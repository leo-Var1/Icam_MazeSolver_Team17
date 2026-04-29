// =============================================================
//  pid.cpp — Implémentation correcteur PID discret
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include <Arduino.h>
#include "config.h"
#include "pid.h"
#include "calibration.h"

// ── Variables internes — PID encodeurs ───────────────────────
static float s_integral   = 0.0f;
static float s_prev_error = 0.0f;

// ── Variables internes — PID ToF ─────────────────────────────
static float s_tof_integral      = 0.0f;
static float s_tof_prev_error    = 0.0f;
static int   s_tof_last_correction = 0;  // fallback outlier : dernière correction valide
static bool  s_tof_enabled       = true; // toggle debug (touche 'p' en Serial)

void pid_tof_set_enabled(bool enabled) {
    s_tof_enabled = enabled;
    s_tof_integral = 0.0f;
    s_tof_prev_error = 0.0f;
    s_tof_last_correction = 0;
}
bool pid_tof_is_enabled() { return s_tof_enabled; }

// ── Variables internes — PID pivot ───────────────────────────
static float s_piv_integral   = 0.0f;
static float s_piv_prev_error = 0.0f;

// ── pid_init ──────────────────────────────────────────────────
void pid_init() {
    s_integral       = 0.0f;
    s_prev_error     = 0.0f;
    s_tof_integral       = 0.0f;
    s_tof_prev_error     = 0.0f;
    s_tof_last_correction = 0;
    s_piv_integral   = 0.0f;
    s_piv_prev_error = 0.0f;
}

// ── pid_update ────────────────────────────────────────────────
void pid_update(long ticks_left, long ticks_right, int pwm_base,
                int& pwm_left, int& pwm_right) {

    // Erreur : si e > 0, gauche est en avance → il faut la ralentir
    float e = (float)(ticks_left - ticks_right);

    // Terme intégral : accumule l'erreur dans le temps
    // Clamp anti-windup : on limite pour éviter une saturation de l'intégrale
    s_integral += e;
    s_integral  = constrain(s_integral, -500.0f, 500.0f);

    // Terme dérivé : variation de l'erreur depuis la dernière itération
    float derivative = e - s_prev_error;
    s_prev_error = e;

    // Correction PID
    float correction = calib_get_pid_kp() * e
                     + calib_get_pid_ki() * s_integral
                     + calib_get_pid_kd() * derivative;

    // Application : on soustrait la correction à gauche, on l'ajoute à droite
    // → si gauche trop rapide (e>0), correction>0 → on freine gauche, on accélère droite
    pwm_left  = constrain((int)(pwm_base - correction), 0, 255);
    pwm_right = constrain((int)(pwm_base + correction), 0, 255);
}

// ── pid_update_tof ────────────────────────────────────────────
// 4 cas selon les murs visibles :
//   - 2 ouvertures → ligne droite (encodeurs prennent le relais)
//   - 1 mur seul   → maintient distance cible (calib_center) sur ce mur
//   - 2 murs       → centre via SL-SR
//   - Outlier      → réutilise la dernière correction valide
//
// Avec "1 mur seul" on garantit que le robot se recentre même quand UN
// côté est ouvert (cas typique après un smooth turn où il est biaisé).
void pid_update_tof(float side_left_mm, float side_right_mm,
                    int opening_l_mm, int opening_r_mm,
                    int pwm_base, int& pwm_left, int& pwm_right) {

    // ── Toggle debug ──────────────────────────────────────────
    if (!s_tof_enabled) {
        pwm_left  = pwm_base;
        pwm_right = pwm_base;
        return;
    }

    // ── Rejet outlier : réutilise la dernière correction valide ──
    bool sl_ok = (side_left_mm  >= TOF_SIDE_MIN_MM && side_left_mm  <= TOF_SIDE_MAX_MM);
    bool sr_ok = (side_right_mm >= TOF_SIDE_MIN_MM && side_right_mm <= TOF_SIDE_MAX_MM);
    if (!sl_ok || !sr_ok) {
        pwm_left  = constrain(pwm_base - s_tof_last_correction, 0, 255);
        pwm_right = constrain(pwm_base + s_tof_last_correction, 0, 255);
        return;
    }

    // ── Détection des ouvertures ──────────────────────────────
    bool sl_open = (side_left_mm  > (float)opening_l_mm);
    bool sr_open = (side_right_mm > (float)opening_r_mm);

    float e;
    if (sl_open && sr_open) {
        // Cas 1 : 2 ouvertures → ligne droite
        e = 0.0f;
    } else if (sl_open) {
        // Cas 2 : ouverture gauche → cale sur le mur DROIT
        // Si SR > target : trop loin du mur droit → tourner à droite (e < 0)
        e = (float)calib_get_center() - side_right_mm;
    } else if (sr_open) {
        // Cas 3 : ouverture droite → cale sur le mur GAUCHE
        // Si SL > target : trop loin du mur gauche → tourner à gauche (e > 0)
        e = side_left_mm - (float)calib_get_center();
    } else {
        // Cas 4 : 2 murs → centrage SL-SR
        e = side_left_mm - side_right_mm;
    }

    s_tof_integral += e;
    s_tof_integral  = constrain(s_tof_integral, -200.0f, 200.0f);

    float derivative = e - s_tof_prev_error;
    s_tof_prev_error = e;

    float correction = calib_get_pid_tof_kp() * e
                     + calib_get_pid_tof_ki() * s_tof_integral
                     + calib_get_pid_tof_kd() * derivative;
    correction = constrain(correction, -(float)calib_get_pid_tof_max_corr(), (float)calib_get_pid_tof_max_corr());
    s_tof_last_correction = (int)correction;

    pwm_left  = constrain((int)(pwm_base - correction), 0, 255);
    pwm_right = constrain((int)(pwm_base + correction), 0, 255);
}

// ── pid_update_pivot ─────────────────────────────────────────
void pid_update_pivot(long ticks_left, long ticks_right, int direction,
                      int pwm_base, int& pwm_left, int& pwm_right) {

    // Erreur sur les valeurs absolues : on veut que les deux roues parcourent
    // le même nombre de ticks (en sens opposés) pour pivoter autour du centre.
    // e > 0 : roue gauche en avance → la freiner, accélérer la droite.
    float e = (float)(labs(ticks_left) - labs(ticks_right));

    s_piv_integral += e;
    s_piv_integral  = constrain(s_piv_integral, -300.0f, 300.0f);

    float derivative = e - s_piv_prev_error;
    s_piv_prev_error = e;

    float correction = calib_get_pid_piv_kp() * e
                     + calib_get_pid_piv_ki() * s_piv_integral
                     + calib_get_pid_piv_kd() * derivative;
    correction = constrain(correction, -(float)calib_get_pid_piv_max_corr(), (float)calib_get_pid_piv_max_corr());

    // Magnitudes corrigées (toujours ≥ PWM minimum pour ne pas staller)
    int mag_l = constrain((int)(pwm_base - correction), PID_PIVOT_MIN_PWM, 255);
    int mag_r = constrain((int)(pwm_base + correction), PID_PIVOT_MIN_PWM, 255);

    // Application des signes selon la direction du pivot.
    //   droite (+1) : gauche en avant, droite en arrière
    //   gauche (-1) : gauche en arrière, droite en avant
    if (direction > 0) {
        pwm_left  =  mag_l;
        pwm_right = -mag_r;
    } else {
        pwm_left  = -mag_l;
        pwm_right =  mag_r;
    }
}
