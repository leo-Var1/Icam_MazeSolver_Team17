// =============================================================
//  pid.cpp — Implémentation correcteur PID discret
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include <Arduino.h>
#include "config.h"
#include "pid.h"

// ── Variables internes — PID encodeurs ───────────────────────
static float s_integral   = 0.0f;
static float s_prev_error = 0.0f;

// ── Variables internes — PID ToF ─────────────────────────────
static float s_tof_integral      = 0.0f;
static float s_tof_prev_error    = 0.0f;
static int   s_tof_last_correction = 0;  // fallback outlier : dernière correction valide

// ── pid_init ──────────────────────────────────────────────────
void pid_init() {
    s_integral       = 0.0f;
    s_prev_error     = 0.0f;
    s_tof_integral       = 0.0f;
    s_tof_prev_error     = 0.0f;
    s_tof_last_correction = 0;
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
    float correction = PID_KP * e
                     + PID_KI * s_integral
                     + PID_KD * derivative;

    // Application : on soustrait la correction à gauche, on l'ajoute à droite
    // → si gauche trop rapide (e>0), correction>0 → on freine gauche, on accélère droite
    pwm_left  = constrain((int)(pwm_base - correction), 0, 255);
    pwm_right = constrain((int)(pwm_base + correction), 0, 255);
}

// ── pid_update_tof ────────────────────────────────────────────
void pid_update_tof(float side_left_mm, float side_right_mm,
                    int opening_l_mm, int opening_r_mm,
                    int pwm_base, int& pwm_left, int& pwm_right) {

    // ── Rejet outlier : capteur hors plage physique ───────────
    // Si une lecture est aberrante (bruit électronique, coin, void),
    // on réutilise la dernière correction valide pour éviter les secousses.
    bool sl_ok = (side_left_mm  >= TOF_SIDE_MIN_MM && side_left_mm  <= TOF_SIDE_MAX_MM);
    bool sr_ok = (side_right_mm >= TOF_SIDE_MIN_MM && side_right_mm <= TOF_SIDE_MAX_MM);
    if (!sl_ok || !sr_ok) {
        pwm_left  = constrain(pwm_base - s_tof_last_correction, 0, 255);
        pwm_right = constrain(pwm_base + s_tof_last_correction, 0, 255);
        return;
    }

    // ── Passage unilatéral (ouverture d'un côté) : correction = 0 ──
    // Quand un côté est ouvert (> seuil), il n'y a plus de référence latérale
    // fiable → on avance droit et on laisse l'encodeur PID prendre le relais.
    if (side_left_mm > opening_l_mm || side_right_mm > opening_r_mm) {
        s_tof_last_correction = 0;
        pwm_left  = pwm_base;
        pwm_right = pwm_base;
        return;
    }

    // ── PID classique ─────────────────────────────────────────
    // Erreur = SL - SR : si > 0, trop près du mur gauche → freiner gauche
    float e = side_left_mm - side_right_mm;

    s_tof_integral += e;
    s_tof_integral  = constrain(s_tof_integral, -200.0f, 200.0f);

    float derivative = e - s_tof_prev_error;
    s_tof_prev_error = e;

    float correction = PID_TOF_KP * e
                     + PID_TOF_KI * s_tof_integral
                     + PID_TOF_KD * derivative;

    // Sauvegarde pour fallback outlier (prochaine lecture aberrante)
    s_tof_last_correction = (int)correction;

    pwm_left  = constrain((int)(pwm_base - correction), 0, 255);
    pwm_right = constrain((int)(pwm_base + correction), 0, 255);
}
