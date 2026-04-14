// =============================================================
//  pid.cpp — Implémentation correcteur PID discret
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include <Arduino.h>
#include "config.h"
#include "pid.h"

// ── Variables internes du PID ─────────────────────────────────
static float s_integral   = 0.0f;  // somme des erreurs (terme I)
static float s_prev_error = 0.0f;  // erreur à t-1 (terme D)

// ── pid_init ──────────────────────────────────────────────────
void pid_init() {
    s_integral   = 0.0f;
    s_prev_error = 0.0f;
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
