// =============================================================
//  pid.h — Correcteur PID discret sur encodeurs
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
// Ce PID corrige la dérive du robot pendant l'avance en ligne droite.
// Signal d'erreur = ticks_gauche - ticks_droite
// Si e > 0 : gauche trop rapide → on la freine
// Si e < 0 : droite trop rapide → on la freine
// =============================================================
#pragma once

#include <Arduino.h>

// ── pid_init ──────────────────────────────────────────────────
// Remet les deux PID à zéro (intégrales, erreurs précédentes).
// À appeler avant chaque mouvement.
void pid_init();

// ── pid_update ────────────────────────────────────────────────
// PID sur encodeurs : corrige la dérive ticks_gauche - ticks_droite.
// Utilisé quand les capteurs latéraux ne voient pas de murs.
//   ticks_left  : compteur encodeur gauche
//   ticks_right : compteur encodeur droit
//   pwm_base    : PWM de base (ex: PWM_RUN1)
//   pwm_left    : [sortie] PWM moteur gauche corrigé
//   pwm_right   : [sortie] PWM moteur droit corrigé
void pid_update(long ticks_left, long ticks_right, int pwm_base,
                int& pwm_left, int& pwm_right);

// ── pid_update_tof ────────────────────────────────────────────
// PID sur capteurs ToF latéraux avec rejet d'outliers et détection
// de passage unilatéral.
//   side_left_mm, side_right_mm : lectures SL/SR en mm (0 si invalide)
//   opening_l_mm, opening_r_mm  : seuils de passage ouvert (calib_get_opening_*)
//   pwm_base                    : PWM de base (ex: PWM_RUN1)
//   pwm_left, pwm_right         : [sortie] PWM corrigés
// Comportement :
//   - Si SL ou SR hors [TOF_SIDE_MIN_MM, TOF_SIDE_MAX_MM] → conserve la
//     dernière correction (évite les secousses sur outlier)
//   - Si SL > opening_l_mm OU SR > opening_r_mm → correction = 0
//     (passage unilatéral : plus de référence latérale fiable)
void pid_update_tof(float side_left_mm, float side_right_mm,
                    int opening_l_mm, int opening_r_mm,
                    int pwm_base, int& pwm_left, int& pwm_right);
