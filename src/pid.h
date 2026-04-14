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
// Remet le PID à zéro (intégrale, erreur précédente).
// À appeler avant chaque mouvement.
void pid_init();

// ── pid_update ────────────────────────────────────────────────
// Calcule la correction à partir de l'erreur actuelle.
// Paramètres :
//   ticks_left  : compteur encodeur gauche
//   ticks_right : compteur encodeur droit
//   pwm_base    : PWM de base (ex: PWM_RUN1)
//   pwm_left    : [sortie] PWM moteur gauche corrigé
//   pwm_right   : [sortie] PWM moteur droit corrigé
void pid_update(long ticks_left, long ticks_right, int pwm_base,
                int& pwm_left, int& pwm_right);
