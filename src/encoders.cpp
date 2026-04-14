// =============================================================
//  encoders.cpp — Implémentation encodeurs moteurs
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include <Arduino.h>
#include "config.h"
#include "encoders.h"

// ── Compteurs de ticks (partagés entre ISR et code principal) ─
// volatile : indique au compilateur que ces variables peuvent changer
// à tout moment (dans une ISR), donc ne pas les mettre en cache registre.
static volatile long s_ticks_left  = 0;
static volatile long s_ticks_right = 0;

// ── ISR encodeur GAUCHE ───────────────────────────────────────
// Déclenchée sur chaque front montant de ENC_L_A (D5/pin14).
// On lit ENC_L_B pour déterminer le sens de rotation :
//   B=HIGH au moment du front A → sens avant (+1)
//   B=LOW  au moment du front A → sens arrière (-1)
// IRAM_ATTR : place la fonction en RAM (obligatoire pour les ISR sur ESP8266)
IRAM_ATTR void isr_encoder_left() {
    // Phase B inversée physiquement → on inverse la logique pour compter positif en avant
    if (digitalRead(ENC_L_B) == HIGH) {
        s_ticks_left--;
    } else {
        s_ticks_left++;
    }
}

// ── ISR encodeur DROIT ────────────────────────────────────────
// Déclenchée sur chaque front montant de ENC_R_A (D6/pin12).
// Phase B droit = A0 analogique → on lit analogRead et compare à 512
// (tension 3.3V sur ADC 1V max, mais on lit uniquement HIGH/LOW logique)
// ⚠️ analogRead() dans une ISR est lent (~17µs) mais acceptable ici
//    car on n'a pas d'autre choix avec l'ESP8266 (A0 = pas d'interruption)
IRAM_ATTR void isr_encoder_right() {
    // analogRead > 512 équivaut à un niveau logique HIGH
    if (analogRead(A0) > 512) {
        s_ticks_right++;
    } else {
        s_ticks_right--;
    }
}

// ── encoders_init ─────────────────────────────────────────────
void encoders_init() {
    // Phase B gauche : entrée numérique standard
    pinMode(ENC_L_B, INPUT);

    // Phase A gauche : interruption sur front montant
    // RISING = déclenche quand le signal passe de LOW à HIGH
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_encoder_left, RISING);

    // Phase B droit : A0 est configuré en entrée analogique par défaut
    // (pas de pinMode nécessaire)

    // Phase A droit : interruption sur front montant
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_encoder_right, RISING);

    Serial.println("[ENC] Encodeurs initialisés (ISR actives)");
}

// ── encoders_get_left ─────────────────────────────────────────
long encoders_get_left() {
    // Lecture atomique : on désactive les interruptions le temps de copier
    // la valeur 32 bits (sur ESP8266 une lecture 32 bits n'est pas atomique)
    noInterrupts();
    long val = s_ticks_left;
    interrupts();
    return val;
}

// ── encoders_get_right ────────────────────────────────────────
long encoders_get_right() {
    noInterrupts();
    long val = s_ticks_right;
    interrupts();
    return val;
}

// ── encoders_reset ────────────────────────────────────────────
void encoders_reset() {
    noInterrupts();
    s_ticks_left  = 0;
    s_ticks_right = 0;
    interrupts();
}
