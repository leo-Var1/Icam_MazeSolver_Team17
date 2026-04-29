#pragma once

// =============================================================
//  encoders.h — Interface module encodeurs (ISR + comptage ticks)
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  Chaque moteur a deux signaux :
//    - Phase A : déclenche une interruption à chaque front montant
//    - Phase B : lu dans l'ISR pour déterminer le sens (avant/arrière)
//
//  Encodeur GAUCHE : ENC_L_A (D5/pin14) + ENC_L_B (D0/pin16)
//  Encodeur DROIT  : ENC_R_A (D6/pin12) + Phase B sur A0 (analogRead>512)
//
//  Usage :
//    encoders_init();                    // dans setup()
//    long tL = encoders_get_left();      // ticks depuis le dernier reset
//    long tR = encoders_get_right();
//    encoders_reset();                   // remet les deux compteurs à 0

// ── Initialisation ───────────────────────────────────────────
// Attache les interruptions sur ENC_L_A et ENC_R_A.
// ⚠️ Wire.begin() et la config des pins doivent être faits avant.
void encoders_init();

// ── Lecture des compteurs ────────────────────────────────────
// Retourne le nombre de ticks depuis le dernier reset (ou depuis init).
// Valeur positive = avance, négative = recul.
long encoders_get_left();
long encoders_get_right();

// ── Remise à zéro ────────────────────────────────────────────
// Remet les deux compteurs à 0 (typiquement avant une manœuvre mesurée).
void encoders_reset();
