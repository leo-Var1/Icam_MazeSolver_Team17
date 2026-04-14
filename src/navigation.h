// =============================================================
//  navigation.h — Primitives de mouvement du robot
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  Toutes les fonctions sont NON-BLOQUANTES :
//  elles démarrent un mouvement puis retournent NAV_BUSY.
//  Il faut appeler nav_update() dans loop() jusqu'à NAV_DONE.
//
//  Exemple d'utilisation (avance d'une case) :
//    nav_start_advance();
//    // dans loop() :
//    if (nav_update() == NAV_DONE) { /* mouvement terminé */ }
//
//  Primitives disponibles :
//    nav_start_advance()       → avance de 1 case (200mm), PID encodeurs
//    nav_start_turn(quarters)  → rotation sur place (+1=droite, -1=gauche, +2=180°)
//    nav_start_auto_align()    → recalage frontal contre le mur (si présent)
//    nav_update()              → à appeler dans loop() — retourne NAV_BUSY ou NAV_DONE
//    nav_abort()               → frein immédiat, annule le mouvement
//    nav_get_state()           → état actuel
// =============================================================
#pragma once

#include <Arduino.h>

// ── État de la navigation ────────────────────────────────────
enum NavState {
    NAV_IDLE,   // Aucun mouvement en cours
    NAV_BUSY,   // Mouvement en cours — continuer à appeler nav_update()
    NAV_DONE    // Mouvement terminé — prêt pour le prochain
};

// ── Initialisation ───────────────────────────────────────────
// À appeler dans setup() après motors_init() et encoders_init()
void nav_init();

// ── Démarrage des mouvements ─────────────────────────────────

// Avance d'une case (TICKS_PER_CELL ticks ≈ 200mm)
// Utilise le PID encodeurs pour rester droit
// S'arrête aussi si un mur est détecté à < TOF_STOP_FRONT_MM
void nav_start_advance();

// Rotation sur place
// quarters : +1 = droite (horaire), -1 = gauche (antihoraire), +2 = demi-tour
// Utilise l'IMU (gyro) pour détecter la fin de rotation
void nav_start_turn(int quarters);

// Auto-alignement frontal
// Si un mur est présent devant : avance/recule pour atteindre TOF_ALIGN_TARGET_MM
// et corrige l'angle (FL ≈ FR).
// Si pas de mur devant : retourne NAV_DONE immédiatement
void nav_start_auto_align();

// ── Mise à jour (à appeler dans loop()) ──────────────────────
// Retourne NAV_BUSY tant que le mouvement est en cours,
// NAV_DONE quand il est terminé.
// ⚠️ Après NAV_DONE, l'état repasse à NAV_IDLE
NavState nav_update();

// ── Arrêt d'urgence ──────────────────────────────────────────
// Freine immédiatement et annule le mouvement en cours
void nav_abort();

// ── Lecture de l'état ────────────────────────────────────────
NavState nav_get_state();
