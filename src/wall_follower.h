// =============================================================
//  wall_follower.h — Algorithme MAIN DROITE (right-hand rule)
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  Stratégie SECOURS (utilisée si Trémaux n'aboutit pas) :
//  À chaque case, priorité décroissante :
//    1. DROITE ouverte ?  → smooth_turn(+1) puis advance
//    2. DEVANT ouvert ?   → advance
//    3. GAUCHE ouverte ?  → smooth_turn(-1) puis advance
//    4. Cul-de-sac        → marche arrière (demi-tour sans pivot 180°)
//
//  Mise à jour de la carte (murs + visited) comme Trémaux, mais sans
//  la logique de marquage "passage 2 fois". Pause inter-case identique.
//
//  Arrêt : position courante == cible (web_ui_get_target_row/col).
// =============================================================
#pragma once
#include <Arduino.h>

enum WallFollowPhase {
    WF_SCAN,
    WF_DECIDE,
    WF_ORIENT,    // smooth turn 90° en cours
    WF_REVERSE,   // marche arrière en cours
    WF_MOVE,      // advance en cours
    WF_PAUSE,     // pause inter-case
    WF_UPDATE,    // mise à jour position
    WF_FINISHED   // cible atteinte
};

// Initialise depuis (start_row, start_col) facing start_dir (0=N,1=E,2=S,3=W)
void wall_follower_init(uint8_t start_row, uint8_t start_col, uint8_t start_dir = 0);

// À appeler dans loop() — retourne true quand la cible est atteinte
bool wall_follower_update();

// Phase courante (debug)
WallFollowPhase wall_follower_get_phase();
