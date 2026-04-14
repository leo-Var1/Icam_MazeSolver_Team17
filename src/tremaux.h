// =============================================================
//  tremaux.h — Algorithme de Trémaux (Run 1 — exploration)
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  L'algorithme de Trémaux explore le labyrinthe case par case.
//  Règle fondamentale : ne jamais emprunter un passage marqué 2 fois.
//  À chaque intersection, on choisit le passage le moins visité.
//
//  Cycle par case :
//    SCAN     → lire les capteurs ToF, mettre à jour les murs de la carte
//    DECIDE   → choisir la prochaine direction (Trémaux)
//    ORIENT   → tourner vers cette direction (nav_start_turn)
//    MOVE     → avancer d'une case (nav_start_advance)
//    UPDATE   → mettre à jour la position sur la carte
//
//  Usage :
//    tremaux_init();
//    // dans loop() :
//    if (tremaux_update()) { /* exploration terminée */ }
// =============================================================
#pragma once
#include <Arduino.h>

enum TremauxPhase {
    TREM_SCAN,       // Lecture des capteurs + mise à jour carte
    TREM_DECIDE,     // Choix de la prochaine direction
    TREM_ORIENT,     // Rotation vers la direction choisie
    TREM_MOVE,       // Avance d'une case
    TREM_UPDATE,     // Mise à jour position dans la carte
    TREM_FINISHED    // Exploration terminée (toutes les cases visitées)
};

// Initialise l'algorithme — appeler avant le Run 1
void tremaux_init();

// Exécute une étape de l'algorithme — appeler dans loop()
// Retourne true quand l'exploration est terminée
bool tremaux_update();

// Retourne la phase actuelle (pour debug)
TremauxPhase tremaux_get_phase();
