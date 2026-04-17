// =============================================================
//  web_ui.h — Interface Web WiFi (Access Point)
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  Principe :
//  - Démarre un Access Point WiFi (SSID/MDP dans config.h)
//  - Sert une page HTML sur http://192.168.4.1
//  - Les handlers HTTP async NE font AUCUN travail bloquant :
//    ils se contentent de positionner des FLAGS qui sont consommés
//    par web_ui_update() dans la loop() principale.
//
//  Cycle type :
//    setup() : web_ui_init()
//    loop()  : web_ui_update()  → consomme les flags et appelle
//              les fonctions nav_* correspondantes
// =============================================================
#pragma once

#include <Arduino.h>
#include "navigation.h"

// ── Commandes reçues depuis l'IHM (flags posés par les handlers) ──
enum WebCmd {
    WEB_CMD_NONE,
    WEB_CMD_STOP,       // Arrêt d'urgence (priorité max)
    WEB_CMD_START1,     // Lancer Run 1 (Trémaux)
    WEB_CMD_START2,     // Lancer Run 2 (BFS)
    WEB_CMD_MOVE_UP,       // Avance d'une case (touche ↑)
    WEB_CMD_MOVE_DOWN,     // Demi-tour 180°      (touche ↓)
    WEB_CMD_MOVE_LEFT,     // Virage gauche 90°   (touche ←)
    WEB_CMD_MOVE_RIGHT,    // Virage droite 90°   (touche →)
    // ── Calibration (uniquement en STATE_IDLE) ────────────────
    WEB_CMD_CALIB_CENTER,    // Capture distance latérale centrée
    WEB_CMD_CALIB_TURN,      // Capture distance frontale d'arrêt
    WEB_CMD_CALIB_OPENING_L, // Capture seuil passage gauche
    WEB_CMD_CALIB_OPENING_R, // Capture seuil passage droit
    WEB_CMD_CALIB_RESET      // Remet les seuils à leurs valeurs par défaut
};

// ── État global exposé à l'IHM ─────────────────────────────────
// Rempli par main.cpp puis lu par le handler /state (JSON)
struct WebState {
    uint8_t robot_state;   // RobotState (STATE_IDLE, STATE_RUN1, ...)
    float   heading;       // cap IMU en degrés
    int     tof_fl;
    int     tof_fr;
    int     tof_sl;
    int     tof_sr;
};

// ── Init ───────────────────────────────────────────────────────
// Démarre l'AP WiFi et le serveur web async.
void web_ui_init();

// ── À appeler dans loop() ──────────────────────────────────────
// Met à jour l'état partagé (pour les réponses /state).
void web_ui_set_state(const WebState& st);

// Récupère la prochaine commande en attente (puis la consomme).
// Retourne WEB_CMD_NONE si rien à traiter.
WebCmd web_ui_poll_cmd();

// ── Target (case d'arrivée choisie par l'utilisateur) ──────────
// Valeur par défaut : (4, 4) = coin bas-droit.
// Modifiée via POST /target {row, col}.
uint8_t web_ui_get_target_row();
uint8_t web_ui_get_target_col();

// ── Start (case de départ choisie par l'utilisateur) ───────────
// Valeur par défaut : (0, 0) = coin haut-gauche.
// Modifiée via POST /start_pos {row, col}.
uint8_t web_ui_get_start_row();
uint8_t web_ui_get_start_col();
