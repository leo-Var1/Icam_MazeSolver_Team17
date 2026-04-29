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
    WEB_CMD_RESET_IDLE, // Retour à IDLE depuis EMERGENCY (relance possible)
    WEB_CMD_START1,     // Lancer Run 1 (Trémaux)
    WEB_CMD_START2,     // Lancer Run 2 (BFS)
    WEB_CMD_START_WALL_R, // Lancer Main Droite (fallback)
    WEB_CMD_MOVE_UP,       // Avance d'une case (touche ↑)
    WEB_CMD_MOVE_DOWN,     // Demi-tour 180°      (touche ↓)
    WEB_CMD_MOVE_LEFT,     // Virage gauche 90°   (touche ←)
    WEB_CMD_MOVE_RIGHT,    // Virage droite 90°   (touche →)
    WEB_CMD_SMOOTH_L,      // Virage fluide gauche
    WEB_CMD_SMOOTH_R,      // Virage fluide droite
    // ── Calibration (uniquement en STATE_IDLE) ────────────────
    WEB_CMD_CALIB_CENTER,    // Capture distance latérale centrée
    WEB_CMD_CALIB_TURN,      // Capture distance frontale d'arrêt
    WEB_CMD_CALIB_OPENING_L, // Capture seuil passage gauche
    WEB_CMD_CALIB_OPENING_R, // Capture seuil passage droit
    WEB_CMD_CALIB_RESET,     // Remet les seuils à leurs valeurs par défaut
    WEB_CMD_CALIB_POST,      // Capture seuil détection poteau alu
    WEB_CMD_CALIB_WALL_L,        // Capture distance mur gauche (SL)
    WEB_CMD_CALIB_WALL_R,        // Capture distance mur droit (SR)
    WEB_CMD_CALIB_SET_PIVOT_90,  // Saisie manuelle ticks pivot 90° (s_pending_value)
    WEB_CMD_CALIB_SET_CELL,      // Saisie manuelle ticks par case  (s_pending_value)
    WEB_CMD_CALIB_SET_PIV45_R,   // Ticks pivot 45° droite (smooth turn)
    WEB_CMD_CALIB_SET_PIV45_L,   // Ticks pivot 45° gauche (smooth turn)
    WEB_CMD_CALIB_SET_SMOOTH_MOVE,   // Ticks avance entre les 2 pivots 45°
    WEB_CMD_CALIB_SET_SMOOTH_CENTER, // Distance frontale finale (mm)
    // ── PID tuning (valeurs * 1000 pour les floats) ───────────
    WEB_CMD_CALIB_SET_PID_KP,
    WEB_CMD_CALIB_SET_PID_KI,
    WEB_CMD_CALIB_SET_PID_KD,
    WEB_CMD_CALIB_SET_PID_TOF_KP,
    WEB_CMD_CALIB_SET_PID_TOF_KI,
    WEB_CMD_CALIB_SET_PID_TOF_KD,
    WEB_CMD_CALIB_SET_PID_TOF_MAX_CORR,
    WEB_CMD_CALIB_SET_PID_PIV_KP,
    WEB_CMD_CALIB_SET_PID_PIV_KI,
    WEB_CMD_CALIB_SET_PID_PIV_KD,
    WEB_CMD_CALIB_SET_PID_PIV_MAX_CORR
};

// Récupère la valeur associée à la dernière commande (ex: ticks pour SET_*).
// À appeler dans le même cycle que web_ui_poll_cmd().
int web_ui_last_value();

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

// Orientation initiale du robot (0=N, 1=E, 2=S, 3=W). Défaut : 0=N.
uint8_t web_ui_get_start_dir();
