// =============================================================
//  calibration.h — Seuils ToF fonctionnels ajustables
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//  Stocke 7 seuils en RAM + persistance LittleFS (/calib.json) :
//   center, turn, opening_l, opening_r, post_detect, wall_l, wall_r
//  Au boot : charge /calib.json si présent, sinon défauts config.h.
// =============================================================
#pragma once
#include <stdint.h>

// Initialise LittleFS et charge les seuils.
// Retourne true si /calib.json a été lu, false si défauts appliqués.
bool calibration_init();

// Getters (retournent la valeur courante en mm)
int calib_get_center();
int calib_get_turn();
int calib_get_opening_l();
int calib_get_opening_r();
int calib_get_post_detect();   // seuil détection poteau (côté intérieur du virage)
int calib_get_wall_l();        // distance SL quand mur gauche présent
int calib_get_wall_r();        // distance SR quand mur droit présent
int calib_get_pivot_90_ticks();// ticks encodeur pour un pivot 90°
int calib_get_cell_ticks();    // ticks encodeur pour traverser 1 case (220mm)

// Smooth turn (45° - avance - 45° - centrage)
int calib_get_pivot_45_r();      // ticks pivot 45° côté DROITE
int calib_get_pivot_45_l();      // ticks pivot 45° côté GAUCHE
int calib_get_smooth_move();     // ticks d'avance entre les 2 pivots 45°
int calib_get_smooth_center();   // distance frontale finale (mm)

// --- PID parameters ---
// Encoder PID
float calib_get_pid_kp();
float calib_get_pid_ki();
float calib_get_pid_kd();
// ToF PID
float calib_get_pid_tof_kp();
float calib_get_pid_tof_ki();
float calib_get_pid_tof_kd();
int   calib_get_pid_tof_max_corr();
// Pivot PID
float calib_get_pid_piv_kp();
float calib_get_pid_piv_ki();
float calib_get_pid_piv_kd();
int   calib_get_pid_piv_max_corr();

// Setters (sauvegarde immédiate dans /calib.json)
void calib_set_pivot_90_ticks(int v);
void calib_set_cell_ticks(int v);
void calib_set_pivot_45_r(int v);
void calib_set_pivot_45_l(int v);
void calib_set_smooth_move(int v);
void calib_set_smooth_center(int v);

// Setters PID
void calib_set_pid_kp(float v);
void calib_set_pid_ki(float v);
void calib_set_pid_kd(float v);
void calib_set_pid_tof_kp(float v);
void calib_set_pid_tof_ki(float v);
void calib_set_pid_tof_kd(float v);
void calib_set_pid_tof_max_corr(int v);
void calib_set_pid_piv_kp(float v);
void calib_set_pid_piv_ki(float v);
void calib_set_pid_piv_kd(float v);
void calib_set_pid_piv_max_corr(int v);

// Capture : échantillonne CALIB_SAMPLES lectures ToF, calcule médiane/moyenne.
// ⚠ Bloque ~400 ms : n'appeler qu'en STATE_IDLE.
// Retourne true si succès (≥ 50% échantillons valides).
bool calib_capture_center();       // moyenne (SL + SR) / 2
bool calib_capture_turn();         // moyenne (FL + FR) / 2
bool calib_capture_opening_l();    // moyenne SL
bool calib_capture_opening_r();    // moyenne SR
// Poteau : médiane de MIN(SL,SR) + marge CALIB_POST_DETECT_MARGIN mm.
// Placer le robot à côté d'un poteau alu avant de capturer.
bool calib_capture_post_detect();
// Mur latéral : placer le robot dans un couloir avec mur des deux côtés.
bool calib_capture_wall_l();   // capture SL avec mur gauche présent
bool calib_capture_wall_r();   // capture SR avec mur droit présent
// Persistance
bool calib_save();              // écrit /calib.json
bool calib_reset_defaults();    // remet valeurs config.h + save
