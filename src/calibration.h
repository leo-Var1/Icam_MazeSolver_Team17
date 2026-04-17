// =============================================================
//  calibration.h — Seuils ToF fonctionnels ajustables
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//  Stocke 5 seuils en RAM + persistance LittleFS (/calib.json) :
//   center, turn, opening_l, opening_r, post_detect
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

// Persistance
bool calib_save();              // écrit /calib.json
bool calib_reset_defaults();    // remet valeurs config.h + save
