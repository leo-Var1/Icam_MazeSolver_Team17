// =============================================================
//  calibration.h — Seuils ToF fonctionnels ajustables
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//  Stocke 4 seuils (center, turn, opening_l, opening_r) en RAM
//  + persistance LittleFS (/calib.json).
//  Au boot : charge /calib.json si présent, sinon applique les
//  défauts CALIB_TOF_*_MM de config.h.
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

// Capture : échantillonne CALIB_SAMPLES lectures ToF (séparées de CALIB_SAMPLE_MS ms),
// calcule la moyenne, met à jour la valeur interne.
// ⚠ Bloque ~400 ms : n'appeler qu'en STATE_IDLE.
// Retourne true si succès (≥ 50% échantillons valides).
bool calib_capture_center();       // moyenne (SL + SR) / 2
bool calib_capture_turn();         // moyenne (FL + FR) / 2
bool calib_capture_opening_l();    // moyenne SL
bool calib_capture_opening_r();    // moyenne SR

// Persistance
bool calib_save();              // écrit /calib.json
bool calib_reset_defaults();    // remet valeurs config.h + save
