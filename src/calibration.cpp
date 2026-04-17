// =============================================================
//  calibration.cpp — Seuils ToF fonctionnels + persistance LittleFS
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
#include "calibration.h"
#include "config.h"
#include "sensors.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

static const char* CALIB_PATH = "/calib.json";

// ── État interne (valeurs courantes en mm) ────────────────────
static int s_center    = CALIB_TOF_CENTER_MM;
static int s_turn      = CALIB_TOF_TURN_MM;
static int s_opening_l = CALIB_TOF_OPENING_L_MM;
static int s_opening_r = CALIB_TOF_OPENING_R_MM;

// ── calibration_init ──────────────────────────────────────────
// Monte LittleFS, lit /calib.json si présent.
// Retourne true si les seuils ont été chargés depuis le fichier.
bool calibration_init() {
    if (!LittleFS.begin()) {
        Serial.println("[CALIB] LittleFS mount FAIL — défauts utilisés");
        return false;
    }
    if (!LittleFS.exists(CALIB_PATH)) {
        Serial.println("[CALIB] /calib.json absent — défauts utilisés");
        return false;
    }
    File f = LittleFS.open(CALIB_PATH, "r");
    if (!f) {
        Serial.println("[CALIB] open FAIL — défauts utilisés");
        return false;
    }
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) {
        Serial.print("[CALIB] JSON parse err: "); Serial.println(err.c_str());
        return false;
    }
    s_center    = doc["center"]    | CALIB_TOF_CENTER_MM;
    s_turn      = doc["turn"]      | CALIB_TOF_TURN_MM;
    s_opening_l = doc["opening_l"] | CALIB_TOF_OPENING_L_MM;
    s_opening_r = doc["opening_r"] | CALIB_TOF_OPENING_R_MM;
    Serial.print("[CALIB] chargé: C="); Serial.print(s_center);
    Serial.print(" T=");              Serial.print(s_turn);
    Serial.print(" OL=");             Serial.print(s_opening_l);
    Serial.print(" OR=");             Serial.println(s_opening_r);
    return true;
}

// ── Getters ───────────────────────────────────────────────────
int calib_get_center()    { return s_center; }
int calib_get_turn()      { return s_turn; }
int calib_get_opening_l() { return s_opening_l; }
int calib_get_opening_r() { return s_opening_r; }

// ── Capture générique (moyenne filtrée sur CALIB_SAMPLES lectures) ──
// selector : 0=(SL+SR)/2  1=(FL+FR)/2  2=SL  3=SR
// ⚠ Bloque (CALIB_SAMPLES × CALIB_SAMPLE_MS) ms ≈ 400ms — STATE_IDLE uniquement.
static bool capture_avg(int selector, int& out_value) {
    long sum = 0;
    int  n   = 0;
    for (int i = 0; i < CALIB_SAMPLES; ++i) {
        ToFReadings t;
        sensors_read(t);
        int v = 0;
        switch (selector) {
            case 0: // (SL + SR) / 2
                if (t.side_left > 0 && t.side_right > 0)
                    v = (int)(t.side_left + t.side_right) / 2;
                break;
            case 1: // (FL + FR) / 2
                if (t.front_left > 0 && t.front_right > 0)
                    v = (int)(t.front_left + t.front_right) / 2;
                break;
            case 2: // SL seul
                v = t.side_left;
                break;
            case 3: // SR seul
                v = t.side_right;
                break;
        }
        if (v > 0) { sum += v; n++; }
        delay(CALIB_SAMPLE_MS);
    }
    if (n < CALIB_SAMPLES / 2) {
        Serial.print("[CALIB] échec: ");
        Serial.print(n); Serial.print("/"); Serial.print(CALIB_SAMPLES);
        Serial.println(" échantillons valides");
        return false;
    }
    out_value = (int)(sum / n);
    return true;
}

bool calib_capture_center() {
    int v;
    if (!capture_avg(0, v)) return false;
    s_center = v;
    Serial.print("[CALIB] center = "); Serial.print(v); Serial.println(" mm");
    return true;
}
bool calib_capture_turn() {
    int v;
    if (!capture_avg(1, v)) return false;
    s_turn = v;
    Serial.print("[CALIB] turn = "); Serial.print(v); Serial.println(" mm");
    return true;
}
bool calib_capture_opening_l() {
    int v;
    if (!capture_avg(2, v)) return false;
    s_opening_l = v;
    Serial.print("[CALIB] opening_l = "); Serial.print(v); Serial.println(" mm");
    return true;
}
bool calib_capture_opening_r() {
    int v;
    if (!capture_avg(3, v)) return false;
    s_opening_r = v;
    Serial.print("[CALIB] opening_r = "); Serial.print(v); Serial.println(" mm");
    return true;
}

// ── calib_save ────────────────────────────────────────────────
bool calib_save() {
    StaticJsonDocument<128> doc;
    doc["center"]    = s_center;
    doc["turn"]      = s_turn;
    doc["opening_l"] = s_opening_l;
    doc["opening_r"] = s_opening_r;
    File f = LittleFS.open(CALIB_PATH, "w");
    if (!f) {
        Serial.println("[CALIB] save open FAIL");
        return false;
    }
    if (serializeJson(doc, f) == 0) {
        Serial.println("[CALIB] save write FAIL");
        f.close();
        return false;
    }
    f.close();
    Serial.println("[CALIB] /calib.json sauvegardé");
    return true;
}

// ── calib_reset_defaults ──────────────────────────────────────
bool calib_reset_defaults() {
    s_center    = CALIB_TOF_CENTER_MM;
    s_turn      = CALIB_TOF_TURN_MM;
    s_opening_l = CALIB_TOF_OPENING_L_MM;
    s_opening_r = CALIB_TOF_OPENING_R_MM;
    Serial.println("[CALIB] défauts restaurés");
    return calib_save();
}
