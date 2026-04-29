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
static int s_center       = CALIB_TOF_CENTER_MM;
static int s_turn         = CALIB_TOF_TURN_MM;
static int s_opening_l    = CALIB_TOF_OPENING_L_MM;
static int s_opening_r    = CALIB_TOF_OPENING_R_MM;
static int s_post_detect  = CALIB_TOF_POST_MM;
static int s_wall_l       = TOF_WALL_SIDE_MM;
static int s_wall_r       = TOF_WALL_SIDE_MM;
static int s_pivot_90_ticks = TICKS_PIVOT_90;
static int s_cell_ticks     = TICKS_PER_CELL;
static int s_pivot_45_r     = TICKS_PIVOT_45_R;
static int s_pivot_45_l     = TICKS_PIVOT_45_L;
static int s_smooth_move    = TICKS_SPECIAL_MOVE;
static int s_smooth_center  = SMOOTH_CENTER_TARGET_MM;

// PID parameters
static float s_pid_kp = PID_KP;
static float s_pid_ki = PID_KI;
static float s_pid_kd = PID_KD;
static float s_pid_tof_kp = PID_TOF_KP;
static float s_pid_tof_ki = PID_TOF_KI;
static float s_pid_tof_kd = PID_TOF_KD;
static int   s_pid_tof_max_corr = PID_TOF_MAX_CORR;
static float s_pid_piv_kp = PID_PIVOT_KP;
static float s_pid_piv_ki = PID_PIVOT_KI;
static float s_pid_piv_kd = PID_PIVOT_KD;
static int   s_pid_piv_max_corr = PID_PIVOT_MAX_CORR;

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
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) {
        Serial.print("[CALIB] JSON parse err: "); Serial.println(err.c_str());
        return false;
    }
    s_center      = doc["center"]       | CALIB_TOF_CENTER_MM;
    s_turn        = doc["turn"]         | CALIB_TOF_TURN_MM;
    s_opening_l   = doc["opening_l"]    | CALIB_TOF_OPENING_L_MM;
    s_opening_r   = doc["opening_r"]    | CALIB_TOF_OPENING_R_MM;
    s_post_detect = doc["post_detect"]  | CALIB_TOF_POST_MM;
    s_wall_l         = doc["wall_l"]          | TOF_WALL_SIDE_MM;
    s_wall_r         = doc["wall_r"]          | TOF_WALL_SIDE_MM;
    s_pivot_90_ticks = doc["pivot_90_ticks"]  | TICKS_PIVOT_90;
    s_cell_ticks     = doc["cell_ticks"]      | TICKS_PER_CELL;
    s_pivot_45_r     = doc["pivot_45_r"]      | TICKS_PIVOT_45_R;
    s_pivot_45_l     = doc["pivot_45_l"]      | TICKS_PIVOT_45_L;
    s_smooth_move    = doc["smooth_move"]     | TICKS_SPECIAL_MOVE;
    s_smooth_center  = doc["smooth_center"]   | SMOOTH_CENTER_TARGET_MM;

    s_pid_kp = doc["pid_kp"] | PID_KP;
    s_pid_ki = doc["pid_ki"] | PID_KI;
    s_pid_kd = doc["pid_kd"] | PID_KD;
    s_pid_tof_kp = doc["pid_tof_kp"] | PID_TOF_KP;
    s_pid_tof_ki = doc["pid_tof_ki"] | PID_TOF_KI;
    s_pid_tof_kd = doc["pid_tof_kd"] | PID_TOF_KD;
    s_pid_tof_max_corr = doc["pid_tof_max_corr"] | PID_TOF_MAX_CORR;
    s_pid_piv_kp = doc["pid_piv_kp"] | PID_PIVOT_KP;
    s_pid_piv_ki = doc["pid_piv_ki"] | PID_PIVOT_KI;
    s_pid_piv_kd = doc["pid_piv_kd"] | PID_PIVOT_KD;
    s_pid_piv_max_corr = doc["pid_piv_max_corr"] | PID_PIVOT_MAX_CORR;

    Serial.print("[CALIB] chargé: C="); Serial.print(s_center);
    Serial.print(" T=");              Serial.print(s_turn);
    Serial.print(" OL=");             Serial.print(s_opening_l);
    Serial.print(" OR=");             Serial.print(s_opening_r);
    Serial.print(" POST=");           Serial.print(s_post_detect);
    Serial.print(" WL=");             Serial.print(s_wall_l);
    Serial.print(" WR=");             Serial.println(s_wall_r);
    return true;
}

// ── Getters ───────────────────────────────────────────────────
int calib_get_center()      { return s_center; }
int calib_get_turn()        { return s_turn; }
int calib_get_opening_l()   { return s_opening_l; }
int calib_get_opening_r()   { return s_opening_r; }
int calib_get_post_detect() { return s_post_detect; }
int calib_get_wall_l()         { return s_wall_l; }
int calib_get_wall_r()         { return s_wall_r; }
int calib_get_pivot_90_ticks() { return s_pivot_90_ticks; }
int calib_get_cell_ticks()     { return s_cell_ticks; }
int calib_get_pivot_45_r()     { return s_pivot_45_r; }
int calib_get_pivot_45_l()     { return s_pivot_45_l; }
int calib_get_smooth_move()    { return s_smooth_move; }
int calib_get_smooth_center()  { return s_smooth_center; }
void calib_set_pivot_90_ticks(int v) { s_pivot_90_ticks = v; calib_save(); }
void calib_set_cell_ticks(int v)     { s_cell_ticks     = v; calib_save(); }
void calib_set_pivot_45_r(int v)     { s_pivot_45_r     = v; calib_save(); }
void calib_set_pivot_45_l(int v)     { s_pivot_45_l     = v; calib_save(); }
void calib_set_smooth_move(int v)    { s_smooth_move    = v; calib_save(); }
void calib_set_smooth_center(int v)  { s_smooth_center  = v; calib_save(); }

float calib_get_pid_kp() { return s_pid_kp; }
float calib_get_pid_ki() { return s_pid_ki; }
float calib_get_pid_kd() { return s_pid_kd; }
float calib_get_pid_tof_kp() { return s_pid_tof_kp; }
float calib_get_pid_tof_ki() { return s_pid_tof_ki; }
float calib_get_pid_tof_kd() { return s_pid_tof_kd; }
int   calib_get_pid_tof_max_corr() { return s_pid_tof_max_corr; }
float calib_get_pid_piv_kp() { return s_pid_piv_kp; }
float calib_get_pid_piv_ki() { return s_pid_piv_ki; }
float calib_get_pid_piv_kd() { return s_pid_piv_kd; }
int   calib_get_pid_piv_max_corr() { return s_pid_piv_max_corr; }

void calib_set_pid_kp(float v) { s_pid_kp = v; calib_save(); }
void calib_set_pid_ki(float v) { s_pid_ki = v; calib_save(); }
void calib_set_pid_kd(float v) { s_pid_kd = v; calib_save(); }
void calib_set_pid_tof_kp(float v) { s_pid_tof_kp = v; calib_save(); }
void calib_set_pid_tof_ki(float v) { s_pid_tof_ki = v; calib_save(); }
void calib_set_pid_tof_kd(float v) { s_pid_tof_kd = v; calib_save(); }
void calib_set_pid_tof_max_corr(int v) { s_pid_tof_max_corr = v; calib_save(); }
void calib_set_pid_piv_kp(float v) { s_pid_piv_kp = v; calib_save(); }
void calib_set_pid_piv_ki(float v) { s_pid_piv_ki = v; calib_save(); }
void calib_set_pid_piv_kd(float v) { s_pid_piv_kd = v; calib_save(); }
void calib_set_pid_piv_max_corr(int v) { s_pid_piv_max_corr = v; calib_save(); }

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

bool calib_capture_wall_l() {
    int v;
    if (!capture_avg(2, v)) return false;  // SL seul
    s_wall_l = v;
    Serial.print("[CALIB] wall_l = "); Serial.print(v); Serial.println(" mm");
    return true;
}
bool calib_capture_wall_r() {
    int v;
    if (!capture_avg(3, v)) return false;  // SR seul
    s_wall_r = v;
    Serial.print("[CALIB] wall_r = "); Serial.print(v); Serial.println(" mm");
    return true;
}

// ── calib_capture_post_detect ─────────────────────────────────
// Médiane de MIN(SL, SR) sur CALIB_SAMPLES lectures.
// Utilise la médiane (au lieu de la moyenne) pour rejeter les outliers VL53L0X.
// Stocke median + CALIB_POST_DETECT_MARGIN pour déclencher légèrement avant le poteau.
bool calib_capture_post_detect() {
    int samples[CALIB_SAMPLES];
    int n = 0;
    for (int i = 0; i < CALIB_SAMPLES; ++i) {
        ToFReadings t;
        sensors_read(t);
        int sl = t.side_left;
        int sr = t.side_right;
        // Prendre le MIN des deux côtés : le poteau est vu par l'un ou l'autre
        int v = 0;
        if (sl > 0 && sr > 0)      v = min(sl, sr);
        else if (sl > 0)            v = sl;
        else if (sr > 0)            v = sr;
        if (v > 0) samples[n++] = v;
        delay(CALIB_SAMPLE_MS);
    }
    if (n < CALIB_SAMPLES / 2) {
        Serial.print("[CALIB] post_detect échec: ");
        Serial.print(n); Serial.print("/"); Serial.print(CALIB_SAMPLES);
        Serial.println(" valides");
        return false;
    }
    // Tri insertion pour trouver la médiane (n ≤ 20 → O(n²) OK)
    for (int i = 1; i < n; i++) {
        int key = samples[i], j = i - 1;
        while (j >= 0 && samples[j] > key) { samples[j+1] = samples[j]; j--; }
        samples[j+1] = key;
    }
    int med = samples[n / 2];
    s_post_detect = med + CALIB_POST_DETECT_MARGIN;
    Serial.print("[CALIB] post_detect = mediane("); Serial.print(med);
    Serial.print(") + marge = "); Serial.println(s_post_detect);
    return true;
}

// ── calib_save ────────────────────────────────────────────────
bool calib_save() {
    StaticJsonDocument<512> doc;
    doc["center"]       = s_center;
    doc["turn"]         = s_turn;
    doc["opening_l"]    = s_opening_l;
    doc["opening_r"]    = s_opening_r;
    doc["post_detect"]  = s_post_detect;
    doc["wall_l"]          = s_wall_l;
    doc["wall_r"]          = s_wall_r;
    doc["pivot_90_ticks"]  = s_pivot_90_ticks;
    doc["cell_ticks"]      = s_cell_ticks;
    doc["pivot_45_r"]      = s_pivot_45_r;
    doc["pivot_45_l"]      = s_pivot_45_l;
    doc["smooth_move"]     = s_smooth_move;
    doc["smooth_center"]   = s_smooth_center;
    doc["pid_kp"]          = s_pid_kp;
    doc["pid_ki"]          = s_pid_ki;
    doc["pid_kd"]          = s_pid_kd;
    doc["pid_tof_kp"]      = s_pid_tof_kp;
    doc["pid_tof_ki"]      = s_pid_tof_ki;
    doc["pid_tof_kd"]      = s_pid_tof_kd;
    doc["pid_tof_max_corr"] = s_pid_tof_max_corr;
    doc["pid_piv_kp"]      = s_pid_piv_kp;
    doc["pid_piv_ki"]      = s_pid_piv_ki;
    doc["pid_piv_kd"]      = s_pid_piv_kd;
    doc["pid_piv_max_corr"] = s_pid_piv_max_corr;

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
    s_center      = CALIB_TOF_CENTER_MM;
    s_turn        = CALIB_TOF_TURN_MM;
    s_opening_l   = CALIB_TOF_OPENING_L_MM;
    s_opening_r   = CALIB_TOF_OPENING_R_MM;
    s_post_detect = CALIB_TOF_POST_MM;
    s_wall_l         = TOF_WALL_SIDE_MM;
    s_wall_r         = TOF_WALL_SIDE_MM;
    s_pivot_90_ticks = TICKS_PIVOT_90;
    s_cell_ticks     = TICKS_PER_CELL;
    s_pivot_45_r     = TICKS_PIVOT_45_R;
    s_pivot_45_l     = TICKS_PIVOT_45_L;
    s_smooth_move    = TICKS_SPECIAL_MOVE;
    s_smooth_center  = SMOOTH_CENTER_TARGET_MM;
    s_pid_kp = PID_KP;
    s_pid_ki = PID_KI;
    s_pid_kd = PID_KD;
    s_pid_tof_kp = PID_TOF_KP;
    s_pid_tof_ki = PID_TOF_KI;
    s_pid_tof_kd = PID_TOF_KD;
    s_pid_tof_max_corr = PID_TOF_MAX_CORR;
    s_pid_piv_kp = PID_PIVOT_KP;
    s_pid_piv_ki = PID_PIVOT_KI;
    s_pid_piv_kd = PID_PIVOT_KD;
    s_pid_piv_max_corr = PID_PIVOT_MAX_CORR;
    Serial.println("[CALIB] défauts restaurés");
    return calib_save();
}
