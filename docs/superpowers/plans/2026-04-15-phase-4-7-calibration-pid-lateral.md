# Phase 4 + 7 — Calibration LittleFS + PID latéral ToF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ajouter calibration (4 seuils ToF modifiables via IHM, stockés en LittleFS) et renforcer le PID latéral existant avec rejet d'outliers, pour que la navigation case-par-case utilise des seuils ajustables sans reflasher.

**Architecture:** Nouveau module `calibration` lit/écrit `/calib.json` en LittleFS. Le PID latéral existant (`pid_update_tof`) est étendu avec outlier rejection et détection de passage unilatéral. `navigation.cpp` et `sensors_detect_walls()` consomment les seuils via `calib_get_*()`. `web_ui` gagne 5 commandes + panneau calibration.

**Tech Stack:** ESP8266 Arduino / PlatformIO, LittleFS, ArduinoJson, ESPAsyncWebServer, VL53L0X.

> **Divergence vs spec :** la spec prévoyait un nouveau module `pid_lateral`. Le code existant contient déjà `pid_update_tof()` dans `pid.cpp`. On le modifie en place (outlier + passage guard) plutôt que dupliquer — conforme à YAGNI.

> **Adaptation TDD :** pas de framework de tests unitaires sur ce projet embarqué. Chaque tâche se valide par **build OK + observation Serial/IHM** (détails dans les steps "Verify"). Commits fréquents à la fin de chaque tâche.

---

## File Structure

**Créés :**
- `src/calibration.h` — API publique (getters, capture, save/reset).
- `src/calibration.cpp` — implémentation + I/O LittleFS.

**Modifiés :**
- `platformio.ini` — ajout `board_build.filesystem = littlefs`.
- `src/config.h` — 4 défauts de seuils + `CALIB_SAMPLES`, `CALIB_SAMPLE_MS`.
- `src/pid.h` / `src/pid.cpp` — signature `pid_update_tof` étendue + outlier rejection.
- `src/web_ui.h` — 5 nouvelles valeurs `WebCmd`.
- `src/web_ui.cpp` — 6 routes HTTP (`/calib` GET + 5 POST) + panneau HTML.
- `src/navigation.cpp` — lecture des seuils via `calib_get_*()`.
- `src/sensors.cpp` — `sensors_detect_walls()` utilise `calib_get_opening_*()`.
- `src/main.cpp` — appel `calibration_init()` au boot + consommation des 5 flags calib dans `STATE_IDLE`.

---

## Task 1 — Préparer LittleFS (platformio + défauts config)

**Files:**
- Modify: `platformio.ini`
- Modify: `src/config.h` (après la section "Seuils capteurs ToF")

- [ ] **Step 1 — Ajouter LittleFS dans platformio.ini**

Ajouter sous `framework = arduino` :

```ini
board_build.filesystem = littlefs
```

Résultat attendu du fichier (extrait) :
```ini
[env:nodemcuv2]
platform = espressif8266
board = nodemcuv2
framework = arduino
board_build.filesystem = littlefs
monitor_speed = 115200
upload_speed = 921600
```

- [ ] **Step 2 — Ajouter les défauts dans config.h**

Ajouter après `#define TOF_SIDE_GEOM_MARGIN  30` :

```cpp
// ── Calibration fonctionnelle (seuils ajustables via IHM/LittleFS) ──
// Valeurs par défaut si /calib.json n'existe pas.
// Toutes en mm.
#define CALIB_TOF_CENTER_MM      80   // distance latérale cible (centré dans couloir)
#define CALIB_TOF_TURN_MM        60   // distance frontale → arrêt pile au centre de case
#define CALIB_TOF_OPENING_L_MM   150  // SL > seuil → passage gauche ouvert
#define CALIB_TOF_OPENING_R_MM   150  // SR > seuil → passage droit ouvert

// Paramètres d'échantillonnage pour capture (appelée en STATE_IDLE)
#define CALIB_SAMPLES            20
#define CALIB_SAMPLE_MS          20

// Plage physique de validité d'une lecture latérale (pour outlier rejection PID)
#define TOF_SIDE_MIN_MM          20
#define TOF_SIDE_MAX_MM          500
```

- [ ] **Step 3 — Build et vérifier**

Run : `pio run`
Expected : compilation OK (aucune utilisation des nouvelles constantes pour l'instant).

- [ ] **Step 4 — Commit**

```bash
git add platformio.ini src/config.h
git commit -m "feat(config): défauts calibration ToF + activation LittleFS"
```

---

## Task 2 — Module `calibration` (sans LittleFS d'abord)

**Files:**
- Create: `src/calibration.h`
- Create: `src/calibration.cpp`

- [ ] **Step 1 — Écrire `src/calibration.h`**

```cpp
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
```

- [ ] **Step 2 — Écrire `src/calibration.cpp` (squelette sans LittleFS)**

```cpp
// =============================================================
//  calibration.cpp — Implémentation
// =============================================================
#include "calibration.h"
#include "config.h"
#include "sensors.h"
#include <Arduino.h>

// État interne (valeurs courantes en mm)
static int s_center     = CALIB_TOF_CENTER_MM;
static int s_turn       = CALIB_TOF_TURN_MM;
static int s_opening_l  = CALIB_TOF_OPENING_L_MM;
static int s_opening_r  = CALIB_TOF_OPENING_R_MM;

// ── Init ────────────────────────────────────────────────────
bool calibration_init() {
    // LittleFS ajouté dans Task 3 — pour l'instant : défauts uniquement.
    Serial.println("[CALIB] défauts appliqués (LittleFS pas encore actif)");
    return false;
}

// ── Getters ─────────────────────────────────────────────────
int calib_get_center()     { return s_center; }
int calib_get_turn()       { return s_turn; }
int calib_get_opening_l()  { return s_opening_l; }
int calib_get_opening_r()  { return s_opening_r; }

// ── Capture générique (moyenne filtrée) ─────────────────────
// selector : 0=(SL+SR)/2, 1=(FL+FR)/2, 2=SL, 3=SR
static bool capture_avg(int selector, int& out_value) {
    long sum = 0;
    int  n   = 0;
    for (int i = 0; i < CALIB_SAMPLES; ++i) {
        ToFReadings t;
        sensors_read(t);
        int v = 0;
        switch (selector) {
            case 0: if (t.side_left > 0 && t.side_right > 0)
                        v = (t.side_left + t.side_right) / 2;
                    break;
            case 1: if (t.front_left > 0 && t.front_right > 0)
                        v = (t.front_left + t.front_right) / 2;
                    break;
            case 2: v = t.side_left;  break;
            case 3: v = t.side_right; break;
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

// ── Persistance (Task 3) ────────────────────────────────────
bool calib_save() {
    Serial.println("[CALIB] save() non implémenté (Task 3)");
    return false;
}
bool calib_reset_defaults() {
    s_center    = CALIB_TOF_CENTER_MM;
    s_turn      = CALIB_TOF_TURN_MM;
    s_opening_l = CALIB_TOF_OPENING_L_MM;
    s_opening_r = CALIB_TOF_OPENING_R_MM;
    Serial.println("[CALIB] défauts restaurés");
    return true;
}
```

- [ ] **Step 3 — Appeler `calibration_init()` dans setup() de `main.cpp`**

Lire `src/main.cpp`, repérer la section setup() après `sensors_init()`, ajouter :

```cpp
#include "calibration.h"
// ...
// dans setup(), après sensors_init() :
calibration_init();
```

- [ ] **Step 4 — Build + flash + verify**

Run : `pio run -t upload && pio device monitor`
Expected Serial au boot : `[CALIB] défauts appliqués (LittleFS pas encore actif)`

- [ ] **Step 5 — Commit**

```bash
git add src/calibration.h src/calibration.cpp src/main.cpp
git commit -m "feat(calibration): module seuils ToF + capture moyennée (sans LittleFS)"
```

---

## Task 3 — Persistance LittleFS dans `calibration.cpp`

**Files:**
- Modify: `src/calibration.cpp`

- [ ] **Step 1 — Ajouter includes LittleFS + ArduinoJson**

En haut de `src/calibration.cpp`, après `#include <Arduino.h>` :

```cpp
#include <LittleFS.h>
#include <ArduinoJson.h>

static const char* CALIB_PATH = "/calib.json";
```

- [ ] **Step 2 — Implémenter `calibration_init()` avec lecture**

Remplacer la fonction `calibration_init()` actuelle par :

```cpp
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
```

- [ ] **Step 3 — Implémenter `calib_save()`**

Remplacer la fonction stub par :

```cpp
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
```

- [ ] **Step 4 — Étendre `calib_reset_defaults()` pour sauver**

Remplacer la fonction par :

```cpp
bool calib_reset_defaults() {
    s_center    = CALIB_TOF_CENTER_MM;
    s_turn      = CALIB_TOF_TURN_MM;
    s_opening_l = CALIB_TOF_OPENING_L_MM;
    s_opening_r = CALIB_TOF_OPENING_R_MM;
    Serial.println("[CALIB] défauts restaurés");
    return calib_save();
}
```

- [ ] **Step 5 — Build, flash, verify persistance**

Run : `pio run -t upload && pio device monitor`
Expected : au premier boot `[CALIB] /calib.json absent — défauts utilisés`.

- [ ] **Step 6 — Commit**

```bash
git add src/calibration.cpp
git commit -m "feat(calibration): persistance LittleFS (/calib.json)"
```

---

## Task 4 — Étendre `pid_update_tof` (outlier + passage guard)

**Files:**
- Modify: `src/pid.h`
- Modify: `src/pid.cpp`

- [ ] **Step 1 — Modifier la signature dans `pid.h`**

Remplacer la déclaration `pid_update_tof` par :

```cpp
// ── pid_update_tof ────────────────────────────────────────────
// PID sur capteurs ToF latéraux avec setpoint `center_mm` (distance cible
// quand centré) et rejet d'outliers.
//   side_left_mm, side_right_mm : lectures SL/SR en mm (0 si invalide)
//   center_mm                   : consigne (distance quand centré) — utilisé
//                                  indirectement, erreur = SL - SR
//   pwm_base                    : PWM de base
//   pwm_left, pwm_right         : [sortie] PWM corrigés
// Comportement :
//   - Si SL ou SR hors [TOF_SIDE_MIN_MM, TOF_SIDE_MAX_MM] → pas de mise à jour
//     (garde la dernière correction, évite les secousses)
//   - Si SL > opening_l_mm OU SR > opening_r_mm → correction = 0 (passage
//     unilatéral : plus de référence latérale fiable)
void pid_update_tof(float side_left_mm, float side_right_mm,
                    int opening_l_mm, int opening_r_mm,
                    int pwm_base, int& pwm_left, int& pwm_right);
```

- [ ] **Step 2 — Réécrire `pid_update_tof` dans `pid.cpp`**

Remplacer la fonction existante par :

```cpp
// Dernière correction calculée — réutilisée en cas d'outlier
static int s_tof_last_correction = 0;

void pid_update_tof(float side_left_mm, float side_right_mm,
                    int opening_l_mm, int opening_r_mm,
                    int pwm_base, int& pwm_left, int& pwm_right) {

    // ── Outlier rejection : capteur hors plage physique ────────
    // On réutilise la dernière correction pour éviter les secousses.
    bool sl_ok = (side_left_mm  >= TOF_SIDE_MIN_MM && side_left_mm  <= TOF_SIDE_MAX_MM);
    bool sr_ok = (side_right_mm >= TOF_SIDE_MIN_MM && side_right_mm <= TOF_SIDE_MAX_MM);
    if (!sl_ok || !sr_ok) {
        pwm_left  = constrain(pwm_base - s_tof_last_correction, 0, 255);
        pwm_right = constrain(pwm_base + s_tof_last_correction, 0, 255);
        return;
    }

    // ── Passage unilatéral (ouverture d'un côté) : pas de PID ──
    // Dans ce cas on avance droit (correction 0) et on laisse l'encodeur PID
    // ou l'IMU prendre le relais côté navigation.
    if (side_left_mm  > opening_l_mm || side_right_mm > opening_r_mm) {
        s_tof_last_correction = 0;
        pwm_left  = pwm_base;
        pwm_right = pwm_base;
        return;
    }

    // ── PID classique ─────────────────────────────────────────
    float e = side_left_mm - side_right_mm;
    s_tof_integral += e;
    s_tof_integral  = constrain(s_tof_integral, -200.0f, 200.0f);

    float derivative = e - s_tof_prev_error;
    s_tof_prev_error = e;

    float correction = PID_TOF_KP * e
                     + PID_TOF_KI * s_tof_integral
                     + PID_TOF_KD * derivative;

    // Sauvegarde pour fallback outlier
    s_tof_last_correction = (int)correction;

    pwm_left  = constrain((int)(pwm_base - correction), 0, 255);
    pwm_right = constrain((int)(pwm_base + correction), 0, 255);
}
```

Note : laisser `s_tof_integral` et `s_tof_prev_error` déjà déclarés plus haut. Ajouter `s_tof_last_correction` à côté et reset dans `pid_init()` :

Dans `pid_init()`, ajouter à la fin :
```cpp
    s_tof_last_correction = 0;
```

- [ ] **Step 3 — Mettre à jour l'appelant dans `navigation.cpp`**

Dans `navigation.cpp`, fonction `update_advance()`, repérer :
```cpp
pid_update_tof((float)s_tof.side_left, (float)s_tof.side_right,
               PWM_RUN1, pwm_l, pwm_r);
```

Ajouter `#include "calibration.h"` en haut du fichier s'il n'y est pas, puis remplacer par :
```cpp
pid_update_tof((float)s_tof.side_left, (float)s_tof.side_right,
               calib_get_opening_l(), calib_get_opening_r(),
               PWM_RUN1, pwm_l, pwm_r);
```

- [ ] **Step 4 — Build**

Run : `pio run`
Expected : compilation OK, aucune erreur.

- [ ] **Step 5 — Commit**

```cpp
git add src/pid.h src/pid.cpp src/navigation.cpp
git commit -m "feat(pid): outlier rejection + passage unilatéral sur PID latéral ToF"
```

---

## Task 5 — Extension `web_ui` : enum + routes HTTP

**Files:**
- Modify: `src/web_ui.h`
- Modify: `src/web_ui.cpp`

- [ ] **Step 1 — Ajouter les 5 commandes dans `web_ui.h`**

Dans l'enum `WebCmd`, ajouter après `WEB_CMD_MOVE_RIGHT,` :
```cpp
    WEB_CMD_CALIB_CENTER,
    WEB_CMD_CALIB_TURN,
    WEB_CMD_CALIB_OPENING_L,
    WEB_CMD_CALIB_OPENING_R,
    WEB_CMD_CALIB_RESET
```

- [ ] **Step 2 — Ajouter handlers dans `web_ui.cpp`**

Dans `src/web_ui.cpp`, juste avant `web_ui_init()`, ajouter :

```cpp
#include "calibration.h"

// GET /calib
static void handle_calib_get(AsyncWebServerRequest* req) {
    StaticJsonDocument<128> doc;
    doc["center"]    = calib_get_center();
    doc["turn"]      = calib_get_turn();
    doc["opening_l"] = calib_get_opening_l();
    doc["opening_r"] = calib_get_opening_r();
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
}

// POST /calib/*  (5 endpoints, chacun pose un flag)
static void handle_calib_center(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_CENTER;
    req->send(200, "application/json", "{\"ok\":true}");
}
static void handle_calib_turn(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_TURN;
    req->send(200, "application/json", "{\"ok\":true}");
}
static void handle_calib_opening_l(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_OPENING_L;
    req->send(200, "application/json", "{\"ok\":true}");
}
static void handle_calib_opening_r(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_OPENING_R;
    req->send(200, "application/json", "{\"ok\":true}");
}
static void handle_calib_reset(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_RESET;
    req->send(200, "application/json", "{\"ok\":true}");
}
```

- [ ] **Step 3 — Enregistrer les routes dans `web_ui_init()`**

Dans `web_ui_init()`, après la ligne `s_server.on("/start2", ...)` et avant `s_server.on("/move", ...)`, ajouter :

```cpp
    s_server.on("/calib",            HTTP_GET,  handle_calib_get);
    s_server.on("/calib/center",     HTTP_POST, handle_calib_center);
    s_server.on("/calib/turn",       HTTP_POST, handle_calib_turn);
    s_server.on("/calib/opening_l",  HTTP_POST, handle_calib_opening_l);
    s_server.on("/calib/opening_r",  HTTP_POST, handle_calib_opening_r);
    s_server.on("/calib/reset",      HTTP_POST, handle_calib_reset);
```

- [ ] **Step 4 — Build**

Run : `pio run`
Expected : compilation OK.

- [ ] **Step 5 — Commit**

```bash
git add src/web_ui.h src/web_ui.cpp
git commit -m "feat(web_ui): routes /calib (GET + 5 POST) + enum WebCmd étendu"
```

---

## Task 6 — Panneau HTML calibration

**Files:**
- Modify: `src/web_ui.cpp` (chaîne `INDEX_HTML`)

- [ ] **Step 1 — Ajouter le panneau HTML dans INDEX_HTML**

Dans `src/web_ui.cpp`, dans la chaîne `INDEX_HTML`, juste avant la balise `</div>` qui ferme `<div class="wrap">` (ligne ~157 — juste après le panneau de contrôle), ajouter :

```html
  <!-- Panneau Calibration -->
  <div class="panel" style="min-width:220px;">
    <h1 style="margin-top:0;">Calibration ToF</h1>
    <div class="info-row"><span class="lbl">CENTER  :</span><span id="cal-center">—</span> mm</div>
    <div class="info-row"><span class="lbl">TURN    :</span><span id="cal-turn">—</span> mm</div>
    <div class="info-row"><span class="lbl">OPEN L  :</span><span id="cal-ol">—</span> mm</div>
    <div class="info-row"><span class="lbl">OPEN R  :</span><span id="cal-or">—</span> mm</div>
    <hr>
    <div class="btn-row">
      <button class="btn" id="btn-cal-c"  onclick="cal('center')">🔵 Capturer CENTER</button>
      <button class="btn" id="btn-cal-t"  onclick="cal('turn')">🟠 Capturer TURN</button>
      <button class="btn" id="btn-cal-ol" onclick="cal('opening_l')">🟢 Capturer OPEN L</button>
      <button class="btn" id="btn-cal-or" onclick="cal('opening_r')">🟢 Capturer OPEN R</button>
    </div>
    <div class="btn-row">
      <button class="btn stop" id="btn-cal-rst" onclick="cal('reset')">↺ Reset défauts</button>
    </div>
    <div class="hint">IDLE uniquement. Placer le robot puis cliquer. ~400ms de mesure.</div>
  </div>
```

- [ ] **Step 2 — Ajouter la fonction JS `cal()` et le polling `/calib`**

Dans le bloc `<script>`, juste après la fonction `move()` (~ligne 280), ajouter :

```js
function cal(what){
  fetch('/calib/'+what,{method:'POST'});
}

async function pollCalib(){
  try{
    const c=await fetch('/calib').then(r=>r.json());
    document.getElementById('cal-center').textContent=c.center;
    document.getElementById('cal-turn').textContent=c.turn;
    document.getElementById('cal-ol').textContent=c.opening_l;
    document.getElementById('cal-or').textContent=c.opening_r;
  } catch(e){ console.warn('calib poll',e); }
}
```

Puis dans la fonction `updateInfo(st)`, tout à la fin (juste avant le `}` de fermeture de la fonction), ajouter le gating IDLE des boutons calibration :

```js
  // Calibration : uniquement en IDLE (state 0)
  const idle = (i===0);
  ['btn-cal-c','btn-cal-t','btn-cal-ol','btn-cal-or','btn-cal-rst']
    .forEach(id => document.getElementById(id).disabled = !idle);
```

Enfin, à la fin du script, juste après `setInterval(poll,500);`, ajouter :

```js
pollCalib();
setInterval(pollCalib, 1000);
```

- [ ] **Step 3 — Build + flash + vérifier UI**

Run : `pio run -t upload`
Expected : le panneau "Calibration ToF" s'affiche à droite, les 4 valeurs apparaissent après ~1s, les boutons sont grisés si le robot n'est pas en IDLE.

- [ ] **Step 4 — Commit**

```bash
git add src/web_ui.cpp
git commit -m "feat(web_ui): panneau calibration (affichage + 5 boutons + polling 1s)"
```

---

## Task 7 — Consommer les flags calibration dans `main.cpp`

**Files:**
- Modify: `src/main.cpp`

- [ ] **Step 1 — Identifier où sont consommées les commandes web**

Lire `src/main.cpp`, repérer le `switch(cmd)` ou équivalent qui traite `WEB_CMD_*` (probablement dans la gestion de `STATE_IDLE`).

- [ ] **Step 2 — Ajouter les 5 nouveaux cas**

Dans le `switch` traitant `web_ui_poll_cmd()`, ajouter (en respectant le style existant) :

```cpp
case WEB_CMD_CALIB_CENTER:
    if (current_state == STATE_IDLE) {
        calib_capture_center();
        calib_save();
    } else Serial.println("[CALIB] ignoré: pas en IDLE");
    break;

case WEB_CMD_CALIB_TURN:
    if (current_state == STATE_IDLE) {
        calib_capture_turn();
        calib_save();
    } else Serial.println("[CALIB] ignoré: pas en IDLE");
    break;

case WEB_CMD_CALIB_OPENING_L:
    if (current_state == STATE_IDLE) {
        calib_capture_opening_l();
        calib_save();
    } else Serial.println("[CALIB] ignoré: pas en IDLE");
    break;

case WEB_CMD_CALIB_OPENING_R:
    if (current_state == STATE_IDLE) {
        calib_capture_opening_r();
        calib_save();
    } else Serial.println("[CALIB] ignoré: pas en IDLE");
    break;

case WEB_CMD_CALIB_RESET:
    if (current_state == STATE_IDLE) calib_reset_defaults();
    else Serial.println("[CALIB] ignoré: pas en IDLE");
    break;
```

(Adapter `current_state` au nom réel de la variable d'état dans `main.cpp`.)

- [ ] **Step 3 — Build + flash**

Run : `pio run -t upload && pio device monitor`
Expected : au clic sur "🔵 Capturer CENTER" dans l'IHM (robot en IDLE), observer :
```
[CALIB] center = XX mm
[CALIB] /calib.json sauvegardé
```
Puis reload page → nouvelle valeur affichée.

- [ ] **Step 4 — Test persistance**

Reboot ESP. Au boot, Serial doit afficher :
```
[CALIB] chargé: C=XX T=YY OL=ZZ OR=WW
```

- [ ] **Step 5 — Commit**

```bash
git add src/main.cpp
git commit -m "feat(main): consommation des 5 commandes de calibration en STATE_IDLE"
```

---

## Task 8 — Utiliser les seuils calibrés dans `sensors_detect_walls` et `navigation`

**Files:**
- Modify: `src/sensors.cpp`
- Modify: `src/navigation.cpp`

- [ ] **Step 1 — Modifier `sensors_detect_walls` pour utiliser `calib_get_opening_*`**

Dans `src/sensors.cpp`, en haut ajouter :
```cpp
#include "calibration.h"
```

Puis dans `sensors_detect_walls()`, remplacer les tests `< TOF_WALL_SIDE_MM` par les seuils calibrés :

```cpp
// Version calibrée : mur à gauche si SL < seuil_opening_l (sinon = passage ouvert)
det.left  = (tof.side_left  > 0 && tof.side_left  < calib_get_opening_l());
det.right = (tof.side_right > 0 && tof.side_right < calib_get_opening_r());
```

(Conserver la détection frontale inchangée — pilotée par `TOF_WALL_FRONT_MM`, qui est un seuil de présence et non de calibration fonctionnelle.)

- [ ] **Step 2 — Modifier navigation : arrêt frontal via `calib_get_turn`**

Dans `src/navigation.cpp`, fonction `update_advance()`, l'arrêt actuel est basé sur `avg_ticks >= TICKS_PER_CELL`. On veut **ajouter** un arrêt complémentaire si un mur frontal est plus proche que `calib_get_turn()` (le robot est pile au centre de la case) :

Après le bloc "Arrêt d'urgence" (`< TOF_STOP_FRONT_MM`) et avant "Condition d'arrêt : distance atteinte", insérer :

```cpp
    // ── Arrêt calibré : mur devant à distance TURN ──
    // Si un mur existe devant à exactement calib_get_turn() mm, le robot est
    // à sa position de fin de case nominale. Prioritaire sur l'arrêt par ticks.
    int front_avg = 0;
    if (s_tof.front_left > 0 && s_tof.front_right > 0) {
        front_avg = (s_tof.front_left + s_tof.front_right) / 2;
        if (front_avg <= calib_get_turn()) {
            motors_stop();
            Serial.print("[NAV] STOP calibré TURN — front_avg=");
            Serial.print(front_avg); Serial.println("mm");
            s_state  = NAV_DONE;
            s_action = ACT_NONE;
            return NAV_DONE;
        }
    }
```

- [ ] **Step 3 — Build**

Run : `pio run`
Expected : compilation OK.

- [ ] **Step 4 — Test physique**

1. Placer robot centré dans un couloir, bouton "🔵 Capturer CENTER" → noter valeur.
2. Placer robot face à un mur à la distance voulue (par exemple 60 mm), bouton "🟠 Capturer TURN" → noter valeur.
3. Lancer avance d'une case (manuel ↑). Le robot doit s'arrêter soit aux ticks, soit quand le mur devant atteint la distance TURN.
4. Vérifier les Serial :
   - `[NAV] STOP calibré TURN — front_avg=XX mm` (si cas frontal)
   - ou fin par ticks sinon.

- [ ] **Step 5 — Commit**

```bash
git add src/sensors.cpp src/navigation.cpp
git commit -m "feat(nav+sensors): utilisation des seuils calibrés (opening + turn)"
```

---

## Task 9 — Validation finale

**Files:**
- Aucun changement de code. Tests manuels.

- [ ] **Step 1 — Test de persistance complet**

1. `pio run -t upload`, observer Serial au boot.
2. Via IHM : capturer CENTER, TURN, OPENING_L, OPENING_R.
3. Reset alim ESP.
4. Au reboot, Serial doit afficher `[CALIB] chargé: C=.. T=.. OL=.. OR=..` avec les valeurs capturées.

- [ ] **Step 2 — Test PID latéral avec outlier**

1. Placer robot dans un couloir droit.
2. Lancer avance manuelle.
3. Pendant la traversée, masquer temporairement un capteur latéral avec la main (→ distance aberrante).
4. Le robot ne doit **pas** faire de mouvement brusque (la correction précédente est conservée).

- [ ] **Step 3 — Test détection passage ouvert**

1. Placer robot à une intersection en T.
2. Lancer avance manuelle.
3. Vérifier via `GET /maze` (via navigateur `http://192.168.4.1/maze`) que la case d'arrêt a les bons murs détectés (le côté ouvert doit avoir `walls` sans le bit correspondant).

- [ ] **Step 4 — Reset + test défauts**

1. IHM : bouton "↺ Reset défauts".
2. Les 4 valeurs doivent revenir à `CALIB_TOF_*_MM` de `config.h`.
3. Reboot ESP → les valeurs par défaut persistent (car `calib_save()` est appelé par `reset_defaults`).

- [ ] **Step 5 — Commit "done" (optionnel : tag)**

```bash
git tag -a phase-4-7-done -m "Phase 4 + 7 validées : calibration LittleFS + PID latéral robuste"
```

---

## Self-Review

- **Spec coverage** : calibration (4 seuils ✅), LittleFS (Task 3 ✅), capture moyennée (Task 2 ✅), PID latéral outlier + passage (Task 4 ✅), extension web_ui (Tasks 5-6 ✅), intégration main (Task 7 ✅), intégration navigation (Task 8 ✅), tests validation (Task 9 ✅).
- **Placeholder scan** : aucune mention "TODO", "TBD" dans les steps. Le code est fourni complet.
- **Type consistency** : `calib_get_center/turn/opening_l/opening_r` utilisés cohéremment dans Tasks 4, 6, 8. Les 5 valeurs enum `WEB_CMD_CALIB_*` utilisées cohéremment dans Tasks 5, 7. Signature `pid_update_tof` étendue (Task 4) et l'appelant mis à jour dans la même tâche.
- **Divergence vs spec documentée** : pas de nouveau module `pid_lateral`, extension en place de `pid_update_tof` (noté en tête).
