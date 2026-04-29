# Phase 4 + 7 — PID latéral ToF, calibration LittleFS, extension IHM

**Date** : 2026-04-15
**Branche** : `phase-1-i2c-mcp-tof`
**Auteur** : Équipe 17 — ICAM Strasbourg

---

## 1. Objectif

Finaliser :

- **Phase 4** — Asservissement PID sur les ToF latéraux 45° pour que le robot avance centré dans les couloirs, et s'arrête pile au centre de chaque case quand il approche d'un mur/une intersection.
- **Phase 7** — Compléter l'IHM web existante avec la calibration des 4 seuils fonctionnels (distance "centré", distance "arrêt avant virage", seuils "passage ouvert" gauche/droite), stockés en LittleFS pour être modifiables sans reflasher.

La navigation case-par-case nécessaire à Trémaux (Run 1) et BFS (Run 2) s'appuie directement sur ces 4 seuils.

---

## 2. Scope

### Dans le scope

- Nouveau module `calibration` (seuils + I/O LittleFS + capture moyennée).
- Nouveau module `pid_lateral` (correcteur PID discret Z sur erreur `SL - SR`).
- 4 nouvelles routes HTTP + panneau UI pour calibration (boutons, affichage live, reset).
- Modification de `navigation.cpp` pour utiliser les seuils calibrés dans la primitive case-par-case.
- Défauts dans `config.h` si LittleFS vide.

### Hors scope

- Phases 5, 6, 8 (validation physique Trémaux, BFS exécution, intégration finale).
- Calibration fine des capteurs ToF eux-mêmes (offset par capteur) — non nécessaire, on travaille en relatif.
- Affichage en direct des 4 ToF bruts (déjà en place).
- Réglage live Kp/Ki/Kd (déjà en place).

---

## 3. Architecture

```
src/
├── calibration.h / .cpp   [NOUVEAU]
├── pid_lateral.h / .cpp   [NOUVEAU]
├── navigation.cpp         [MODIFIÉ — consomme seuils calibrés]
├── main.cpp               [MODIFIÉ — init + consommation flags calib]
├── web_ui.h / .cpp        [MODIFIÉ — 5 routes + panneau UI calib]
└── config.h               [MODIFIÉ — défauts 4 seuils + coeffs PID latéral]
```

### Dépendances
```
main.cpp
  ├── calibration (lit seuils)
  ├── pid_lateral (correction PWM)
  ├── navigation (consomme calib_get_* et pid_lat_update)
  └── web_ui (expose commandes et état)

calibration → sensors (lecture ToF), LittleFS
pid_lateral → config.h (coeffs)
```

---

## 4. Module `calibration`

### Données stockées

| Constante | Rôle | Capteurs |
|---|---|---|
| `TOF_CENTER_MM` | Setpoint PID latéral (distance "centré dans couloir") | moyenne SL+SR |
| `TOF_TURN_MM` | Seuil arrêt frontal (pour être pile au centre de la case) | moyenne FL+FR |
| `TOF_OPENING_L_MM` | Seuil au-dessus duquel un passage gauche est ouvert | SL |
| `TOF_OPENING_R_MM` | Seuil au-dessus duquel un passage droit est ouvert | SR |

### API publique (`calibration.h`)

```cpp
void calibration_init();             // monte LittleFS, charge /calib.json ou défauts
int  calib_get_center();
int  calib_get_turn();
int  calib_get_opening_l();
int  calib_get_opening_r();

bool calib_capture_center();         // 20 échantillons × 20ms, moyenne (SL+SR)/2
bool calib_capture_turn();           // 20 échantillons × 20ms, moyenne (FL+FR)/2
bool calib_capture_opening_l();      // 20 échantillons × 20ms, moyenne SL
bool calib_capture_opening_r();      // 20 échantillons × 20ms, moyenne SR

bool calib_save();                   // sérialise vers /calib.json
bool calib_reset_defaults();         // remet aux valeurs de config.h (+ save)
```

### Format `/calib.json`
```json
{"center":80,"turn":60,"opening_l":150,"opening_r":150}
```

### Défauts (config.h)
```cpp
#define TOF_CENTER_MM      80
#define TOF_TURN_MM        60
#define TOF_OPENING_L_MM   150
#define TOF_OPENING_R_MM   150
#define CALIB_SAMPLES      20
#define CALIB_SAMPLE_MS    20
```

### Comportement

- `calibration_init()` : `LittleFS.begin()` → lit `/calib.json` → si succès charge, sinon applique défauts (sans sauver — évite écrire au premier boot).
- `calib_capture_*()` : appelable UNIQUEMENT en `STATE_IDLE` (garde côté `main.cpp`). Boucle de 20 lectures avec `delay(20)` → acceptable car pendant calibration le robot est à l'arrêt et l'utilisateur attend le retour.
- Pas d'allocation dynamique : tout est sur la pile via `StaticJsonDocument<128>`.

---

## 5. Module `pid_lateral`

### Principe

PID discret en Z, échantillonné à 50 Hz.

**Erreur** : `e = tof_sl - tof_sr` (mm)

**Sortie** :
```
u = Kp·e + Ki·Σe + Kd·(e - e_prev)
pwm_L = pwm_base - u
pwm_R = pwm_base + u
```

### API publique (`pid_lateral.h`)

```cpp
void    pid_lat_init();
void    pid_lat_reset();                              // Σe = 0, e_prev = 0 (appeler à chaque début de case)
int16_t pid_lat_update(int tof_sl, int tof_sr);       // retourne u (correction PWM)
```

### Coefficients (config.h)
```cpp
#define PID_LAT_KP     0.4f
#define PID_LAT_KI     0.01f
#define PID_LAT_KD     0.2f
#define PID_LAT_I_MAX  500.0f    // anti-windup : |Σe| ≤ I_MAX
```

### Garde-fous

1. **Outlier rejection** (CLAUDE.md point 8) : si `tof_sl` ou `tof_sr` hors plage `[20, 500]`, retourner la dernière correction `u` calculée (ne pas intégrer la valeur aberrante).
2. **Passage unilatéral** : si `tof_sl > TOF_OPENING_L_MM` OU `tof_sr > TOF_OPENING_R_MM` → retourner 0 (pas d'erreur exploitable, avance PWM symétrique).
3. **Anti-windup** : clamp `Σe` à `±PID_LAT_I_MAX`.
4. **Saturation de sortie** : clamp `u` à `±(PWM_RUN1 / 2)` (jamais inverser un moteur).

---

## 6. Extension `web_ui`

### Nouvelles commandes (`WebCmd` enum)

```cpp
WEB_CMD_CALIB_CENTER,
WEB_CMD_CALIB_TURN,
WEB_CMD_CALIB_OPENING_L,
WEB_CMD_CALIB_OPENING_R,
WEB_CMD_CALIB_RESET
```

### Nouvelles routes HTTP

| Route | Méthode | Action |
|---|---|---|
| `POST /calib/center` | Pose flag capture CENTER | `{"ok":true}` |
| `POST /calib/turn` | Pose flag capture TURN | `{"ok":true}` |
| `POST /calib/opening_l` | Pose flag OPENING_L | `{"ok":true}` |
| `POST /calib/opening_r` | Pose flag OPENING_R | `{"ok":true}` |
| `POST /calib/reset` | Remet défauts | `{"ok":true}` |
| `GET /calib` | Lit les 4 seuils | `{"center":..,"turn":..,"opening_l":..,"opening_r":..}` |

### Modification page HTML

Ajouter un panneau "Calibration" :
- 4 lignes info affichant les valeurs actuelles (poll toutes les 500 ms sur `/calib`).
- 4 boutons colorés : "🔵 Capturer CENTER", "🟠 Capturer TURN", "🟢 Capturer OPENING L", "🟢 Capturer OPENING R".
- 1 bouton "↺ Reset défauts".
- Hint : "Placer le robot dans la position voulue. Uniquement en IDLE."
- Boutons grisés si `state !== IDLE`.

### Consommation dans `main.cpp`

Dans la machine d'états, pour `STATE_IDLE` :
```cpp
case WEB_CMD_CALIB_CENTER:    calib_capture_center();    calib_save(); break;
case WEB_CMD_CALIB_TURN:      calib_capture_turn();      calib_save(); break;
case WEB_CMD_CALIB_OPENING_L: calib_capture_opening_l(); calib_save(); break;
case WEB_CMD_CALIB_OPENING_R: calib_capture_opening_r(); calib_save(); break;
case WEB_CMD_CALIB_RESET:     calib_reset_defaults();                  break;
```

---

## 7. Intégration dans `navigation.cpp`

### Primitive case-par-case

```cpp
bool nav_advance_one_cell() {
    pid_lat_reset();
    uint32_t t_last = millis();

    while (true) {
        // Lecture ToF
        int fl = sensors_read_fl();
        int fr = sensors_read_fr();
        int sl = sensors_read_sl();
        int sr = sensors_read_sr();

        int front_avg = (fl + fr) / 2;
        if (front_avg < calib_get_turn()) break;   // seuil atteint → stop

        // PID latéral à 50 Hz
        if (millis() - t_last >= PID_SAMPLE_MS) {
            int16_t u = pid_lat_update(sl, sr);
            motors_set_pwm(PWM_RUN1 - u, PWM_RUN1 + u);
            t_last = millis();
        }
        yield();   // coopératif (ESPAsyncWebServer, WiFi, etc.)
    }

    motors_brake();

    // Détection murs pour la carte Trémaux
    WallMask w = 0;
    if (sensors_read_fl_avg() < calib_get_turn() + MARGIN) w |= WALL_FRONT;
    if (sensors_read_sl()     < calib_get_opening_l())     w |= WALL_LEFT;
    if (sensors_read_sr()     < calib_get_opening_r())     w |= WALL_RIGHT;
    maze_mark_walls(w);
    return true;
}
```

(Le code exact s'adapte aux fonctions existantes dans `navigation.cpp`.)

---

## 8. Flux de données

### Boot
```
setup()
  → I2C + sensors_init() + imu_init()
  → motors_init() + encoders_init()
  → calibration_init()   [LittleFS + lecture seuils]
  → pid_lat_init()
  → web_ui_init()
```

### Calibration par l'utilisateur
```
Mobile → POST /calib/center
  → handler pose flag WEB_CMD_CALIB_CENTER
  → loop() récupère flag
  → calib_capture_center() échantillonne 400 ms
  → calib_save() écrit LittleFS
  → Serial : "[CALIB] center = 78 mm OK"
  → prochain GET /calib reflète la nouvelle valeur
```

### Navigation case-par-case
```
Run 1/2 → nav_advance_one_cell()
  → pid_lat_reset()
  → boucle 50 Hz : lit ToF → PID → PWM corrigés
  → quand tof_front_avg < calib_get_turn() → brake
  → classification murs via seuils opening_l / opening_r
  → maze mis à jour
```

---

## 9. Tests & validation

| Test | Méthode | Critère |
|---|---|---|
| LittleFS persistance | Capturer CENTER, reboot, `GET /calib` | valeur persiste |
| Capture ok en IDLE | Appel quand IDLE | JSON ok + Serial log |
| Capture bloquée si non-IDLE | Appel pendant Run | bouton grisé côté UI |
| PID latéral seul | Couloir droit, manuel | trajectoire centrée ±5 mm sur 1 m |
| Arrêt TURN | Face à mur | arrêt pile à TURN ± 5 mm |
| Détection ouvertures | Intersection en T | walls corrects dans `/maze` |
| Reset défauts | `POST /calib/reset` puis `GET /calib` | valeurs de config.h |
| Outlier rejection | Capteur occluté (valeur > 500) | correction figée à u_prev, pas de secousse |

---

## 10. Risques et mitigations

| Risque | Mitigation |
|---|---|
| LittleFS corruption | `calib_reset_defaults()` accessible via IHM |
| Capture pendant mouvement | Garde `STATE_IDLE` côté main.cpp |
| Kp trop élevé → oscillations | Défauts conservateurs (Kp=0.4), à tuner en phase de test |
| ToF bruités en intersection | Outlier rejection + désactivation PID si passage unilatéral |
| Boucle navigation bloque WiFi | `yield()` dans la boucle + PID léger |
| Overflow RAM LittleFS | JSON < 128 octets, pas d'I/O dynamique |

---

## 11. Fichiers modifiés / créés

**Créés** :
- `src/calibration.h`
- `src/calibration.cpp`
- `src/pid_lateral.h`
- `src/pid_lateral.cpp`

**Modifiés** :
- `src/config.h` — 4 défauts de seuils + coeffs PID latéral + CALIB_SAMPLES/MS
- `src/web_ui.h` — 5 nouvelles commandes enum
- `src/web_ui.cpp` — 5 nouvelles routes + panneau HTML calibration
- `src/navigation.cpp` — utilisation des seuils calibrés dans primitive case-par-case
- `src/main.cpp` — appel `calibration_init()`, `pid_lat_init()`, consommation des flags calib en IDLE
- `platformio.ini` — ajouter LittleFS (ESP8266 LittleFS est inclus dans le core Arduino, aucune lib_deps à ajouter, mais `board_build.filesystem = littlefs` si pas déjà présent)

---

## 12. Ordre d'implémentation suggéré

1. `config.h` — ajouter défauts + coeffs.
2. `calibration` (header + impl) sans LittleFS d'abord → valider API.
3. Ajouter LittleFS dans `calibration_init/save`.
4. `pid_lateral` (header + impl) — testable à l'oscillo via Serial.
5. `web_ui` — ajouter commandes enum + 5 routes + UI HTML.
6. `main.cpp` — câblage init + consommation flags.
7. `navigation.cpp` — intégration PID latéral + seuils dans primitive avance case.
8. Tests sur table (calibration + PID + détection murs).
