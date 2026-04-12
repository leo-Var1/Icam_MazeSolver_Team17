# MAZEBOT T17 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the complete software stack for a 5x5 maze-solving robot with live web UI, PID tuning, Kalman filtering, Tremaux exploration, and BFS resolution.

**Architecture:** Bottom-up module construction with early web UI for real-time calibration. Each module is a .h/.cpp pair with clear interfaces. The main loop is a non-blocking state machine using millis(). All constants live in config.h.

**Tech Stack:** ESP8266/Arduino (PlatformIO), C++17, ESPAsyncWebServer, VL53L0X (Pololu), MCP23017 (Adafruit), MPU6050 (I2Cdevlib), ArduinoJson 7

---

## File Structure

```
src/
  config.h            <- All constants, pinout, PID params, thresholds
  mcp_io.h / .cpp     <- MCP23017 init, GPIO read/write wrapper
  leds.h / .cpp       <- LED patterns via MCP23017 (blink, solid, off)
  sensors.h / .cpp    <- VL53L0X init (XSHUT), read distances, Kalman filter
  imu.h / .cpp        <- MPU6050 init, yaw angle, rotation detection
  encoders.h / .cpp   <- ISR tick counting, speed calc, distance calc
  motors.h / .cpp     <- PWM setup, direction, brake, ramp
  pid.h / .cpp        <- Discrete PID controller (Z-transform), motor + lateral
  diagnostic.h / .cpp <- Hardware validation at boot
  maze.h / .cpp       <- 5x5 grid, Cell struct, wall bitmask, robot position
  tremaux.h / .cpp    <- Tremaux algorithm, marking, intersection decision
  bfs.h / .cpp        <- BFS shortest path, path reconstruction
  navigation.h / .cpp <- Movement primitives: advance, turn, auto-align, bell curve
  web_ui.h / .cpp     <- Async web server, HTML/CSS/JS in PROGMEM, REST API
  main.cpp            <- setup(), loop(), state machine
platformio.ini        <- Board config, lib_deps, build flags
```

---

## Task 1: Project Foundation — platformio.ini + config.h

**Files:**
- Modify: `platformio.ini`
- Create: `src/config.h`

- [ ] **Step 1: Update platformio.ini with all dependencies**

```ini
; platformio.ini
[env:nodemcuv2]
platform = espressif8266
board = nodemcuv2
framework = arduino
monitor_speed = 115200
upload_speed = 921600

lib_deps =
  pololu/VL53L0X @ ^1.3.1
  adafruit/Adafruit MCP23017 Arduino Library @ ^2.3.2
  electroniccats/MPU6050 @ ^1.3.0
  me-no-dev/ESPAsyncWebServer @ ^1.2.4
  me-no-dev/ESPAsyncTCP @ ^1.2.2
  bblanchon/ArduinoJson @ ^7.0.0

build_flags =
  -D PIO_FRAMEWORK_ARDUINO_LWIP2_LOW_MEMORY
```

- [ ] **Step 2: Create config.h with all constants**

```cpp
// src/config.h
// Fichier de configuration central — TOUTES les constantes du projet
// Modifier ici plutot que dans le code pour eviter les magic numbers
#pragma once

#include <Arduino.h>

// =====================================================================
// I2C Bus
// =====================================================================
#define PIN_SCL         5   // D1 — Bus I2C SCL
#define PIN_SDA         4   // D2 — Bus I2C SDA

// =====================================================================
// Moteur GAUCHE (DRV8833 sorties A)
// NB: HIGH,HIGH au boot = freinage immediat (securite native via pull-ups)
// =====================================================================
#define MOTOR_L_IN1     0   // D3 — AIN1 | Pull-up 10k -> boot HIGH
#define MOTOR_L_IN2     2   // D4 — AIN2 | Pull-up 10k -> boot HIGH

// =====================================================================
// Moteur DROIT (DRV8833 sorties B)
// =====================================================================
#define MOTOR_R_IN1     13  // D7 — BIN1
#define MOTOR_R_IN2     15  // D8 — BIN2 | Pull-down 10k -> boot LOW

// =====================================================================
// Encodeurs
// =====================================================================
#define ENC_L_A         14  // D5 — Phase A gauche | Interruption
#define ENC_L_B         16  // D0 — Phase B gauche | Sens de rotation
#define ENC_R_A         12  // D6 — Phase A droit  | Interruption
// Phase B droit = A0 (analogique, lire dans ISR de ENC_R_A)
// analogRead(A0) > 512 => HIGH, sinon LOW

// =====================================================================
// Adresses I2C
// =====================================================================
#define I2C_MCP23017    0x27  // Expandeur
#define I2C_MPU6050     0x68  // Gyroscope (AD0=GND)

// VL53L0X : adresses assignees dynamiquement via XSHUT au boot
#define TOF_ADDR_FL     0x30  // Front Left
#define TOF_ADDR_FR     0x31  // Front Right
#define TOF_ADDR_SL     0x32  // Side Left 45 deg
#define TOF_ADDR_SR     0x33  // Side Right 45 deg

// =====================================================================
// Pins XSHUT des VL53L0X sur MCP23017 (ports GPA)
// =====================================================================
#define MCP_XSHUT_FL    1   // GPA1 -> VL53L0X Front Left
#define MCP_XSHUT_FR    0   // GPA0 -> VL53L0X Front Right
#define MCP_XSHUT_SL    3   // GPA3 -> VL53L0X Side Left 45 deg
#define MCP_XSHUT_SR    2   // GPA2 -> VL53L0X Side Right 45 deg

// =====================================================================
// Capteur de ligne GT1140 sur MCP23017
// =====================================================================
#define MCP_LINE_SENSOR 4   // GPA4 -> GT1140 (detection case arrivee)

// =====================================================================
// LEDs sur MCP23017 (ports GPB)
// =====================================================================
#define MCP_LED_RED     8  // GPB0 -> LED Rouge  (erreur / urgence)
#define MCP_LED_YELLOW  9  // GPB1 -> LED Jaune  (Run 1 exploration)
#define MCP_LED_GREEN   10  // GPB2 -> LED Verte  (Run 2 / succes / idle)

// =====================================================================
// Parametres moteurs
// =====================================================================
#define PWM_FREQ_HZ     10000  // 10kHz — evite sifflements audibles
#define PWM_RUN1        90     // ~35% de 255 — vitesse exploration
#define PWM_RUN2        230    // ~90% de 255 — vitesse resolution
#define PWM_TURN        70     // PWM pour les rotations sur place
#define PWM_DIAG        50     // PWM faible pour le diagnostic moteurs

// =====================================================================
// Encodeurs — Constantes mecaniques
// =====================================================================
#define TICKS_PER_REV       420    // Ticks par tour en sortie de reducteur
#define WHEEL_DIAMETER_MM   43.0f  // Diametre roue en mm
#define WHEEL_PERIMETER_MM  (PI * WHEEL_DIAMETER_MM)  // ~135.1mm
#define TICKS_PER_MM        (TICKS_PER_REV / WHEEL_PERIMETER_MM) // ~3.11
#define WHEEL_BASE_MM       100.0f // Entraxe roues en mm (A MESURER ET AJUSTER)

// Distance d'une case en ticks
#define CELL_SIZE_MM        200
#define TICKS_PER_CELL      ((int)(CELL_SIZE_MM * TICKS_PER_MM)) // ~622

// Ticks pour une rotation 90 deg sur place
// = (PI/2 * entraxe / 2) * ticks_per_mm pour chaque roue
#define TICKS_PER_90DEG     ((int)(PI / 2.0f * WHEEL_BASE_MM / 2.0f * TICKS_PER_MM))

// =====================================================================
// Labyrinthe
// =====================================================================
#define MAZE_SIZE       5     // Grille 5x5
#define START_ROW       0     // Case depart
#define START_COL       0
#define START_DIR       0     // 0=Nord
#define END_ROW         4     // Case arrivee (configurable via IHM)
#define END_COL         4

// Direction bitmask pour les murs
#define DIR_N           0     // Index Nord
#define DIR_E           1     // Index Est
#define DIR_S           2     // Index Sud
#define DIR_W           3     // Index Ouest
#define WALL_N          0x01  // Bitmask mur Nord
#define WALL_E          0x02  // Bitmask mur Est
#define WALL_S          0x04  // Bitmask mur Sud
#define WALL_W          0x08  // Bitmask mur Ouest

// =====================================================================
// PID — Valeurs par defaut (ajustables via IHM)
// =====================================================================
#define PID_SAMPLE_MS       20     // Periode echantillonnage = 50Hz

// PID vitesse moteur (transformee en Z)
#define PID_MOTOR_KP        1.2f
#define PID_MOTOR_KI        0.05f
#define PID_MOTOR_KD        0.8f

// PID correction laterale (centrage couloir)
#define PID_LAT_KP          0.8f
#define PID_LAT_KI          0.02f
#define PID_LAT_KD          0.5f

// =====================================================================
// Seuils capteurs ToF (ajustables via IHM)
// =====================================================================
#define TOF_WALL_FRONT_MM       120   // Mur frontal si distance < 120mm
#define TOF_WALL_SIDE_MM        100   // Mur lateral si distance < 100mm (a 45 deg)
#define TOF_NO_WALL_SIDE_MM     150   // Pas de mur lateral si distance > 150mm
#define TOF_STOP_FRONT_MM       50    // Arret d'urgence si mur < 50mm devant
#define TOF_ALIGN_TARGET_MM     40    // Distance cible pour auto-calage frontal
#define TOF_ALIGN_TOLERANCE_MM  2     // Tolerance alignement FL vs FR

// =====================================================================
// Filtre de Kalman — Valeurs par defaut (ajustables via IHM)
// =====================================================================
#define KALMAN_Q        0.01f  // Bruit processus (faible = mur stable)
#define KALMAN_R        5.0f   // Bruit mesure (eleve = poteaux)

// =====================================================================
// WiFi Access Point
// =====================================================================
#define WIFI_SSID       "Robot_Laby_Eq17"
#define WIFI_PASSWORD   "icam2026"
#define WIFI_PORT       80

// =====================================================================
// Timings non-bloquants (millis)
// =====================================================================
#define LED_BLINK_MS        500   // Periode clignotement LED
#define SENSOR_READ_MS      50    // Lecture capteurs ToF toutes les 50ms
#define WEB_POLL_MS         200   // Rafraichissement donnees IHM
#define MAZE_POLL_MS        300   // Rafraichissement carte IHM
#define DIAG_MOTOR_MS       500   // Duree test moteur diagnostic
#define TURN_SETTLE_MS      50    // Stabilisation avant rotation

// =====================================================================
// Machine d'etats
// =====================================================================
enum RobotState {
    STATE_IDLE,
    STATE_DIAGNOSTIC,
    STATE_CALIBRATION,
    STATE_RUN1_TREMAUX,
    STATE_MAZE_COMPLETE,
    STATE_RUN2_BFS,
    STATE_FINISHED,
    STATE_EMERGENCY
};
```

- [ ] **Step 3: Update main.cpp with minimal skeleton**

```cpp
// src/main.cpp
#include <Arduino.h>
#include "config.h"

RobotState currentState = STATE_DIAGNOSTIC;

void setup() {
    Serial.begin(115200);
    delay(500); // Seul delay autorise : au boot pour stabiliser Serial
    Serial.println("\n=== MAZEBOT T17 — Equipe 17 ===");
    Serial.println("Demarrage...\n");
}

void loop() {
    // Machine d'etats non-bloquante (millis)
    // Sera remplie au fur et a mesure des tasks
}
```

- [ ] **Step 4: Compile to verify setup**

Run: `pio run`
Expected: compilation reussie, 0 erreurs

- [ ] **Step 5: Commit**

```bash
git add platformio.ini src/config.h src/main.cpp
git commit -m "feat: project foundation — config.h + platformio.ini with all deps"
```

---

## Task 2: MCP23017 Wrapper + LEDs

**Files:**
- Create: `src/mcp_io.h`
- Create: `src/mcp_io.cpp`
- Create: `src/leds.h`
- Create: `src/leds.cpp`

- [ ] **Step 1: Create mcp_io.h — interface MCP23017**

```cpp
// src/mcp_io.h
// Wrapper pour le MCP23017 (expandeur GPIO I2C)
// Gere les pins XSHUT des VL53L0X, les LEDs, et le capteur de ligne GT1140
#pragma once

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

// Initialise le MCP23017 sur le bus I2C
// Configure les pins XSHUT en OUTPUT, LED en OUTPUT, GT1140 en INPUT
// Retourne true si OK, false si echec
bool mcpInit();

// Met une pin XSHUT a HIGH (active le capteur) ou LOW (desactive)
void mcpSetXshut(uint8_t pin, bool state);

// Met toutes les pins XSHUT a LOW (eteint tous les VL53L0X)
void mcpAllXshutLow();

// Controle une LED (pin = MCP_LED_RED/YELLOW/GREEN)
void mcpSetLed(uint8_t pin, bool state);

// Eteint toutes les LEDs
void mcpAllLedsOff();

// Lit l'etat du capteur de ligne GT1140
// Retourne true si ligne noire detectee
bool mcpReadLineSensor();

// Acces a l'objet MCP pour usage avance (diagnostic)
Adafruit_MCP23X17& mcpGetDevice();
```

- [ ] **Step 2: Create mcp_io.cpp — implementation**

```cpp
// src/mcp_io.cpp
#include "mcp_io.h"
#include "config.h"

// Instance statique du MCP23017 — pas d'allocation dynamique
static Adafruit_MCP23X17 mcp;

bool mcpInit() {
    if (!mcp.begin_I2C(I2C_MCP23017)) {
        Serial.println("[MCP] ERREUR: MCP23017 non trouve a 0x20");
        return false;
    }

    // Configurer les 4 pins XSHUT en sortie (GPA0-GPA3)
    mcp.pinMode(MCP_XSHUT_FL, OUTPUT);
    mcp.pinMode(MCP_XSHUT_FR, OUTPUT);
    mcp.pinMode(MCP_XSHUT_SL, OUTPUT);
    mcp.pinMode(MCP_XSHUT_SR, OUTPUT);

    // Capteur de ligne GT1140 en entree (GPA4)
    mcp.pinMode(MCP_LINE_SENSOR, INPUT);

    // LEDs en sortie (GPB4-GPB6)
    mcp.pinMode(MCP_LED_RED, OUTPUT);
    mcp.pinMode(MCP_LED_YELLOW, OUTPUT);
    mcp.pinMode(MCP_LED_GREEN, OUTPUT);

    // Etat initial : tous XSHUT LOW, toutes LEDs OFF
    mcpAllXshutLow();
    mcpAllLedsOff();

    Serial.println("[MCP] MCP23017 initialise OK");
    return true;
}

void mcpSetXshut(uint8_t pin, bool state) {
    mcp.digitalWrite(pin, state ? HIGH : LOW);
}

void mcpAllXshutLow() {
    mcp.digitalWrite(MCP_XSHUT_FL, LOW);
    mcp.digitalWrite(MCP_XSHUT_FR, LOW);
    mcp.digitalWrite(MCP_XSHUT_SL, LOW);
    mcp.digitalWrite(MCP_XSHUT_SR, LOW);
}

void mcpSetLed(uint8_t pin, bool state) {
    mcp.digitalWrite(pin, state ? HIGH : LOW);
}

void mcpAllLedsOff() {
    mcp.digitalWrite(MCP_LED_RED, LOW);
    mcp.digitalWrite(MCP_LED_YELLOW, LOW);
    mcp.digitalWrite(MCP_LED_GREEN, LOW);
}

bool mcpReadLineSensor() {
    return mcp.digitalRead(MCP_LINE_SENSOR) == HIGH;
}

Adafruit_MCP23X17& mcpGetDevice() {
    return mcp;
}
```

- [ ] **Step 3: Create leds.h — interface LED**

```cpp
// src/leds.h
// Gestion des 3 LEDs (Rouge, Jaune, Verte) via MCP23017
// Supporte : allumage fixe, clignotement non-bloquant, extinction
#pragma once

#include <Arduino.h>

// Initialise le systeme de LEDs (appeler apres mcpInit)
void ledsInit();

// Met a jour les clignotements — appeler dans loop()
void ledsUpdate();

// Modes de LED
void ledSetSolid(uint8_t pin, bool on);   // Allumage fixe
void ledSetBlink(uint8_t pin);            // Clignotement
void ledSetOff(uint8_t pin);              // Eteindre

// Patterns pre-definis pour chaque etat
void ledsSetIdle();        // Vert fixe
void ledsSetDiagnostic();  // Jaune fixe
void ledsSetRun1();        // Jaune clignotant
void ledsSetMazeComplete();// Vert + Jaune fixes
void ledsSetRun2();        // Vert clignotant
void ledsSetFinished();    // Vert fixe
void ledsSetEmergency();   // Rouge clignotant rapide
void ledsSetAllOff();      // Tout eteint
```

- [ ] **Step 4: Create leds.cpp — implementation avec clignotement non-bloquant**

```cpp
// src/leds.cpp
#include "leds.h"
#include "mcp_io.h"
#include "config.h"

// Etat de chaque LED : 0=off, 1=solid, 2=blink
static uint8_t ledMode[3];
// Pins correspondantes
static const uint8_t ledPins[3] = { MCP_LED_RED, MCP_LED_YELLOW, MCP_LED_GREEN };
// Timer pour le clignotement
static unsigned long lastBlinkMs = 0;
static bool blinkState = false;

void ledsInit() {
    for (int i = 0; i < 3; i++) ledMode[i] = 0;
    mcpAllLedsOff();
}

void ledsUpdate() {
    unsigned long now = millis();

    // Mise a jour du clignotement toutes les LED_BLINK_MS
    if (now - lastBlinkMs >= LED_BLINK_MS) {
        lastBlinkMs = now;
        blinkState = !blinkState;
    }

    // Appliquer l'etat de chaque LED
    for (int i = 0; i < 3; i++) {
        switch (ledMode[i]) {
            case 0: mcpSetLed(ledPins[i], false); break;       // Off
            case 1: mcpSetLed(ledPins[i], true); break;        // Solid
            case 2: mcpSetLed(ledPins[i], blinkState); break;  // Blink
        }
    }
}

void ledSetSolid(uint8_t pin, bool on) {
    for (int i = 0; i < 3; i++) {
        if (ledPins[i] == pin) ledMode[i] = on ? 1 : 0;
    }
}

void ledSetBlink(uint8_t pin) {
    for (int i = 0; i < 3; i++) {
        if (ledPins[i] == pin) ledMode[i] = 2;
    }
}

void ledSetOff(uint8_t pin) {
    for (int i = 0; i < 3; i++) {
        if (ledPins[i] == pin) ledMode[i] = 0;
    }
}

void ledsSetIdle()         { ledsSetAllOff(); ledSetSolid(MCP_LED_GREEN, true); }
void ledsSetDiagnostic()   { ledsSetAllOff(); ledSetSolid(MCP_LED_YELLOW, true); }
void ledsSetRun1()         { ledsSetAllOff(); ledSetBlink(MCP_LED_YELLOW); }
void ledsSetMazeComplete() { ledsSetAllOff(); ledSetSolid(MCP_LED_GREEN, true); ledSetSolid(MCP_LED_YELLOW, true); }
void ledsSetRun2()         { ledsSetAllOff(); ledSetBlink(MCP_LED_GREEN); }
void ledsSetFinished()     { ledsSetAllOff(); ledSetSolid(MCP_LED_GREEN, true); }
void ledsSetEmergency()    { ledsSetAllOff(); ledSetBlink(MCP_LED_RED); }
void ledsSetAllOff()       { for (int i = 0; i < 3; i++) ledMode[i] = 0; }
```

- [ ] **Step 5: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 6: Commit**

```bash
git add src/mcp_io.h src/mcp_io.cpp src/leds.h src/leds.cpp
git commit -m "feat: MCP23017 wrapper + LED patterns with non-blocking blink"
```

---

## Task 3: VL53L0X Sensors + Kalman Filter

**Files:**
- Create: `src/sensors.h`
- Create: `src/sensors.cpp`

- [ ] **Step 1: Create sensors.h — interface capteurs ToF + Kalman**

```cpp
// src/sensors.h
// Gestion des 4 capteurs VL53L0X Time-of-Flight
// - Init avec adressage sequentiel via XSHUT (MCP23017)
// - Lecture distances brutes et filtrees (Kalman 1D)
// - Seuils de detection de mur configurables
#pragma once

#include <Arduino.h>

// Structure pour un filtre de Kalman 1D
struct KalmanFilter {
    float estimate;     // Estimation courante (mm)
    float errorEst;     // Erreur d'estimation
    float errorMeas;    // Bruit de mesure (R) — ajustable via IHM
    float processNoise; // Bruit de processus (Q) — ajustable via IHM
};

// Initialise les 4 VL53L0X via adressage XSHUT sequentiel
// Retourne un bitmask : bit 0=FL, 1=FR, 2=SL, 3=SR (1=OK, 0=FAIL)
uint8_t sensorsInit();

// Lit les 4 capteurs (appeler periodiquement, non-bloquant)
void sensorsRead();

// Distances brutes (mm) — derniere lecture
uint16_t sensorRawFL();
uint16_t sensorRawFR();
uint16_t sensorRawSL();
uint16_t sensorRawSR();

// Distances filtrees Kalman (mm)
float sensorFilteredFL();
float sensorFilteredFR();
float sensorFilteredSL();
float sensorFilteredSR();

// Detection de murs (basee sur distances filtrees)
bool wallFront();    // Mur devant ?
bool wallLeft();     // Mur a gauche ?
bool wallRight();    // Mur a droite ?

// Mise a jour des parametres Kalman depuis l'IHM
void sensorsSetKalmanQ(float q);
void sensorsSetKalmanR(float r);

// Mise a jour des seuils de detection depuis l'IHM
void sensorsSetThresholdFront(uint16_t mm);
void sensorsSetThresholdSide(uint16_t mm);
void sensorsSetThresholdNoWall(uint16_t mm);
```

- [ ] **Step 2: Create sensors.cpp — implementation avec XSHUT + Kalman**

```cpp
// src/sensors.cpp
#include "sensors.h"
#include "mcp_io.h"
#include "config.h"
#include <VL53L0X.h>
#include <Wire.h>

// 4 instances de capteurs VL53L0X — statiques, pas de new/malloc
static VL53L0X tofFL, tofFR, tofSL, tofSR;

// Distances brutes en mm
static uint16_t rawFL = 0, rawFR = 0, rawSL = 0, rawSR = 0;

// Filtres de Kalman (un par capteur)
static KalmanFilter kfFL, kfFR, kfSL, kfSR;

// Seuils de detection (modifiables via IHM)
static uint16_t threshFront = TOF_WALL_FRONT_MM;
static uint16_t threshSide  = TOF_WALL_SIDE_MM;
static uint16_t threshNoWall = TOF_NO_WALL_SIDE_MM;

// ----- Kalman 1D -----

// Initialise un filtre de Kalman
static void kalmanInit(KalmanFilter& kf, float initValue) {
    kf.estimate     = initValue;
    kf.errorEst     = 50.0f;        // Incertitude initiale elevee
    kf.errorMeas    = KALMAN_R;     // Bruit de mesure (poteaux)
    kf.processNoise = KALMAN_Q;     // Bruit de processus (mur stable)
}

// Met a jour le filtre avec une nouvelle mesure
// Retourne la nouvelle estimation filtree
static float kalmanUpdate(KalmanFilter& kf, float measurement) {
    // Prediction : l'estimation ne change pas (modele statique)
    kf.errorEst += kf.processNoise;

    // Gain de Kalman
    float K = kf.errorEst / (kf.errorEst + kf.errorMeas);

    // Mise a jour
    kf.estimate = kf.estimate + K * (measurement - kf.estimate);
    kf.errorEst = (1.0f - K) * kf.errorEst;

    return kf.estimate;
}

// ----- Init VL53L0X avec adressage XSHUT -----

// Initialise un seul capteur VL53L0X :
// 1. Active son XSHUT
// 2. Attend 10ms pour le boot du capteur
// 3. Change son adresse I2C
// 4. Lance la mesure continue
static bool initOneSensor(VL53L0X& sensor, uint8_t xshutPin, uint8_t newAddr, const char* name) {
    mcpSetXshut(xshutPin, true);  // Activer le capteur
    delay(10);                     // Attendre le boot (seul delay acceptable: init)

    sensor.setAddress(newAddr);

    if (!sensor.init()) {
        Serial.printf("[TOF] %s (0x%02X): INIT FAIL\n", name, newAddr);
        return false;
    }

    // Mode lecture continue — non-bloquant apres init
    sensor.setMeasurementTimingBudget(20000); // 20ms par mesure
    sensor.startContinuous();

    Serial.printf("[TOF] %s (0x%02X): OK\n", name, newAddr);
    return true;
}

uint8_t sensorsInit() {
    uint8_t result = 0;

    // Etape 1 : Tous les XSHUT a LOW (eteindre tous les capteurs)
    mcpAllXshutLow();
    delay(50); // Attendre l'extinction complete

    // Etape 2 : Activer un par un et assigner une adresse unique
    if (initOneSensor(tofFL, MCP_XSHUT_FL, TOF_ADDR_FL, "FL")) result |= 0x01;
    if (initOneSensor(tofFR, MCP_XSHUT_FR, TOF_ADDR_FR, "FR")) result |= 0x02;
    if (initOneSensor(tofSL, MCP_XSHUT_SL, TOF_ADDR_SL, "SL")) result |= 0x04;
    if (initOneSensor(tofSR, MCP_XSHUT_SR, TOF_ADDR_SR, "SR")) result |= 0x08;

    // Initialiser les filtres de Kalman
    kalmanInit(kfFL, 200.0f);
    kalmanInit(kfFR, 200.0f);
    kalmanInit(kfSL, 100.0f);
    kalmanInit(kfSR, 100.0f);

    return result; // 0x0F = tous OK
}

void sensorsRead() {
    // Lecture non-bloquante (readRangeContinuousMillimeters retourne immediatement
    // si une mesure est prete, sinon retourne la derniere valeur)
    uint16_t fl = tofFL.readRangeContinuousMillimeters();
    uint16_t fr = tofFR.readRangeContinuousMillimeters();
    uint16_t sl = tofSL.readRangeContinuousMillimeters();
    uint16_t sr = tofSR.readRangeContinuousMillimeters();

    // Ignorer les lectures aberrantes (timeout du capteur = 65535)
    if (fl < 2000) rawFL = fl;
    if (fr < 2000) rawFR = fr;
    if (sl < 2000) rawSL = sl;
    if (sr < 2000) rawSR = sr;

    // Appliquer le filtre de Kalman
    kalmanUpdate(kfFL, (float)rawFL);
    kalmanUpdate(kfFR, (float)rawFR);
    kalmanUpdate(kfSL, (float)rawSL);
    kalmanUpdate(kfSR, (float)rawSR);
}

// ----- Getters distances brutes -----
uint16_t sensorRawFL() { return rawFL; }
uint16_t sensorRawFR() { return rawFR; }
uint16_t sensorRawSL() { return rawSL; }
uint16_t sensorRawSR() { return rawSR; }

// ----- Getters distances filtrees -----
float sensorFilteredFL() { return kfFL.estimate; }
float sensorFilteredFR() { return kfFR.estimate; }
float sensorFilteredSL() { return kfSL.estimate; }
float sensorFilteredSR() { return kfSR.estimate; }

// ----- Detection de murs -----
bool wallFront() {
    // Mur devant si FL et FR < seuil frontal
    return (kfFL.estimate < threshFront) && (kfFR.estimate < threshFront);
}

bool wallLeft() {
    // Mur a gauche si SL < seuil lateral (capteur a 45 deg)
    return kfSL.estimate < threshSide;
}

bool wallRight() {
    // Mur a droite si SR < seuil lateral (capteur a 45 deg)
    return kfSR.estimate < threshSide;
}

// ----- Parametres Kalman ajustables -----
void sensorsSetKalmanQ(float q) {
    kfFL.processNoise = q;
    kfFR.processNoise = q;
    kfSL.processNoise = q;
    kfSR.processNoise = q;
}

void sensorsSetKalmanR(float r) {
    kfFL.errorMeas = r;
    kfFR.errorMeas = r;
    kfSL.errorMeas = r;
    kfSR.errorMeas = r;
}

// ----- Seuils ajustables -----
void sensorsSetThresholdFront(uint16_t mm) { threshFront = mm; }
void sensorsSetThresholdSide(uint16_t mm)  { threshSide = mm; }
void sensorsSetThresholdNoWall(uint16_t mm) { threshNoWall = mm; }
```

- [ ] **Step 3: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 4: Commit**

```bash
git add src/sensors.h src/sensors.cpp
git commit -m "feat: VL53L0X sensors with XSHUT sequencing + Kalman 1D filter"
```

---

## Task 4: MPU6050 IMU

**Files:**
- Create: `src/imu.h`
- Create: `src/imu.cpp`

- [ ] **Step 1: Create imu.h**

```cpp
// src/imu.h
// Gestion du gyroscope MPU6050
// - Lecture angle yaw (rotation autour de l'axe vertical)
// - Detection de rotation (90 deg, 180 deg)
// - Verification secondaire pour les virages (encodeurs = primaire)
#pragma once

#include <Arduino.h>

// Initialise le MPU6050
// Retourne true si OK, false si echec
bool imuInit();

// Met a jour l'angle yaw — appeler le plus souvent possible dans loop()
// Integre la vitesse angulaire du gyroscope pour obtenir l'angle
void imuUpdate();

// Retourne l'angle yaw actuel en degres (cumule depuis le dernier reset)
float imuGetYaw();

// Retourne la vitesse angulaire brute en deg/s
float imuGetYawRate();

// Remet l'angle yaw a zero (avant un virage)
void imuResetYaw();

// Verifie si le MPU6050 est vivant (valeurs non figees)
bool imuIsAlive();
```

- [ ] **Step 2: Create imu.cpp**

```cpp
// src/imu.cpp
#include "imu.h"
#include "config.h"
#include <MPU6050.h>
#include <Wire.h>

static MPU6050 mpu;

// Angle yaw cumule (integration du gyroscope)
static float yawAngle = 0.0f;
static float yawRate  = 0.0f;
static unsigned long lastUpdateUs = 0;

// Offset de calibration du gyroscope (derive au repos)
static float gyroZOffset = 0.0f;

// Calibre le gyroscope en mesurant la derive au repos
// Le robot doit etre IMMOBILE pendant cette phase
static void calibrateGyro() {
    Serial.print("[IMU] Calibration gyroscope...");
    float sum = 0;
    const int samples = 500;

    for (int i = 0; i < samples; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sum += gz;
        delay(2); // Acceptable pendant l'init uniquement
    }

    gyroZOffset = sum / samples;
    Serial.printf(" offset Z = %.1f\n", gyroZOffset);
}

bool imuInit() {
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("[IMU] ERREUR: MPU6050 non trouve a 0x68");
        return false;
    }

    // Sensibilite gyroscope : ±250 deg/s (la plus precise)
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    // Filtre passe-bas interne du MPU6050
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);

    calibrateGyro();

    lastUpdateUs = micros();
    yawAngle = 0.0f;

    Serial.println("[IMU] MPU6050 initialise OK");
    return true;
}

void imuUpdate() {
    unsigned long now = micros();
    float dt = (now - lastUpdateUs) / 1000000.0f; // Delta en secondes
    lastUpdateUs = now;

    // Eviter les sauts temporels aberrants
    if (dt > 0.1f) dt = 0.1f;

    // Lire la vitesse angulaire brute sur l'axe Z
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // Convertir en deg/s (sensibilite 250 deg/s => 131 LSB/deg/s)
    yawRate = (gz - gyroZOffset) / 131.0f;

    // Filtrer le bruit : ignorer les petites rotations (< 0.5 deg/s)
    if (fabs(yawRate) < 0.5f) yawRate = 0.0f;

    // Integrer pour obtenir l'angle
    yawAngle += yawRate * dt;
}

float imuGetYaw()     { return yawAngle; }
float imuGetYawRate() { return yawRate; }

void imuResetYaw() {
    yawAngle = 0.0f;
}

bool imuIsAlive() {
    // Verifie que le gyroscope n'est pas fige
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    // Si les 3 axes sont exactement 0, c'est suspect
    return !(gx == 0 && gy == 0 && gz == 0);
}
```

- [ ] **Step 3: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 4: Commit**

```bash
git add src/imu.h src/imu.cpp
git commit -m "feat: MPU6050 IMU with yaw integration and gyro calibration"
```

---

## Task 5: Encodeurs

**Files:**
- Create: `src/encoders.h`
- Create: `src/encoders.cpp`

- [ ] **Step 1: Create encoders.h**

```cpp
// src/encoders.h
// Comptage de ticks via interruptions sur les phases A des encodeurs
// Phase B gauche = GPIO16 (D0) — lecture digitale dans l'ISR
// Phase B droite = A0 — lecture analogique dans l'ISR (seuil 512)
#pragma once

#include <Arduino.h>

// Initialise les interruptions sur ENC_L_A et ENC_R_A
void encodersInit();

// Retourne le nombre de ticks depuis le dernier reset
// Valeur signee : positif = avant, negatif = arriere
volatile long encoderGetLeft();
volatile long encoderGetRight();

// Remet les compteurs a zero
void encoderResetLeft();
void encoderResetRight();
void encoderResetBoth();

// Calcule la vitesse en ticks/s (appeler periodiquement)
void encodersComputeSpeed();

// Retourne la vitesse en ticks/s
float encoderSpeedLeft();
float encoderSpeedRight();

// Retourne la distance parcourue en mm depuis le dernier reset
float encoderDistanceLeft();
float encoderDistanceRight();
float encoderDistanceAvg();
```

- [ ] **Step 2: Create encoders.cpp**

```cpp
// src/encoders.cpp
#include "encoders.h"
#include "config.h"

// Compteurs de ticks — volatile car modifies dans les ISR
static volatile long ticksL = 0;
static volatile long ticksR = 0;

// Vitesses calculees
static float speedL = 0.0f;
static float speedR = 0.0f;

// Ticks precedents pour le calcul de vitesse
static long prevTicksL = 0;
static long prevTicksR = 0;
static unsigned long lastSpeedCalcMs = 0;

// --- ISR Encodeur Gauche ---
// Declenchee sur front montant de ENC_L_A (D5 = GPIO14)
// Lit ENC_L_B (D0 = GPIO16) pour determiner le sens
IRAM_ATTR void isrEncoderLeft() {
    // digitalRead est safe sur GPIO16 dans une ISR
    if (digitalRead(ENC_L_B) == HIGH) {
        ticksL++;   // Sens avant
    } else {
        ticksL--;   // Sens arriere
    }
}

// --- ISR Encodeur Droit ---
// Declenchee sur front montant de ENC_R_A (D6 = GPIO12)
// Lit A0 en analogique pour determiner le sens (pas d'interrupt sur A0)
IRAM_ATTR void isrEncoderRight() {
    // analogRead dans une ISR sur ESP8266 : rapide (~10us), acceptable
    if (analogRead(A0) > 512) {
        ticksR++;   // Sens avant
    } else {
        ticksR--;   // Sens arriere
    }
}

void encodersInit() {
    // Phase A : entrees avec interruption
    pinMode(ENC_L_A, INPUT);
    pinMode(ENC_R_A, INPUT);

    // Phase B gauche : entree digitale
    pinMode(ENC_L_B, INPUT);
    // Phase B droite : A0 est analogique par defaut, pas de pinMode

    // Attacher les interruptions sur front montant
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncoderRight, RISING);

    ticksL = 0;
    ticksR = 0;
    lastSpeedCalcMs = millis();

    Serial.println("[ENC] Encodeurs initialises OK");
}

volatile long encoderGetLeft()  { return ticksL; }
volatile long encoderGetRight() { return ticksR; }

void encoderResetLeft()  { ticksL = 0; }
void encoderResetRight() { ticksR = 0; }
void encoderResetBoth()  { ticksL = 0; ticksR = 0; }

void encodersComputeSpeed() {
    unsigned long now = millis();
    unsigned long dt = now - lastSpeedCalcMs;

    if (dt < PID_SAMPLE_MS) return; // Pas encore le moment

    // Desactiver les interruptions pour lire les ticks de facon atomique
    noInterrupts();
    long currentL = ticksL;
    long currentR = ticksR;
    interrupts();

    // Vitesse en ticks/s
    float dtSec = dt / 1000.0f;
    speedL = (currentL - prevTicksL) / dtSec;
    speedR = (currentR - prevTicksR) / dtSec;

    prevTicksL = currentL;
    prevTicksR = currentR;
    lastSpeedCalcMs = now;
}

float encoderSpeedLeft()  { return speedL; }
float encoderSpeedRight() { return speedR; }

float encoderDistanceLeft()  { return ticksL / TICKS_PER_MM; }
float encoderDistanceRight() { return ticksR / TICKS_PER_MM; }
float encoderDistanceAvg()   { return (encoderDistanceLeft() + encoderDistanceRight()) / 2.0f; }
```

- [ ] **Step 3: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 4: Commit**

```bash
git add src/encoders.h src/encoders.cpp
git commit -m "feat: encoder ISR with tick counting, speed and distance calculation"
```

---

## Task 6: Motors

**Files:**
- Create: `src/motors.h`
- Create: `src/motors.cpp`

- [ ] **Step 1: Create motors.h**

```cpp
// src/motors.h
// Controle des 2 moteurs DC via DRV8833
// PWM sur les 4 pins IN1/IN2 pour chaque moteur
// Convention : PWM positif = avant, negatif = arriere
#pragma once

#include <Arduino.h>

// Initialise les pins moteur en sortie PWM
void motorsInit();

// Definit la vitesse de chaque moteur
// pwm : -255 a +255 (negatif = arriere)
void motorSetLeft(int pwm);
void motorSetRight(int pwm);

// Definit les deux moteurs en meme temps
void motorSetBoth(int pwmLeft, int pwmRight);

// Freinage actif (court-circuit moteur via DRV8833 : IN1=IN2=HIGH)
void motorBrake();

// Roue libre (IN1=IN2=LOW)
void motorCoast();

// Securite : coupe tout (appele par arret d'urgence)
void motorEmergencyStop();
```

- [ ] **Step 2: Create motors.cpp**

```cpp
// src/motors.cpp
#include "motors.h"
#include "config.h"

// Applique un PWM signe a un moteur via ses 2 pins
// DRV8833 : IN1=PWM, IN2=LOW => avant
//           IN1=LOW, IN2=PWM => arriere
//           IN1=HIGH, IN2=HIGH => frein
//           IN1=LOW, IN2=LOW => roue libre
static void setMotor(uint8_t pin1, uint8_t pin2, int pwm) {
    // Clamper le PWM entre -255 et 255
    if (pwm > 255)  pwm = 255;
    if (pwm < -255) pwm = -255;

    if (pwm > 0) {
        analogWrite(pin1, pwm);
        analogWrite(pin2, 0);
    } else if (pwm < 0) {
        analogWrite(pin1, 0);
        analogWrite(pin2, -pwm);
    } else {
        // PWM = 0 => roue libre
        analogWrite(pin1, 0);
        analogWrite(pin2, 0);
    }
}

void motorsInit() {
    // Configurer les pins en sortie
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);

    // Configurer la frequence PWM (10kHz pour eviter les sifflements)
    analogWriteFreq(PWM_FREQ_HZ);

    // Freiner au demarrage par securite
    motorBrake();

    Serial.println("[MOT] Moteurs initialises OK");
}

void motorSetLeft(int pwm)  { setMotor(MOTOR_L_IN1, MOTOR_L_IN2, pwm); }
void motorSetRight(int pwm) { setMotor(MOTOR_R_IN1, MOTOR_R_IN2, pwm); }

void motorSetBoth(int pwmLeft, int pwmRight) {
    motorSetLeft(pwmLeft);
    motorSetRight(pwmRight);
}

void motorBrake() {
    // Freinage actif : IN1=HIGH, IN2=HIGH pour les deux moteurs
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, HIGH);
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, HIGH);
}

void motorCoast() {
    analogWrite(MOTOR_L_IN1, 0);
    analogWrite(MOTOR_L_IN2, 0);
    analogWrite(MOTOR_R_IN1, 0);
    analogWrite(MOTOR_R_IN2, 0);
}

void motorEmergencyStop() {
    motorBrake();
}
```

- [ ] **Step 3: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 4: Commit**

```bash
git add src/motors.h src/motors.cpp
git commit -m "feat: motor control with PWM, brake and coast via DRV8833"
```

---

## Task 7: Diagnostic Module

**Files:**
- Create: `src/diagnostic.h`
- Create: `src/diagnostic.cpp`
- Modify: `src/main.cpp`

- [ ] **Step 1: Create diagnostic.h**

```cpp
// src/diagnostic.h
// Routines de diagnostic hardware executees au boot
// Teste chaque composant et affiche un resume OK/FAIL en Serial
#pragma once

#include <Arduino.h>

// Structure pour stocker les resultats du diagnostic
struct DiagResult {
    bool i2cMcp;      // MCP23017 trouve sur I2C
    bool i2cMpu;      // MPU6050 trouve sur I2C
    bool ledRed;      // LED Rouge fonctionne
    bool ledYellow;   // LED Jaune fonctionne
    bool ledGreen;    // LED Verte fonctionne
    bool tofFL;       // VL53L0X Front Left OK
    bool tofFR;       // VL53L0X Front Right OK
    bool tofSL;       // VL53L0X Side Left OK
    bool tofSR;       // VL53L0X Side Right OK
    bool imu;         // MPU6050 fonctionnel
    bool lineSensor;  // GT1140 accessible
    bool motorLeft;   // Moteur gauche + encodeur OK
    bool motorRight;  // Moteur droit + encodeur OK

    // Retourne true si tous les composants critiques sont OK
    bool allCriticalOk() const;
};

// Execute le diagnostic complet
// Initialise tous les peripheriques et teste chacun
// Retourne la structure de resultats
DiagResult diagnosticRun();

// Affiche le resume du diagnostic en Serial
void diagnosticPrint(const DiagResult& result);

// Acces au dernier resultat (pour l'IHM web)
const DiagResult& diagnosticGetResult();
```

- [ ] **Step 2: Create diagnostic.cpp**

```cpp
// src/diagnostic.cpp
#include "diagnostic.h"
#include "config.h"
#include "mcp_io.h"
#include "leds.h"
#include "sensors.h"
#include "imu.h"
#include "encoders.h"
#include "motors.h"
#include <Wire.h>

static DiagResult lastResult;

bool DiagResult::allCriticalOk() const {
    // Critiques : MCP, au moins 2 ToF frontaux, encodeurs, moteurs
    return i2cMcp && tofFL && tofFR && motorLeft && motorRight;
}

// Scanne le bus I2C et verifie la presence des peripheriques attendus
static void scanI2C(DiagResult& r) {
    Serial.println("\n[I2C] Scan du bus...");

    r.i2cMcp = false;
    r.i2cMpu = false;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[I2C] Peripherique trouve a 0x%02X", addr);
            if (addr == I2C_MCP23017) { r.i2cMcp = true; Serial.print(" (MCP23017)"); }
            if (addr == I2C_MPU6050)  { r.i2cMpu = true; Serial.print(" (MPU6050)"); }
            Serial.println();
        }
    }

    Serial.printf("[I2C] Scan: MCP=0x%02X %s | MPU=0x%02X %s\n",
        I2C_MCP23017, r.i2cMcp ? "OK" : "FAIL",
        I2C_MPU6050,  r.i2cMpu ? "OK" : "FAIL");
}

// Teste les LEDs en les allumant une par une
static void testLEDs(DiagResult& r) {
    Serial.println("\n[MCP] Test LEDs...");

    // On assume OK si le MCP est present (pas de feedback LED)
    r.ledRed = r.i2cMcp;
    r.ledYellow = r.i2cMcp;
    r.ledGreen = r.i2cMcp;

    if (!r.i2cMcp) return;

    // Sequence visuelle : R -> J -> V, 300ms chacune
    mcpSetLed(MCP_LED_RED, true);
    delay(300);
    mcpSetLed(MCP_LED_RED, false);

    mcpSetLed(MCP_LED_YELLOW, true);
    delay(300);
    mcpSetLed(MCP_LED_YELLOW, false);

    mcpSetLed(MCP_LED_GREEN, true);
    delay(300);
    mcpSetLed(MCP_LED_GREEN, false);

    Serial.printf("[MCP] LEDs: R=%s J=%s V=%s\n",
        r.ledRed ? "OK" : "FAIL",
        r.ledYellow ? "OK" : "FAIL",
        r.ledGreen ? "OK" : "FAIL");
}

// Teste les capteurs ToF
static void testToF(DiagResult& r) {
    Serial.println("\n[TOF] Init VL53L0X...");

    uint8_t tofResult = sensorsInit();
    r.tofFL = (tofResult & 0x01) != 0;
    r.tofFR = (tofResult & 0x02) != 0;
    r.tofSL = (tofResult & 0x04) != 0;
    r.tofSR = (tofResult & 0x08) != 0;

    // Faire une premiere lecture pour verifier
    if (tofResult) {
        delay(50);
        sensorsRead();
        Serial.printf("[TOF] FL(0x%02X): %dmm %s\n", TOF_ADDR_FL, sensorRawFL(), r.tofFL ? "OK" : "FAIL");
        Serial.printf("[TOF] FR(0x%02X): %dmm %s\n", TOF_ADDR_FR, sensorRawFR(), r.tofFR ? "OK" : "FAIL");
        Serial.printf("[TOF] SL(0x%02X): %dmm %s\n", TOF_ADDR_SL, sensorRawSL(), r.tofSL ? "OK" : "FAIL");
        Serial.printf("[TOF] SR(0x%02X): %dmm %s\n", TOF_ADDR_SR, sensorRawSR(), r.tofSR ? "OK" : "FAIL");
    }
}

// Teste le MPU6050
static void testIMU(DiagResult& r) {
    Serial.println("\n[IMU] Test MPU6050...");
    r.imu = imuInit();
    if (r.imu) {
        r.imu = imuIsAlive();
    }
    Serial.printf("[IMU] MPU6050: %s\n", r.imu ? "OK" : "FAIL");
}

// Teste le capteur de ligne
static void testLineSensor(DiagResult& r) {
    Serial.println("\n[LINE] Test GT1140...");
    // On verifie juste qu'on peut lire la pin
    r.lineSensor = r.i2cMcp; // Si MCP OK, la pin est accessible
    bool lineState = mcpReadLineSensor();
    Serial.printf("[LINE] GT1140: %s (etat=%s)\n",
        r.lineSensor ? "OK" : "FAIL",
        lineState ? "LIGNE" : "PAS DE LIGNE");
}

// Teste les moteurs avec les encodeurs
static void testMotors(DiagResult& r) {
    Serial.println("\n[MOT] Test moteurs + encodeurs...");

    encodersInit();
    motorsInit();

    // Test moteur gauche : avant puis arriere
    encoderResetBoth();
    motorSetLeft(PWM_DIAG);
    delay(DIAG_MOTOR_MS);
    motorBrake();
    delay(100);
    long fwdL = encoderGetLeft();

    encoderResetLeft();
    motorSetLeft(-PWM_DIAG);
    delay(DIAG_MOTOR_MS);
    motorBrake();
    delay(100);
    long bwdL = encoderGetLeft();

    r.motorLeft = (fwdL > 10 && bwdL < -10); // Ticks dans les deux sens
    Serial.printf("[MOT] Left:  FWD=%ldt BWD=%ldt %s\n", fwdL, bwdL, r.motorLeft ? "OK" : "FAIL");

    // Test moteur droit : avant puis arriere
    encoderResetBoth();
    motorSetRight(PWM_DIAG);
    delay(DIAG_MOTOR_MS);
    motorBrake();
    delay(100);
    long fwdR = encoderGetRight();

    encoderResetRight();
    motorSetRight(-PWM_DIAG);
    delay(DIAG_MOTOR_MS);
    motorBrake();
    delay(100);
    long bwdR = encoderGetRight();

    r.motorRight = (fwdR > 10 && bwdR < -10);
    Serial.printf("[MOT] Right: FWD=%ldt BWD=%ldt %s\n", fwdR, bwdR, r.motorRight ? "OK" : "FAIL");

    motorCoast();
}

DiagResult diagnosticRun() {
    DiagResult r = {};

    Serial.println("\n========================================");
    Serial.println("=== DIAGNOSTIC Robot Eq17 ===");
    Serial.println("========================================");

    // Initialiser I2C
    Wire.begin(PIN_SDA, PIN_SCL);

    // 1. Scan I2C
    scanI2C(r);

    // 2. Init MCP23017 (necessaire pour tout le reste)
    if (r.i2cMcp) {
        r.i2cMcp = mcpInit();
    }

    // 3. Test LEDs
    testLEDs(r);

    // 4. Test VL53L0X
    if (r.i2cMcp) {
        testToF(r);
    }

    // 5. Test MPU6050
    if (r.i2cMpu) {
        testIMU(r);
    }

    // 6. Test capteur de ligne
    testLineSensor(r);

    // 7. Test moteurs + encodeurs
    testMotors(r);

    // Resume
    diagnosticPrint(r);

    lastResult = r;
    return r;
}

void diagnosticPrint(const DiagResult& r) {
    Serial.println("\n========================================");
    Serial.println("=== RESUME DIAGNOSTIC ===");
    Serial.println("========================================");
    Serial.printf("MCP23017:  %s\n", r.i2cMcp ? "OK" : "FAIL");
    Serial.printf("MPU6050:   %s\n", r.imu ? "OK" : "FAIL (non-critique)");
    Serial.printf("TOF FL:    %s\n", r.tofFL ? "OK" : "FAIL");
    Serial.printf("TOF FR:    %s\n", r.tofFR ? "OK" : "FAIL");
    Serial.printf("TOF SL:    %s\n", r.tofSL ? "OK" : "FAIL");
    Serial.printf("TOF SR:    %s\n", r.tofSR ? "OK" : "FAIL");
    Serial.printf("GT1140:    %s\n", r.lineSensor ? "OK" : "FAIL");
    Serial.printf("Moteur L:  %s\n", r.motorLeft ? "OK" : "FAIL");
    Serial.printf("Moteur R:  %s\n", r.motorRight ? "OK" : "FAIL");
    Serial.println("========================================");

    if (r.allCriticalOk()) {
        Serial.println("=== TOUS SYSTEMES CRITIQUES OK ===");
    } else {
        Serial.println("=== ECHEC CRITIQUE — EMERGENCY ===");
    }
    Serial.println("========================================\n");
}

const DiagResult& diagnosticGetResult() {
    return lastResult;
}
```

- [ ] **Step 3: Update main.cpp with diagnostic state machine**

```cpp
// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "mcp_io.h"
#include "leds.h"
#include "sensors.h"
#include "imu.h"
#include "encoders.h"
#include "motors.h"
#include "diagnostic.h"

RobotState currentState = STATE_DIAGNOSTIC;
static bool diagDone = false;

void setup() {
    Serial.begin(115200);
    delay(500); // Seul delay autorise : au boot pour stabiliser Serial
    Serial.println("\n=== MAZEBOT T17 — Equipe 17 ===");
    Serial.println("Demarrage...\n");
}

void loop() {
    // Toujours mettre a jour les LEDs (clignotement)
    ledsUpdate();

    switch (currentState) {
        case STATE_DIAGNOSTIC: {
            if (!diagDone) {
                ledsSetDiagnostic();
                DiagResult result = diagnosticRun();

                if (result.allCriticalOk()) {
                    currentState = STATE_IDLE;
                    ledsSetIdle();
                    Serial.println("[MAIN] Diagnostic OK -> IDLE");
                    Serial.println("[MAIN] En attente de commande (IHM web)...");
                } else {
                    currentState = STATE_EMERGENCY;
                    ledsSetEmergency();
                    Serial.println("[MAIN] Diagnostic FAIL -> EMERGENCY");
                }
                diagDone = true;
            }
            break;
        }

        case STATE_IDLE: {
            // Attente de commande depuis l'IHM web
            // Lire les capteurs periodiquement pour affichage
            static unsigned long lastSensorRead = 0;
            if (millis() - lastSensorRead >= SENSOR_READ_MS) {
                lastSensorRead = millis();
                sensorsRead();
                imuUpdate();
                encodersComputeSpeed();
            }
            break;
        }

        case STATE_EMERGENCY: {
            motorEmergencyStop();
            // LED rouge clignotante (geree par ledsUpdate)
            break;
        }

        default:
            break;
    }
}
```

- [ ] **Step 4: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 5: Flash and test on robot**

Run: `pio run --target upload && pio device monitor -b 115200`
Expected: le diagnostic s'execute, affiche le tableau OK/FAIL, LEDs clignotent

- [ ] **Step 6: Commit**

```bash
git add src/diagnostic.h src/diagnostic.cpp src/main.cpp
git commit -m "feat: hardware diagnostic at boot with full Serial report"
```

---

## Task 8: PID Controller (Discrete Z-Transform)

**Files:**
- Create: `src/pid.h`
- Create: `src/pid.cpp`

- [ ] **Step 1: Create pid.h**

```cpp
// src/pid.h
// Correcteur PID discret (transformee en Z)
// H(z) = Kp + Ki*Ts/(1 - z^-1) + Kd*(1 - z^-1)/Ts
//
// Deux usages :
// 1. PID vitesse moteur (un par roue) : consigne ticks/s vs mesure encodeur
// 2. PID lateral : erreur distance gauche-droite (ToF) -> correction vitesse
#pragma once

#include <Arduino.h>

struct PIDController {
    // Gains (ajustables via IHM)
    float kp;
    float ki;
    float kd;

    // Periode d'echantillonnage en secondes
    float ts;

    // Etats internes
    float prevError;    // e[n-1] pour le terme derive
    float integral;     // Somme des erreurs pour le terme integral

    // Limites de sortie (anti-windup)
    float outMin;
    float outMax;
};

// Initialise un controleur PID
void pidInit(PIDController& pid, float kp, float ki, float kd,
             float sampleMs, float outMin, float outMax);

// Calcule la sortie du PID pour une erreur donnee
// erreur = consigne - mesure
// Retourne la correction a appliquer
float pidCompute(PIDController& pid, float error);

// Remet les etats internes a zero (avant un nouveau mouvement)
void pidReset(PIDController& pid);

// Met a jour les gains (depuis l'IHM, sans reset des etats)
void pidSetGains(PIDController& pid, float kp, float ki, float kd);
```

- [ ] **Step 2: Create pid.cpp**

```cpp
// src/pid.cpp
#include "pid.h"

void pidInit(PIDController& pid, float kp, float ki, float kd,
             float sampleMs, float outMin, float outMax) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.ts = sampleMs / 1000.0f; // Convertir ms -> secondes
    pid.prevError = 0.0f;
    pid.integral = 0.0f;
    pid.outMin = outMin;
    pid.outMax = outMax;
}

float pidCompute(PIDController& pid, float error) {
    // --- Correcteur discret en Z ---
    // H(z) = Kp + Ki*Ts/(1 - z^-1) + Kd*(1 - z^-1)/Ts
    //
    // Terme proportionnel : P = Kp * e[n]
    float P = pid.kp * error;

    // Terme integral : I += Ki * Ts * e[n]  (approximation rectangulaire)
    pid.integral += pid.ki * pid.ts * error;

    // Anti-windup : clamper l'integral
    if (pid.integral > pid.outMax) pid.integral = pid.outMax;
    if (pid.integral < pid.outMin) pid.integral = pid.outMin;

    // Terme derive : D = Kd * (e[n] - e[n-1]) / Ts
    float D = pid.kd * (error - pid.prevError) / pid.ts;

    // Sauvegarder l'erreur pour le prochain cycle
    pid.prevError = error;

    // Sortie = P + I + D
    float output = P + pid.integral + D;

    // Clamper la sortie
    if (output > pid.outMax) output = pid.outMax;
    if (output < pid.outMin) output = pid.outMin;

    return output;
}

void pidReset(PIDController& pid) {
    pid.prevError = 0.0f;
    pid.integral = 0.0f;
}

void pidSetGains(PIDController& pid, float kp, float ki, float kd) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
}
```

- [ ] **Step 3: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 4: Commit**

```bash
git add src/pid.h src/pid.cpp
git commit -m "feat: discrete PID controller (Z-transform) with anti-windup"
```

---

## Task 9: Web UI — Server + Dashboard + Sensors API

**Files:**
- Create: `src/web_ui.h`
- Create: `src/web_ui.cpp`
- Modify: `src/main.cpp` — ajouter init WiFi + web server

Ce task est le plus gros : il contient tout le HTML/CSS/JS embarque en PROGMEM.

- [ ] **Step 1: Create web_ui.h**

```cpp
// src/web_ui.h
// Serveur web asynchrone embarque (ESPAsyncWebServer)
// - Access Point WiFi
// - Page HTML/CSS/JS en PROGMEM
// - API REST JSON pour capteurs, PID, carte, controle
#pragma once

#include <Arduino.h>

// Demarre le WiFi AP et le serveur web
void webUiInit();

// Flag d'arret d'urgence (set par le handler POST /stop)
// Le main loop doit verifier ce flag
extern volatile bool webEmergencyStop;

// Flags de commande (set par les handlers POST)
extern volatile bool webStartRun1;
extern volatile bool webStartRun2;
extern volatile bool webCalibrateFlag;
```

- [ ] **Step 2: Create web_ui.cpp — serveur + HTML PROGMEM**

```cpp
// src/web_ui.cpp
#include "web_ui.h"
#include "config.h"
#include "sensors.h"
#include "imu.h"
#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "maze.h"
#include "mcp_io.h"
#include "diagnostic.h"

#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Serveur sur le port 80
static AsyncWebServer server(WIFI_PORT);

// Flags de commande (volatiles car modifies dans les handlers)
volatile bool webEmergencyStop = false;
volatile bool webStartRun1 = false;
volatile bool webStartRun2 = false;
volatile bool webCalibrateFlag = false;

// PID externes (definis dans main.cpp ou navigation.cpp)
extern PIDController pidMotorL;
extern PIDController pidMotorR;
extern PIDController pidLateral;
extern RobotState currentState;

// =====================================================================
// Page HTML embarquee en PROGMEM
// =====================================================================

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MAZEBOT T17</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0a0a0a;color:#e0e0e0;min-height:100vh;padding-bottom:80px}
:root{--accent:#00ff88;--bg-card:#141414;--bg-card-hover:#1a1a1a;--border:#222;--red:#ff3b3b;--yellow:#ffd93d;--green:#00ff88}

/* Header */
.header{background:#111;border-bottom:1px solid var(--border);padding:16px 20px;display:flex;align-items:center;justify-content:space-between}
.header h1{font-size:1.3em;font-weight:700;letter-spacing:1px;color:#fff}
.header h1 span{color:var(--accent)}
.badge{display:inline-flex;align-items:center;gap:6px;padding:4px 12px;border-radius:20px;font-size:0.75em;font-weight:600;text-transform:uppercase;border:1px solid var(--border)}
.badge .dot{width:8px;height:8px;border-radius:50%;animation:pulse 2s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.badge-idle .dot{background:var(--green)}
.badge-run1 .dot{background:var(--yellow)}
.badge-run2 .dot{background:var(--green)}
.badge-emergency .dot{background:var(--red)}

/* Layout */
.panels{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:16px;padding:16px;max-width:1200px;margin:auto}
.card{background:var(--bg-card);border:1px solid var(--border);border-radius:12px;padding:20px;transition:border-color .2s}
.card:hover{border-color:#333}
.card-title{font-size:.8em;text-transform:uppercase;color:#666;letter-spacing:1px;margin-bottom:12px;display:flex;align-items:center;gap:8px}
.card-title svg{width:16px;height:16px;fill:var(--accent)}

/* Sensor values */
.sensor-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.sensor-item{text-align:center;padding:10px;background:#111;border-radius:8px}
.sensor-label{font-size:.7em;color:#888;text-transform:uppercase;margin-bottom:4px}
.sensor-val{font-size:1.6em;font-weight:700;color:var(--accent);font-family:'Courier New',monospace}
.sensor-val .unit{font-size:.5em;color:#555;margin-left:2px}

/* Robot top view */
.robot-view{position:relative;width:160px;height:180px;margin:10px auto;border:1px dashed #333;border-radius:8px}
.rv-label{position:absolute;font-size:.7em;color:var(--accent);font-family:monospace;font-weight:700}
.rv-fl{top:8px;left:20px}.rv-fr{top:8px;right:20px}
.rv-sl{top:50%;left:4px;transform:translateY(-50%)}.rv-sr{top:50%;right:4px;transform:translateY(-50%)}
.rv-bot{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);width:40px;height:50px;border:2px solid var(--accent);border-radius:4px}
.rv-bot::after{content:'';position:absolute;top:-8px;left:50%;transform:translateX(-50%);border-left:6px solid transparent;border-right:6px solid transparent;border-bottom:8px solid var(--accent)}

/* PID sliders */
.pid-group{margin-bottom:16px}
.pid-row{display:flex;align-items:center;gap:10px;margin:6px 0}
.pid-row label{width:30px;font-size:.8em;font-weight:600;color:#aaa}
.pid-row input[type=range]{flex:1;accent-color:var(--accent)}
.pid-row .pid-val{width:60px;text-align:right;font-family:monospace;font-size:.85em;color:var(--accent)}
.pid-section-title{font-size:.75em;color:#888;margin:12px 0 4px;text-transform:uppercase}
.btn{padding:8px 20px;border:none;border-radius:8px;font-weight:600;cursor:pointer;font-size:.85em;transition:all .2s}
.btn-accent{background:var(--accent);color:#000}.btn-accent:hover{background:#00cc6e}
.btn-sm{padding:5px 12px;font-size:.75em}

/* PID guide */
.guide-step{background:#111;border-radius:8px;padding:12px;margin:8px 0;border-left:3px solid var(--accent)}
.guide-step.active{border-left-color:var(--yellow)}
.guide-step h4{font-size:.8em;color:var(--accent);margin-bottom:4px}
.guide-step p{font-size:.75em;color:#888}

/* Canvas graph */
canvas{width:100%;height:120px;background:#111;border-radius:8px;margin-top:10px}

/* Maze */
.maze-container{display:flex;flex-direction:column;align-items:center}
.maze-info{display:flex;gap:16px;margin-top:12px;font-size:.75em;color:#888}

/* Kalman */
.kalman-display{margin-top:12px;padding:10px;background:#111;border-radius:8px}
.kalman-row{display:flex;justify-content:space-between;font-size:.75em;margin:2px 0}
.kalman-raw{color:#ff6b6b}
.kalman-filtered{color:var(--accent)}

/* Bottom bar */
.bottom-bar{position:fixed;bottom:0;left:0;right:0;background:#111;border-top:1px solid var(--border);padding:12px 20px;display:flex;justify-content:center;gap:12px;z-index:100}
.btn-start1{background:var(--yellow);color:#000}
.btn-start2{background:var(--green);color:#000}
.btn-stop{background:var(--red);color:#fff;min-width:120px;font-size:1em;font-weight:700}
.btn-stop:hover{background:#cc0000}
</style>
</head>
<body>

<div class="header">
  <h1>MAZE<span>BOT</span> T17</h1>
  <div id="badge" class="badge badge-idle"><span class="dot"></span><span id="state-text">IDLE</span></div>
</div>

<div class="panels">
  <!-- Sensors Card -->
  <div class="card">
    <div class="card-title"><svg viewBox="0 0 24 24"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/></svg>CAPTEURS LIVE</div>
    <div class="robot-view">
      <div class="rv-label rv-fl" id="d-fl">--</div>
      <div class="rv-label rv-fr" id="d-fr">--</div>
      <div class="rv-label rv-sl" id="d-sl">--</div>
      <div class="rv-label rv-sr" id="d-sr">--</div>
      <div class="rv-bot"></div>
    </div>
    <div class="sensor-grid">
      <div class="sensor-item"><div class="sensor-label">YAW</div><div class="sensor-val" id="v-yaw">--<span class="unit">deg</span></div></div>
      <div class="sensor-item"><div class="sensor-label">LIGNE</div><div class="sensor-val" id="v-line">--</div></div>
      <div class="sensor-item"><div class="sensor-label">ENC L</div><div class="sensor-val" id="v-encl">--<span class="unit">t/s</span></div></div>
      <div class="sensor-item"><div class="sensor-label">ENC R</div><div class="sensor-val" id="v-encr">--<span class="unit">t/s</span></div></div>
    </div>
    <div class="kalman-display">
      <div style="font-size:.7em;color:#666;margin-bottom:4px">KALMAN (brut / filtre)</div>
      <div class="kalman-row"><span>FL:</span><span class="kalman-raw" id="k-fl-r">--</span><span class="kalman-filtered" id="k-fl-f">--</span></div>
      <div class="kalman-row"><span>FR:</span><span class="kalman-raw" id="k-fr-r">--</span><span class="kalman-filtered" id="k-fr-f">--</span></div>
      <div class="kalman-row"><span>SL:</span><span class="kalman-raw" id="k-sl-r">--</span><span class="kalman-filtered" id="k-sl-f">--</span></div>
      <div class="kalman-row"><span>SR:</span><span class="kalman-raw" id="k-sr-r">--</span><span class="kalman-filtered" id="k-sr-f">--</span></div>
    </div>
  </div>

  <!-- PID Tuning Card -->
  <div class="card">
    <div class="card-title"><svg viewBox="0 0 24 24"><path d="M3 17v2h6v-2H3zM3 5v2h10V5H3zm10 16v-2h8v-2h-8v-2h-2v6h2zM7 9v2H3v2h4v2h2V9H7zm14 4v-2H11v2h10zm-6-4h2V7h4V5h-4V3h-2v6z"/></svg>TUNING PID</div>

    <div class="pid-section-title">PID Moteur (par roue)</div>
    <div class="pid-group">
      <div class="pid-row"><label>Kp</label><input type="range" id="s-mkp" min="0" max="5" step="0.1" value="1.2"><span class="pid-val" id="v-mkp">1.2</span></div>
      <div class="pid-row"><label>Ki</label><input type="range" id="s-mki" min="0" max="1" step="0.01" value="0.05"><span class="pid-val" id="v-mki">0.05</span></div>
      <div class="pid-row"><label>Kd</label><input type="range" id="s-mkd" min="0" max="3" step="0.1" value="0.8"><span class="pid-val" id="v-mkd">0.8</span></div>
    </div>

    <div class="pid-section-title">PID Lateral (centrage)</div>
    <div class="pid-group">
      <div class="pid-row"><label>Kp</label><input type="range" id="s-lkp" min="0" max="5" step="0.1" value="0.8"><span class="pid-val" id="v-lkp">0.8</span></div>
      <div class="pid-row"><label>Ki</label><input type="range" id="s-lki" min="0" max="1" step="0.01" value="0.02"><span class="pid-val" id="v-lki">0.02</span></div>
      <div class="pid-row"><label>Kd</label><input type="range" id="s-lkd" min="0" max="3" step="0.1" value="0.5"><span class="pid-val" id="v-lkd">0.5</span></div>
    </div>

    <div class="pid-section-title">Kalman ToF</div>
    <div class="pid-group">
      <div class="pid-row"><label>Q</label><input type="range" id="s-kq" min="0.001" max="1" step="0.001" value="0.01"><span class="pid-val" id="v-kq">0.01</span></div>
      <div class="pid-row"><label>R</label><input type="range" id="s-kr" min="0.5" max="50" step="0.5" value="5"><span class="pid-val" id="v-kr">5.0</span></div>
    </div>

    <button class="btn btn-accent" onclick="applyPID()">Appliquer</button>
    <button class="btn btn-sm" style="margin-left:8px;background:#333;color:#aaa" onclick="testPID()">Test PID</button>

    <canvas id="pid-graph"></canvas>

    <!-- PID Guide -->
    <div style="margin-top:16px">
      <div class="pid-section-title">Assistant de Reglage</div>
      <div class="guide-step active" id="gs1"><h4>Etape 1 — Regler Kp</h4><p>Ki=0, Kd=0. Augmenter Kp jusqu'a oscillation.</p></div>
      <div class="guide-step" id="gs2"><h4>Etape 2 — Regler Ki</h4><p>Garder Kp. Augmenter Ki pour eliminer l'erreur statique.</p></div>
      <div class="guide-step" id="gs3"><h4>Etape 3 — Regler Kd</h4><p>Ajouter Kd pour amortir les oscillations.</p></div>
      <div class="guide-step" id="gs4"><h4>Etape 4 — Valider</h4><p>Lancer un test : ligne droite 1m + retour.</p></div>
    </div>
  </div>

  <!-- Maze Card -->
  <div class="card" style="grid-column:span 2">
    <div class="card-title"><svg viewBox="0 0 24 24"><path d="M3 3v18h18V3H3zm16 16H5V5h14v14zM7 7h4v4H7V7zm0 6h4v4H7v-4zm6-6h4v4h-4V7zm0 6h4v4h-4v-4z"/></svg>CARTE LABYRINTHE</div>
    <div class="maze-container">
      <canvas id="maze-canvas" width="400" height="400"></canvas>
      <div class="maze-info">
        <span>Position: <b id="m-pos">(0,0)</b></span>
        <span>Direction: <b id="m-dir">N</b></span>
        <span>Cases visitees: <b id="m-visited">0</b>/25</span>
      </div>
    </div>
  </div>
</div>

<!-- Bottom bar -->
<div class="bottom-bar">
  <button class="btn btn-start1" onclick="startRun(1)">START RUN 1</button>
  <button class="btn btn-start2" onclick="startRun(2)">START RUN 2</button>
  <button class="btn btn-stop" onclick="emergencyStop()">STOP</button>
</div>

<script>
// --- Polling des capteurs ---
function fetchSensors(){
  fetch('/sensors').then(r=>r.json()).then(d=>{
    document.getElementById('d-fl').textContent=d.fl+'mm';
    document.getElementById('d-fr').textContent=d.fr+'mm';
    document.getElementById('d-sl').textContent=d.sl+'mm';
    document.getElementById('d-sr').textContent=d.sr+'mm';
    document.getElementById('v-yaw').innerHTML=d.yaw.toFixed(1)+'<span class="unit">deg</span>';
    document.getElementById('v-line').textContent=d.line?'LIGNE':'---';
    document.getElementById('v-line').style.color=d.line?'var(--red)':'var(--accent)';
    document.getElementById('v-encl').innerHTML=d.enc_l.toFixed(0)+'<span class="unit">t/s</span>';
    document.getElementById('v-encr').innerHTML=d.enc_r.toFixed(0)+'<span class="unit">t/s</span>';
    // Kalman display
    document.getElementById('k-fl-r').textContent=d.raw_fl;
    document.getElementById('k-fl-f').textContent=d.fl;
    document.getElementById('k-fr-r').textContent=d.raw_fr;
    document.getElementById('k-fr-f').textContent=d.fr;
    document.getElementById('k-sl-r').textContent=d.raw_sl;
    document.getElementById('k-sl-f').textContent=d.sl;
    document.getElementById('k-sr-r').textContent=d.raw_sr;
    document.getElementById('k-sr-f').textContent=d.sr;
  }).catch(()=>{});
}

// --- Polling etat ---
function fetchState(){
  fetch('/state').then(r=>r.json()).then(d=>{
    const b=document.getElementById('badge');
    const t=document.getElementById('state-text');
    const states={0:'IDLE',1:'DIAGNOSTIC',2:'CALIBRATION',3:'RUN 1',4:'MAZE OK',5:'RUN 2',6:'FINI',7:'URGENCE'};
    const cls={0:'idle',1:'idle',2:'idle',3:'run1',4:'idle',5:'run2',6:'idle',7:'emergency'};
    t.textContent=states[d.state]||'?';
    b.className='badge badge-'+(cls[d.state]||'idle');
  }).catch(()=>{});
}

// --- Polling carte ---
function fetchMaze(){
  fetch('/maze').then(r=>r.json()).then(d=>{
    drawMaze(d);
    document.getElementById('m-pos').textContent='('+d.robot.row+','+d.robot.col+')';
    const dirs=['N','E','S','W'];
    document.getElementById('m-dir').textContent=dirs[d.robot.dir]||'?';
    let v=0;
    for(let r=0;r<5;r++)for(let c=0;c<5;c++)if(d.cells[r][c].v>0)v++;
    document.getElementById('m-visited').textContent=v+'/25';
  }).catch(()=>{});
}

// --- Dessin du labyrinthe ---
function drawMaze(data){
  const cv=document.getElementById('maze-canvas');
  const ctx=cv.getContext('2d');
  const s=cv.width/5;
  ctx.clearRect(0,0,cv.width,cv.height);

  for(let r=0;r<5;r++){
    for(let c=0;c<5;c++){
      const x=c*s, y=r*s;
      const cell=data.cells[r][c];

      // Fond de la case selon visites
      if(cell.v>0){
        const g=Math.min(cell.v*40,120);
        ctx.fillStyle='rgba(0,'+g+',0,0.3)';
        ctx.fillRect(x+2,y+2,s-4,s-4);
        // Nombre de visites
        ctx.fillStyle='rgba(255,255,255,0.4)';
        ctx.font='12px monospace';
        ctx.textAlign='center';
        ctx.fillText(cell.v,x+s/2,y+s/2+4);
      }

      // Case arrivee
      if(r==data.endRow&&c==data.endCol){
        ctx.fillStyle='rgba(255,59,59,0.2)';
        ctx.fillRect(x+2,y+2,s-4,s-4);
      }

      // Murs
      ctx.strokeStyle='#fff';
      ctx.lineWidth=2;
      const w=cell.w;
      if(w&1){ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+s,y);ctx.stroke();}        // N
      if(w&2){ctx.beginPath();ctx.moveTo(x+s,y);ctx.lineTo(x+s,y+s);ctx.stroke();}    // E
      if(w&4){ctx.beginPath();ctx.moveTo(x,y+s);ctx.lineTo(x+s,y+s);ctx.stroke();}    // S
      if(w&8){ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x,y+s);ctx.stroke();}        // W

      // Murs inconnus (gris pointille)
      ctx.strokeStyle='#333';
      ctx.lineWidth=1;
      ctx.setLineDash([4,4]);
      if(!(w&1)&&r>0){ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+s,y);ctx.stroke();}
      if(!(w&2)&&c<4){ctx.beginPath();ctx.moveTo(x+s,y);ctx.lineTo(x+s,y+s);ctx.stroke();}
      if(!(w&4)&&r<4){ctx.beginPath();ctx.moveTo(x,y+s);ctx.lineTo(x+s,y+s);ctx.stroke();}
      if(!(w&8)&&c>0){ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x,y+s);ctx.stroke();}
      ctx.setLineDash([]);
    }
  }

  // Chemin BFS (si disponible)
  if(data.path&&data.path.length>1){
    ctx.strokeStyle='rgba(255,59,59,0.8)';
    ctx.lineWidth=3;
    ctx.beginPath();
    ctx.moveTo(data.path[0].c*s+s/2,data.path[0].r*s+s/2);
    for(let i=1;i<data.path.length;i++){
      ctx.lineTo(data.path[i].c*s+s/2,data.path[i].r*s+s/2);
    }
    ctx.stroke();
  }

  // Robot (triangle)
  const rx=data.robot.col*s+s/2, ry=data.robot.row*s+s/2;
  ctx.save();
  ctx.translate(rx,ry);
  ctx.rotate(data.robot.dir*Math.PI/2);
  ctx.fillStyle='var(--accent)';
  ctx.beginPath();
  ctx.moveTo(0,-14);
  ctx.lineTo(-10,10);
  ctx.lineTo(10,10);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

// --- PID graph ---
let pidData=[];
const MAX_PID_PTS=100;
function drawPIDGraph(setpoint,measured){
  pidData.push({s:setpoint,m:measured});
  if(pidData.length>MAX_PID_PTS)pidData.shift();
  const cv=document.getElementById('pid-graph');
  const ctx=cv.getContext('2d');
  const w=cv.width,h=cv.height;
  ctx.clearRect(0,0,w,h);
  if(pidData.length<2)return;
  const max=Math.max(...pidData.map(d=>Math.max(d.s,d.m)),1);
  const dx=w/MAX_PID_PTS;
  // Setpoint line
  ctx.strokeStyle='#555';ctx.lineWidth=1;ctx.beginPath();
  pidData.forEach((d,i)=>{const x=i*dx,y=h-d.s/max*h;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});
  ctx.stroke();
  // Measured line
  ctx.strokeStyle='#00ff88';ctx.lineWidth=2;ctx.beginPath();
  pidData.forEach((d,i)=>{const x=i*dx,y=h-d.m/max*h;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});
  ctx.stroke();
}

// --- Slider updates ---
document.querySelectorAll('input[type=range]').forEach(s=>{
  const vId='v-'+s.id.substring(2);
  s.addEventListener('input',()=>{document.getElementById(vId).textContent=s.value;});
});

// --- Actions ---
function applyPID(){
  const body={
    mkp:+document.getElementById('s-mkp').value,
    mki:+document.getElementById('s-mki').value,
    mkd:+document.getElementById('s-mkd').value,
    lkp:+document.getElementById('s-lkp').value,
    lki:+document.getElementById('s-lki').value,
    lkd:+document.getElementById('s-lkd').value,
    kq:+document.getElementById('s-kq').value,
    kr:+document.getElementById('s-kr').value
  };
  fetch('/pid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
}
function testPID(){fetch('/calibrate/pid',{method:'POST'});}
function startRun(n){fetch('/start'+n,{method:'POST'});}
function emergencyStop(){fetch('/stop',{method:'POST'});}

// --- Polling intervals ---
setInterval(fetchSensors,200);
setInterval(fetchState,1000);
setInterval(fetchMaze,300);
fetchSensors();fetchState();fetchMaze();
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// Handlers API
// =====================================================================

static void setupRoutes() {
    // Page principale
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", INDEX_HTML);
    });

    // Etat du robot
    server.on("/state", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["state"] = (int)currentState;
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // Capteurs (bruts + filtres + encodeurs + IMU + ligne)
    server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["fl"]     = (int)sensorFilteredFL();
        doc["fr"]     = (int)sensorFilteredFR();
        doc["sl"]     = (int)sensorFilteredSL();
        doc["sr"]     = (int)sensorFilteredSR();
        doc["raw_fl"] = sensorRawFL();
        doc["raw_fr"] = sensorRawFR();
        doc["raw_sl"] = sensorRawSL();
        doc["raw_sr"] = sensorRawSR();
        doc["yaw"]    = imuGetYaw();
        doc["enc_l"]  = encoderSpeedLeft();
        doc["enc_r"]  = encoderSpeedRight();
        doc["line"]   = mcpReadLineSensor();
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // Carte du labyrinthe
    server.on("/maze", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        JsonArray cells = doc["cells"].to<JsonArray>();
        for (int r = 0; r < MAZE_SIZE; r++) {
            JsonArray row = cells.add<JsonArray>();
            for (int c = 0; c < MAZE_SIZE; c++) {
                JsonObject cell = row.add<JsonObject>();
                cell["w"] = mazeGetWalls(r, c);
                cell["v"] = mazeGetVisited(r, c);
            }
        }
        JsonObject robot = doc["robot"].to<JsonObject>();
        robot["row"] = mazeGetRobotRow();
        robot["col"] = mazeGetRobotCol();
        robot["dir"] = mazeGetRobotDir();
        doc["endRow"] = END_ROW;
        doc["endCol"] = END_COL;

        // Chemin BFS (si calcule)
        // Sera rempli par bfs.cpp quand le chemin est disponible

        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // Lecture PID actuel
    server.on("/pid", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["mkp"] = pidMotorL.kp;
        doc["mki"] = pidMotorL.ki;
        doc["mkd"] = pidMotorL.kd;
        doc["lkp"] = pidLateral.kp;
        doc["lki"] = pidLateral.ki;
        doc["lkd"] = pidLateral.kd;
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // Mise a jour PID
    server.on("/pid", HTTP_POST, [](AsyncWebServerRequest* req) {},
        NULL,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            deserializeJson(doc, data, len);

            if (doc.containsKey("mkp")) {
                float mkp = doc["mkp"];
                float mki = doc["mki"];
                float mkd = doc["mkd"];
                pidSetGains(pidMotorL, mkp, mki, mkd);
                pidSetGains(pidMotorR, mkp, mki, mkd);
            }
            if (doc.containsKey("lkp")) {
                float lkp = doc["lkp"];
                float lki = doc["lki"];
                float lkd = doc["lkd"];
                pidSetGains(pidLateral, lkp, lki, lkd);
            }
            if (doc.containsKey("kq")) {
                sensorsSetKalmanQ(doc["kq"]);
            }
            if (doc.containsKey("kr")) {
                sensorsSetKalmanR(doc["kr"]);
            }
            req->send(200, "application/json", "{\"ok\":true}");
        });

    // Commandes
    server.on("/start1", HTTP_POST, [](AsyncWebServerRequest* req) {
        webStartRun1 = true;
        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/start2", HTTP_POST, [](AsyncWebServerRequest* req) {
        webStartRun2 = true;
        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/stop", HTTP_POST, [](AsyncWebServerRequest* req) {
        webEmergencyStop = true;
        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/calibrate/pid", HTTP_POST, [](AsyncWebServerRequest* req) {
        webCalibrateFlag = true;
        req->send(200, "application/json", "{\"ok\":true}");
    });
}

void webUiInit() {
    // Demarrer le WiFi en mode Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("[WEB] WiFi AP demarre");
    Serial.printf("[WEB] SSID: %s\n", WIFI_SSID);
    Serial.printf("[WEB] IP: %s\n", WiFi.softAPIP().toString().c_str());

    // Configurer les routes
    setupRoutes();

    // Demarrer le serveur
    server.begin();
    Serial.println("[WEB] Serveur web demarre sur port 80");
}
```

- [ ] **Step 3: Update main.cpp to include web UI and PID globals**

```cpp
// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "mcp_io.h"
#include "leds.h"
#include "sensors.h"
#include "imu.h"
#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "diagnostic.h"
#include "maze.h"
#include "web_ui.h"

// Etat global du robot
RobotState currentState = STATE_DIAGNOSTIC;

// Controleurs PID globaux (accessibles par web_ui.cpp via extern)
PIDController pidMotorL;
PIDController pidMotorR;
PIDController pidLateral;

static bool diagDone = false;
static unsigned long lastSensorMs = 0;
static unsigned long lastPidMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== MAZEBOT T17 — Equipe 17 ===");
    Serial.println("Demarrage...\n");

    // Initialiser les PID avec les valeurs par defaut
    pidInit(pidMotorL, PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD,
            PID_SAMPLE_MS, -255, 255);
    pidInit(pidMotorR, PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD,
            PID_SAMPLE_MS, -255, 255);
    pidInit(pidLateral, PID_LAT_KP, PID_LAT_KI, PID_LAT_KD,
            PID_SAMPLE_MS, -100, 100);
}

void loop() {
    unsigned long now = millis();

    // Toujours mettre a jour les LEDs
    ledsUpdate();

    // Verifier l'arret d'urgence depuis l'IHM
    if (webEmergencyStop) {
        webEmergencyStop = false;
        motorEmergencyStop();
        currentState = STATE_EMERGENCY;
        ledsSetEmergency();
        Serial.println("[MAIN] ARRET D'URGENCE !");
    }

    switch (currentState) {
        case STATE_DIAGNOSTIC: {
            if (!diagDone) {
                ledsSetDiagnostic();
                DiagResult result = diagnosticRun();

                // Demarrer le serveur web (meme si diagnostic echoue)
                mazeInit();
                webUiInit();

                if (result.allCriticalOk()) {
                    currentState = STATE_IDLE;
                    ledsSetIdle();
                    Serial.println("[MAIN] Diagnostic OK -> IDLE");
                } else {
                    currentState = STATE_EMERGENCY;
                    ledsSetEmergency();
                    Serial.println("[MAIN] Diagnostic FAIL -> EMERGENCY");
                }
                diagDone = true;
            }
            break;
        }

        case STATE_IDLE: {
            // Lecture capteurs periodique
            if (now - lastSensorMs >= SENSOR_READ_MS) {
                lastSensorMs = now;
                sensorsRead();
                imuUpdate();
                encodersComputeSpeed();
            }

            // Verifier les commandes IHM
            if (webStartRun1) {
                webStartRun1 = false;
                currentState = STATE_RUN1_TREMAUX;
                ledsSetRun1();
                encoderResetBoth();
                pidReset(pidMotorL);
                pidReset(pidMotorR);
                pidReset(pidLateral);
                Serial.println("[MAIN] Lancement Run 1 (Tremaux)");
            }
            if (webStartRun2) {
                webStartRun2 = false;
                currentState = STATE_RUN2_BFS;
                ledsSetRun2();
                encoderResetBoth();
                pidReset(pidMotorL);
                pidReset(pidMotorR);
                pidReset(pidLateral);
                Serial.println("[MAIN] Lancement Run 2 (BFS)");
            }
            break;
        }

        case STATE_RUN1_TREMAUX: {
            // Sera implemente dans Task 13
            // Pour l'instant, lecture capteurs
            if (now - lastSensorMs >= SENSOR_READ_MS) {
                lastSensorMs = now;
                sensorsRead();
                imuUpdate();
                encodersComputeSpeed();
            }
            break;
        }

        case STATE_MAZE_COMPLETE: {
            if (webStartRun2) {
                webStartRun2 = false;
                currentState = STATE_RUN2_BFS;
                ledsSetRun2();
                Serial.println("[MAIN] Lancement Run 2 (BFS)");
            }
            break;
        }

        case STATE_RUN2_BFS: {
            // Sera implemente dans Task 15
            if (now - lastSensorMs >= SENSOR_READ_MS) {
                lastSensorMs = now;
                sensorsRead();
                imuUpdate();
                encodersComputeSpeed();
            }
            break;
        }

        case STATE_FINISHED: {
            motorBrake();
            break;
        }

        case STATE_EMERGENCY: {
            motorEmergencyStop();
            // Permettre de revenir a IDLE depuis l'IHM
            if (webStartRun1 || webStartRun2) {
                webStartRun1 = false;
                webStartRun2 = false;
                currentState = STATE_IDLE;
                ledsSetIdle();
                Serial.println("[MAIN] Sortie EMERGENCY -> IDLE");
            }
            break;
        }

        default:
            break;
    }
}
```

- [ ] **Step 4: Compile**

Run: `pio run`
Expected: erreur car maze.h n'existe pas encore. On le cree dans le step suivant.

- [ ] **Step 5: Commit (sera possible apres Task 10 quand maze.h existe)**

Note: ce commit sera fait apres Task 10 car web_ui.cpp depend de maze.h

---

## Task 10: Maze Data Structure

**Files:**
- Create: `src/maze.h`
- Create: `src/maze.cpp`

- [ ] **Step 1: Create maze.h**

```cpp
// src/maze.h
// Structure de donnees pour le labyrinthe 5x5
// - Grille de cellules avec murs en bitmask
// - Position et orientation du robot
// - Marquage Tremaux par direction
#pragma once

#include <Arduino.h>
#include "config.h"

// Structure d'une cellule du labyrinthe
struct Cell {
    uint8_t walls;      // Bitmask murs : WALL_N | WALL_E | WALL_S | WALL_W
    uint8_t visited;    // Nombre de fois que le robot est passe
    uint8_t marks[4];   // Marquage Tremaux par direction (N/E/S/W) : 0, 1 ou 2
};

// Initialise la grille : murs exterieurs pleins, interieur vide
void mazeInit();

// Acces a une cellule
Cell& mazeGetCell(uint8_t row, uint8_t col);

// Getters pour les murs et visites (utilises par l'API web)
uint8_t mazeGetWalls(uint8_t row, uint8_t col);
uint8_t mazeGetVisited(uint8_t row, uint8_t col);

// Ajoute un mur a une cellule ET a la cellule voisine (symetrique)
void mazeSetWall(uint8_t row, uint8_t col, uint8_t direction);

// Marque la cellule actuelle comme visitee (+1)
void mazeMarkVisited(uint8_t row, uint8_t col);

// Position et orientation du robot
uint8_t mazeGetRobotRow();
uint8_t mazeGetRobotCol();
uint8_t mazeGetRobotDir();
void mazeSetRobotPos(uint8_t row, uint8_t col, uint8_t dir);

// Deplace le robot d'une case dans la direction actuelle
// Met a jour row/col selon la direction
void mazeAdvanceRobot();

// Tourne le robot (0=N, 1=E, 2=S, 3=W)
void mazeRotateRobot(int quarters); // +1 = droite, -1 = gauche, +2 = demi-tour

// Convertit les lectures capteurs en murs absolus (N/E/S/W)
// selon l'orientation du robot
// front/left/right = murs relatifs detectes par les capteurs
void mazeUpdateWalls(uint8_t row, uint8_t col, uint8_t robotDir,
                     bool front, bool left, bool right);

// Verifie si toutes les cases accessibles ont ete visitees
bool mazeIsFullyExplored();

// Affiche la carte en Serial (debug)
void mazePrintSerial();
```

- [ ] **Step 2: Create maze.cpp**

```cpp
// src/maze.cpp
#include "maze.h"

// Grille statique 5x5 — zero allocation dynamique
static Cell grid[MAZE_SIZE][MAZE_SIZE];

// Position et orientation du robot
static uint8_t robotRow = START_ROW;
static uint8_t robotCol = START_COL;
static uint8_t robotDir = START_DIR;

// Tables de direction pour convertir relatif -> absolu
// Index = direction du robot (0=N, 1=E, 2=S, 3=W)
// delta[dir] = {deltaRow, deltaCol}
static const int8_t deltaRow[4] = { -1,  0,  1,  0 }; // N, E, S, W
static const int8_t deltaCol[4] = {  0,  1,  0, -1 }; // N, E, S, W

// Mur oppose : N<->S, E<->W
static const uint8_t oppositeWall[4] = { WALL_S, WALL_W, WALL_N, WALL_E };
static const uint8_t wallBit[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

void mazeInit() {
    // Reinitialiser toutes les cellules
    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            grid[r][c].walls = 0;
            grid[r][c].visited = 0;
            for (int d = 0; d < 4; d++) grid[r][c].marks[d] = 0;
        }
    }

    // Murs exterieurs : tous pleins
    for (int i = 0; i < MAZE_SIZE; i++) {
        grid[0][i].walls |= WALL_N;           // Bord nord
        grid[MAZE_SIZE-1][i].walls |= WALL_S; // Bord sud
        grid[i][0].walls |= WALL_W;           // Bord ouest
        grid[i][MAZE_SIZE-1].walls |= WALL_E; // Bord est
    }

    // Position initiale du robot
    robotRow = START_ROW;
    robotCol = START_COL;
    robotDir = START_DIR;

    Serial.println("[MAZE] Grille 5x5 initialisee (murs exterieurs pleins)");
}

Cell& mazeGetCell(uint8_t row, uint8_t col) {
    return grid[row][col];
}

uint8_t mazeGetWalls(uint8_t row, uint8_t col) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return 0x0F;
    return grid[row][col].walls;
}

uint8_t mazeGetVisited(uint8_t row, uint8_t col) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return 0;
    return grid[row][col].visited;
}

void mazeSetWall(uint8_t row, uint8_t col, uint8_t direction) {
    // Ajouter le mur a cette cellule
    grid[row][col].walls |= wallBit[direction];

    // Ajouter le mur symetrique a la cellule voisine
    int8_t nr = row + deltaRow[direction];
    int8_t nc = col + deltaCol[direction];
    if (nr >= 0 && nr < MAZE_SIZE && nc >= 0 && nc < MAZE_SIZE) {
        grid[nr][nc].walls |= oppositeWall[direction];
    }
}

void mazeMarkVisited(uint8_t row, uint8_t col) {
    grid[row][col].visited++;
}

uint8_t mazeGetRobotRow() { return robotRow; }
uint8_t mazeGetRobotCol() { return robotCol; }
uint8_t mazeGetRobotDir() { return robotDir; }

void mazeSetRobotPos(uint8_t row, uint8_t col, uint8_t dir) {
    robotRow = row;
    robotCol = col;
    robotDir = dir;
}

void mazeAdvanceRobot() {
    int8_t newRow = robotRow + deltaRow[robotDir];
    int8_t newCol = robotCol + deltaCol[robotDir];
    if (newRow >= 0 && newRow < MAZE_SIZE && newCol >= 0 && newCol < MAZE_SIZE) {
        robotRow = newRow;
        robotCol = newCol;
    }
}

void mazeRotateRobot(int quarters) {
    robotDir = (robotDir + quarters + 4) % 4;
}

void mazeUpdateWalls(uint8_t row, uint8_t col, uint8_t dir,
                     bool front, bool left, bool right) {
    // Convertir directions relatives (avant/gauche/droite) en absolues (N/E/S/W)
    // Avant = direction du robot
    // Gauche = direction - 1 (mod 4)
    // Droite = direction + 1 (mod 4)

    uint8_t frontDir = dir;
    uint8_t leftDir  = (dir + 3) % 4; // -1 mod 4
    uint8_t rightDir = (dir + 1) % 4; // +1 mod 4

    if (front) mazeSetWall(row, col, frontDir);
    if (left)  mazeSetWall(row, col, leftDir);
    if (right) mazeSetWall(row, col, rightDir);

    // Le mur derriere est implicite (on vient de la)
}

bool mazeIsFullyExplored() {
    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            // Si une case n'a pas ete visitee et est accessible
            // (pas entouree de murs), l'exploration n'est pas complete
            if (grid[r][c].visited == 0) {
                // Verifier si cette case est accessible (au moins un cote sans mur)
                bool enclosed = (grid[r][c].walls == 0x0F);
                if (!enclosed) return false;
            }
        }
    }
    return true;
}

void mazePrintSerial() {
    Serial.println("\n--- CARTE LABYRINTHE ---");
    for (int r = 0; r < MAZE_SIZE; r++) {
        // Ligne du haut (murs nord)
        for (int c = 0; c < MAZE_SIZE; c++) {
            Serial.print("+");
            Serial.print((grid[r][c].walls & WALL_N) ? "---" : "   ");
        }
        Serial.println("+");

        // Ligne du milieu (murs ouest + contenu + murs est)
        for (int c = 0; c < MAZE_SIZE; c++) {
            Serial.print((grid[r][c].walls & WALL_W) ? "|" : " ");
            if (r == robotRow && c == robotCol) {
                const char* arrows[] = { " ^ ", " > ", " v ", " < " };
                Serial.print(arrows[robotDir]);
            } else if (grid[r][c].visited > 0) {
                Serial.printf(" %d ", grid[r][c].visited);
            } else {
                Serial.print("   ");
            }
        }
        Serial.println("|");
    }
    // Derniere ligne (murs sud)
    for (int c = 0; c < MAZE_SIZE; c++) {
        Serial.print("+---");
    }
    Serial.println("+\n");
}
```

- [ ] **Step 3: Compile everything**

Run: `pio run`
Expected: 0 erreurs — maintenant tous les modules de base compilent

- [ ] **Step 4: Flash and test**

Run: `pio run --target upload && pio device monitor -b 115200`
Expected: diagnostic + WiFi AP + page web accessible a 192.168.4.1

- [ ] **Step 5: Commit everything accumulated**

```bash
git add src/maze.h src/maze.cpp src/web_ui.h src/web_ui.cpp src/pid.h src/pid.cpp src/main.cpp
git commit -m "feat: web UI with live dashboard, PID tuning, maze display + maze data structure"
```

---

## Task 11: Navigation Primitives

**Files:**
- Create: `src/navigation.h`
- Create: `src/navigation.cpp`

- [ ] **Step 1: Create navigation.h**

```cpp
// src/navigation.h
// Primitives de mouvement du robot
// - advanceOneCell() : avancer de 200mm (Run 1, stop-turn-go)
// - turnLeft() / turnRight() / turnAround() : rotation sur place
// - autoAlign() : alignement sur mur frontal
// - advanceWithCurve() : virage en cloche sans arret (Run 2)
//
// Toutes les fonctions sont non-bloquantes : elles retournent un etat
// (NAV_BUSY ou NAV_DONE) et doivent etre appelees dans loop()
#pragma once

#include <Arduino.h>

enum NavState {
    NAV_IDLE,       // Pas de mouvement en cours
    NAV_BUSY,       // Mouvement en cours, rappeler dans loop()
    NAV_DONE        // Mouvement termine
};

// Initialise le module de navigation
void navigationInit();

// --- Primitives Run 1 (stop-turn-go) ---

// Commence a avancer d'une case (200mm)
// Appeler une fois, puis appeler navigationUpdate() dans loop()
void navStartAdvance();

// Commence une rotation (quarters: +1=droite, -1=gauche, +2=demi-tour)
void navStartTurn(int quarters);

// Commence l'auto-calage contre un mur frontal
void navStartAutoAlign();

// --- Primitive Run 2 (trajectoire en cloche) ---

// Commence un mouvement avec virage en cloche
// nextTurn : 0=tout droit, +1=droite, -1=gauche
void navStartAdvanceWithCurve(int nextTurn);

// --- Controle ---

// Met a jour le mouvement en cours — appeler dans loop()
// Retourne NAV_BUSY si encore en mouvement, NAV_DONE si termine
NavState navigationUpdate();

// Annule le mouvement en cours et freine
void navAbort();

// Retourne l'etat actuel
NavState navGetState();
```

- [ ] **Step 2: Create navigation.cpp**

```cpp
// src/navigation.cpp
#include "navigation.h"
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "imu.h"
#include "pid.h"
#include "mcp_io.h"

// PID externes
extern PIDController pidMotorL;
extern PIDController pidMotorR;
extern PIDController pidLateral;

// Etat interne de la navigation
static NavState state = NAV_IDLE;

// Type de mouvement en cours
enum NavAction {
    ACT_NONE,
    ACT_ADVANCE,
    ACT_TURN,
    ACT_AUTO_ALIGN,
    ACT_CURVE
};

static NavAction currentAction = ACT_NONE;

// Parametres du mouvement en cours
static int targetTicks = 0;       // Ticks a parcourir (advance)
static int targetQuarters = 0;    // Quarts de tour (turn)
static int curveNextTurn = 0;     // Prochain virage (curve)
static unsigned long actionStartMs = 0;
static bool turnSettled = false;   // Flag stabilisation avant rotation

// Vitesse cible actuelle (PWM base)
static int basePWM = PWM_RUN1;

void navigationInit() {
    state = NAV_IDLE;
    currentAction = ACT_NONE;
}

// ----- ADVANCE ONE CELL -----

void navStartAdvance() {
    encoderResetBoth();
    pidReset(pidMotorL);
    pidReset(pidMotorR);
    pidReset(pidLateral);
    imuResetYaw();

    targetTicks = TICKS_PER_CELL;
    state = NAV_BUSY;
    currentAction = ACT_ADVANCE;
    actionStartMs = millis();
}

static NavState updateAdvance() {
    long ticksL = encoderGetLeft();
    long ticksR = encoderGetRight();
    long avgTicks = (ticksL + ticksR) / 2;

    // Condition d'arret : distance atteinte
    if (avgTicks >= targetTicks) {
        motorBrake();
        state = NAV_DONE;
        return NAV_DONE;
    }

    // Arret d'urgence si mur trop proche devant
    if (sensorFilteredFL() < TOF_STOP_FRONT_MM && sensorFilteredFR() < TOF_STOP_FRONT_MM) {
        motorBrake();
        state = NAV_DONE;
        return NAV_DONE;
    }

    // PID lateral : correction centrage avec ToF 45 deg
    float errorLat = sensorFilteredSL() - sensorFilteredSR();
    float correction = pidCompute(pidLateral, errorLat);

    // PID moteur : appliquer la correction laterale
    int pwmL = basePWM - (int)correction;
    int pwmR = basePWM + (int)correction;

    // Clamper
    pwmL = constrain(pwmL, 0, 255);
    pwmR = constrain(pwmR, 0, 255);

    motorSetBoth(pwmL, pwmR);

    return NAV_BUSY;
}

// ----- TURN -----

void navStartTurn(int quarters) {
    motorBrake();
    targetQuarters = quarters;
    turnSettled = false;
    actionStartMs = millis();
    encoderResetBoth();
    imuResetYaw();

    state = NAV_BUSY;
    currentAction = ACT_TURN;
}

static NavState updateTurn() {
    // Phase 1 : stabilisation avant rotation
    if (!turnSettled) {
        if (millis() - actionStartMs >= TURN_SETTLE_MS) {
            turnSettled = true;
            encoderResetBoth();
            imuResetYaw();
        }
        return NAV_BUSY;
    }

    // Calculer les ticks necessaires pour la rotation
    int targetTurnTicks = abs(targetQuarters) * TICKS_PER_90DEG;
    long ticksL = abs(encoderGetLeft());
    long ticksR = abs(encoderGetRight());
    long avgTicks = (ticksL + ticksR) / 2;

    // Condition d'arret primaire : encodeurs
    if (avgTicks >= targetTurnTicks) {
        motorBrake();

        // Verification secondaire : gyroscope
        float expectedAngle = targetQuarters * 90.0f;
        float actualAngle = imuGetYaw();
        float angleError = fabs(actualAngle - expectedAngle);

        if (angleError > 5.0f) {
            Serial.printf("[NAV] WARNING: ecart gyro %.1f deg\n", angleError);
        }

        state = NAV_DONE;
        return NAV_DONE;
    }

    // Rotation sur place : une roue avant, l'autre arriere
    if (targetQuarters > 0) {
        // Tourner a droite : gauche avance, droite recule
        motorSetBoth(PWM_TURN, -PWM_TURN);
    } else {
        // Tourner a gauche : gauche recule, droite avance
        motorSetBoth(-PWM_TURN, PWM_TURN);
    }

    return NAV_BUSY;
}

// ----- AUTO ALIGN -----

void navStartAutoAlign() {
    state = NAV_BUSY;
    currentAction = ACT_AUTO_ALIGN;
    actionStartMs = millis();
}

static NavState updateAutoAlign() {
    float fl = sensorFilteredFL();
    float fr = sensorFilteredFR();

    // Pas de mur devant ? Rien a faire
    if (fl > TOF_WALL_FRONT_MM || fr > TOF_WALL_FRONT_MM) {
        motorBrake();
        state = NAV_DONE;
        return NAV_DONE;
    }

    // Etape 1 : aligner l'angle (FL doit etre ~ FR)
    float angleError = fl - fr;
    if (fabs(angleError) > TOF_ALIGN_TOLERANCE_MM) {
        // Micro-rotation pour corriger
        int turnPwm = (int)(angleError * 2.0f); // Gain proportionnel simple
        turnPwm = constrain(turnPwm, -PWM_DIAG, PWM_DIAG);
        motorSetBoth(-turnPwm, turnPwm);
        return NAV_BUSY;
    }

    // Etape 2 : ajuster la distance au mur
    float avgDist = (fl + fr) / 2.0f;
    float distError = avgDist - TOF_ALIGN_TARGET_MM;

    if (fabs(distError) > 3.0f) {
        int movePwm = (int)(distError * 1.5f);
        movePwm = constrain(movePwm, -PWM_DIAG, PWM_DIAG);
        motorSetBoth(movePwm, movePwm);
        return NAV_BUSY;
    }

    // Alignement termine
    motorBrake();
    state = NAV_DONE;
    return NAV_DONE;
}

// ----- ADVANCE WITH CURVE (Run 2) -----

void navStartAdvanceWithCurve(int nextTurn) {
    encoderResetBoth();
    pidReset(pidMotorL);
    pidReset(pidMotorR);
    pidReset(pidLateral);
    imuResetYaw();

    curveNextTurn = nextTurn;
    targetTicks = TICKS_PER_CELL;
    basePWM = PWM_RUN2;

    state = NAV_BUSY;
    currentAction = ACT_CURVE;
    actionStartMs = millis();
}

static NavState updateCurve() {
    long ticksL = encoderGetLeft();
    long ticksR = encoderGetRight();
    long avgTicks = (ticksL + ticksR) / 2;

    // Distance restante en ticks
    int remaining = targetTicks - avgTicks;

    if (remaining <= 0) {
        // Si pas de virage, juste freiner
        if (curveNextTurn == 0) {
            motorBrake();
            state = NAV_DONE;
            return NAV_DONE;
        }
    }

    // Seuil d'anticipation du virage (en ticks)
    int anticipationTicks = (int)(30.0f * TICKS_PER_MM); // 30mm avant la fin

    if (curveNextTurn != 0 && remaining < anticipationTicks && remaining > -TICKS_PER_90DEG) {
        // Phase de virage en cloche
        // Profil en S : la roue interieure ralentit, l'exterieure accelere
        float progress = 1.0f - (float)remaining / anticipationTicks;
        if (progress < 0) progress = 0;
        if (progress > 2.0f) progress = 2.0f;

        // Courbe en cloche (sinusoide)
        float curve = sin(progress * PI / 2.0f);

        int innerPWM = basePWM - (int)(curve * basePWM * 0.6f);
        int outerPWM = basePWM + (int)(curve * basePWM * 0.2f);

        innerPWM = constrain(innerPWM, 0, 255);
        outerPWM = constrain(outerPWM, 0, 255);

        if (curveNextTurn > 0) {
            // Virage droite : gauche=ext, droite=int
            motorSetBoth(outerPWM, innerPWM);
        } else {
            // Virage gauche : gauche=int, droite=ext
            motorSetBoth(innerPWM, outerPWM);
        }

        // Verifier si le virage est complet (gyroscope)
        float targetAngle = curveNextTurn * 90.0f;
        if (fabs(imuGetYaw()) >= fabs(targetAngle) - 5.0f) {
            // Virage termine, passer a la case suivante
            encoderResetBoth();
            imuResetYaw();
            state = NAV_DONE;
            return NAV_DONE;
        }
    } else {
        // Phase de ligne droite — PID lateral actif
        float errorLat = sensorFilteredSL() - sensorFilteredSR();
        float correction = pidCompute(pidLateral, errorLat);

        int pwmL = basePWM - (int)correction;
        int pwmR = basePWM + (int)correction;
        pwmL = constrain(pwmL, 0, 255);
        pwmR = constrain(pwmR, 0, 255);

        motorSetBoth(pwmL, pwmR);
    }

    return NAV_BUSY;
}

// ----- UPDATE PRINCIPAL -----

NavState navigationUpdate() {
    // Lire les capteurs a chaque update
    sensorsRead();
    imuUpdate();
    encodersComputeSpeed();

    switch (currentAction) {
        case ACT_ADVANCE:    return updateAdvance();
        case ACT_TURN:       return updateTurn();
        case ACT_AUTO_ALIGN: return updateAutoAlign();
        case ACT_CURVE:      return updateCurve();
        default:             return NAV_IDLE;
    }
}

void navAbort() {
    motorBrake();
    state = NAV_IDLE;
    currentAction = ACT_NONE;
}

NavState navGetState() {
    return state;
}
```

- [ ] **Step 3: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 4: Commit**

```bash
git add src/navigation.h src/navigation.cpp
git commit -m "feat: navigation primitives — advance, turn, auto-align, bell curve"
```

---

## Task 12: Tremaux Algorithm

**Files:**
- Create: `src/tremaux.h`
- Create: `src/tremaux.cpp`

- [ ] **Step 1: Create tremaux.h**

```cpp
// src/tremaux.h
// Algorithme de Tremaux pour l'exploration du labyrinthe (Run 1)
// Fonctionne case par case en cycle : SCAN -> DECIDE -> ORIENT -> MOVE -> UPDATE
#pragma once

#include <Arduino.h>

// Etat de l'algorithme Tremaux
enum TremauxState {
    TREM_SCAN,          // Scanner les murs de la case actuelle
    TREM_DECIDE,        // Choisir la prochaine direction
    TREM_ORIENT,        // Tourner vers la direction choisie
    TREM_MOVE,          // Avancer d'une case
    TREM_CHECK_LINE,    // Verifier le capteur de ligne
    TREM_AUTO_ALIGN,    // Auto-calage si mur devant
    TREM_UPDATE,        // Mettre a jour la carte
    TREM_FINISHED       // Exploration terminee
};

// Initialise l'algorithme (a appeler avant le Run 1)
void tremauxInit();

// Execute un pas de l'algorithme — appeler dans loop()
// Retourne true quand l'exploration est terminee
bool tremauxUpdate();

// Retourne l'etat actuel (pour debug / IHM)
TremauxState tremauxGetState();
```

- [ ] **Step 2: Create tremaux.cpp**

```cpp
// src/tremaux.cpp
#include "tremaux.h"
#include "maze.h"
#include "sensors.h"
#include "navigation.h"
#include "mcp_io.h"
#include "config.h"

static TremauxState tState = TREM_SCAN;
static uint8_t chosenDir = 0; // Direction choisie par l'algorithme

void tremauxInit() {
    tState = TREM_SCAN;
    // Marquer la case de depart comme visitee
    mazeMarkVisited(START_ROW, START_COL);
}

// Choisit la meilleure direction selon Tremaux
// Retourne la direction absolue (0=N, 1=E, 2=S, 3=W)
static uint8_t tremauxChooseDirection() {
    uint8_t row = mazeGetRobotRow();
    uint8_t col = mazeGetRobotCol();
    uint8_t dir = mazeGetRobotDir();
    Cell& cell = mazeGetCell(row, col);

    // Lister les directions possibles (pas de mur)
    // Priorite : non marque > marque 1 fois > jamais marque 2 fois
    uint8_t bestDir = dir;           // Par defaut, tout droit
    uint8_t bestMark = 255;          // Plus petit = meilleur

    // Direction d'ou on vient (pour Tremaux : marquer cette entree)
    uint8_t fromDir = (dir + 2) % 4; // Oppose de la direction du robot

    // Marquer l'entree (+1)
    if (cell.marks[fromDir] < 2) {
        cell.marks[fromDir]++;
    }

    // Evaluer chaque direction
    for (int d = 0; d < 4; d++) {
        // Ignorer si mur
        if (cell.walls & (1 << d)) continue;

        // Ignorer si marque 2 (dead-end confirmee)
        if (cell.marks[d] >= 2) continue;

        // Priorite : 0 (non visite) > 1 (visite une fois)
        if (cell.marks[d] < bestMark) {
            bestMark = cell.marks[d];
            bestDir = d;
        }
    }

    // Marquer la sortie (+1)
    if (cell.marks[bestDir] < 2) {
        cell.marks[bestDir]++;
    }

    return bestDir;
}

bool tremauxUpdate() {
    switch (tState) {
        case TREM_SCAN: {
            // Scanner les murs de la case actuelle avec les capteurs
            sensorsRead();
            bool front = wallFront();
            bool left  = wallLeft();
            bool right = wallRight();

            // Mettre a jour les murs dans la carte
            uint8_t row = mazeGetRobotRow();
            uint8_t col = mazeGetRobotCol();
            uint8_t dir = mazeGetRobotDir();
            mazeUpdateWalls(row, col, dir, front, left, right);

            Serial.printf("[TREM] Scan (%d,%d) dir=%d : F=%d L=%d R=%d\n",
                row, col, dir, front, left, right);

            tState = TREM_DECIDE;
            break;
        }

        case TREM_DECIDE: {
            chosenDir = tremauxChooseDirection();
            uint8_t currentDir = mazeGetRobotDir();

            Serial.printf("[TREM] Decide: direction choisie = %d\n", chosenDir);

            if (chosenDir == currentDir) {
                // Tout droit — pas besoin de tourner
                tState = TREM_MOVE;
                navStartAdvance();
            } else {
                // Calculer la rotation necessaire
                int quarters = (chosenDir - currentDir + 4) % 4;
                if (quarters == 3) quarters = -1; // Optimiser : 3 quarts = -1 quart

                tState = TREM_ORIENT;
                navStartTurn(quarters);
            }
            break;
        }

        case TREM_ORIENT: {
            NavState ns = navigationUpdate();
            if (ns == NAV_DONE) {
                // Mettre a jour l'orientation du robot dans la carte
                mazeSetRobotPos(mazeGetRobotRow(), mazeGetRobotCol(), chosenDir);

                // Verifier si mur devant apres rotation
                sensorsRead();
                if (wallFront()) {
                    // Mur devant apres rotation — auto-calage
                    tState = TREM_AUTO_ALIGN;
                    navStartAutoAlign();
                } else {
                    // Avancer
                    tState = TREM_MOVE;
                    navStartAdvance();
                }
            }
            break;
        }

        case TREM_MOVE: {
            NavState ns = navigationUpdate();
            if (ns == NAV_DONE) {
                tState = TREM_CHECK_LINE;
            }
            break;
        }

        case TREM_CHECK_LINE: {
            // Verifier le capteur de ligne GT1140
            if (mcpReadLineSensor()) {
                Serial.println("[TREM] LIGNE NOIRE DETECTEE — ARRIVEE !");
                tState = TREM_FINISHED;
                return true;
            }

            // Auto-calage si mur devant
            sensorsRead();
            if (wallFront()) {
                tState = TREM_AUTO_ALIGN;
                navStartAutoAlign();
            } else {
                tState = TREM_UPDATE;
            }
            break;
        }

        case TREM_AUTO_ALIGN: {
            NavState ns = navigationUpdate();
            if (ns == NAV_DONE) {
                tState = TREM_UPDATE;
            }
            break;
        }

        case TREM_UPDATE: {
            // Avancer la position du robot dans la carte
            mazeAdvanceRobot();
            uint8_t row = mazeGetRobotRow();
            uint8_t col = mazeGetRobotCol();

            // Marquer la case comme visitee
            mazeMarkVisited(row, col);

            Serial.printf("[TREM] Arrive en (%d,%d) — visites=%d\n",
                row, col, mazeGetVisited(row, col));

            // Afficher la carte en Serial
            mazePrintSerial();

            // Verifier si exploration complete
            if (mazeIsFullyExplored()) {
                Serial.println("[TREM] Exploration complete !");
                tState = TREM_FINISHED;
                return true;
            }

            // Prochain cycle
            tState = TREM_SCAN;
            break;
        }

        case TREM_FINISHED:
            return true;
    }

    return false;
}

TremauxState tremauxGetState() {
    return tState;
}
```

- [ ] **Step 3: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 4: Commit**

```bash
git add src/tremaux.h src/tremaux.cpp
git commit -m "feat: Tremaux algorithm — scan, decide, orient, move, update cycle"
```

---

## Task 13: BFS Algorithm

**Files:**
- Create: `src/bfs.h`
- Create: `src/bfs.cpp`

- [ ] **Step 1: Create bfs.h**

```cpp
// src/bfs.h
// Algorithme BFS (Breadth-First Search) pour le chemin optimal (Run 2)
// Utilise la carte memorisee par Tremaux pour trouver le plus court chemin
#pragma once

#include <Arduino.h>
#include "config.h"

// Structure pour un point du chemin
struct PathPoint {
    uint8_t row;
    uint8_t col;
};

// Calcule le chemin optimal de (startRow,startCol) vers (endRow,endCol)
// Retourne le nombre de cases dans le chemin (0 = pas de chemin)
uint8_t bfsComputePath(uint8_t startRow, uint8_t startCol,
                       uint8_t endRow, uint8_t endCol);

// Retourne le chemin calcule (tableau statique)
const PathPoint* bfsGetPath();

// Retourne le nombre de points dans le chemin
uint8_t bfsGetPathLength();

// Convertit le chemin en sequence de virages
// Pour chaque paire de cases consecutives, determine si le robot doit
// tourner a gauche, a droite, ou aller tout droit
// nextTurn[i] : 0=tout droit, +1=droite, -1=gauche
// startDir : direction initiale du robot
void bfsComputeTurns(uint8_t startDir);

// Retourne la sequence de virages
const int8_t* bfsGetTurns();
```

- [ ] **Step 2: Create bfs.cpp**

```cpp
// src/bfs.cpp
#include "bfs.h"
#include "maze.h"

// Chemin statique (max 25 cases pour un labyrinthe 5x5)
static PathPoint path[MAZE_SIZE * MAZE_SIZE];
static uint8_t pathLength = 0;

// Sequence de virages
static int8_t turns[MAZE_SIZE * MAZE_SIZE];

// BFS interne
static const int8_t dRow[4] = { -1,  0,  1,  0 }; // N, E, S, W
static const int8_t dCol[4] = {  0,  1,  0, -1 };

uint8_t bfsComputePath(uint8_t startRow, uint8_t startCol,
                       uint8_t endRow, uint8_t endCol) {
    // File FIFO statique
    struct BFSNode {
        uint8_t row, col;
    };
    BFSNode queue[MAZE_SIZE * MAZE_SIZE];
    uint8_t head = 0, tail = 0;

    // Tableau de parents pour reconstruire le chemin
    // parent[r][c] = direction d'ou on est arrive (0-3), ou 255 si pas visite
    uint8_t parent[MAZE_SIZE][MAZE_SIZE];
    bool visited[MAZE_SIZE][MAZE_SIZE];

    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            parent[r][c] = 255;
            visited[r][c] = false;
        }
    }

    // Enfiler le depart
    queue[tail++] = { startRow, startCol };
    visited[startRow][startCol] = true;

    bool found = false;

    while (head < tail && !found) {
        BFSNode current = queue[head++];

        // Explorer les 4 voisins
        for (int d = 0; d < 4; d++) {
            // Verifier s'il y a un mur dans cette direction
            uint8_t walls = mazeGetWalls(current.row, current.col);
            if (walls & (1 << d)) continue; // Mur = pas de passage

            int8_t nr = current.row + dRow[d];
            int8_t nc = current.col + dCol[d];

            // Hors limites
            if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;

            // Deja visite
            if (visited[nr][nc]) continue;

            visited[nr][nc] = true;
            parent[nr][nc] = d;
            queue[tail++] = { (uint8_t)nr, (uint8_t)nc };

            if (nr == endRow && nc == endCol) {
                found = true;
                break;
            }
        }
    }

    if (!found) {
        pathLength = 0;
        Serial.println("[BFS] Pas de chemin trouve !");
        return 0;
    }

    // Reconstruire le chemin en remontant les parents
    pathLength = 0;
    uint8_t r = endRow, c = endCol;

    while (r != startRow || c != startCol) {
        path[pathLength++] = { r, c };
        uint8_t d = parent[r][c];
        // Remonter dans la direction opposee
        r -= dRow[d];
        c -= dCol[d];
    }
    path[pathLength++] = { startRow, startCol };

    // Inverser le chemin (il est de la fin vers le debut)
    for (int i = 0; i < pathLength / 2; i++) {
        PathPoint tmp = path[i];
        path[i] = path[pathLength - 1 - i];
        path[pathLength - 1 - i] = tmp;
    }

    Serial.printf("[BFS] Chemin trouve : %d cases\n", pathLength);
    for (int i = 0; i < pathLength; i++) {
        Serial.printf("  -> (%d,%d)\n", path[i].row, path[i].col);
    }

    return pathLength;
}

const PathPoint* bfsGetPath() {
    return path;
}

uint8_t bfsGetPathLength() {
    return pathLength;
}

void bfsComputeTurns(uint8_t startDir) {
    if (pathLength < 2) return;

    uint8_t currentDir = startDir;

    for (int i = 0; i < pathLength - 1; i++) {
        // Direction necessaire pour aller de path[i] a path[i+1]
        int8_t dr = path[i+1].row - path[i].row;
        int8_t dc = path[i+1].col - path[i].col;

        uint8_t neededDir;
        if (dr == -1) neededDir = DIR_N;
        else if (dc ==  1) neededDir = DIR_E;
        else if (dr ==  1) neededDir = DIR_S;
        else neededDir = DIR_W;

        // Calculer le virage relatif
        int diff = (neededDir - currentDir + 4) % 4;
        if (diff == 0)      turns[i] = 0;   // Tout droit
        else if (diff == 1) turns[i] = 1;   // Droite
        else if (diff == 3) turns[i] = -1;  // Gauche
        else                turns[i] = 2;   // Demi-tour (ne devrait pas arriver)

        currentDir = neededDir;
    }
    turns[pathLength - 1] = 0; // Derniere case : pas de virage

    Serial.print("[BFS] Virages : ");
    for (int i = 0; i < pathLength - 1; i++) {
        if (turns[i] == 0) Serial.print("^ ");
        else if (turns[i] == 1) Serial.print("> ");
        else if (turns[i] == -1) Serial.print("< ");
        else Serial.print("U ");
    }
    Serial.println();
}

const int8_t* bfsGetTurns() {
    return turns;
}
```

- [ ] **Step 3: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 4: Commit**

```bash
git add src/bfs.h src/bfs.cpp
git commit -m "feat: BFS shortest path with turn sequence computation"
```

---

## Task 14: Integration — Full State Machine

**Files:**
- Modify: `src/main.cpp` — integrer Tremaux + BFS + navigation Run 2

- [ ] **Step 1: Update main.cpp with complete state machine**

```cpp
// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "mcp_io.h"
#include "leds.h"
#include "sensors.h"
#include "imu.h"
#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "diagnostic.h"
#include "maze.h"
#include "tremaux.h"
#include "bfs.h"
#include "navigation.h"
#include "web_ui.h"

// Etat global du robot
RobotState currentState = STATE_DIAGNOSTIC;

// Controleurs PID globaux
PIDController pidMotorL;
PIDController pidMotorR;
PIDController pidLateral;

static bool diagDone = false;
static unsigned long lastSensorMs = 0;

// --- Run 2 : suivi du chemin BFS ---
static uint8_t bfsStep = 0; // Index dans le chemin BFS
static bool bfsRunning = false;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== MAZEBOT T17 — Equipe 17 ===");
    Serial.println("Demarrage...\n");

    // Initialiser les PID
    pidInit(pidMotorL, PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD,
            PID_SAMPLE_MS, -255, 255);
    pidInit(pidMotorR, PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD,
            PID_SAMPLE_MS, -255, 255);
    pidInit(pidLateral, PID_LAT_KP, PID_LAT_KI, PID_LAT_KD,
            PID_SAMPLE_MS, -100, 100);
}

void loop() {
    unsigned long now = millis();

    ledsUpdate();

    // Arret d'urgence depuis l'IHM (priorite absolue)
    if (webEmergencyStop) {
        webEmergencyStop = false;
        motorEmergencyStop();
        navAbort();
        currentState = STATE_EMERGENCY;
        ledsSetEmergency();
        Serial.println("[MAIN] ARRET D'URGENCE !");
    }

    switch (currentState) {

        // ===================== DIAGNOSTIC =====================
        case STATE_DIAGNOSTIC: {
            if (!diagDone) {
                ledsSetDiagnostic();
                DiagResult result = diagnosticRun();
                mazeInit();
                navigationInit();
                webUiInit();

                if (result.allCriticalOk()) {
                    currentState = STATE_IDLE;
                    ledsSetIdle();
                    Serial.println("[MAIN] -> IDLE (en attente sur http://192.168.4.1)");
                } else {
                    currentState = STATE_EMERGENCY;
                    ledsSetEmergency();
                    Serial.println("[MAIN] -> EMERGENCY (diagnostic echoue)");
                }
                diagDone = true;
            }
            break;
        }

        // ===================== IDLE =====================
        case STATE_IDLE: {
            if (now - lastSensorMs >= SENSOR_READ_MS) {
                lastSensorMs = now;
                sensorsRead();
                imuUpdate();
                encodersComputeSpeed();
            }

            if (webStartRun1) {
                webStartRun1 = false;
                mazeInit();
                tremauxInit();
                encoderResetBoth();
                pidReset(pidMotorL);
                pidReset(pidMotorR);
                pidReset(pidLateral);
                currentState = STATE_RUN1_TREMAUX;
                ledsSetRun1();
                Serial.println("[MAIN] -> RUN 1 (Tremaux)");
            }
            break;
        }

        // ===================== RUN 1 — TREMAUX =====================
        case STATE_RUN1_TREMAUX: {
            bool finished = tremauxUpdate();

            if (finished) {
                motorBrake();
                mazePrintSerial();
                currentState = STATE_MAZE_COMPLETE;
                ledsSetMazeComplete();
                Serial.println("[MAIN] -> MAZE COMPLETE");
            }
            break;
        }

        // ===================== MAZE COMPLETE =====================
        case STATE_MAZE_COMPLETE: {
            if (now - lastSensorMs >= SENSOR_READ_MS) {
                lastSensorMs = now;
                sensorsRead();
            }

            if (webStartRun2) {
                webStartRun2 = false;

                // Calculer le chemin BFS
                uint8_t len = bfsComputePath(START_ROW, START_COL, END_ROW, END_COL);

                if (len > 0) {
                    bfsComputeTurns(START_DIR);
                    bfsStep = 0;
                    bfsRunning = false;

                    // Repositionner le robot au depart
                    mazeSetRobotPos(START_ROW, START_COL, START_DIR);
                    encoderResetBoth();
                    pidReset(pidMotorL);
                    pidReset(pidMotorR);
                    pidReset(pidLateral);

                    currentState = STATE_RUN2_BFS;
                    ledsSetRun2();
                    Serial.println("[MAIN] -> RUN 2 (BFS)");
                } else {
                    Serial.println("[MAIN] BFS: pas de chemin ! Reste en MAZE_COMPLETE");
                }
            }
            break;
        }

        // ===================== RUN 2 — BFS =====================
        case STATE_RUN2_BFS: {
            const int8_t* turnSeq = bfsGetTurns();
            uint8_t pathLen = bfsGetPathLength();

            // Verifier si on a fini le chemin
            if (bfsStep >= pathLen - 1) {
                motorBrake();
                currentState = STATE_FINISHED;
                ledsSetFinished();
                Serial.println("[MAIN] -> FINISHED ! Labyrinthe resolu !");
                break;
            }

            if (!bfsRunning) {
                // Lancer le prochain mouvement en cloche
                int nextTurn = (bfsStep < pathLen - 2) ? turnSeq[bfsStep + 1] : 0;
                navStartAdvanceWithCurve(turnSeq[bfsStep]);
                bfsRunning = true;
            }

            NavState ns = navigationUpdate();
            if (ns == NAV_DONE) {
                mazeAdvanceRobot();
                if (turnSeq[bfsStep] != 0) {
                    mazeRotateRobot(turnSeq[bfsStep]);
                }
                bfsStep++;
                bfsRunning = false;
                Serial.printf("[BFS] Step %d/%d\n", bfsStep, pathLen - 1);
            }
            break;
        }

        // ===================== FINISHED =====================
        case STATE_FINISHED: {
            motorBrake();
            break;
        }

        // ===================== EMERGENCY =====================
        case STATE_EMERGENCY: {
            motorEmergencyStop();
            if (webStartRun1) {
                webStartRun1 = false;
                currentState = STATE_IDLE;
                ledsSetIdle();
                Serial.println("[MAIN] -> IDLE (sortie urgence)");
            }
            break;
        }
    }
}
```

- [ ] **Step 2: Compile**

Run: `pio run`
Expected: 0 erreurs

- [ ] **Step 3: Flash and test**

Run: `pio run --target upload && pio device monitor -b 115200`

Expected:
1. Diagnostic s'execute
2. WiFi AP "Robot_Laby_Eq17" apparait
3. Page web accessible a 192.168.4.1
4. Capteurs visibles en temps reel
5. Bouton START RUN 1 lance Tremaux
6. Carte se remplit en temps reel

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "feat: complete state machine — diagnostic, run1 tremaux, run2 bfs, web UI"
```

- [ ] **Step 5: Push to GitHub**

```bash
git push origin main
```

---

## Task 15: Update CLAUDE.md with GT1140 + Final Config

**Files:**
- Modify: `CLAUDE.md` — ajouter GT1140, corriger les phases, mettre a jour config

- [ ] **Step 1: Update CLAUDE.md pinout section to include GT1140**

Add to the MCP23017 section in CLAUDE.md:
```
#define MCP_LINE_SENSOR 4   // GPA4 -> GT1140 capteur de ligne
```

- [ ] **Step 2: Update phases in CLAUDE.md to match actual implementation**

Update the phases table to reflect the "IHM early" approach and Kalman/bell curve additions.

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md with GT1140 pinout and revised phases"
```

---

## Post-Implementation Notes

### Calibration Workflow (sur le robot physique)

1. **Flasher le code** : `pio run --target upload`
2. **Verifier le diagnostic** : ouvrir Serial Monitor, tout doit etre OK
3. **Se connecter au WiFi** : SSID `Robot_Laby_Eq17`, mdp `icam2026`
4. **Ouvrir l'IHM** : `http://192.168.4.1`
5. **Calibrer les seuils ToF** :
   - Placer le robot dans un couloir droit avec murs
   - Ajuster les sliders Kalman Q/R jusqu'a lisser les poteaux
6. **Calibrer le PID moteur** :
   - Suivre l'assistant de tuning (Etapes 1-4)
   - Commencer par Kp seul, puis Ki, puis Kd
7. **Mesurer l'entraxe des roues** :
   - Faire tourner le robot de 360 deg et compter les ticks
   - Ajuster `WHEEL_BASE_MM` dans config.h
8. **Tester l'avance d'une case** :
   - Placer le robot, lancer depuis l'IHM
   - Verifier qu'il avance bien de 200mm
9. **Tester Run 1 sur un mini-labyrinthe** (2x2 ou 3x3)
10. **Lancer Run 1 sur le vrai labyrinthe 5x5**
11. **Lancer Run 2**

### Valeurs a mesurer sur le robot physique

| Parametre | Comment mesurer | Ou modifier |
|-----------|----------------|-------------|
| `WHEEL_BASE_MM` | Distance entre centres des roues (regle/pied a coulisse) | `config.h` |
| `TICKS_PER_90DEG` | Faire tourner 360 deg, diviser ticks/4 | Calcule auto dans config.h |
| Seuils ToF mur | IHM, calibration auto | Via IHM (RAM) |
| PID Kp/Ki/Kd | IHM, assistant de tuning | Via IHM (RAM) |
| Kalman Q/R | IHM, observation courbes | Via IHM (RAM) |
