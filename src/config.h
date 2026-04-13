#pragma once

// =============================================================
//  config.h — Toutes les constantes du projet Robot Labyrinthe
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

// ── I2C Bus ──────────────────────────────────────────────────
#define PIN_SCL         5    // D1
#define PIN_SDA         4    // D2

// ── Moteur GAUCHE (DRV8833 canal A) ──────────────────────────
#define MOTOR_L_IN1     0    // D3 — pull-up 10kΩ → HIGH au boot (frein natif)
#define MOTOR_L_IN2     2    // D4 — pull-up 10kΩ → HIGH au boot

// ── Moteur DROIT (DRV8833 canal B) ───────────────────────────
#define MOTOR_R_IN1     13   // D7
#define MOTOR_R_IN2     15   // D8 — pull-down 10kΩ → LOW au boot

// ── Encodeur GAUCHE ───────────────────────────────────────────
#define ENC_L_A         14   // D5 — phase A (interruption)
#define ENC_L_B         16   // D0 — phase B (sens)

// ── Encodeur DROIT ────────────────────────────────────────────
#define ENC_R_A         12   // D6 — phase A (interruption)
// Phase B droit : analogRead(A0) > 512 dans l'ISR de ENC_R_A
// ⚠️ Ne JAMAIS attacher une interruption sur A0

// ── Adresses I2C ─────────────────────────────────────────────
#define I2C_MCP23017    0x27  // A0=A1=A2=VCC
#define I2C_MPU6050     0x68  // AD0=GND

// Adresses VL53L0X assignées dynamiquement via XSHUT au boot
#define TOF_ADDR_FRONT_L  0x30
#define TOF_ADDR_FRONT_R  0x31
#define TOF_ADDR_SIDE_L   0x32  // 45° gauche
#define TOF_ADDR_SIDE_R   0x33  // 45° droite

// ── Pins XSHUT sur MCP23017 (GPA) ────────────────────────────
// NB : numérotation Adafruit MCP23X17 → GPA0=0 … GPA7=7, GPB0=8 … GPB7=15
#define MCP_XSHUT_FR    0    // GPA0 → Front Right
#define MCP_XSHUT_FL    1    // GPA1 → Front Left
#define MCP_XSHUT_SL    2    // GPA2 → Side Left
#define MCP_XSHUT_SR    3    // GPA3 → Side Right

// ── LEDs sur MCP23017 (GPB) ───────────────────────────────────
#define MCP_LED_RED     8    // GPB0 (pin MCP = 8+0 = 8)  — erreur
#define MCP_LED_YELLOW  9    // GPB1 (pin MCP = 8+1 = 9)  — exploration Run 1
#define MCP_LED_GREEN   10   // GPB2 (pin MCP = 8+2 = 10) — succès / idle

// ── Moteurs : PWM ─────────────────────────────────────────────
#define PWM_RUN1        90   // ~35% — vitesse exploration
#define PWM_RUN2        230  // ~90% — vitesse résolution

// ── Labyrinthe ────────────────────────────────────────────────
#define MAZE_SIZE       5    // grille 5×5
#define CELL_SIZE_MM    200  // 200 mm par case

// ── PID ───────────────────────────────────────────────────────
#define PID_KP          1.2f
#define PID_KI          0.05f
#define PID_KD          0.8f
#define PID_SAMPLE_MS   20   // 50 Hz

// ── WiFi Access Point ─────────────────────────────────────────
#define WIFI_SSID       "Robot_Laby_Eq17"
#define WIFI_PASSWORD   "icam2026"
#define WIFI_PORT       80

// ── Timeout capteurs (ms) ─────────────────────────────────────
#define TOF_TIMEOUT_MS  500   // timeout lecture VL53L0X
#define TOF_MAX_MM      1200  // distance max valide (labyrinthe 5x5 = 1000mm max)

// ── Bitmask murs (maze.h) ─────────────────────────────────────
#define WALL_N  0b0001   // Nord
#define WALL_E  0b0010   // Est
#define WALL_S  0b0100   // Sud
#define WALL_W  0b1000   // Ouest

// ── IMU MPU6050 ───────────────────────────────────────────────
// Sensibilité gyroscope : plage ±250°/s → 131.0 LSB/(°/s)
#define IMU_GYRO_SENSITIVITY    131.0f

// Nombre d'échantillons pour calibrer le biais gyro au démarrage
// Robot immobile pendant (IMU_CALIB_SAMPLES × PID_SAMPLE_MS) ms ≈ 1s
#define IMU_CALIB_SAMPLES       50

// Seuil en dessous duquel le gyro est considéré "à zéro" (anti-drift)
// En degrés/seconde — en dessous de cette valeur, on n'intègre pas
#define IMU_DEADBAND_DPS        0.5f

// Tolérance pour déclarer une rotation de 90° terminée (±5°)
#define IMU_ROTATION_TOLERANCE  5.0f

// Période d'échantillonnage IMU = même que PID (50 Hz)
#define IMU_SAMPLE_MS           PID_SAMPLE_MS
