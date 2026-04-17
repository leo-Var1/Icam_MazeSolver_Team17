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
#define PWM_RUN1        65   // ~25% — vitesse exploration
#define PWM_RUN2        230  // ~90% — vitesse résolution

// ── Labyrinthe ────────────────────────────────────────────────
#define MAZE_SIZE       5    // grille 5×5
#define CELL_SIZE_MM    220  // 200 mm par case (220mm en réel)

// ── PID encodeurs (correction dérive ticks gauche/droite) ────
#define PID_KP          2.0f
#define PID_KI          0.05f
#define PID_KD          0.8f
#define PID_SAMPLE_MS   20   // 50 Hz

// ── PID ToF latéraux (centrage dans le couloir) ───────────────
// Signal d'erreur = side_left_mm - side_right_mm
// Si > 0 : trop proche du mur gauche → corriger vers droite
// Les gains sont plus faibles : la mesure en mm est plus bruitée que les ticks
#define PID_TOF_KP      0.8f
#define PID_TOF_KI      0.01f
#define PID_TOF_KD      0.4f

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

// ── Encodeurs ────────────────────────────────────────────────
// Nombre de ticks par tour de roue (à mesurer physiquement)
// Valeur typique pour encodeur 20 CPR + réducteur : ajuster après calibration
#define ENC_TICKS_PER_REV   20

// Diamètre de la roue en mm → circonférence = π × D
#define WHEEL_DIAMETER_MM   65.0f

// Distance entre les deux roues (voie) en mm — entraxe mesuré physiquement
#define WHEEL_BASE_MM       125.0f

// Ticks calibrés physiquement (2026-04-14) :
// 308 ticks mesurés → 220mm réels → recalibré à 280 ticks pour 200mm
// Roue ∅43mm → périmètre 135.1mm → ~210 ticks/tour (ratio réducteur inclus)
#define TICKS_PER_MM        1.40f   // 280 ticks / 200mm
#define TICKS_PER_CELL      245     // 200mm = 1 case

// ── Navigation ────────────────────────────────────────────────
// PWM pour rotations sur place — lent pour éviter le glissement
#define PWM_TURN            120  // roue intérieure en sens inverse (frottement statique élevé)
// (non utilisé pour l'instant — rotation en une seule phase lente)
#define PWM_TURN_SLOW       50
// PWM faible pour les micro-corrections (auto-alignement, diagnostic)
#define PWM_DIAG            40

// Ticks pour une rotation de 90° sur place :
// Chaque roue parcourt un arc = (PI/2) × (WHEEL_BASE_MM/2)
// WHEEL_BASE = 125mm → arc = 1.5708 × 62.5 ≈ 98.2mm → 98.2 × 1.40 ≈ 137 ticks
#define TICKS_PER_90DEG     125

// Seuil de décélération pour la rotation (~68% de TICKS_PER_90DEG)
// 137 × 0.68 ≈ 93 ticks
#define TICKS_TURN_DECEL    93

// Durée du frein intermédiaire entre phase rapide et phase lente (ms)
// Le DRV8833 freine activement (IN1=IN2=HIGH) → tue l'inertie en ~50-80ms
#define TURN_BRAKE_MS       60

// Impulsion inverse après l'arrêt du virage (correction de dépassement résiduel)
// Le robot recule brièvement dans le sens opposé pour revenir sur 90° pile
#define TURN_REVERSE_MS     35    // durée de l'impulsion (ms) — augmenter si trop court
#define TURN_REVERSE_PWM    70    // PWM de l'impulsion inverse

// Délai de stabilisation après freinage avant de démarrer une rotation (ms)
#define TURN_SETTLE_MS      80

// ── Demi-tour 180° (constantes séparées — à calibrer indépendamment) ──────
// Le 180° accumule plus d'inertie que le 90° → frein et reverse plus longs.
// Ticks cible : légèrement sous 2×90° pour compenser l'inertie accumulée
#define TICKS_PER_180DEG    300   // ≈ 2×130 - marge. Augmenter si trop court.
// Seuil décélération 180° (~72% de TICKS_PER_180DEG)
#define TICKS_TURN_DECEL_180  180
// Frein actif après la rotation 180° (plus long qu'un 90°)
#define TURN_BRAKE_MS_180   90    // ms — augmenter si le robot continue de glisser
// Impulsion inverse 180° : plus longue pour corriger le dépassement résiduel
#define TURN_REVERSE_MS_180 55    // ms — augmenter si encore trop court

// ── Seuils capteurs ToF ───────────────────────────────────────
// Mur frontal détecté si distance < 120mm (les deux capteurs FL et FR)
#define TOF_WALL_FRONT_MM   120
// Mur latéral détecté par les capteurs à 45° (SL, SR).
// Géométrie : mur à 100mm latéral → capteur à 45° lit 100/cos(45°) ≈ 141mm.
// On prend 160mm comme seuil avec marge de sécurité → à calibrer physiquement.
#define TOF_WALL_SIDE_MM    160
// Arrêt d'urgence si mur < 50mm devant
#define TOF_STOP_FRONT_MM   60
// Distance cible pour l'auto-alignement frontal
#define TOF_ALIGN_TARGET_MM 40
// Tolérance d'alignement FL vs FR (en mm)
#define TOF_ALIGN_TOL_MM    2

// Marge de rejet géométrique capteurs latéraux à 45°.
// Un mur frontal à distance d projette sur le capteur 45° à d × √2.
// Le capteur latéral est déclaré "vrai mur latéral" seulement si sa
// lecture est inférieure à (d_frontal × 1.414 - TOF_SIDE_GEOM_MARGIN).
// → Augmenter si faux positifs persistent, diminuer si vrais murs manqués.
// Limitation connue : un coin (mur frontal + mur latéral équidistants)
// peut ne pas être détecté → à calibrer physiquement.
#define TOF_SIDE_GEOM_MARGIN  30

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

// ── IMU MPU6050 ───────────────────────────────────────────────
// Sensibilité gyroscope : plage ±250°/s → 131.0 LSB/(°/s)
#define IMU_GYRO_SENSITIVITY    131.0f

// Nombre d'échantillons pour calibrer le biais gyro au démarrage
// Robot immobile pendant (IMU_CALIB_SAMPLES × PID_SAMPLE_MS) ms ≈ 1s
#define IMU_CALIB_SAMPLES       50

// Seuil en dessous duquel le gyro est considéré "à zéro" (anti-drift)
// En degrés/seconde — en dessous de cette valeur, on n'intègre pas
#define IMU_DEADBAND_DPS        0.5f

// Tolérance pour déclarer une rotation de 90° terminée (±2°)
// Réduit de 5° à 2° : la décélération par encodeurs absorbe l'inertie,
// donc on peut s'arrêter plus près de la cible.
#define IMU_ROTATION_TOLERANCE  2.0f

// Période d'échantillonnage IMU = même que PID (50 Hz)
#define IMU_SAMPLE_MS           PID_SAMPLE_MS
