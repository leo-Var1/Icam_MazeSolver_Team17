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
#define PWM_RUN1        80   // ~31% — vitesse exploration (couple ok, PID stable)
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
// KP faible : évite les oscillations (mesure en mm bruitée)
// KD élevé  : amortit les oscillations
// KI minimal : évite le windup (dérive lente vers un côté)
#define PID_TOF_KP      0.15f   // ↑ correction proportionnelle plus marquée
#define PID_TOF_KI      0.001f
#define PID_TOF_KD      0.5f    // ↓ moins sensible au bruit ToF (5-10mm cycle à cycle)
// Cap de correction : permet de récupérer après un smooth turn (biais ~50mm)
#define PID_TOF_MAX_CORR  25

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
// (Les PWM de pivot sont définis plus bas dans la section "pivot 90°/180°".)
// PWM faible pour les micro-corrections (auto-alignement, diagnostic)
#define PWM_DIAG            40

// ── PID pivot (force |ticks_L| ≈ |ticks_R| pendant rotation sur place) ──
// Erreur = labs(ticks_L) - labs(ticks_R) → correction symétrique des magnitudes
// PWM : on accélère la roue lente, on freine la roue rapide.
// PWM minimum garanti pour ne pas staller (vaincre frottement statique).
#define PID_PIVOT_KP        1.5f
#define PID_PIVOT_KI        0.04f
#define PID_PIVOT_KD        0.6f
#define PID_PIVOT_MAX_CORR  35     // cap correction (PWM units)
#define PID_PIVOT_MIN_PWM   75     // plancher PWM (au-dessus du seuil de stall)

// ── Rotation — pivot sur place 90° et 180° ───────────────────
// Cinématique : les 2 roues tournent en sens opposés autour du centre
// du robot. Le PID pivot équilibre |ticks_L| ≈ |ticks_R|.
//
// RÉFÉRENCE PRIMAIRE D'ARRÊT : ticks encodeur (plus précis que l'IMU).
// L'IMU sert de garde-fou : si l'angle dépasse IMU_PIVOT_TARGET avant les
// ticks, on stoppe quand même (problème de glissement → log d'alerte).
//
// Calcul théorique des ticks pour 90° :
//   arc roue = (π/2) × (WHEEL_BASE/2) = 1.5708 × 62.5 ≈ 98 mm
//   ticks   = 98 × TICKS_PER_MM = 98 × 1.40 ≈ 137 ticks
// En pratique le glissement augmente cette valeur (typiquement 150-170).
// → Ajustable en direct via l'IHM (calib_get_pivot_90_ticks()).
#define TICKS_PIVOT_90        137     // défaut — à régler physiquement
#define TICKS_PIVOT_45_R      63      // Virage 45° Droite (réduit de 69)
#define TICKS_PIVOT_45_L      60      // Virage 45° Gauche (réduit de 66)
#define TICKS_PIVOT_180       274     // ≈ 2 × TICKS_PIVOT_90


// ── Mouvement spécial 45-40-45 ───────────────────────────────
#define TICKS_SPECIAL_MOVE    98      // 70mm * 1.4 ticks/mm

// Distance frontale cible pour se caler au milieu de la case en fin de smooth turn
#define SMOOTH_CENTER_TARGET_MM   77
// Tolérance d'arrêt sur la distance cible (±mm)
#define SMOOTH_CENTER_TOL_MM      4
// PWM min/max pour la phase de centrage (au-dessus du stall, sous PWM_RUN1)
#define SMOOTH_CENTER_PWM_MIN     65
#define SMOOTH_CENTER_PWM_MAX     90

// Garde-fou IMU : stop forcé si l'angle dépasse cette cible AVANT les ticks
// (cas pathologique : roues qui glissent énormément).
// Cible LARGE pour ne pas couper la rotation prématurément quand l'utilisateur
// augmente les ticks de calibration (sinon on plafonne à ~90°/180° côté IMU).
#define IMU_PIVOT_TARGET      120.0f   // marge large : 30° de tolérance au-delà de 90°
#define IMU_PIVOT_180_TARGET  220.0f   // idem pour le 180°

// Rampe linéaire de décélération :
//   À PIVOT_DECEL_PCT% des ticks, on commence à interpoler PWM_PIVOT_FAST
//   vers PWM_PIVOT_SLOW (linéaire jusqu'à 100% des ticks).
#define PIVOT_DECEL_PCT       50      // début de la rampe à 50% des ticks
#define PWM_PIVOT_FAST        110     // PWM phase rapide (vainc frottement)
#define PWM_PIVOT_SLOW        75      // PWM fin de rampe (juste > stall)

// Délai de stabilisation avant chaque rotation (ms)
#define TURN_SETTLE_MS        80

// Pause entre 2 cases dans Trémaux (laisse le temps de stabiliser + lire ToF propre)
#define TREM_PAUSE_MS         600

// Marge de stricte main droite : pour considérer le passage à droite ouvert,
// la lecture SR doit dépasser opening_r de cette marge (en mm). Évite que
// le wall_follower colle au mur droit en tournant dès la moindre bosse.
#define WF_RIGHT_MARGIN_MM    25

// ── Snapshot détection murs (capteurs à 45°) ─────────────────
// Valeur en % de TICKS_PER_CELL (40% = milieu de la case).
#define WALL_SNAP_PCT       40

// ── Demi-tour 180° ────────────────────────────────────────────
// 180° = pivot continu (pas de décomposition cross).
#define TICKS_PER_180DEG    300

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
// Tolérance d'alignement FL vs FR (en mm) — ancien auto-align
#define TOF_ALIGN_TOL_MM    5
// Tolérance distance translation
#define ALIGN_DIST_TOL_MM   8


// ── Pré-alignement avant pivot (mode bang-bang) ──────────────
// PWM_DIAG (40) est sous le seuil de décrochage → la roue ne démarre pas et
// quand elle finit par bouger, elle dépasse. On utilise un PWM constant
// au-dessus du seuil de stall avec tolérances larges pour éviter le hunting.
#define PWM_ALIGN_TURN          80   // PWM rotation pré-align (au-dessus stall)
#define PWM_ALIGN_MOVE          95   // PWM avance pré-align (vainc frottement)
#define ALIGN_ANGLE_TOL_MM      5    // tolérance |FL - FR|
#define ALIGN_DIST_TOL_MM       8    // tolérance |avg_dist - target|

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
#define CALIB_TOF_TURN_MM        50   // distance frontale → déclenchement virage
#define CALIB_TOF_OPENING_L_MM   150  // SL > seuil → passage gauche ouvert
#define CALIB_TOF_OPENING_R_MM   150  // SR > seuil → passage droit ouvert
#define CALIB_TOF_POST_MM        40   // seuil détection poteau alu (+ marge 10mm incluse)
// Marge ajoutée automatiquement lors de la capture : median + POST_DETECT_MARGIN_MM
#define CALIB_POST_DETECT_MARGIN 10

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
