#pragma once

// =============================================================
//  imu.h — Interface module MPU6050 (gyroscope cap + rotation)
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  Ce module gère :
//    1. L'initialisation du MPU6050 via I2C
//    2. La calibration du biais gyro (robot immobile au boot)
//    3. L'intégration du gyro Z → cap en degrés (yaw)
//    4. La détection de rotation à 90° (±IMU_ROTATION_TOLERANCE)
//
//  Usage typique dans la loop() :
//    imu_update();                         // appeler toutes les IMU_SAMPLE_MS ms
//    float cap = imu_get_heading();        // cap actuel en degrés
//    imu_reset_heading();                  // avant une rotation
//    if (imu_rotation_complete(90.0f)) {}  // détecter fin de rotation

// ── Initialisation ───────────────────────────────────────────
// Démarre la communication I2C avec le MPU6050,
// règle le gyro en ±250°/s, et calibre le biais.
// Retourne true si le MPU6050 est trouvé et initialisé.
bool imu_init();

// ── Mise à jour (à appeler périodiquement) ───────────────────
// Lit le gyro Z brut, soustrait le biais, applique le deadband,
// puis intègre sur dt pour mettre à jour le cap interne.
// ⚠️ Doit être appelé toutes les IMU_SAMPLE_MS ms (utiliser millis())
void imu_update();

// ── Lecture du cap courant ───────────────────────────────────
// Retourne le cap intégré en degrés depuis le dernier reset.
// Positif = sens antihoraire (convention gyro Z+ vers le haut).
float imu_get_heading();

// ── Remise à zéro du cap ─────────────────────────────────────
// À appeler juste AVANT de commencer une rotation,
// pour mesurer l'angle parcouru depuis ce point.
void imu_reset_heading();

// ── Détection fin de rotation ────────────────────────────────
// Retourne true si |cap| >= target_deg - IMU_ROTATION_TOLERANCE.
// Exemple : imu_rotation_complete(90.0f) → vrai entre 85° et 95°
bool imu_rotation_complete(float target_deg);
