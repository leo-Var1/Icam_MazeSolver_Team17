// =============================================================
//  imu.cpp — Implémentation module MPU6050
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>   // electroniccats/MPU6050

#include "config.h"
#include "imu.h"

// ── Objet MPU6050 ─────────────────────────────────────────────
static MPU6050 mpu;

// ── Variables internes ────────────────────────────────────────

// Biais gyro Z mesuré à la calibration (en LSB bruts)
// Ce biais est soustrait à chaque lecture pour annuler la dérive
static float s_gyro_z_bias = 0.0f;

// Cap intégré en degrés depuis le dernier imu_reset_heading()
static float s_heading_deg = 0.0f;

// Timestamp de la dernière imu_update() (pour calculer dt)
static uint32_t s_last_update_ms = 0;

// True si l'IMU est correctement initialisé
static bool s_imu_ready = false;

// ── imu_init ──────────────────────────────────────────────────
bool imu_init() {
    // Initialise le MPU6050 (envoie les commandes I2C de configuration)
    mpu.initialize();

    // Vérifie que le MPU6050 répond bien (lit le registre WHO_AM_I)
    if (!mpu.testConnection()) {
        Serial.println("[IMU] ERREUR : MPU6050 ne répond pas (testConnection failed)");
        s_imu_ready = false;
        return false;
    }
    Serial.println("[IMU] MPU6050 connecté ✓");

    // ── Configuration de la plage du gyroscope ────────────────
    // MPU6050_GYRO_FS_250 → ±250°/s → sensibilité 131 LSB/(°/s)
    // C'est la plage la plus précise, parfaite pour les rotations lentes du robot
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    // ── Calibration du biais gyro ─────────────────────────────
    // Le gyro n'est jamais parfaitement à zéro quand le robot est immobile.
    // On moyenne IMU_CALIB_SAMPLES lectures pour estimer ce biais,
    // puis on le soustraira à chaque mesure.
    Serial.print("[IMU] Calibration gyro (robot immobile)...");
    float sum = 0.0f;
    for (int i = 0; i < IMU_CALIB_SAMPLES; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);  // lecture brute des 3 axes
        sum += (float)gz;
        delay(IMU_SAMPLE_MS);  // délai OK ici : on est dans setup(), pas dans loop()
    }
    s_gyro_z_bias = sum / (float)IMU_CALIB_SAMPLES;
    Serial.print(" biais Z = ");
    Serial.print(s_gyro_z_bias, 1);
    Serial.println(" LSB");

    // Initialise le timestamp pour le premier appel à imu_update()
    s_last_update_ms = millis();
    s_heading_deg    = 0.0f;
    s_imu_ready      = true;

    Serial.println("[IMU] Prêt — cap initialisé à 0°");
    return true;
}

// ── imu_update ────────────────────────────────────────────────
void imu_update() {
    if (!s_imu_ready) return;

    uint32_t now = millis();
    // dt = temps écoulé depuis la dernière mise à jour, en SECONDES
    // On divise par 1000 pour convertir ms → s
    float dt_s = (float)(now - s_last_update_ms) / 1000.0f;
    s_last_update_ms = now;

    // ── Lecture gyro Z brut ───────────────────────────────────
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // ── Soustraction du biais de calibration ─────────────────
    float gz_corrected = (float)gz - s_gyro_z_bias;

    // ── Conversion LSB → degrés/seconde ──────────────────────
    // À ±250°/s, chaque LSB = 1/131 °/s
    float gz_dps = gz_corrected / IMU_GYRO_SENSITIVITY;

    // ── Deadband (zone morte) ─────────────────────────────────
    // Si la vitesse angulaire est inférieure au seuil,
    // on considère que le robot est immobile et on n'intègre pas.
    // Cela évite que le cap dérive lentement quand le robot ne bouge pas.
    if (gz_dps > -IMU_DEADBAND_DPS && gz_dps < IMU_DEADBAND_DPS) {
        gz_dps = 0.0f;
    }

    // ── Intégration : vitesse angulaire × temps = angle ──────
    // angle (°) += vitesse (°/s) × dt (s)
    s_heading_deg += gz_dps * dt_s;
}

// ── imu_get_heading ───────────────────────────────────────────
float imu_get_heading() {
    return s_heading_deg;
}

// ── imu_reset_heading ─────────────────────────────────────────
void imu_reset_heading() {
    s_heading_deg = 0.0f;
    // On remet aussi le timestamp à jour pour que le prochain dt soit correct
    s_last_update_ms = millis();
}

// ── imu_rotation_complete ─────────────────────────────────────
bool imu_rotation_complete(float target_deg) {
    // On travaille sur la valeur absolue du cap :
    // une rotation à droite donne un cap négatif sur le gyro Z,
    // une rotation à gauche donne un cap positif.
    // La tolérance est définie dans config.h (IMU_ROTATION_TOLERANCE = 5°)
    float abs_heading = fabsf(s_heading_deg);
    float threshold   = fabsf(target_deg) - IMU_ROTATION_TOLERANCE;

    return (abs_heading >= threshold);
}
