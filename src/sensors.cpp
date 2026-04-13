#include "sensors.h"
#include "config.h"
#include <VL53L0X.h>
#include <Wire.h>

// Les 4 objets capteurs ToF (instances statiques, pas d'allocation dynamique)
static VL53L0X tof_fl;  // Front Left
static VL53L0X tof_fr;  // Front Right
static VL53L0X tof_sl;  // Side Left
static VL53L0X tof_sr;  // Side Right

// ──────────────────────────────────────────────────────────────
//  Fonction utilitaire : active un XSHUT, attend que le ToF
//  démarre (~2ms), puis lui assigne une adresse I2C unique.
//  Retourne false si le capteur ne répond pas.
// ──────────────────────────────────────────────────────────────
static bool init_one_tof(Adafruit_MCP23X17& mcp,
                         uint8_t xshut_pin,
                         VL53L0X& sensor,
                         uint8_t new_address)
{
    // Active le XSHUT (HIGH = actif)
    mcp.digitalWrite(xshut_pin, HIGH);
    delay(2);  // ← seul delay() autorisé : initialisation hardware unique

    sensor.setTimeout(TOF_TIMEOUT_MS);

    if (!sensor.init()) {
        Serial.print("[TOF] ERREUR init capteur @ xshut=");
        Serial.println(xshut_pin);
        return false;
    }

    // Réassigne l'adresse I2C (par défaut 0x29 au boot)
    sensor.setAddress(new_address);

    // Mode continu = lecture plus rapide, moins de latence I2C
    sensor.startContinuous(20);  // mesure toutes les 20ms

    Serial.print("[TOF] OK → adresse 0x");
    Serial.println(new_address, HEX);
    return true;
}

// ──────────────────────────────────────────────────────────────
//  sensors_init — Protocole XSHUT complet
// ──────────────────────────────────────────────────────────────
bool sensors_init(Adafruit_MCP23X17& mcp) {
    // Étape 1 : met tous les XSHUT en sortie et en LOW → reset tous les ToF
    mcp.pinMode(MCP_XSHUT_FL, OUTPUT);
    mcp.pinMode(MCP_XSHUT_FR, OUTPUT);
    mcp.pinMode(MCP_XSHUT_SL, OUTPUT);
    mcp.pinMode(MCP_XSHUT_SR, OUTPUT);

    mcp.digitalWrite(MCP_XSHUT_FL, LOW);
    mcp.digitalWrite(MCP_XSHUT_FR, LOW);
    mcp.digitalWrite(MCP_XSHUT_SL, LOW);
    mcp.digitalWrite(MCP_XSHUT_SR, LOW);
    delay(10);  // laisse le temps au reset de se stabiliser

    bool ok = true;

    // Étapes 2–5 : active UN capteur à la fois et lui donne son adresse
    // Ordre : GPA0=FR, GPA1=FL, GPA2=SL, GPA3=SR
    ok &= init_one_tof(mcp, MCP_XSHUT_FR, tof_fr, TOF_ADDR_FRONT_R);
    ok &= init_one_tof(mcp, MCP_XSHUT_FL, tof_fl, TOF_ADDR_FRONT_L);
    ok &= init_one_tof(mcp, MCP_XSHUT_SL, tof_sl, TOF_ADDR_SIDE_L);
    ok &= init_one_tof(mcp, MCP_XSHUT_SR, tof_sr, TOF_ADDR_SIDE_R);

    return ok;
}

// ──────────────────────────────────────────────────────────────
//  sensors_read — Lit les 4 capteurs, filtre les valeurs aberrantes
// ──────────────────────────────────────────────────────────────

// Valide une lecture : renvoie 0 si la valeur est hors plage physique
static uint16_t filter_reading(uint16_t raw) {
    // 65535 = timeout du capteur, 0 = erreur hardware
    // TOF_MAX_MM = distance max crédible dans notre labyrinthe
    if (raw == 0 || raw == 65535 || raw > TOF_MAX_MM) {
        return 0;
    }
    return raw;
}

void sensors_read(ToFReadings& out) {
    out.front_left  = filter_reading(tof_fl.readRangeContinuousMillimeters());
    out.front_right = filter_reading(tof_fr.readRangeContinuousMillimeters());
    out.side_left   = filter_reading(tof_sl.readRangeContinuousMillimeters());
    out.side_right  = filter_reading(tof_sr.readRangeContinuousMillimeters());
}
