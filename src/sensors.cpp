#include "sensors.h"
#include "config.h"
#include "calibration.h"
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
    // Valeurs parasites connues du VL53L0X :
    //   0     = erreur hardware
    //   8190  = out-of-range (objet trop proche ou angle)
    //   8191  = out-of-range variante
    //   65535 = timeout I2C
    // TOF_MAX_MM = distance max crédible dans notre labyrinthe (1200mm)
    if (raw == 0 || raw == 8190 || raw == 8191 || raw >= 65535 || raw > TOF_MAX_MM) {
        return 0;
    }
    return raw;
}

void sensors_read(ToFReadings& out) {
    out.front_left  = filter_reading(tof_fl.readRangeContinuousMillimeters());
    out.front_right = filter_reading(tof_fr.readRangeContinuousMillimeters());
    // Capteurs latéraux physiquement inversés sur le châssis → on croise les lectures
    out.side_left   = filter_reading(tof_sr.readRangeContinuousMillimeters());
    out.side_right  = filter_reading(tof_sl.readRangeContinuousMillimeters());
}

// ──────────────────────────────────────────────────────────────
//  sensors_detect_walls
//  Convertit des distances brutes en présence de murs.
//
//  Logique frontale : FL ET FR doivent tous les deux confirmer
//  (on évite les faux positifs dus à un mur de biais ou une poussière).
//
//  Logique latérale (capteurs à 45°) :
//   - Un seul capteur suffit (l'autre n'est pas sur ce côté).
//   - Seuil = TOF_WALL_SIDE_MM (≈ 160mm, à calibrer).
//   - Si le capteur renvoie 0 (invalide), on considère "pas de mur"
//     plutôt que "mur présent" pour éviter les faux positifs.
// ──────────────────────────────────────────────────────────────
WallDetection sensors_detect_walls(const ToFReadings& tof) {
    WallDetection w = { false, false, false };

    // ── Mur devant ────────────────────────────────────────────
    // Les deux capteurs frontaux doivent confirmer (évite les faux positifs
    // sur un mur de biais ou un obstacle ponctuel).
    w.front = (tof.front_left  > 0 && tof.front_left  < TOF_WALL_FRONT_MM &&
               tof.front_right > 0 && tof.front_right < TOF_WALL_FRONT_MM);

    // ── Rejet géométrique des faux murs latéraux ──────────────
    // Problème : si un mur frontal est à distance d, le capteur à 45°
    // homolatéral projette ce mur à d × √2 ≈ d × 1.414.
    //   Ex : FL=107mm → SL devrait lire ~151mm par projection frontale.
    //   Si SL=138mm < 151mm, est-ce un vrai mur gauche ou du bruit ?
    //
    // Règle : on déclare un vrai mur latéral SEULEMENT si la lecture
    // est inférieure à (d_frontal × 1.414 - TOF_SIDE_GEOM_MARGIN).
    // → Si capteur frontal invalide (0) : pas de rejet → seuil très grand.
    float fl = (tof.front_left  > 0) ? (float)tof.front_left  : 9999.0f;
    float fr = (tof.front_right > 0) ? (float)tof.front_right : 9999.0f;

    // Projection attendue du mur frontal sur chaque capteur 45°
    // (FL cross-check SL car ils sont du même côté gauche ; FR cross-check SR)
    float expected_sl = fl * 1.414f;
    float expected_sr = fr * 1.414f;

    // Mur gauche : valide + sous seuil calibré (opening_l) + non expliqué par le mur frontal
    // calib_get_opening_l() = distance au-delà de laquelle le passage est considéré ouvert
    bool sl_in_range = (tof.side_left  > 0 && tof.side_left  < calib_get_opening_l());
    w.left  = sl_in_range && ((float)tof.side_left  < expected_sl - TOF_SIDE_GEOM_MARGIN);

    // Mur droit : symétrique
    bool sr_in_range = (tof.side_right > 0 && tof.side_right < calib_get_opening_r());
    w.right = sr_in_range && ((float)tof.side_right < expected_sr - TOF_SIDE_GEOM_MARGIN);

    return w;
}
