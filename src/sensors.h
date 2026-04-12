#pragma once
#include <stdint.h>
#include <Adafruit_MCP23X17.h>

// =============================================================
//  sensors.h — Initialisation et lecture des 4 VL53L0X
//
//  Ordre d'init (protocole XSHUT) :
//   1. Tous les XSHUT à LOW  → tous les ToF en reset
//   2. Activer XSHUT_FL seul → assigner adresse TOF_ADDR_FRONT_L
//   3. Activer XSHUT_FR seul → assigner adresse TOF_ADDR_FRONT_R
//   4. Activer XSHUT_SL seul → assigner adresse TOF_ADDR_SIDE_L
//   5. Activer XSHUT_SR seul → assigner adresse TOF_ADDR_SIDE_R
// =============================================================

// Structure contenant les 4 distances en mm (0 = capteur injoignable)
struct ToFReadings {
    uint16_t front_left;   // mm
    uint16_t front_right;  // mm
    uint16_t side_left;    // mm (45°)
    uint16_t side_right;   // mm (45°)
};

// Initialise les 4 VL53L0X avec adressage dynamique via XSHUT
// Retourne true si tous les capteurs ont répondu
bool sensors_init(Adafruit_MCP23X17& mcp);

// Lit les 4 capteurs et remplit la structure
// Les valeurs hors plage (> TOF_MAX_MM ou 65535) sont remplacées par 0
void sensors_read(ToFReadings& out);
