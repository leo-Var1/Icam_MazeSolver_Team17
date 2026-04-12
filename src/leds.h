#pragma once
#include <Adafruit_MCP23X17.h>

// =============================================================
//  leds.h — Contrôle des 3 LEDs via MCP23017
// =============================================================

// Initialise les pins LED en sortie (à appeler après mcp.begin())
void leds_init(Adafruit_MCP23X17& mcp);

// Allume / éteint une LED individuelle
void led_set(Adafruit_MCP23X17& mcp, uint8_t pin, bool state);

// Éteint toutes les LEDs
void leds_all_off(Adafruit_MCP23X17& mcp);
