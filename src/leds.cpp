#include "leds.h"
#include "config.h"

void leds_init(Adafruit_MCP23X17& mcp) {
    mcp.pinMode(MCP_LED_RED,    OUTPUT);
    mcp.pinMode(MCP_LED_YELLOW, OUTPUT);
    mcp.pinMode(MCP_LED_GREEN,  OUTPUT);
    leds_all_off(mcp);
}

void led_set(Adafruit_MCP23X17& mcp, uint8_t pin, bool state) {
    mcp.digitalWrite(pin, state ? HIGH : LOW);
}

void leds_all_off(Adafruit_MCP23X17& mcp) {
    mcp.digitalWrite(MCP_LED_RED,    LOW);
    mcp.digitalWrite(MCP_LED_YELLOW, LOW);
    mcp.digitalWrite(MCP_LED_GREEN,  LOW);
}
