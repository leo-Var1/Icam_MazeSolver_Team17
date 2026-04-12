#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#include "config.h"
#include "leds.h"
#include "sensors.h"
#include "maze.h"

// ── Objet MCP23017 partagé entre les modules ──────────────────
Adafruit_MCP23X17 mcp;

// ── Machine d'états ───────────────────────────────────────────
enum RobotState {
    STATE_IDLE,           // LED verte fixe — attente commande
    STATE_RUN1_TREMAUX,   // LED jaune clignotante — exploration
    STATE_MAZE_COMPLETE,  // LED verte + jaune — carte ok
    STATE_RUN2_BFS,       // LED verte clignotante — résolution
    STATE_FINISHED,       // LED verte fixe + 3 bips
    STATE_EMERGENCY       // LED rouge fixe — STOP
};
static RobotState robot_state = STATE_IDLE;

// ── Timer pour lecture périodique des capteurs ────────────────
static uint32_t last_sensor_ms = 0;

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);  // laisse l'UART se stabiliser
    Serial.println("\n[BOOT] Robot Labyrinthe — Équipe 17");

    // 1) I2C en Fast Mode (400kHz) obligatoire avec 6 devices sur le bus
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    Serial.println("[I2C] Bus démarré @ 400kHz");

    // 2) MCP23017 — expandeur GPIO (adresse 0x20)
    if (!mcp.begin_I2C(I2C_MCP23017)) {
        Serial.println("[MCP] ERREUR : MCP23017 non trouvé !");
        // On reste bloqué : sans MCP, pas de XSHUT ni de LEDs
        while (true) { delay(1000); }
    }
    Serial.println("[MCP] MCP23017 OK");

    // 3) LEDs — init et allume rouge le temps du boot
    leds_init(mcp);
    led_set(mcp, MCP_LED_RED, true);

    // 4) VL53L0X — adressage XSHUT
    Serial.println("[TOF] Initialisation des 4 capteurs VL53L0X...");
    if (!sensors_init(mcp)) {
        Serial.println("[TOF] ERREUR : un ou plusieurs capteurs manquants !");
        // LED rouge clignote pour signaler l'erreur
        while (true) {
            led_set(mcp, MCP_LED_RED, true);
            delay(200);
            led_set(mcp, MCP_LED_RED, false);
            delay(200);
        }
    }
    Serial.println("[TOF] Tous les capteurs OK");

    // 5) Grille labyrinthe
    maze_init();
    Serial.println("[MAZE] Grille initialisée (5×5)");

    // Boot réussi : éteint rouge, allume verte
    led_set(mcp, MCP_LED_RED,   false);
    led_set(mcp, MCP_LED_GREEN, true);
    robot_state = STATE_IDLE;
    Serial.println("[BOOT] Système prêt — état IDLE");
    Serial.println("────────────────────────────────");
}

// ─────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // Phase 1 : lecture des 4 capteurs toutes les 100ms et affichage
    if (now - last_sensor_ms >= 100) {
        last_sensor_ms = now;

        ToFReadings tof;
        sensors_read(tof);

        Serial.print("[ToF] FL=");
        Serial.print(tof.front_left);
        Serial.print("mm  FR=");
        Serial.print(tof.front_right);
        Serial.print("mm  SL=");
        Serial.print(tof.side_left);
        Serial.print("mm  SR=");
        Serial.print(tof.side_right);
        Serial.println("mm");
    }
}
