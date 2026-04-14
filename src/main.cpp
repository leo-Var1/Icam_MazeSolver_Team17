#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#include "config.h"
#include "leds.h"
#include "sensors.h"
#include "maze.h"
#include "imu.h"
#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "navigation.h"

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

// ── Timers pour les lectures périodiques ─────────────────────
static uint32_t last_sensor_ms = 0;
static uint32_t last_imu_ms    = 0;

// ── Lecture des touches fléchées (séquences ANSI) ─────────────
// Les touches fléchées envoient 3 octets : ESC (0x1B) + '[' + lettre
//   ↑ = ESC[A   ↓ = ESC[B   → = ESC[C   ← = ESC[D
// On détecte l'ESC puis on lit les 2 octets suivants.
static bool s_esc_received = false;  // vrai quand on a reçu ESC, on attend '['
static bool s_bracket_received = false; // vrai quand on a reçu ESC+'[', on attend la lettre

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[BOOT] Robot Labyrinthe — Équipe 17");

    // 1) I2C en Fast Mode (400kHz) obligatoire avec 6 devices sur le bus
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    Serial.println("[I2C] Bus démarré @ 400kHz");

    // 2) MCP23017
    if (!mcp.begin_I2C(I2C_MCP23017)) {
        Serial.println("[MCP] ERREUR : MCP23017 non trouvé !");
        while (true) { delay(1000); }
    }
    Serial.println("[MCP] MCP23017 OK");

    // 3) LEDs — rouge pendant le boot
    leds_init(mcp);
    led_set(mcp, MCP_LED_RED, true);

    // 4) MPU6050 — calibration gyro (~1s, robot immobile)
    if (!imu_init()) {
        Serial.println("[IMU] AVERTISSEMENT : IMU non disponible");
    }

    // 5) VL53L0X
    Serial.println("[TOF] Initialisation des 4 capteurs VL53L0X...");
    sensors_init(mcp);

    // 6) Moteurs + encodeurs + navigation
    motors_init();
    encoders_init();
    nav_init();

    // 7) Grille labyrinthe
    maze_init();
    Serial.println("[MAZE] Grille initialisée (5×5)");

    // Boot OK
    led_set(mcp, MCP_LED_RED,   false);
    led_set(mcp, MCP_LED_GREEN, true);
    robot_state = STATE_IDLE;
    Serial.println("[BOOT] Système prêt — état IDLE");
    Serial.println("────────────────────────────────");
    Serial.println("[TEST] Commandes Serial disponibles :");
    Serial.println("  ↑  (flèche haut)   → avance 1 case (~200mm)");
    Serial.println("  ↓  (flèche bas)    → demi-tour 180°");
    Serial.println("  →  (flèche droite) → virage DROITE 90°");
    Serial.println("  ←  (flèche gauche) → virage GAUCHE 90°");
    Serial.println("  s  → STOP d'urgence");
    Serial.println("  l  → moteur GAUCHE seul (diagnostic)");
    Serial.println("  R  → moteur DROIT seul (diagnostic)");
    Serial.println("  p  → affiche les numéros de pins moteurs");
    Serial.println("  (affichage live automatique pendant les mouvements)");
}

// ─────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // ── Mise à jour IMU @ 50Hz ────────────────────────────────
    // Doit être appelée très régulièrement pour que l'intégration gyro soit précise
    if (now - last_imu_ms >= IMU_SAMPLE_MS) {
        last_imu_ms = now;
        imu_update();
    }

    // ── Navigation : mise à jour du mouvement en cours ────────
    // nav_update() gère en interne PID, détection fin de mouvement,
    // et l'affichage Serial live. On récupère l'état pour les LEDs.
    NavState nav_st = nav_update();
    if (nav_st == NAV_DONE) {
        led_set(mcp, MCP_LED_YELLOW, false);
        led_set(mcp, MCP_LED_GREEN, true);
    }

    // ── Lecture touches Serial (touches fléchées ANSI + touches simples) ──
    // Les touches fléchées génèrent 3 octets : ESC (0x1B), '[', puis A/B/C/D.
    // On lit octet par octet avec un mini automate d'état (s_esc_received,
    // s_bracket_received) pour ne pas bloquer la boucle.
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (s_bracket_received) {
            // 3ème octet de la séquence fléchée → action
            s_esc_received     = false;
            s_bracket_received = false;

            // On n'accepte les commandes de mouvement que si le robot est libre
            NavState cur = nav_get_state();
            bool libre = (cur == NAV_IDLE || cur == NAV_DONE);

            if (c == 'A') {
                // ↑ — avance 1 case
                if (libre) {
                    led_set(mcp, MCP_LED_YELLOW, true);
                    nav_start_advance();
                } else {
                    Serial.println("[TEST] Mouvement en cours, ignoré");
                }
            } else if (c == 'B') {
                // ↓ — demi-tour 180°
                if (libre) {
                    led_set(mcp, MCP_LED_YELLOW, true);
                    nav_start_turn(2);
                } else {
                    Serial.println("[TEST] Mouvement en cours, ignoré");
                }
            } else if (c == 'C') {
                // → — virage droite 90°
                if (libre) {
                    led_set(mcp, MCP_LED_YELLOW, true);
                    nav_start_turn(1);
                } else {
                    Serial.println("[TEST] Mouvement en cours, ignoré");
                }
            } else if (c == 'D') {
                // ← — virage gauche 90°
                if (libre) {
                    led_set(mcp, MCP_LED_YELLOW, true);
                    nav_start_turn(-1);
                } else {
                    Serial.println("[TEST] Mouvement en cours, ignoré");
                }
            }

        } else if (s_esc_received) {
            // 2ème octet : doit être '[' pour continuer la séquence fléchée
            if (c == '[') {
                s_bracket_received = true;
            } else {
                s_esc_received = false;  // séquence invalide → on ignore
            }

        } else if (c == 0x1B) {
            // 1er octet ESC → début possible d'une séquence fléchée
            s_esc_received = true;

        } else {
            // ── Touches simples (non-fléchées) ────────────────
            if (c == 's') {
                // Arrêt d'urgence
                nav_abort();
                led_set(mcp, MCP_LED_YELLOW, false);
                led_set(mcp, MCP_LED_RED,    true);
                delay(200);
                led_set(mcp, MCP_LED_RED,    false);
                led_set(mcp, MCP_LED_GREEN,  true);
                Serial.println("[TEST] STOP");

            } else if (c == 'l') {
                // Moteur GAUCHE seul (diagnostic câblage)
                nav_abort();
                Serial.println("[DIAG] Moteur GAUCHE seul");
                analogWrite(MOTOR_L_IN1, 120);
                digitalWrite(MOTOR_L_IN2, LOW);

            } else if (c == 'R') {
                // Moteur DROIT seul (diagnostic câblage)
                nav_abort();
                Serial.println("[DIAG] Moteur DROIT seul");
                analogWrite(MOTOR_R_IN1, 120);
                digitalWrite(MOTOR_R_IN2, LOW);

            } else if (c == 'p') {
                // Affiche les pins pour vérifier le câblage
                Serial.println("[DIAG] Pins moteurs :");
                Serial.print("  MOTOR_L_IN1 = pin "); Serial.println(MOTOR_L_IN1);
                Serial.print("  MOTOR_L_IN2 = pin "); Serial.println(MOTOR_L_IN2);
                Serial.print("  MOTOR_R_IN1 = pin "); Serial.println(MOTOR_R_IN1);
                Serial.print("  MOTOR_R_IN2 = pin "); Serial.println(MOTOR_R_IN2);
            }
        }
    }

    // ── Affichage ToF + cap @ 500ms quand le robot est à l'arrêt ─
    // On réduit la fréquence pour ne pas noyer l'affichage live de la navigation
    if (nav_get_state() == NAV_IDLE && now - last_sensor_ms >= 500) {
        last_sensor_ms = now;

        ToFReadings tof;
        sensors_read(tof);
        float heading = imu_get_heading();

        Serial.print("[IDLE] FL="); Serial.print(tof.front_left);
        Serial.print("mm FR=");    Serial.print(tof.front_right);
        Serial.print("mm SL=");    Serial.print(tof.side_left);
        Serial.print("mm SR=");    Serial.print(tof.side_right);
        Serial.print("mm  cap=");  Serial.print(heading, 1);
        Serial.println("°");
    }
}
