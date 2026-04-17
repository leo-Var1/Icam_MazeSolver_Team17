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
#include "web_ui.h"
#include "calibration.h"

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

// ── Mode test murs (touche 'w') ───────────────────────────────
// Quand actif : affiche distances + murs détectés toutes les 200ms
static bool      s_wall_test_mode = false;
static uint32_t  s_last_wall_ms   = 0;

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
    calibration_init();  // charge /calib.json ou applique les défauts

    // 6) Moteurs + encodeurs + navigation
    motors_init();
    encoders_init();
    nav_init();

    // 7) Grille labyrinthe
    maze_init();
    Serial.println("[MAZE] Grille initialisée (5×5)");

    // 8) IHM Web (Access Point + serveur async)
    //    À faire APRÈS maze_init() car les handlers lisent la grille.
    web_ui_init();

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
    Serial.println("  w  → toggle mode test ToF (distances + murs toutes les 200ms)");
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

    // ── Consommation des commandes reçues depuis l'IHM Web ────
    // Les handlers async ne font QUE poser des flags.
    // Ici (contexte loop), on peut appeler les fonctions nav_* en sécurité.
    {
        WebCmd cmd = web_ui_poll_cmd();
        NavState cur = nav_get_state();
        bool libre = (cur == NAV_IDLE || cur == NAV_DONE);

        switch (cmd) {
            case WEB_CMD_STOP:
                // Priorité absolue : frein immédiat quel que soit l'état
                nav_abort();
                robot_state = STATE_EMERGENCY;
                led_set(mcp, MCP_LED_YELLOW, false);
                led_set(mcp, MCP_LED_GREEN,  false);
                led_set(mcp, MCP_LED_RED,    true);
                Serial.println("[WEB] STOP urgence");
                break;

            case WEB_CMD_START1:
                // TODO phase 5 : lancer Trémaux — pour l'instant, juste l'état+LED
                robot_state = STATE_RUN1_TREMAUX;
                led_set(mcp, MCP_LED_GREEN,  false);
                led_set(mcp, MCP_LED_YELLOW, true);
                Serial.println("[WEB] Run 1 démarré (stub)");
                break;

            case WEB_CMD_START2:
                // TODO phase 6 : lancer BFS vers (target_row, target_col)
                robot_state = STATE_RUN2_BFS;
                Serial.print("[WEB] Run 2 démarré (stub) — cible (");
                Serial.print(web_ui_get_target_row()); Serial.print(",");
                Serial.print(web_ui_get_target_col()); Serial.println(")");
                break;

            case WEB_CMD_MOVE_UP:
                if (libre) { led_set(mcp, MCP_LED_YELLOW, true); nav_start_advance(); }
                break;
            case WEB_CMD_MOVE_DOWN:
                if (libre) { led_set(mcp, MCP_LED_YELLOW, true); nav_start_turn(2); }
                break;
            case WEB_CMD_MOVE_LEFT:
                if (libre) { led_set(mcp, MCP_LED_YELLOW, true); nav_start_turn(-1); }
                break;
            case WEB_CMD_MOVE_RIGHT:
                if (libre) { led_set(mcp, MCP_LED_YELLOW, true); nav_start_turn(1); }
                break;

            case WEB_CMD_CALIB_CENTER:
                if (robot_state == STATE_IDLE) { calib_capture_center();    calib_save(); }
                else Serial.println("[CALIB] ignoré: pas en IDLE");
                break;

            case WEB_CMD_CALIB_TURN:
                if (robot_state == STATE_IDLE) { calib_capture_turn();      calib_save(); }
                else Serial.println("[CALIB] ignoré: pas en IDLE");
                break;

            case WEB_CMD_CALIB_OPENING_L:
                if (robot_state == STATE_IDLE) { calib_capture_opening_l(); calib_save(); }
                else Serial.println("[CALIB] ignoré: pas en IDLE");
                break;

            case WEB_CMD_CALIB_OPENING_R:
                if (robot_state == STATE_IDLE) { calib_capture_opening_r(); calib_save(); }
                else Serial.println("[CALIB] ignoré: pas en IDLE");
                break;

            case WEB_CMD_CALIB_RESET:
                if (robot_state == STATE_IDLE) calib_reset_defaults();
                else Serial.println("[CALIB] ignoré: pas en IDLE");
                break;

            case WEB_CMD_NONE:
            default:
                break;
        }
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

            } else if (c == 'w') {
                // Toggle mode test ToF
                s_wall_test_mode = !s_wall_test_mode;
                if (s_wall_test_mode) {
                    Serial.println("[TOF-TEST] Mode murs ACTIF (200ms) — appuie 'w' pour arrêter");
                    Serial.print("[TOF-TEST] Seuils : FRONT<"); Serial.print(TOF_WALL_FRONT_MM);
                    Serial.print("mm  SIDE<");                  Serial.print(TOF_WALL_SIDE_MM);
                    Serial.println("mm");
                } else {
                    Serial.println("[TOF-TEST] Mode murs INACTIF");
                }
            }
        }
    }

    // ── Affichage ToF + cap @ 500ms + push état IHM ──────────
    // Mise à jour toujours (pas seulement en IDLE) pour que l'IHM reste vivante
    // pendant les mouvements. À 500ms d'intervalle, la lecture I2C ne perturbe
    // pas le PID (qui tourne à 50Hz = 20ms).
    if (now - last_sensor_ms >= 500) {
        last_sensor_ms = now;

        ToFReadings tof;
        sensors_read(tof);
        float heading = imu_get_heading();

        Serial.print("[SENS] FL="); Serial.print(tof.front_left);
        Serial.print("mm FR=");    Serial.print(tof.front_right);
        Serial.print("mm SL=");    Serial.print(tof.side_left);
        Serial.print("mm SR=");    Serial.print(tof.side_right);
        Serial.print("mm  cap=");  Serial.print(heading, 1);
        Serial.println("°");

        // Push vers l'IHM Web
        WebState ws;
        ws.robot_state = (uint8_t)robot_state;
        ws.heading     = heading;
        ws.tof_fl      = tof.front_left;
        ws.tof_fr      = tof.front_right;
        ws.tof_sl      = tof.side_left;
        ws.tof_sr      = tof.side_right;
        web_ui_set_state(ws);
    }

    // ── Mode test murs @ 200ms ────────────────────────────────
    // Indépendant du timer 500ms — période plus courte pour voir les
    // changements en temps réel quand on approche le robot d'un mur.
    if (s_wall_test_mode && (now - s_last_wall_ms >= 200)) {
        s_last_wall_ms = now;

        ToFReadings tof;
        sensors_read(tof);
        WallDetection walls = sensors_detect_walls(tof);

        // Ligne 1 : distances brutes (pour calibrer les seuils)
        Serial.print("[TOF] FL="); Serial.print(tof.front_left);
        Serial.print("  FR=");     Serial.print(tof.front_right);
        Serial.print("  SL=");     Serial.print(tof.side_left);
        Serial.print("  SR=");     Serial.println(tof.side_right);

        // Ligne 2 : murs détectés (affichage graphique ASCII)
        //   [X] = mur présent   [ ] = pas de mur
        Serial.print("[MUR]       ");
        Serial.println(walls.front ? "[ DEVANT ]" : "[        ]");
        Serial.print("[MUR] ");
        Serial.print(walls.left  ? "[GAUCHE]" : "[      ]");
        Serial.print("  robot  ");
        Serial.println(walls.right ? "[DROITE]" : "[      ]");
        Serial.println("---");
    }
}
