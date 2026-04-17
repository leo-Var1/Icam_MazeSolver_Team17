// =============================================================
//  motors.cpp — Implémentation pilotage moteurs DRV8833
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include <Arduino.h>
#include "config.h"
#include "motors.h"

// ── Fonctions internes (non exposées) ─────────────────────────

// Pilote UN moteur à partir de deux pins IN1/IN2
// pwm dans [-255..+255] : positif=avant, négatif=arrière, 0=frein
static void motor_drive(uint8_t in1, uint8_t in2, int pwm) {
    if (pwm > 0) {
        // Avant : IN1=PWM, IN2=LOW
        analogWrite(in1, pwm);
        digitalWrite(in2, LOW);
    } else if (pwm < 0) {
        // Arrière : IN1=LOW, IN2=PWM (valeur absolue)
        digitalWrite(in1, LOW);
        analogWrite(in2, -pwm);
    } else {
        // Frein : IN1=HIGH, IN2=HIGH → court-circuit bobine
        digitalWrite(in1, HIGH);
        digitalWrite(in2, HIGH);
    }
}

// ── motors_init ───────────────────────────────────────────────
void motors_init() {
    // Configure les 4 pins moteur en sortie
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);

    // Frein par défaut (sécurité : robot immobile au démarrage)
    motors_stop();

    Serial.println("[MOT] Moteurs initialisés (frein actif)");
}

// ── motors_set ────────────────────────────────────────────────
void motors_set(int pwm_left, int pwm_right) {
    // Clamp : on s'assure de rester dans [-255..+255]
    pwm_left  = constrain(pwm_left,  -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);

    motor_drive(MOTOR_L_IN1, MOTOR_L_IN2, -pwm_left); // inversé : câblage moteur gauche en sens inverse
    motor_drive(MOTOR_R_IN1, MOTOR_R_IN2, pwm_right);
}

// ── motors_stop ───────────────────────────────────────────────
void motors_stop() {
    // Frein : HIGH/HIGH sur les deux moteurs
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, HIGH);
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, HIGH);
}

// ── motors_turn_left / motors_turn_right ─────────────────────
// Rotation sur place asymétrique :
//   - pwm_outer (paramètre) = roue extérieure (avant)
//   - PWM_TURN_INNER (config) = roue intérieure (arrière)
//
// La roue intérieure va en sens inverse et doit vaincre un frottement
// statique plus élevé → on lui applique un PWM plus fort (PWM_TURN_INNER).
// Si PWM_TURN_INNER est trop faible, le moteur stalle et le robot pivote
// autour de la roue intérieure au lieu de tourner sur place.
void motors_turn_left(int pwm_outer) {
    // Rotation antihoraire (vue du dessus) :
    //   Gauche = intérieure → arrière (fort)
    //   Droite = extérieure → avant (normal)
    motors_set(-PWM_TURN_INNER, pwm_outer);
}

void motors_turn_right(int pwm_outer) {
    // Rotation horaire :
    //   Gauche = extérieure → avant (normal)
    //   Droite = intérieure → arrière (fort)
    motors_set(pwm_outer, -PWM_TURN_INNER);
}

// ── motors_coast ──────────────────────────────────────────────
void motors_coast() {
    // Roue libre : LOW/LOW sur les deux moteurs
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
}
