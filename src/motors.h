#pragma once

// =============================================================
//  motors.h — Interface module moteurs (DRV8833)
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  Le DRV8833 contrôle 2 moteurs via des ponts en H.
//  Chaque moteur est piloté par 2 pins PWM (IN1, IN2) :
//    IN1=PWM, IN2=LOW  → avant
//    IN1=LOW,  IN2=PWM → arrière
//    IN1=HIGH, IN2=HIGH → frein (court-circuit bobine)
//    IN1=LOW,  IN2=LOW  → roue libre
//
//  ⚠️ Au boot ESP8266 : D3/D4 sont HIGH (pull-up) → frein natif OK
//                        D8 est LOW  (pull-down) → pas de mouvement OK
//
//  Usage :
//    motors_init();
//    motors_set(150, 150);    // avance à PWM=150 les deux moteurs
//    motors_set(-100, -100);  // recule
//    motors_stop();           // frein immédiat
//    motors_coast();          // roue libre (glisse)

// ── Initialisation ───────────────────────────────────────────
void motors_init();

// ── Commande vitesse ─────────────────────────────────────────
// pwm_left / pwm_right : [-255 .. +255]
//   positif = avant, négatif = arrière, 0 = frein
void motors_set(int pwm_left, int pwm_right);

// ── Frein immédiat ───────────────────────────────────────────
// Court-circuite les bobines → arrêt net (préférer pour l'urgence)
void motors_stop();

// ── Roue libre ───────────────────────────────────────────────
// Déconnecte les bobines → le robot glisse jusqu'à l'arrêt
void motors_coast();

// ── Rotation sur place ───────────────────────────────────────
// Gauche arrière + Droite avant = tourne à gauche (antihoraire)
// Gauche avant  + Droite arrière = tourne à droite (horaire)
// pwm : valeur positive [0..255]
void motors_turn_left(int pwm);
void motors_turn_right(int pwm);
