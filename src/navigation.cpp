// =============================================================
//  navigation.cpp — Implémentation des primitives de mouvement
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include "navigation.h"
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "imu.h"
#include "pid.h"

// ── Type d'action en cours ───────────────────────────────────
enum NavAction {
    ACT_NONE,
    ACT_ADVANCE,
    ACT_TURN,
    ACT_TURN_SETTLING,   // phase d'attente avant rotation (stabilisation)
    ACT_AUTO_ALIGN
};

// ── Variables d'état internes ─────────────────────────────────
static NavState    s_state          = NAV_IDLE;
static NavAction   s_action         = ACT_NONE;
static int         s_turn_quarters  = 0;      // +1=droite, -1=gauche, +2=180°
static uint32_t    s_settle_start   = 0;      // timestamp début stabilisation

// ── Cache capteurs (mis à jour toutes les PID_SAMPLE_MS ms) ──
// Les VL53L0X tournent en mode continu (mesure toutes les 20ms).
// readRangeContinuousMillimeters() peut bloquer si appelé trop souvent.
// On ne lit les capteurs que toutes les PID_SAMPLE_MS = 20ms.
static ToFReadings s_tof            = { 0, 0, 0, 0 };
static uint32_t    s_last_tof_ms    = 0;

// ── nav_init ──────────────────────────────────────────────────
void nav_init() {
    s_state  = NAV_IDLE;
    s_action = ACT_NONE;
}

// =============================================================
//  NAV_START_ADVANCE
//  Lance l'avance d'une case (≈200mm via TICKS_PER_CELL).
//  Le PID encodeurs centre les deux roues.
// =============================================================
void nav_start_advance() {
    encoders_reset();
    pid_init();             // remet l'intégrateur à zéro
    motors_set(PWM_RUN1, PWM_RUN1);

    s_action = ACT_ADVANCE;
    s_state  = NAV_BUSY;
}

// ── Mise à jour de l'avance ───────────────────────────────────
static NavState update_advance() {
    uint32_t now = millis();

    // ── Lecture capteurs (non-bloquante : on réutilise le cache) ──
    // Les VL53L0X tournent en continu toutes les 20ms.
    // On ne relance une lecture que si au moins PID_SAMPLE_MS ms se sont écoulées.
    if (now - s_last_tof_ms >= (uint32_t)PID_SAMPLE_MS) {
        sensors_read(s_tof);
        s_last_tof_ms = now;
    }

    long tL = encoders_get_left();
    long tR = encoders_get_right();

    // ── Arrêt d'urgence : mur trop proche devant ──────────────
    if (s_tof.front_left  > 0 && s_tof.front_left  < TOF_STOP_FRONT_MM &&
        s_tof.front_right > 0 && s_tof.front_right < TOF_STOP_FRONT_MM) {
        motors_stop();
        Serial.println("[NAV] ARRET urgence — mur frontal < 50mm");
        s_state  = NAV_DONE;
        s_action = ACT_NONE;
        return NAV_DONE;
    }

    // ── Condition d'arrêt : distance atteinte ─────────────────
    long avg_ticks = (labs(tL) + labs(tR)) / 2;
    if (avg_ticks >= TICKS_PER_CELL) {
        motors_stop();
        s_state  = NAV_DONE;
        s_action = ACT_NONE;
        return NAV_DONE;
    }

    // ── PID encodeurs : correction dérive gauche/droite ───────
    int pwm_l, pwm_r;
    pid_update(tL, tR, PWM_RUN1, pwm_l, pwm_r);
    motors_set(pwm_l, pwm_r);

    return NAV_BUSY;
}

// =============================================================
//  NAV_START_TURN
//  Rotation sur place, guidée par l'IMU (gyroscope).
//  Phase 1 : freinage + stabilisation (TURN_SETTLE_MS)
//  Phase 2 : rotation jusqu'à angle cible ±IMU_ROTATION_TOLERANCE
// =============================================================
void nav_start_turn(int quarters) {
    motors_stop();              // frein immédiat avant de tourner
    s_turn_quarters = quarters;
    s_settle_start  = millis();
    s_action = ACT_TURN_SETTLING;
    s_state  = NAV_BUSY;
}

// ── Mise à jour de la rotation ────────────────────────────────
static NavState update_turn() {
    // Phase 1 : attente de stabilisation après freinage
    if (s_action == ACT_TURN_SETTLING) {
        if (millis() - s_settle_start >= TURN_SETTLE_MS) {
            // Le robot est arrêté → reset IMU et démarrer la rotation
            imu_reset_heading();
            if (s_turn_quarters > 0) {
                motors_turn_right(PWM_TURN);
            } else {
                motors_turn_left(PWM_TURN);
            }
            s_action = ACT_TURN;
        }
        return NAV_BUSY;
    }

    // Phase 2 : détection fin de rotation via IMU
    float target_deg = abs(s_turn_quarters) * 90.0f;
    if (imu_rotation_complete(target_deg)) {
        motors_stop();
        float actual = imu_get_heading();
        Serial.print("[NAV] Virage terminé — angle mesuré = ");
        Serial.print(abs(actual), 1);
        Serial.print("°  (cible = ");
        Serial.print(target_deg, 0);
        Serial.println("°)");

        s_state  = NAV_DONE;
        s_action = ACT_NONE;
        return NAV_DONE;
    }

    return NAV_BUSY;
}

// =============================================================
//  NAV_START_AUTO_ALIGN
//  Si un mur est présent devant (FL et FR < TOF_WALL_FRONT_MM) :
//   1. Corrige l'angle : micro-rotation jusqu'à FL ≈ FR
//   2. Ajuste la distance : avance/recule jusqu'à (FL+FR)/2 ≈ TOF_ALIGN_TARGET_MM
//  Sinon : NAV_DONE immédiatement (pas de mur = rien à faire)
// =============================================================
void nav_start_auto_align() {
    s_action = ACT_AUTO_ALIGN;
    s_state  = NAV_BUSY;
}

// ── Mise à jour de l'auto-alignement ─────────────────────────
static NavState update_auto_align() {
    uint32_t now = millis();
    if (now - s_last_tof_ms >= (uint32_t)PID_SAMPLE_MS) {
        sensors_read(s_tof);
        s_last_tof_ms = now;
    }
    float fl = (float)s_tof.front_left;
    float fr = (float)s_tof.front_right;

    // Pas de mur devant → rien à faire
    if (fl == 0 || fl > TOF_WALL_FRONT_MM ||
        fr == 0 || fr > TOF_WALL_FRONT_MM) {
        motors_stop();
        s_state  = NAV_DONE;
        s_action = ACT_NONE;
        return NAV_DONE;
    }

    // Étape 1 : correction angulaire (FL doit être ≈ FR)
    float angle_error = fl - fr;
    if (fabsf(angle_error) > TOF_ALIGN_TOL_MM) {
        // Gain proportionnel simple : 1mm d'erreur → 2 unités PWM
        int turn_pwm = (int)(angle_error * 2.0f);
        turn_pwm = constrain(turn_pwm, -PWM_DIAG, PWM_DIAG);
        // Si FL > FR : tourner légèrement vers la droite pour se rapprocher du mur gauche
        motors_set(-turn_pwm, turn_pwm);
        return NAV_BUSY;
    }

    // Étape 2 : correction de distance
    float avg_dist  = (fl + fr) / 2.0f;
    float dist_err  = avg_dist - (float)TOF_ALIGN_TARGET_MM;

    if (fabsf(dist_err) > 3.0f) {
        // dist_err > 0 → trop loin → avancer ; dist_err < 0 → trop proche → reculer
        int move_pwm = (int)(dist_err * 1.5f);
        move_pwm = constrain(move_pwm, -PWM_DIAG, PWM_DIAG);
        motors_set(move_pwm, move_pwm);
        return NAV_BUSY;
    }

    // Alignement terminé
    motors_stop();
    Serial.print("[NAV] Auto-align OK — dist=");
    Serial.print(avg_dist, 0);
    Serial.println("mm");
    s_state  = NAV_DONE;
    s_action = ACT_NONE;
    return NAV_DONE;
}

// =============================================================
//  NAV_UPDATE — dispatcher principal
//  À appeler dans loop() à chaque itération.
// =============================================================
NavState nav_update() {
    switch (s_action) {
        case ACT_ADVANCE:
            return update_advance();

        case ACT_TURN:
        case ACT_TURN_SETTLING:
            return update_turn();

        case ACT_AUTO_ALIGN:
            return update_auto_align();

        default:
            return NAV_IDLE;
    }
}

// ── nav_abort ────────────────────────────────────────────────
void nav_abort() {
    motors_stop();
    s_state  = NAV_IDLE;
    s_action = ACT_NONE;
    Serial.println("[NAV] Mouvement annulé (abort)");
}

// ── nav_get_state ────────────────────────────────────────────
NavState nav_get_state() {
    return s_state;
}
