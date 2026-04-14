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
    ACT_TURN,            // rotation rapide (PWM_TURN)
    ACT_TURN_BRAKE,      // frein court entre phase rapide et lente (tue l'inertie)
    ACT_TURN_SLOW,       // rotation lente (PWM_TURN_SLOW) — phase de précision
    ACT_TURN_REVERSE,    // impulsion inverse brève pour corriger le dépassement résiduel
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

// ── Timer pour l'affichage Serial live ───────────────────────
// Affiche l'état du mouvement en cours toutes les PID_SAMPLE_MS ms
static uint32_t    s_last_log_ms    = 0;

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
    s_last_log_ms = millis();
    motors_set(PWM_RUN1, PWM_RUN1);

    s_action = ACT_ADVANCE;
    s_state  = NAV_BUSY;
    Serial.println("[ADV] Avance 1 case — démarrage");
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

    // ── Affichage live @ PID_SAMPLE_MS ────────────────────────
    if (now - s_last_log_ms >= (uint32_t)PID_SAMPLE_MS) {
        s_last_log_ms = now;
        Serial.print("[ADV] t="); Serial.print(now);
        Serial.print("ms  tL="); Serial.print(tL);
        Serial.print("  tR=");   Serial.print(tR);
        Serial.print("  ecart="); Serial.print(tL - tR);
        Serial.print("  pwm=");  Serial.print(pwm_l);
        Serial.print("/");       Serial.println(pwm_r);
    }

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
    encoders_reset();           // reset encodeurs — on va s'en servir pour la décélération
    s_turn_quarters = quarters;
    s_settle_start  = millis();
    s_action = ACT_TURN_SETTLING;
    s_state  = NAV_BUSY;
}

// ── Mise à jour de la rotation ────────────────────────────────
// Phases :
//   SETTLING → robot immobile, on attend TURN_SETTLE_MS avant de démarrer
//   TURN     → rotation rapide (PWM_TURN) jusqu'à TICKS_PER_90DEG encodeurs
//   BRAKE    → frein actif TURN_BRAKE_MS pour tuer l'inertie
//   REVERSE  → impulsion inverse TURN_REVERSE_MS pour corriger le dépassement
static NavState update_turn() {
    uint32_t now = millis();

    // ── Phase de stabilisation ─────────────────────────────────
    if (s_action == ACT_TURN_SETTLING) {
        if (now - s_settle_start >= TURN_SETTLE_MS) {
            // Le robot est bien arrêté → reset IMU + redémarrer encodeurs proprement
            imu_reset_heading();
            encoders_reset();
            s_last_log_ms = now;
            if (s_turn_quarters > 0) {
                motors_turn_right(PWM_TURN);
            } else {
                motors_turn_left(PWM_TURN);
            }
            s_action = ACT_TURN;
            Serial.println("[TURN] Démarrage rotation rapide");
        }
        return NAV_BUSY;
    }

    // ── Lecture commune encodeurs (moyenne des deux roues) ─────
    // Pour une rotation sur place, une roue avance et l'autre recule.
    // On prend la valeur absolue de chaque côté puis on moyenne.
    long tL = encoders_get_left();
    long tR = encoders_get_right();
    long avg_ticks = (labs(tL) + labs(tR)) / 2;

    // Cibles scalées selon le nombre de quarts de tour (90°, 180°…)
    int nb = abs(s_turn_quarters);
    long ticks_decel   = (long)nb * TICKS_TURN_DECEL;
    long ticks_target  = (long)nb * TICKS_PER_90DEG;
    long ticks_fallback = ticks_target + ticks_target / 10;  // 110%
    float target_deg   = nb * 90.0f;

    // ── Phase rapide → décélération ────────────────────────────
    if (s_action == ACT_TURN) {
        // Correction symétrie : ticks_left + ticks_right doit être ≈ 0
        // Si > 0 : roue gauche en avance → ralentir gauche / accélérer droite
        // Kp volontairement faible (0.3) : on veut juste équilibrer, pas osciller
        if (now - s_last_log_ms >= (uint32_t)PID_SAMPLE_MS) {
            s_last_log_ms = now;
            float err = (float)(tL + tR);
            int corr = (int)(1.3f * err);
            corr = constrain(corr, -40, 40);
            if (s_turn_quarters > 0) {
                // virage droite : gauche avant (+), droite arrière (-)
                motors_set(PWM_TURN - corr, -(PWM_TURN + corr));
            } else {
                // virage gauche : droite avant (+), gauche arrière (-)
                motors_set(-(PWM_TURN + corr), PWM_TURN - corr);
            }
            Serial.print("[TURN] gyro="); Serial.print(fabsf(imu_get_heading()), 1);
            Serial.print("°  ticks=");   Serial.print(avg_ticks);
            Serial.print("  sym_err=");  Serial.print(tL + tR);
            Serial.println("  phase=RAPIDE");
        }

        if (avg_ticks >= ticks_target) {
            motors_stop();
            s_settle_start = now;
            s_action = ACT_TURN_BRAKE;
            Serial.println("[TURN] → Frein");
        }
        return NAV_BUSY;
    }

    // ── Frein ─────────────────────────────────────────────────
    if (s_action == ACT_TURN_BRAKE) {
        if (now - s_settle_start >= TURN_BRAKE_MS) {
            float actual = fabsf(imu_get_heading());
            Serial.print("[TURN] Après frein — gyro="); Serial.print(actual, 1);
            Serial.print("°  ticks="); Serial.print(avg_ticks);
            Serial.println(" → impulsion inverse...");
            if (s_turn_quarters > 0) {
                motors_turn_left(TURN_REVERSE_PWM);
            } else {
                motors_turn_right(TURN_REVERSE_PWM);
            }
            s_settle_start = now;
            s_action = ACT_TURN_REVERSE;
        }
        return NAV_BUSY;
    }

    // ── Impulsion inverse ──────────────────────────────────────
    if (s_action == ACT_TURN_REVERSE) {
        if (now - s_settle_start >= TURN_REVERSE_MS) {
            motors_stop();
            float actual = fabsf(imu_get_heading());
            Serial.print("[TURN] TERMINÉ — gyro="); Serial.print(actual, 1);
            Serial.print("°  cible="); Serial.print(target_deg, 0);
            Serial.println("°");
            s_state  = NAV_DONE;
            s_action = ACT_NONE;
            return NAV_DONE;
        }
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
        case ACT_TURN_BRAKE:
        case ACT_TURN_REVERSE:
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
