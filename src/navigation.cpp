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
#include "calibration.h"

// ── Type d'action en cours ───────────────────────────────────
enum NavAction {
    ACT_NONE,
    ACT_ADVANCE,
    ACT_REVERSE_ALIGN,   // pré-alignement frontal AVANT le reverse (cul-de-sac)
    ACT_REVERSE,         // marche arrière d'une case
    ACT_TURN_PRE_ALIGN,  // auto-align frontal AVANT le pivot (si mur devant)
    ACT_TURN_SETTLING,   // stabilisation avant de commencer la rotation
    ACT_TURN_PIVOT,      // pivot sur place (90° ou 180°), arrêt sur ticks (IMU = garde)
    ACT_AUTO_ALIGN,
    ACT_SMOOTH_45_1,     // Étape 1 : premier pivot 45°
    ACT_SMOOTH_SETTLE_1, // Étape 2 : pause
    ACT_SMOOTH_MOVE,     // Étape 3 : avance 40mm
    ACT_SMOOTH_SETTLE_2, // Étape 4 : pause
    ACT_SMOOTH_45_2,     // Étape 5 : second pivot 45°
    ACT_SMOOTH_ALIGN,    // Étape 6 : recalage final (legacy)
    ACT_SMOOTH_CENTER    // Étape 7 : centrage à 77mm du mur frontal
};

// ── Variables d'état internes ─────────────────────────────────
static NavState    s_state          = NAV_IDLE;
static NavAction   s_action         = ACT_NONE;
static int         s_turn_quarters  = 0;      // +1=droite, -1=gauche, +2=180°
static int         s_smooth_dir     = 0;      // +1=droite, -1=gauche (virage 45-40-45)
static uint32_t    s_settle_start   = 0;      // timestamp début stabilisation

// ── Cache capteurs (mis à jour toutes les PID_SAMPLE_MS ms) ──
// Les VL53L0X tournent en mode continu (mesure toutes les 20ms).
// readRangeContinuousMillimeters() peut bloquer si appelé trop souvent.
// On ne lit les capteurs que toutes les PID_SAMPLE_MS = 20ms.
static ToFReadings s_tof            = { 0, 0, 0, 0 };
static uint32_t    s_last_tof_ms    = 0;

// ── Snapshot murs au milieu de la case ───────────────────────
// Capteurs à 45° : si on lit les murs à la FIN de la case, on voit
// déjà la case suivante. On snapshote à WALL_SNAP_PCT% de la case.
static WallDetection s_wall_snap       = { false, false, false };
static bool          s_wall_snap_valid = false;

// ── Timer pour l'affichage Serial live ───────────────────────
// Affiche l'état du mouvement en cours toutes les PID_SAMPLE_MS ms
static uint32_t    s_last_log_ms    = 0;

// ── Prototypes des fonctions internes ────────────────────────
static NavState update_advance();
static NavState update_reverse_align();
static NavState update_reverse();
static NavState update_turn();
static NavState update_auto_align();
static NavState update_smooth_turn();
static void     start_pivot();
static void     apply_pivot_pwm(long avg_ticks, long target_ticks);
static int      pivot_base_pwm(long avg_ticks, long target_ticks);

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
    s_wall_snap_valid = false;  // reset snapshot pour cette nouvelle case
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
    // UN seul capteur valide suffit pour l'urgence (sécurité maximale)
    {
        bool fl_danger = (s_tof.front_left  > 0 && s_tof.front_left  < TOF_STOP_FRONT_MM);
        bool fr_danger = (s_tof.front_right > 0 && s_tof.front_right < TOF_STOP_FRONT_MM);
        if (fl_danger || fr_danger) {
            motors_stop();
            Serial.println("[NAV] ARRET urgence — mur frontal < 60mm");
            s_state  = NAV_DONE;
            s_action = ACT_NONE;
            return NAV_DONE;
        }
    }

    // ── Arrêt calibré : mur devant à distance TURN ───────────
    // Calcul de la distance frontale à partir de UN ou DEUX capteurs.
    // Si les deux sont valides → moyenne. Sinon → on prend celui qui est valide.
    // Un seul capteur peut suffire si l'autre timeout à courte portée.
    {
        bool fl_ok = (s_tof.front_left  > 0 && s_tof.front_left  < TOF_MAX_MM);
        bool fr_ok = (s_tof.front_right > 0 && s_tof.front_right < TOF_MAX_MM);
        int front_dist = 0;
        if      (fl_ok && fr_ok) front_dist = ((int)s_tof.front_left + (int)s_tof.front_right) / 2;
        else if (fl_ok)          front_dist = s_tof.front_left;
        else if (fr_ok)          front_dist = s_tof.front_right;

        if (front_dist > 0 && front_dist <= calib_get_turn()) {
            motors_stop();
            Serial.print("[NAV] STOP calibré TURN — front=");
            Serial.print(front_dist); Serial.println("mm");
            s_state  = NAV_DONE;
            s_action = ACT_NONE;
            return NAV_DONE;
        }
    }

    // ── Condition d'arrêt : distance atteinte ─────────────────
    long cell_target = (long)calib_get_cell_ticks();
    long avg_ticks   = (labs(tL) + labs(tR)) / 2;
    if (avg_ticks >= cell_target) {
        motors_stop();
        s_state  = NAV_DONE;
        s_action = ACT_NONE;
        return NAV_DONE;
    }

    // ── Snapshot murs au milieu de la case ───────────────────────
    // Pris à WALL_SNAP_PCT% des ticks pour avoir les murs de la case courante
    // (pas ceux de la suivante, que les capteurs à 45° voient en fin de case).
    if (!s_wall_snap_valid && avg_ticks >= (cell_target * WALL_SNAP_PCT / 100L)) {
        s_wall_snap = sensors_detect_walls(s_tof);
        s_wall_snap_valid = true;
        Serial.print("[NAV] Wall snap @ ticks="); Serial.print(avg_ticks);
        Serial.print(" F="); Serial.print((int)s_wall_snap.front);
        Serial.print(" G="); Serial.print((int)s_wall_snap.left);
        Serial.print(" D="); Serial.println((int)s_wall_snap.right);
    }

    // ── PID combiné (Encodeurs + ToF Latéral) ───────────────────
    // 1. PID Encodeurs : maintient les roues synchronisées
    int pwm_l_enc, pwm_r_enc;
    pid_update(tL, tR, PWM_RUN1, pwm_l_enc, pwm_r_enc);
    int corr_enc = pwm_r_enc - PWM_RUN1;

    // 2. PID ToF : maintient le robot centré dans le couloir
    int pwm_l_tof, pwm_r_tof;
    pid_update_tof((float)s_tof.side_left, (float)s_tof.side_right,
                   calib_get_opening_l(), calib_get_opening_r(),
                   PWM_RUN1, pwm_l_tof, pwm_r_tof);
    int corr_tof = pwm_r_tof - PWM_RUN1;

    // 3. Somme des corrections
    int final_l = constrain(PWM_RUN1 - corr_enc - corr_tof, 0, 255);
    int final_r = constrain(PWM_RUN1 + corr_enc + corr_tof, 0, 255);
    motors_set(final_l, final_r);

    // ── Affichage live @ PID_SAMPLE_MS ────────────────────────
    if (now - s_last_log_ms >= (uint32_t)PID_SAMPLE_MS) {
        s_last_log_ms = now;
        Serial.print("[ADV] tL="); Serial.print(tL);
        Serial.print("  tR=");     Serial.print(tR);
        Serial.print("  SL=");     Serial.print(s_tof.side_left);
        Serial.print("  SR=");     Serial.print(s_tof.side_right);
        Serial.print("  corr(E/T)="); Serial.print(corr_enc);
        Serial.print("/");            Serial.print(corr_tof);
        Serial.print("  pwm=");    Serial.print(final_l);
        Serial.print("/");         Serial.println(final_r);
    }

    return NAV_BUSY;
}

// =============================================================
//  NAV_START_REVERSE — demi-tour par marche arrière
//  Phase 1 : alignement frontal sur le mur du cul-de-sac (FL≈FR)
//  Phase 2 : marche arrière 1 case avec PID latéral (centrage couloir)
//
//  Les capteurs 45° voient toujours les murs latéraux pendant le recul,
//  donc le PID latéral reste valide.
// =============================================================
void nav_start_reverse() {
    encoders_reset();
    pid_init();
    s_wall_snap_valid = false;
    s_last_tof_ms = 0;          // force lecture ToF immédiate
    s_last_log_ms = millis();
    motors_stop();              // frein avant pré-align
    s_action = ACT_REVERSE_ALIGN;
    s_state  = NAV_BUSY;
    Serial.println("[REV] Démarrage — phase 1 : alignement frontal");
}

// ── Phase 1 : pré-alignement frontal (FL≈FR) ─────────────────
// Pivot sur place doux pour égaliser les distances frontales.
// Ne fait rien si pas de mur devant (alignement pas critique).
static NavState update_reverse_align() {
    uint32_t now = millis();
    if (now - s_last_tof_ms >= (uint32_t)PID_SAMPLE_MS) {
        sensors_read(s_tof);
        s_last_tof_ms = now;
    }
    float fl = (float)s_tof.front_left;
    float fr = (float)s_tof.front_right;

    // Pas de mur frontal → on saute l'alignement
    bool has_wall = (fl > 0 && fl < TOF_WALL_FRONT_MM &&
                     fr > 0 && fr < TOF_WALL_FRONT_MM);
    if (!has_wall) {
        Serial.println("[REV] Pas de mur frontal -> reverse direct");
        encoders_reset();
        pid_init();
        s_action = ACT_REVERSE;
        s_last_log_ms = now;
        return NAV_BUSY;
    }

    float angle_err = fl - fr;
    if (fabsf(angle_err) > ALIGN_ANGLE_TOL_MM) {
        int sign = (angle_err > 0) ? 1 : -1;
        motors_set(-sign * PWM_ALIGN_TURN, sign * PWM_ALIGN_TURN);
        return NAV_BUSY;
    }

    // Aligné → passage à la marche arrière
    motors_stop();
    encoders_reset();
    pid_init();
    s_action = ACT_REVERSE;
    s_last_log_ms = now;
    Serial.print("[REV] Alignement OK (FL="); Serial.print(fl, 0);
    Serial.print(" FR=");                    Serial.print(fr, 0);
    Serial.println(") -> marche arrière");
    return NAV_BUSY;
}

// ── Phase 2 : marche arrière avec PID encodeurs + PID latéral ──
static NavState update_reverse() {
    uint32_t now = millis();

    if (now - s_last_tof_ms >= (uint32_t)PID_SAMPLE_MS) {
        sensors_read(s_tof);
        s_last_tof_ms = now;
    }

    long tL = encoders_get_left();
    long tR = encoders_get_right();
    long avg_ticks = (labs(tL) + labs(tR)) / 2;
    long cell_target = (long)calib_get_cell_ticks();

    if (avg_ticks >= cell_target) {
        motors_stop();
        s_state  = NAV_DONE;
        s_action = ACT_NONE;
        Serial.println("[REV] Marche arrière terminée");
        return NAV_DONE;
    }

    // 1. PID encodeurs (synchronisation roues)
    int pwm_l_enc, pwm_r_enc;
    pid_update(tL, tR, PWM_RUN1, pwm_l_enc, pwm_r_enc);
    int corr_enc = pwm_r_enc - PWM_RUN1;

    // 2. PID latéral (centrage couloir) — capteurs 45° voient toujours.
    //    EN MARCHE ARRIÈRE le sens de correction est INVERSÉ : si on est
    //    trop à gauche (SL petit), accélérer le moteur GAUCHE en arrière
    //    (et non droit) pour s'écarter du mur gauche.
    int pwm_l_tof, pwm_r_tof;
    pid_update_tof((float)s_tof.side_left, (float)s_tof.side_right,
                   calib_get_opening_l(), calib_get_opening_r(),
                   PWM_RUN1, pwm_l_tof, pwm_r_tof);
    int corr_tof = pwm_r_tof - PWM_RUN1;

    // Application : signes négatifs pour reculer, corrections inversées
    int final_l = constrain(-PWM_RUN1 + corr_enc + corr_tof, -255, 0);
    int final_r = constrain(-PWM_RUN1 - corr_enc - corr_tof, -255, 0);
    motors_set(final_l, final_r);

    if (now - s_last_log_ms >= (uint32_t)PID_SAMPLE_MS) {
        s_last_log_ms = now;
        Serial.print("[REV] tL="); Serial.print(tL);
        Serial.print(" tR=");      Serial.print(tR);
        Serial.print(" SL=");      Serial.print(s_tof.side_left);
        Serial.print(" SR=");      Serial.print(s_tof.side_right);
        Serial.print(" corr(E/T)="); Serial.print(corr_enc);
        Serial.print("/");           Serial.print(corr_tof);
        Serial.print(" pwm=");     Serial.print(final_l);
        Serial.print("/");         Serial.println(final_r);
    }
    return NAV_BUSY;
}

// =============================================================
//  NAV_START_SMOOTH_TURN
//  Démarre la séquence 45° -> 40mm -> 45°
// =============================================================
void nav_start_smooth_turn(int direction) {
    motors_stop();
    encoders_reset();
    pid_init();
    s_smooth_dir = (direction > 0) ? 1 : -1;
    s_action     = ACT_SMOOTH_45_1;
    s_state      = NAV_BUSY;
    s_settle_start = millis();
    Serial.print("[SMOOTH] Début virage ");
    Serial.println(s_smooth_dir > 0 ? "DROITE" : "GAUCHE");
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
    s_last_tof_ms   = 0;        // force lecture ToF immédiate dans pre-align
    s_action = ACT_TURN_PRE_ALIGN;
    s_state  = NAV_BUSY;
    Serial.println("[TURN] Pré-alignement frontal");
}

// ── Calcule le PWM de base pivot via rampe linéaire de décélération ──
// De 0 à PIVOT_DECEL_PCT% des ticks → PWM_PIVOT_FAST.
// De PIVOT_DECEL_PCT% à 100% → interpolation linéaire vers PWM_PIVOT_SLOW.
// Au-delà de 100% → PWM_PIVOT_SLOW (au cas où l'IMU n'a pas encore stoppé).
static int pivot_base_pwm(long avg_ticks, long target_ticks) {
    long decel_start = (target_ticks * PIVOT_DECEL_PCT) / 100L;
    if (avg_ticks <= decel_start)  return PWM_PIVOT_FAST;
    if (avg_ticks >= target_ticks) return PWM_PIVOT_SLOW;
    // interpolation linéaire entre decel_start et target_ticks
    long span = target_ticks - decel_start;
    long pos  = avg_ticks - decel_start;
    int  diff = PWM_PIVOT_FAST - PWM_PIVOT_SLOW;
    return PWM_PIVOT_FAST - (int)((diff * pos) / span);
}

// ── Démarre le pivot dans la bonne direction ─────────────────
// Roues en sens opposés. PWM symétrique au-dessus du seuil de stall ;
// le PID pivot prend le relais à chaque cycle.
static void start_pivot() {
    encoders_reset();
    pid_init();
    int dir  = (s_turn_quarters > 0) ? 1 : -1;
    int kick = PWM_PIVOT_FAST;
    if (dir > 0) motors_set( kick, -kick);   // horaire (droite)
    else         motors_set(-kick,  kick);   // antihoraire (gauche)
}

// ── Applique le PID pivot avec base = rampe linéaire ─────────
// Force |ticks_L| ≈ |ticks_R| → pivot autour du centre du robot.
static void apply_pivot_pwm(long avg_ticks, long target_ticks) {
    int  base = pivot_base_pwm(avg_ticks, target_ticks);
    // Direction : s_turn_quarters (90/180) ou s_smooth_dir (45)
    int  dir  = (s_action == ACT_TURN_PIVOT) ? ((s_turn_quarters > 0) ? 1 : -1) : s_smooth_dir;
    long tL   = encoders_get_left();
    long tR   = encoders_get_right();
    int pwm_l, pwm_r;
    pid_update_pivot(tL, tR, dir, base, pwm_l, pwm_r);
    motors_set(pwm_l, pwm_r);
}

// ── Mise à jour du virage complexe 45-40-45 ───────────────────
static NavState update_smooth_turn() {
    uint32_t now = millis();
    long tL = encoders_get_left();
    long tR = encoders_get_right();
    long avg_ticks = (labs(tL) + labs(tR)) / 2;
    
    // On choisit la cible de ticks selon la direction (calibrable via web UI)
    long target_45 = (s_smooth_dir > 0) ? (long)calib_get_pivot_45_r()
                                         : (long)calib_get_pivot_45_l();
    long target_move = (long)calib_get_smooth_move();

    switch (s_action) {
        case ACT_SMOOTH_45_1:
            apply_pivot_pwm(avg_ticks, target_45);
            if (avg_ticks >= target_45) {
                motors_stop();
                s_settle_start = now;
                s_action = ACT_SMOOTH_SETTLE_1;
                Serial.println("[SMOOTH] 45° (1) OK -> Pause");
            }
            break;

        case ACT_SMOOTH_SETTLE_1:
            if (now - s_settle_start >= (uint32_t)TURN_SETTLE_MS) {
                encoders_reset();
                pid_init();
                s_action = ACT_SMOOTH_MOVE;
                Serial.println("[SMOOTH] Avance 40mm...");
            }
            break;

        case ACT_SMOOTH_MOVE:
            {
                // PID combiné pour l'avance de 40mm
                if (now - s_last_tof_ms >= (uint32_t)PID_SAMPLE_MS) {
                    sensors_read(s_tof);
                    s_last_tof_ms = now;
                }
                int pwm_l_enc, pwm_r_enc;
                pid_update(tL, tR, PWM_RUN1, pwm_l_enc, pwm_r_enc);
                int corr_enc = pwm_r_enc - PWM_RUN1;

                int pwm_l_tof, pwm_r_tof;
                pid_update_tof((float)s_tof.side_left, (float)s_tof.side_right,
                               calib_get_opening_l(), calib_get_opening_r(),
                               PWM_RUN1, pwm_l_tof, pwm_r_tof);
                int corr_tof = pwm_r_tof - PWM_RUN1;

                motors_set(constrain(PWM_RUN1 - corr_enc - corr_tof, 0, 255),
                           constrain(PWM_RUN1 + corr_enc + corr_tof, 0, 255));

                if (avg_ticks >= target_move) {
                    motors_stop();
                    s_settle_start = now;
                    s_action = ACT_SMOOTH_SETTLE_2;
                    Serial.println("[SMOOTH] 40mm OK -> Pause");
                }
            }
            break;

        case ACT_SMOOTH_SETTLE_2:
            if (now - s_settle_start >= (uint32_t)TURN_SETTLE_MS) {
                encoders_reset();
                pid_init();
                s_action = ACT_SMOOTH_45_2;
                Serial.println("[SMOOTH] 45° (2)...");
            }
            break;

        case ACT_SMOOTH_45_2:
            apply_pivot_pwm(avg_ticks, target_45);
            if (avg_ticks >= target_45) {
                motors_stop();
                s_action = ACT_SMOOTH_CENTER;
                s_last_tof_ms = 0;       // force lecture ToF immédiate
                s_settle_start = now;    // réutilisé comme timeout
                Serial.println("[SMOOTH] 45° (2) OK -> Centrage 77mm...");
            }
            break;

        case ACT_SMOOTH_CENTER:
            {
                if (now - s_last_tof_ms >= (uint32_t)PID_SAMPLE_MS) {
                    sensors_read(s_tof);
                    s_last_tof_ms = now;
                }
                float fl = (float)s_tof.front_left;
                float fr = (float)s_tof.front_right;

                // On ne tente le centrage QUE si un mur est PROCHE (< 250mm).
                // Sinon le robot fonce en avant pour atteindre 77mm sans
                // rien y voir → comportement erratique. Mieux vaut s'arrêter.
                const float CENTER_GUARD_MM = 250.0f;
                bool fl_ok = (fl > 0 && fl < CENTER_GUARD_MM);
                bool fr_ok = (fr > 0 && fr < CENTER_GUARD_MM);
                if (!fl_ok && !fr_ok) {
                    motors_stop();
                    Serial.println("[SMOOTH] Centrage : pas de mur proche -> FIN (skip)");
                    s_state  = NAV_DONE;
                    s_action = ACT_NONE;
                    return NAV_DONE;
                }

                // Distance frontale = moyenne des 2 capteurs valides, ou le seul valide
                float front;
                if (fl_ok && fr_ok)  front = (fl + fr) * 0.5f;
                else if (fl_ok)      front = fl;
                else                 front = fr;

                float dist_err = front - (float)calib_get_smooth_center();

                // Arrivé dans la tolérance → fini
                if (fabsf(dist_err) <= (float)SMOOTH_CENTER_TOL_MM) {
                    motors_stop();
                    Serial.print("[SMOOTH] Centrage OK -> FIN, front=");
                    Serial.print(front, 0); Serial.println("mm");
                    s_state  = NAV_DONE;
                    s_action = ACT_NONE;
                    return NAV_DONE;
                }

                // Timeout 1.5s pour éviter de rester coincé
                if (now - s_settle_start > 1500) {
                    motors_stop();
                    Serial.print("[SMOOTH] Centrage TIMEOUT -> FIN, front=");
                    Serial.print(front, 0); Serial.println("mm");
                    s_state  = NAV_DONE;
                    s_action = ACT_NONE;
                    return NAV_DONE;
                }

                // P proportionnel + clamp + signe (avant si trop loin, recule si trop près)
                int sign = (dist_err > 0) ? 1 : -1;
                int pwm  = (int)fabsf(dist_err * 2.0f);
                pwm = constrain(pwm, SMOOTH_CENTER_PWM_MIN, SMOOTH_CENTER_PWM_MAX);
                motors_set(sign * pwm, sign * pwm);
            }
            break;

        case ACT_SMOOTH_ALIGN:
            {
                // On avance doucement pour se centrer dans la case suivante
                if (now - s_last_tof_ms >= (uint32_t)PID_SAMPLE_MS) {
                    sensors_read(s_tof);
                    s_last_tof_ms = now;
                }
                float fl = (float)s_tof.front_left;
                float fr = (float)s_tof.front_right;

                // 1. Correction d'angle (toujours prioritaire)
                float angle_err = fl - fr;
                if (fl > 0 && fr > 0 && fabsf(angle_err) > (float)TOF_ALIGN_TOL_MM) {
                    // Facteur réduit à 3.0 (au lieu de 5.0)
                    int turn_pwm = (int)(angle_err * 3.0f);
                    // Minimum baissé à 60
                    int sign = (turn_pwm > 0) ? 1 : -1;
                    turn_pwm = sign * constrain(abs(turn_pwm), 60, 80);
                    motors_set(-turn_pwm, turn_pwm);
                    return NAV_BUSY;
                }

                // 2. Avance vers la distance cible de la case (TOF_ALIGN_TARGET_MM)
                if (fl > 0 && fr > 0) {
                    float avg_dist = (fl + fr) / 2.0f;
                    float dist_err = avg_dist - (float)TOF_ALIGN_TARGET_MM;
                    
                    if (fabsf(dist_err) > (float)ALIGN_DIST_TOL_MM) {
                        // Translation proportionnelle plus douce
                        int move_base = (int)(dist_err * 3.0f);
                        int sign = (move_base > 0) ? 1 : -1;
                        move_base = sign * constrain(abs(move_base), 65, 85);

                        int pwm_l, pwm_r;
                        pid_update_tof((float)s_tof.side_left, (float)s_tof.side_right,
                                       calib_get_opening_l(), calib_get_opening_r(),
                                       move_base, pwm_l, pwm_r);
                        motors_set(pwm_l, pwm_r);
                        return NAV_BUSY;
                    }
                }

 else {
                    // Pas de mur devant ? On avance juste de 20mm pour "entrer" dans la case
                    // (Optionnel : si tu veux une avance aveugle sans mur frontal)
                    motors_stop();
                    s_state = NAV_DONE;
                    s_action = ACT_NONE;
                    return NAV_DONE;
                }

                // Centrage terminé
                motors_stop();
                Serial.println("[SMOOTH] Recalage et centrage OK -> FIN");
                s_state = NAV_DONE;
                s_action = ACT_NONE;
                return NAV_DONE;
            }
            break;

        default: break;
    }
    return NAV_BUSY;
}

// ── Mise à jour de la rotation ────────────────────────────────
// 90° et 180° : pivot sur place. Arrêt PRINCIPAL sur ticks (calib).
// L'IMU est un garde-fou : si l'angle dépasse la cible avant les ticks
// (glissement excessif), on stoppe quand même avec un log d'alerte.
static NavState update_turn() {
    uint32_t now = millis();

    // ── Phase pré-alignement frontal ──────────────────────────
    if (s_action == ACT_TURN_PRE_ALIGN) {
        if (now - s_last_tof_ms >= (uint32_t)PID_SAMPLE_MS) {
            sensors_read(s_tof);
            s_last_tof_ms = now;
        }
        float fl = (float)s_tof.front_left;
        float fr = (float)s_tof.front_right;

        bool has_wall = (fl > 0 && fl <= TOF_WALL_FRONT_MM &&
                         fr > 0 && fr <= TOF_WALL_FRONT_MM);
        if (!has_wall) {
            motors_stop();
            s_settle_start = now;
            s_action = ACT_TURN_SETTLING;
            Serial.println("[TURN] Pas de mur frontal — settling direct");
            return NAV_BUSY;
        }

        // Étape 1 : correction angulaire (FL ≈ FR) — bang-bang
        float angle_err = fl - fr;
        if (fabsf(angle_err) > ALIGN_ANGLE_TOL_MM) {
            int sign = (angle_err > 0) ? 1 : -1;
            motors_set(-sign * PWM_ALIGN_TURN, sign * PWM_ALIGN_TURN);
            return NAV_BUSY;
        }

        // Étape 2 : distance cible TOF_ALIGN_TARGET_MM — bang-bang
        float avg_dist = (fl + fr) / 2.0f;
        float dist_err = avg_dist - (float)TOF_ALIGN_TARGET_MM;
        if (fabsf(dist_err) > ALIGN_DIST_TOL_MM) {
            int sign = (dist_err > 0) ? 1 : -1;
            motors_set(sign * PWM_ALIGN_MOVE, sign * PWM_ALIGN_MOVE);
            return NAV_BUSY;
        }

        // Aligné → settling
        motors_stop();
        encoders_reset();
        s_settle_start = now;
        s_action = ACT_TURN_SETTLING;
        Serial.print("[TURN] Pré-align OK — dist=");
        Serial.print(avg_dist, 0); Serial.println("mm");
        return NAV_BUSY;
    }

    bool is_180 = (abs(s_turn_quarters) == 2);

    // ── Phase de stabilisation ─────────────────────────────────
    if (s_action == ACT_TURN_SETTLING) {
        if (now - s_settle_start >= TURN_SETTLE_MS) {
            s_last_log_ms = now;
            start_pivot();
            s_action = ACT_TURN_PIVOT;
            Serial.print("[TURN] Pivot ");
            Serial.print(is_180 ? "180" : "90");
            Serial.print("° démarré — cible ticks=");
            Serial.println(is_180 ? TICKS_PIVOT_180 : calib_get_pivot_90_ticks());
        }
        return NAV_BUSY;
    }

    long tL        = encoders_get_left();
    long tR        = encoders_get_right();
    long avg_ticks = (labs(tL) + labs(tR)) / 2;

    // ── Phase PIVOT : 90° (calib) ou 180° (TICKS_PIVOT_180) ──
    // Arrêt UNIQUEMENT sur ticks encodeurs (IMU non utilisée).
    if (s_action == ACT_TURN_PIVOT) {
        long tick_target = is_180 ? (long)TICKS_PIVOT_180 : (long)calib_get_pivot_90_ticks();

        apply_pivot_pwm(avg_ticks, tick_target);

        if (now - s_last_log_ms >= (uint32_t)PID_SAMPLE_MS) {
            s_last_log_ms = now;
            Serial.print("[PIV] ticks="); Serial.print(avg_ticks);
            Serial.print("/");            Serial.print(tick_target);
            Serial.print(" pwm=");        Serial.print(pivot_base_pwm(avg_ticks, tick_target));
            Serial.print(" tL=");         Serial.print(tL);
            Serial.print(" tR=");         Serial.println(tR);
        }

        if (avg_ticks >= tick_target) {
            motors_stop();
            Serial.print("[TURN] Pivot ");
            Serial.print(is_180 ? "180" : "90");
            Serial.println("° terminé (ticks)");
            s_state  = NAV_DONE;
            s_action = ACT_NONE;
            return NAV_DONE;
        }
        return NAV_BUSY;
    }
    return NAV_BUSY;
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

    if (fl == 0 || fl > TOF_WALL_FRONT_MM ||
        fr == 0 || fr > TOF_WALL_FRONT_MM) {
        motors_stop();
        s_state  = NAV_DONE;
        s_action = ACT_NONE;
        return NAV_DONE;
    }

    float angle_error = fl - fr;
    if (fabsf(angle_error) > TOF_ALIGN_TOL_MM) {
        int sign = (angle_error > 0) ? 1 : -1;
        motors_set(-sign * 70, sign * 70);
        return NAV_BUSY;
    }

    float avg_dist  = (fl + fr) / 2.0f;
    float dist_err  = avg_dist - (float)TOF_ALIGN_TARGET_MM;

    if (fabsf(dist_err) > 3.0f) {
        int sign = (dist_err > 0) ? 1 : -1;
        motors_set(sign * 75, sign * 75);
        return NAV_BUSY;
    }

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

        case ACT_REVERSE_ALIGN:
            return update_reverse_align();

        case ACT_REVERSE:
            return update_reverse();

        case ACT_TURN_PRE_ALIGN:
        case ACT_TURN_SETTLING:
        case ACT_TURN_PIVOT:
            return update_turn();

        case ACT_AUTO_ALIGN:
            return update_auto_align();

        case ACT_SMOOTH_45_1:
        case ACT_SMOOTH_SETTLE_1:
        case ACT_SMOOTH_MOVE:
        case ACT_SMOOTH_SETTLE_2:
        case ACT_SMOOTH_45_2:
        case ACT_SMOOTH_ALIGN:
        case ACT_SMOOTH_CENTER:
            return update_smooth_turn();

        default:
            return NAV_IDLE;
    }
}

// ── nav_get_wall_snap ─────────────────────────────────────────
bool nav_get_wall_snap(WallDetection& out) {
    if (!s_wall_snap_valid) return false;
    out = s_wall_snap;
    return true;
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
