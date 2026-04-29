// =============================================================
//  wall_follower.cpp — Algorithme main droite
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include "wall_follower.h"
#include "config.h"
#include "maze.h"
#include "sensors.h"
#include "navigation.h"
#include "web_ui.h"
#include "calibration.h"
#include <Arduino.h>

// ── État interne ──────────────────────────────────────────────
static WallFollowPhase s_phase             = WF_SCAN;
static uint32_t        s_pause_start       = 0;
static bool            s_did_reverse       = false;
static uint8_t         s_target_row        = 0;
static uint8_t         s_target_col        = 0;
static uint8_t         s_consecutive_rev   = 0;  // garde anti-boucle 180°
#define WF_MAX_CONSECUTIVE_REV  3

// La détection murs/ouvertures utilise sensors_detect_walls() qui applique
// les seuils calibrés (calib_get_opening_l/r) + rejet géométrique.

// ── wall_follower_init ────────────────────────────────────────
void wall_follower_init(uint8_t start_row, uint8_t start_col, uint8_t start_dir) {
    s_phase = WF_SCAN;
    s_did_reverse = false;
    s_consecutive_rev = 0;
    s_target_row = web_ui_get_target_row();
    s_target_col = web_ui_get_target_col();
    maze_set_pos(start_row, start_col, start_dir);
    maze_mark_visited(start_row, start_col);
    Serial.print("[WF] Main droite - depart (");
    Serial.print(start_row); Serial.print(",");
    Serial.print(start_col); Serial.print(") facing=");
    Serial.print("NESW"[start_dir]);
    Serial.print(" cible (");
    Serial.print(s_target_row); Serial.print(",");
    Serial.print(s_target_col); Serial.println(")");
}

// ── wall_follower_update ──────────────────────────────────────
bool wall_follower_update() {
    switch (s_phase) {

        // ── SCAN : lire murs (snap si dispo, sinon ToF direct) ──
        case WF_SCAN: {
            // Si on vient juste d'arriver à la cible, on s'arrête là
            if (maze_get_row() == s_target_row && maze_get_col() == s_target_col) {
                Serial.println("[WF] Cible atteinte -> FIN");
                s_phase = WF_FINISHED;
                return true;
            }

            // Scan moyenné : 3 lectures espacées de 30ms pour fiabilité.
            // Le snap pris pendant l'avance est ignoré ici — on veut une mesure
            // robot ARRÊTÉ, pas en mouvement (post-pause = robot stable).
            ToFReadings sum = {0,0,0,0};
            int n_fl=0, n_fr=0, n_sl=0, n_sr=0;
            for (int i = 0; i < 3; i++) {
                ToFReadings t;
                sensors_read(t);
                if (t.front_left  > 0) { sum.front_left  += t.front_left;  n_fl++; }
                if (t.front_right > 0) { sum.front_right += t.front_right; n_fr++; }
                if (t.side_left   > 0) { sum.side_left   += t.side_left;   n_sl++; }
                if (t.side_right  > 0) { sum.side_right  += t.side_right;  n_sr++; }
                delay(30);
            }
            ToFReadings tof = {
                (uint16_t)(n_fl ? sum.front_left  / n_fl : 0),
                (uint16_t)(n_fr ? sum.front_right / n_fr : 0),
                (uint16_t)(n_sl ? sum.side_left   / n_sl : 0),
                (uint16_t)(n_sr ? sum.side_right  / n_sr : 0)
            };
            WallDetection wd = sensors_detect_walls(tof);
            bool front = wd.front, left = wd.left, right = wd.right;

            maze_update_walls(front, left, right);

            Serial.printf("[WF] Scan (%d,%d) dir=%c | FL=%d FR=%d SL=%d SR=%d | F=%d G=%d D=%d\n",
                maze_get_row(), maze_get_col(), "NESW"[maze_get_dir()],
                tof.front_left, tof.front_right, tof.side_left, tof.side_right,
                (int)front, (int)left, (int)right);

            s_phase = WF_DECIDE;
            break;
        }

        // ── DECIDE : règle main droite ─────────────────────────
        case WF_DECIDE: {
            uint8_t dir = maze_get_dir();
            uint8_t walls = maze[maze_get_row()][maze_get_col()].walls;

            // Murs RELATIFS au facing courant (depuis la carte)
            const uint8_t wall_bit[4] = { WALL_N, WALL_E, WALL_S, WALL_W };
            bool blocked_front = walls & wall_bit[dir];
            bool blocked_right = walls & wall_bit[(dir + 1) % 4];
            bool blocked_left  = walls & wall_bit[(dir + 3) % 4];

            // ── Garde stricte main droite : double-check via lecture fresh ──
            // Évite de virer à droite à la moindre bosse de mur (collision
            // avec le mur droit). On exige SR > opening_r + marge.
            if (!blocked_right) {
                ToFReadings tof;
                sensors_read(tof);
                int seuil = calib_get_opening_r() + WF_RIGHT_MARGIN_MM;
                if (tof.side_right == 0 || tof.side_right < seuil) {
                    Serial.printf("[WF] Garde droite : SR=%d < seuil=%d -> bloque\n",
                        tof.side_right, seuil);
                    blocked_right = true;
                }
            }

            s_did_reverse = false;

            Serial.printf("[WF] Murs relatifs : front=%d droite=%d gauche=%d\n",
                (int)blocked_front, (int)blocked_right, (int)blocked_left);

            if (!blocked_right) {
                // Priorité 1 : DROITE ouverte → smooth turn droite
                Serial.println("[WF] Decision : DROITE (smooth +1)");
                s_phase = WF_ORIENT;
                maze_set_pos(maze_get_row(), maze_get_col(), (dir + 1) % 4);
                nav_start_smooth_turn(+1);
            } else if (!blocked_front) {
                // Priorité 2 : DEVANT
                Serial.println("[WF] Decision : DEVANT (advance)");
                s_phase = WF_MOVE;
                nav_start_advance();
            } else if (!blocked_left) {
                // Priorité 3 : GAUCHE → smooth turn gauche
                Serial.println("[WF] Decision : GAUCHE (smooth -1)");
                s_phase = WF_ORIENT;
                maze_set_pos(maze_get_row(), maze_get_col(), (dir + 3) % 4);
                nav_start_smooth_turn(-1);
            } else {
                // Priorité 4 : cul-de-sac → marche arrière
                // Garde anti-boucle : si on a déjà fait 3 reverse consécutifs,
                // on est probablement bloqué (détection murs cassée). Abort.
                if (s_consecutive_rev >= WF_MAX_CONSECUTIVE_REV) {
                    Serial.printf("[WF] !! BOUCLE 180° detectee (%d reverse consecutifs) -> ABORT\n",
                        s_consecutive_rev);
                    Serial.println("[WF] Verifie : detection murs (touche 'w' pour test ToF), ou recalibre opening_l/r");
                    s_phase = WF_FINISHED;
                    return true;
                }
                Serial.printf("[WF] Decision : CUL-DE-SAC -> reverse #%d\n", s_consecutive_rev + 1);
                s_did_reverse = true;
                s_consecutive_rev++;
                s_phase = WF_REVERSE;
                nav_start_reverse();
            }
            // Reset du compteur si on n'a PAS fait reverse
            if (!s_did_reverse) s_consecutive_rev = 0;
            break;
        }

        // ── ORIENT : attente fin smooth turn 90° puis advance ──
        case WF_ORIENT: {
            if (nav_get_state() == NAV_DONE) {
                s_phase = WF_MOVE;
                nav_start_advance();
            }
            break;
        }

        // ── REVERSE : attente fin marche arrière ──────────────
        case WF_REVERSE: {
            if (nav_get_state() == NAV_DONE) {
                s_phase = WF_PAUSE;
                s_pause_start = millis();
            }
            break;
        }

        // ── MOVE : attente fin avance ─────────────────────────
        case WF_MOVE: {
            if (nav_get_state() == NAV_DONE) {
                s_phase = WF_PAUSE;
                s_pause_start = millis();
            }
            break;
        }

        // ── PAUSE inter-case ──────────────────────────────────
        case WF_PAUSE: {
            if (millis() - s_pause_start >= (uint32_t)TREM_PAUSE_MS) {
                s_phase = WF_UPDATE;
            }
            break;
        }

        // ── UPDATE position dans la carte ─────────────────────
        case WF_UPDATE: {
            if (s_did_reverse) maze_reverse_robot();
            else               maze_advance_robot();
            uint8_t r = maze_get_row();
            uint8_t c = maze_get_col();
            maze_mark_visited(r, c);
            Serial.printf("[WF] Position -> (%d,%d) visites=%d\n",
                r, c, maze_get_visited(r, c));
            maze_print();
            s_phase = WF_SCAN;
            break;
        }

        case WF_FINISHED:
            return true;
    }
    return false;
}

WallFollowPhase wall_follower_get_phase() { return s_phase; }
