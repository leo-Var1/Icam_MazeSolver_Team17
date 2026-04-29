// =============================================================
//  tremaux.cpp — Implémentation algorithme de Trémaux
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include "tremaux.h"
#include "config.h"
#include "maze.h"
#include "sensors.h"
#include "navigation.h"
#include <Arduino.h>

// ── État interne ──────────────────────────────────────────────
static TremauxPhase s_phase     = TREM_SCAN;
static uint8_t      s_chosen_dir = 0;  // Direction absolue choisie (0=N,1=E,2=S,3=W)
static uint32_t     s_pause_start = 0;  // timestamp début pause inter-case
static bool         s_did_reverse = false; // true si dernier mouvement = marche arrière
static uint8_t      s_consecutive_rev = 0; // garde anti-boucle 180°
#define TREM_MAX_CONSECUTIVE_REV  3

// ── Détection murs ───────────────────────────────────────────
// On délègue à sensors_detect_walls() qui applique les seuils calibrés
// (calib_get_opening_l/r) + rejet géométrique (anti faux mur latéral
// dû à un mur frontal proche). Cohérent avec le wall_snap.

// ── Choix de la direction (cœur de Trémaux) ──────────────────
// Règles (par priorité décroissante) :
//  1. Direction non visitée (visited = 0) → prioritaire
//  2. Direction visitée 1 fois → acceptable
//  3. Direction visitée 2 fois → mort confirmé, on évite
//  4. Si toutes les directions sont bloquées ou visitées 2x → faire demi-tour
//
// On incrémente le compteur de visite du passage EMPRUNTÉ (pas tous).
static uint8_t tremaux_choose_direction() {
    uint8_t row = maze_get_row();
    uint8_t col = maze_get_col();
    uint8_t dir = maze_get_dir();

    // Direction d'où on vient (opposée à la direction courante)
    // On la marque pour indiquer qu'on est entré par ici
    uint8_t from_dir = (dir + 2) % 4;

    // Seuils des compteurs de visites par direction pour cette case.
    // On encode les visites dans maze[r][c].visited (global) mais Trémaux
    // a besoin de savoir combien de fois CHAQUE PASSAGE a été emprunté.
    // Simplification : on utilise une table marks[row][col][4] locale.
    // Stockage : on réutilise le champ visited pour le compteur global,
    // et on code les passages dans les 4 bits supérieurs de walls.
    // → Pour rester simple, on utilise une approche globale :
    //   visited[row][col] = nombre de fois que le robot est passé par cette case.
    //   On choisit la direction vers la case la moins visitée.

    // Directions accessibles (pas de mur)
    uint8_t best_dir   = 255;
    uint8_t best_count = 255;

    // Deltas de position
    const int8_t dR[4] = { -1,  0,  1,  0 };
    const int8_t dC[4] = {  0,  1,  0, -1 };
    const uint8_t wall_bit[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

    for (int d = 0; d < 4; d++) {
        // Pas de passage si mur
        if (maze[row][col].walls & wall_bit[d]) continue;

        // Calculer la case voisine
        int8_t nr = row + dR[d];
        int8_t nc = col + dC[d];
        if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;

        // Compter les visites de la case voisine
        uint8_t visits = maze_get_visited(nr, nc);

        // Règle Trémaux : éviter les cases visitées 2 fois SAUF si c'est la seule option
        // (pour pouvoir rebrousser chemin dans un cul-de-sac)
        if (visits < best_count) {
            // Préférer les directions non-retour (ne pas revenir d'où on vient sauf necessity)
            if (d == from_dir && best_dir != 255 && best_count <= visits) continue;
            best_count = visits;
            best_dir   = d;
        }
    }

    // Si aucune direction libre trouvée (case complètement encerclée) : rester sur place
    if (best_dir == 255) {
        Serial.println("[TREM] Aucune direction accessible ! Retour arrière.");
        return from_dir;
    }

    return best_dir;
}

// ── tremaux_init ──────────────────────────────────────────────
void tremaux_init(uint8_t start_row, uint8_t start_col, uint8_t start_dir) {
    s_phase      = TREM_SCAN;
    s_chosen_dir = start_dir;
    s_consecutive_rev = 0;
    maze_set_pos(start_row, start_col, start_dir);
    maze_mark_visited(start_row, start_col);
    Serial.print("[TREM] Initialisation — depart (");
    Serial.print(start_row); Serial.print(",");
    Serial.print(start_col); Serial.print(") facing=");
    Serial.println("NESW"[start_dir]);
}

// ── tremaux_update ────────────────────────────────────────────
bool tremaux_update() {
    switch (s_phase) {

        // ── SCAN : lire les murs + mettre à jour la carte ────────
        // Priorité au snapshot pris au milieu de la case pendant l'avance.
        // Les capteurs à 45° voient la case SUIVANTE en fin de case → ne pas
        // lire les capteurs en direct ici (sauf au tout premier scan, sans avance).
        case TREM_SCAN: {
            // Scan moyenné sur 3 lectures (~90ms) pour fiabilité.
            // Robot stable après pause → meilleure mesure que le snap en mouvement.
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

            Serial.printf("[TREM] Scan (%d,%d) dir=%c | FL=%d FR=%d SL=%d SR=%d | F=%d G=%d D=%d\n",
                maze_get_row(), maze_get_col(), "NESW"[maze_get_dir()],
                tof.front_left, tof.front_right, tof.side_left, tof.side_right,
                (int)front, (int)left, (int)right);

            s_phase = TREM_DECIDE;
            break;
        }

        // ── DECIDE : choisir la prochaine direction ────────────
        case TREM_DECIDE: {
            s_chosen_dir = tremaux_choose_direction();
            uint8_t current_dir = maze_get_dir();
            s_did_reverse = false;

            Serial.printf("[TREM] Décision : dir=%d (actuelle=%d)\n",
                s_chosen_dir, current_dir);

            if (s_chosen_dir == current_dir) {
                // Déjà orienté dans la bonne direction → avancer directement
                s_consecutive_rev = 0;
                s_phase = TREM_MOVE;
                nav_start_advance();
            } else {
                // Calculer le virage en quarts de tour signés [-2..+2]
                int diff = (int)s_chosen_dir - (int)current_dir;
                if (diff >  2) diff -= 4;
                if (diff < -2) diff += 4;

                if (diff == 2 || diff == -2) {
                    // Garde anti-boucle : si 3 reverse consécutifs → abort
                    if (s_consecutive_rev >= TREM_MAX_CONSECUTIVE_REV) {
                        Serial.printf("[TREM] !! BOUCLE 180° (%d reverse consecutifs) -> ABORT\n",
                            s_consecutive_rev);
                        Serial.println("[TREM] Detection murs probablement en cause. Touche 'w' pour test ToF.");
                        s_phase = TREM_FINISHED;
                        break;
                    }
                    Serial.printf("[TREM] Demi-tour → marche arrière #%d\n", s_consecutive_rev + 1);
                    s_did_reverse = true;
                    s_consecutive_rev++;
                    s_phase = TREM_REVERSE;
                    nav_start_reverse();
                } else {
                    s_consecutive_rev = 0;
                    // ── 90° gauche/droite → smooth turn (45-avance-45) ──
                    Serial.printf("[TREM] Virage 90° %s (smooth)\n",
                        diff > 0 ? "DROITE" : "GAUCHE");
                    s_phase = TREM_ORIENT;
                    nav_start_smooth_turn(diff);
                }
            }
            break;
        }

        // ── ORIENT : attendre la fin du smooth turn 90° ────────
        case TREM_ORIENT: {
            if (nav_get_state() == NAV_DONE) {
                maze_set_pos(maze_get_row(), maze_get_col(), s_chosen_dir);
                s_phase = TREM_MOVE;
                nav_start_advance();
            }
            break;
        }

        // ── REVERSE : attendre la fin de la marche arrière ─────
        case TREM_REVERSE: {
            if (nav_get_state() == NAV_DONE) {
                // Position recule (facing inchangé), on saute MOVE et on passe à PAUSE.
                s_phase = TREM_PAUSE;
                s_pause_start = millis();
            }
            break;
        }

        // ── MOVE : attendre la fin de l'avance ────────────────
        case TREM_MOVE: {
            if (nav_get_state() == NAV_DONE) {
                s_phase = TREM_PAUSE;
                s_pause_start = millis();
            }
            break;
        }

        // ── PAUSE : attendre TREM_PAUSE_MS avant de scanner la nouvelle case ──
        // Laisse le temps aux capteurs ToF (mode continu) de produire des
        // lectures stables et au robot de se stabiliser physiquement.
        case TREM_PAUSE: {
            if (millis() - s_pause_start >= (uint32_t)TREM_PAUSE_MS) {
                s_phase = TREM_UPDATE;
            }
            break;
        }

        // ── UPDATE : mise à jour de la position dans la carte ─
        case TREM_UPDATE: {
            // Avance OU recule la position logique selon le dernier mouvement
            if (s_did_reverse) maze_reverse_robot();
            else               maze_advance_robot();
            uint8_t row = maze_get_row();
            uint8_t col = maze_get_col();

            // Marquer la case comme visitée (+1)
            maze_mark_visited(row, col);

            Serial.printf("[TREM] Arrivé en (%d,%d) — visites=%d\n",
                row, col, maze_get_visited(row, col));

            // Afficher la carte en Serial
            maze_print();

            // Vérifier si l'exploration est terminée
            if (maze_is_fully_explored()) {
                Serial.println("[TREM] Exploration complète !");
                s_phase = TREM_FINISHED;
                return true;
            }

            // Prochain cycle : scanner la nouvelle case
            s_phase = TREM_SCAN;
            break;
        }

        case TREM_FINISHED:
            return true;
    }

    return false;
}

// ── tremaux_get_phase ────────────────────────────────────────
TremauxPhase tremaux_get_phase() {
    return s_phase;
}
