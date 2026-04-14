// =============================================================
//  tremaux.cpp — Implémentation algorithme de Trémaux
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include "tremaux.h"
#include "config.h"
#include "maze.h"
#include "sensors.h"
#include "navigation.h"

// ── État interne ──────────────────────────────────────────────
static TremauxPhase s_phase     = TREM_SCAN;
static uint8_t      s_chosen_dir = 0;  // Direction absolue choisie (0=N,1=E,2=S,3=W)

// ── Helpers de détection de murs ─────────────────────────────
// On lit les capteurs ToF et on seuille pour décider si un mur est présent.
// Les seuils sont dans config.h (TOF_WALL_FRONT_MM, TOF_WALL_SIDE_MM).

static bool wall_front(const ToFReadings& tof) {
    // Mur frontal si les deux capteurs FL et FR détectent < TOF_WALL_FRONT_MM
    // (valeur 0 = capteur non disponible → ignorer)
    bool fl = (tof.front_left  > 0 && tof.front_left  < TOF_WALL_FRONT_MM);
    bool fr = (tof.front_right > 0 && tof.front_right < TOF_WALL_FRONT_MM);
    return fl || fr;  // au moins un détecte = considéré comme mur
}

static bool wall_left(const ToFReadings& tof) {
    // Capteur latéral gauche à 45° — mur si distance < TOF_WALL_SIDE_MM
    return (tof.side_left > 0 && tof.side_left < TOF_WALL_SIDE_MM);
}

static bool wall_right(const ToFReadings& tof) {
    return (tof.side_right > 0 && tof.side_right < TOF_WALL_SIDE_MM);
}

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
void tremaux_init() {
    s_phase      = TREM_SCAN;
    s_chosen_dir = 0;
    // Position de départ : case (0,0), direction Nord
    maze_set_pos(0, 0, 0);
    // Marquer la case de départ comme visitée (on y est)
    maze_mark_visited(0, 0);
    Serial.println("[TREM] Initialisation — départ (0,0) direction Nord");
}

// ── tremaux_update ────────────────────────────────────────────
bool tremaux_update() {
    switch (s_phase) {

        // ── SCAN : lire les capteurs + mettre à jour la carte ──
        case TREM_SCAN: {
            ToFReadings tof;
            sensors_read(tof);

            bool front = wall_front(tof);
            bool left  = wall_left(tof);
            bool right = wall_right(tof);

            // Mettre à jour les murs de la case courante dans la carte
            maze_update_walls(front, left, right);

            Serial.printf("[TREM] Scan (%d,%d) dir=%d : F=%d G=%d D=%d\n",
                maze_get_row(), maze_get_col(), maze_get_dir(),
                (int)front, (int)left, (int)right);

            s_phase = TREM_DECIDE;
            break;
        }

        // ── DECIDE : choisir la prochaine direction ────────────
        case TREM_DECIDE: {
            s_chosen_dir = tremaux_choose_direction();
            uint8_t current_dir = maze_get_dir();

            Serial.printf("[TREM] Décision : dir=%d (actuelle=%d)\n",
                s_chosen_dir, current_dir);

            if (s_chosen_dir == current_dir) {
                // Déjà orienté dans la bonne direction → avancer directement
                s_phase = TREM_MOVE;
                nav_start_advance();
            } else {
                // Calculer le virage nécessaire en quarts de tour
                int diff = (int)s_chosen_dir - (int)current_dir;
                // Normaliser dans [-2, 2] (le plus court chemin de rotation)
                if (diff >  2) diff -= 4;
                if (diff < -2) diff += 4;
                // diff : +1=droite, -1=gauche, +2 ou -2=demi-tour → on prend +2

                s_phase = TREM_ORIENT;
                nav_start_turn(diff);
            }
            break;
        }

        // ── ORIENT : attendre la fin de la rotation ────────────
        case TREM_ORIENT: {
            NavState ns = nav_update();
            if (ns == NAV_DONE) {
                // Mettre à jour l'orientation du robot dans la carte
                maze_set_pos(maze_get_row(), maze_get_col(), s_chosen_dir);

                // Démarrer l'avance vers la case suivante
                s_phase = TREM_MOVE;
                nav_start_advance();
            }
            break;
        }

        // ── MOVE : attendre la fin de l'avance ────────────────
        case TREM_MOVE: {
            NavState ns = nav_update();
            if (ns == NAV_DONE) {
                s_phase = TREM_UPDATE;
            }
            break;
        }

        // ── UPDATE : mise à jour de la position dans la carte ─
        case TREM_UPDATE: {
            // Avancer la position logique du robot dans la carte
            maze_advance_robot();
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
