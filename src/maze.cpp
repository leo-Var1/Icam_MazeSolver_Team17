#include "maze.h"
#include <Arduino.h>

// Déclaration de la grille globale
Cell maze[MAZE_SIZE][MAZE_SIZE];

void maze_init() {
    for (uint8_t r = 0; r < MAZE_SIZE; r++) {
        for (uint8_t c = 0; c < MAZE_SIZE; c++) {
            maze[r][c].walls   = 0;
            maze[r][c].visited = 0;
            maze[r][c].known   = false;
        }
    }
}

// Retourne le mur opposé (N↔S, E↔W)
static uint8_t opposite_wall(uint8_t w) {
    switch (w) {
        case WALL_N: return WALL_S;
        case WALL_S: return WALL_N;
        case WALL_E: return WALL_W;
        case WALL_W: return WALL_E;
    }
    return 0;
}

void maze_set_wall(uint8_t row, uint8_t col, uint8_t wall_dir) {
    maze[row][col].walls |= wall_dir;

    // Met à jour la case voisine (cohérence de la carte)
    if (wall_dir == WALL_N && row > 0)
        maze[row-1][col].walls |= WALL_S;
    else if (wall_dir == WALL_S && row < MAZE_SIZE-1)
        maze[row+1][col].walls |= WALL_N;
    else if (wall_dir == WALL_E && col < MAZE_SIZE-1)
        maze[row][col+1].walls |= WALL_W;
    else if (wall_dir == WALL_W && col > 0)
        maze[row][col-1].walls |= WALL_E;
}

void maze_clear_wall(uint8_t row, uint8_t col, uint8_t wall_dir) {
    maze[row][col].walls &= ~wall_dir;

    if (wall_dir == WALL_N && row > 0)
        maze[row-1][col].walls &= ~WALL_S;
    else if (wall_dir == WALL_S && row < MAZE_SIZE-1)
        maze[row+1][col].walls &= ~WALL_N;
    else if (wall_dir == WALL_E && col < MAZE_SIZE-1)
        maze[row][col+1].walls &= ~WALL_W;
    else if (wall_dir == WALL_W && col > 0)
        maze[row][col-1].walls &= ~WALL_E;
}

// =============================================================
//  Position du robot
// =============================================================

// Directions : 0=N, 1=E, 2=S, 3=W
// Deltas de position pour avancer dans chaque direction
static const int8_t DIR_DR[4] = { -1,  0,  1,  0 }; // delta row
static const int8_t DIR_DC[4] = {  0,  1,  0, -1 }; // delta col

// Correspondance direction → bitmask mur
static const uint8_t DIR_WALL[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

static uint8_t s_robot_row = 0;
static uint8_t s_robot_col = 0;
static uint8_t s_robot_dir = 0; // 0=Nord au démarrage

uint8_t maze_get_row() { return s_robot_row; }
uint8_t maze_get_col() { return s_robot_col; }
uint8_t maze_get_dir() { return s_robot_dir; }

void maze_set_pos(uint8_t row, uint8_t col, uint8_t dir) {
    s_robot_row = row;
    s_robot_col = col;
    s_robot_dir = dir;
}

void maze_advance_robot() {
    int8_t nr = s_robot_row + DIR_DR[s_robot_dir];
    int8_t nc = s_robot_col + DIR_DC[s_robot_dir];
    // Garder dans les limites (sécurité)
    if (nr >= 0 && nr < MAZE_SIZE && nc >= 0 && nc < MAZE_SIZE) {
        s_robot_row = nr;
        s_robot_col = nc;
    }
}

// =============================================================
//  Mise à jour des murs depuis les capteurs
// =============================================================

void maze_update_walls(bool front, bool left, bool right) {
    uint8_t row = s_robot_row;
    uint8_t col = s_robot_col;
    uint8_t dir = s_robot_dir;

    // Direction frontale = dir du robot
    // Direction gauche = (dir + 3) % 4
    // Direction droite = (dir + 1) % 4
    uint8_t dir_front = dir;
    uint8_t dir_left  = (dir + 3) % 4;
    uint8_t dir_right = (dir + 1) % 4;

    if (front) maze_set_wall(row, col, DIR_WALL[dir_front]);
    if (left)  maze_set_wall(row, col, DIR_WALL[dir_left]);
    if (right) maze_set_wall(row, col, DIR_WALL[dir_right]);

    // Marquer la case comme connue (murs vérifiés)
    maze[row][col].known = true;
}

uint8_t maze_get_walls(uint8_t row, uint8_t col) {
    return maze[row][col].walls;
}

void maze_mark_visited(uint8_t row, uint8_t col) {
    if (maze[row][col].visited < 2) {
        maze[row][col].visited++;
    }
}

uint8_t maze_get_visited(uint8_t row, uint8_t col) {
    return maze[row][col].visited;
}

bool maze_is_fully_explored() {
    // Considère l'exploration terminée quand toutes les cases connues
    // ont been visitées au moins une fois.
    // Une case non connue = impossible à atteindre = ignorée.
    for (uint8_t r = 0; r < MAZE_SIZE; r++) {
        for (uint8_t c = 0; c < MAZE_SIZE; c++) {
            if (maze[r][c].known && maze[r][c].visited == 0) {
                return false;
            }
        }
    }
    return true;
}

void maze_print() {
    Serial.println("\n=== CARTE DU LABYRINTHE ===");
    for (uint8_t r = 0; r < MAZE_SIZE; r++) {
        // Ligne du dessus
        for (uint8_t c = 0; c < MAZE_SIZE; c++) {
            Serial.print("+");
            Serial.print((maze[r][c].walls & WALL_N) ? "---" : "   ");
        }
        Serial.println("+");
        // Côtés
        for (uint8_t c = 0; c < MAZE_SIZE; c++) {
            Serial.print((maze[r][c].walls & WALL_W) ? "|" : " ");
            Serial.print(" ");
            Serial.print(maze[r][c].visited);
            Serial.print(" ");
        }
        Serial.println("|");
    }
    // Dernière ligne du bas
    for (uint8_t c = 0; c < MAZE_SIZE; c++) {
        Serial.print("+");
        Serial.print((maze[MAZE_SIZE-1][c].walls & WALL_S) ? "---" : "   ");
    }
    Serial.println("+");
}
