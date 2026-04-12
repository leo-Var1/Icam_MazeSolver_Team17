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
