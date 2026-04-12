#pragma once
#include <stdint.h>
#include "config.h"

// =============================================================
//  maze.h — Représentation de la grille 5×5
//
//  Chaque case a :
//   - walls    : bitmask des murs présents (WALL_N/E/S/W)
//   - visited  : nombre de fois traversée (Trémaux)
//   - known    : true si les murs ont été détectés
// =============================================================

struct Cell {
    uint8_t walls;    // bitmask : WALL_N | WALL_E | WALL_S | WALL_W
    uint8_t visited;  // compteur Trémaux (0, 1 ou 2)
    bool    known;    // true = murs de cette case cartographiés
};

// Grille globale (statique, pas d'allocation dynamique)
extern Cell maze[MAZE_SIZE][MAZE_SIZE];

// Initialise toutes les cases (murs inconnus, visited=0)
void maze_init();

// Pose un mur entre deux cases adjacentes (met à jour les deux côtés)
void maze_set_wall(uint8_t row, uint8_t col, uint8_t wall_direction);

// Retire un mur (passage libre)
void maze_clear_wall(uint8_t row, uint8_t col, uint8_t wall_direction);

// Affiche la carte en Serial (debug)
void maze_print();
