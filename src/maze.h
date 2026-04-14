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

// =============================================================
//  Position et orientation du robot dans le labyrinthe
//  Directions : 0=Nord, 1=Est, 2=Sud, 3=Ouest
// =============================================================

// Retourne la position et direction actuelles du robot
uint8_t maze_get_row();
uint8_t maze_get_col();
uint8_t maze_get_dir();

// Modifie la position/direction du robot (sans déplacement physique)
void maze_set_pos(uint8_t row, uint8_t col, uint8_t dir);

// Avance la position du robot d'une case dans la direction courante
// (à appeler APRÈS un nav_start_advance() réussi)
void maze_advance_robot();

// =============================================================
//  Mise à jour de la carte depuis les capteurs
// =============================================================

// Met à jour les murs de la case courante selon les lectures ToF.
// front/left/right : true = mur présent dans cette direction RELATIVE au robot
// (left/right sont les capteurs latéraux à 45° → mur si distance < TOF_WALL_SIDE_MM)
void maze_update_walls(bool front, bool left, bool right);

// Retourne les murs d'une case (bitmask WALL_N/E/S/W)
uint8_t maze_get_walls(uint8_t row, uint8_t col);

// =============================================================
//  Trémaux — suivi des visites
// =============================================================

// Incrémente le compteur de visites de la case (max 2)
void maze_mark_visited(uint8_t row, uint8_t col);

// Retourne le nombre de visites (0, 1, ou 2)
uint8_t maze_get_visited(uint8_t row, uint8_t col);

// Retourne true si toutes les cases accessibles ont been visited >= 1
bool maze_is_fully_explored();
