// =============================================================
//  bfs.h — Algorithme BFS (Breadth-First Search) — Run 2
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  Après le Run 1 (Trémaux), la carte est complète.
//  Le BFS calcule le chemin le plus court de la case de départ
//  vers la case d'arrivée.
//
//  Usage :
//    uint8_t len = bfs_compute(0, 0, 4, 4);
//    if (len > 0) {
//        bfs_compute_turns(maze_get_dir());
//        const PathPoint* path  = bfs_get_path();
//        const int8_t*    turns = bfs_get_turns();
//    }
// =============================================================
#pragma once
#include <Arduino.h>
#include "config.h"

// Un point du chemin (coordonnées d'une case)
struct PathPoint {
    uint8_t row;
    uint8_t col;
};

// Calcule le chemin optimal entre (sr,sc) et (er,ec)
// Retourne le nombre de cases dans le chemin (0 = pas de chemin)
uint8_t bfs_compute(uint8_t start_row, uint8_t start_col,
                    uint8_t end_row,   uint8_t end_col);

// Retourne le tableau de cases du chemin (taille = bfs_get_path_length())
const PathPoint* bfs_get_path();

// Retourne le nombre de cases dans le chemin calculé
uint8_t bfs_get_path_length();

// Convertit le chemin en séquence de virages relatifs au robot.
// start_dir : direction initiale du robot (0=N, 1=E, 2=S, 3=W)
// Résultat : turns[i] pour le déplacement de path[i] vers path[i+1]
//   0 = tout droit, +1 = virage droite, -1 = virage gauche, +2 = demi-tour
void bfs_compute_turns(uint8_t start_dir);

// Retourne le tableau de virages
const int8_t* bfs_get_turns();
