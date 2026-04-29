// =============================================================
//  bfs.cpp — Implémentation BFS (Breadth-First Search)
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================

#include "bfs.h"
#include "maze.h"

// ── Tableaux statiques (pas d'allocation dynamique) ───────────
// Max 25 cases pour un labyrinthe 5×5
static PathPoint s_path[MAZE_SIZE * MAZE_SIZE];
static int8_t    s_turns[MAZE_SIZE * MAZE_SIZE];
static uint8_t   s_path_length = 0;

// Directions : 0=Nord, 1=Est, 2=Sud, 3=Ouest
static const int8_t  dRow[4]    = { -1,  0,  1,  0 };
static const int8_t  dCol[4]    = {  0,  1,  0, -1 };
static const uint8_t wallBit[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

// =============================================================
//  bfs_compute — Calcul du plus court chemin par BFS
// =============================================================
uint8_t bfs_compute(uint8_t start_row, uint8_t start_col,
                    uint8_t end_row,   uint8_t end_col) {
    s_path_length = 0;

    // ── File FIFO statique ────────────────────────────────────
    struct BFSNode { uint8_t row, col; };
    BFSNode queue[MAZE_SIZE * MAZE_SIZE];
    uint8_t head = 0, tail = 0;

    // parent[r][c] = direction d'où on est arrivé en (r,c)
    // 255 = non visité
    uint8_t parent[MAZE_SIZE][MAZE_SIZE];
    bool    visited[MAZE_SIZE][MAZE_SIZE];

    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            parent[r][c]  = 255;
            visited[r][c] = false;
        }
    }

    // ── Enfiler le point de départ ────────────────────────────
    queue[tail++]                  = { start_row, start_col };
    visited[start_row][start_col]  = true;

    bool found = false;

    // ── Boucle BFS ────────────────────────────────────────────
    while (head < tail && !found) {
        BFSNode cur = queue[head++];

        for (int d = 0; d < 4; d++) {
            // Mur dans cette direction ? → bloquer
            if (maze_get_walls(cur.row, cur.col) & wallBit[d]) continue;

            int8_t nr = cur.row + dRow[d];
            int8_t nc = cur.col + dCol[d];

            // Hors limites
            if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;

            // Déjà visité
            if (visited[nr][nc]) continue;

            visited[nr][nc]  = true;
            parent[nr][nc]   = d;
            queue[tail++]    = { (uint8_t)nr, (uint8_t)nc };

            if (nr == end_row && nc == end_col) {
                found = true;
                break;
            }
        }
    }

    if (!found) {
        Serial.println("[BFS] Pas de chemin trouvé !");
        return 0;
    }

    // ── Reconstruction du chemin en remontant les parents ─────
    // On part de la fin et on remonte jusqu'au départ
    uint8_t r = end_row, c = end_col;
    while (r != start_row || c != start_col) {
        s_path[s_path_length++] = { r, c };
        uint8_t d = parent[r][c];
        // Remonter dans la direction opposée
        r = (uint8_t)((int8_t)r - dRow[d]);
        c = (uint8_t)((int8_t)c - dCol[d]);
    }
    s_path[s_path_length++] = { start_row, start_col };

    // Inverser le chemin (il est stocké fin→départ, on veut départ→fin)
    for (int i = 0; i < s_path_length / 2; i++) {
        PathPoint tmp              = s_path[i];
        s_path[i]                  = s_path[s_path_length - 1 - i];
        s_path[s_path_length-1-i]  = tmp;
    }

    // ── Affichage debug ───────────────────────────────────────
    Serial.printf("[BFS] Chemin trouvé : %d cases\n", s_path_length);
    for (int i = 0; i < s_path_length; i++) {
        Serial.printf("  [%d] (%d,%d)\n", i, s_path[i].row, s_path[i].col);
    }

    return s_path_length;
}

// ── Accesseurs ───────────────────────────────────────────────
const PathPoint* bfs_get_path()        { return s_path; }
uint8_t          bfs_get_path_length() { return s_path_length; }

// =============================================================
//  bfs_compute_turns — Séquence de virages
// =============================================================
void bfs_compute_turns(uint8_t start_dir) {
    if (s_path_length < 2) return;

    uint8_t current_dir = start_dir;

    for (int i = 0; i < s_path_length - 1; i++) {
        // Direction nécessaire pour aller de path[i] à path[i+1]
        int8_t dr = s_path[i+1].row - s_path[i].row;
        int8_t dc = s_path[i+1].col - s_path[i].col;

        uint8_t needed_dir;
        if      (dr == -1) needed_dir = 0;  // Nord
        else if (dc ==  1) needed_dir = 1;  // Est
        else if (dr ==  1) needed_dir = 2;  // Sud
        else               needed_dir = 3;  // Ouest

        // Calcul du virage relatif (la différence de direction)
        int diff = (int)needed_dir - (int)current_dir;
        if (diff >  2) diff -= 4;
        if (diff < -2) diff += 4;

        // diff = 0 (tout droit), +1 (droite), -1 (gauche), ±2 (demi-tour → +2)
        s_turns[i]  = (int8_t)diff;
        current_dir = needed_dir;
    }

    // Dernière case : pas de virage après
    s_turns[s_path_length - 1] = 0;

    // Affichage debug
    Serial.print("[BFS] Virages : ");
    for (int i = 0; i < s_path_length - 1; i++) {
        if      (s_turns[i] ==  0) Serial.print("^ ");
        else if (s_turns[i] ==  1) Serial.print("> ");
        else if (s_turns[i] == -1) Serial.print("< ");
        else                       Serial.print("U ");
    }
    Serial.println();
}

const int8_t* bfs_get_turns() { return s_turns; }
