#ifndef MAZEH
#define MAZEH

#include "include.h"

struct cell1;
struct DCell;
typedef struct DCell DCell;
typedef struct cell1 cell1;

/* Definición de celda */
struct cell1 {
    // Coordenadas
    int x, y;
    // Indicador de obstaculo
    short obstacle;
    // Indica que tiene un obstaculo
    short blocked[100];
    short blockedAgent[NAGENTS][100];
    // costo
    double g;
    double g_backup;
    double penalty;
    double h;
    double key;
    long int key3;
    short pathlength;
    // Contador de iteaciones
    int iteration;
    // Index dentro del heap
    int heapindex;
    // Index dentro del dijkstra heap
    int heapindex3;
    int overexpanded;
    int update_iteration;
    // Movimientod de agente [a][costo][j]
    int agentMovingTo[NAGENTS][100][NAGENTS];
    int toTransition[NAGENTS][100];
    int fromTransition[NAGENTS][100];
    int numConflicts[NAGENTS];
    int depth[NAGENTS];
    int degree[NAGENTS];
    int marked[NAGENTS][100];
    int tmpdepth[NAGENTS];
    double cost[DIRECTIONS];
    // Movimientos posibles dentro de la casilla actual
    cell1 *move[DIRECTIONS];
    cell1 *succ[DIRECTIONS];
    cell1 *searchtree;
    cell1 *parent[NAGENTS];
    cell1 *tmpsearchtree[150];
    // Guarda la ruta realizada por el agente
    cell1 *trace;
};

struct DCell {
    int x, y;
    double velx, vely;

};


DCell **DM;
cell1 *position[NAGENTS];
cell1 *intermediate_position[NAGENTS];
cell1 *goal[NAGENTS];
//cell1 *track[NAGENTS][NAGENTS][MEMORY];

cell1 **maze1;
cell1 **mazeall;
cell1 **maze_actual;

int **mazeh;
cell1 *mazestart1, *mazegoal1;
cell1 *targetall;
cell1 *robotall;

int mazeiteration, searchiteration;
int RUN;

#endif
