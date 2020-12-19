/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#ifndef MAZEH
#define MAZEH

#include "include.h"

struct cell1;
typedef struct cell1 cell1;

struct cell1 {
    int x, y;
    short obstacle;
    short blocked;
    double g;
    double h;
    double key;
    long int key3;
    int iteration;
    int heapindex;
    int heapindex3;
    int overexpanded;
    int update_iteration;
    double cost[DIRECTIONS];
    cell1 *move[DIRECTIONS];
    cell1 *succ[DIRECTIONS];
    cell1 *searchtree;
    cell1 *trace;
};

cell1 *position[NAGENTS];
cell1 *goal[NAGENTS];

cell1 **maze1;
cell1 **mazeall;
cell1 **maze_actual;

int **mazeh;
cell1 *mazestart1, *mazegoal1;
cell1 *targetall;
cell1 *robotall;

int mazeiteration, searchiteration;
int RUN;

void newrandommaze_astar();

#endif
