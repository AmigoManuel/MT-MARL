/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#ifndef AGENT
#define AGENT

#include "include.h"

// int blockedAgent[MAZEWIDTH][MAZEHEIGHT][NAGENTS][100];
// int mostProbPositionY[NAGENTS][NAGENTS][100]; //Max lookahead 100

int track[NAGENTS][NAGENTS][MEMORY][3];
int obsNextCell[NAGENTS][NAGENTS][DIRECTIONS+1];
int lastMove[NAGENTS][NAGENTS];
int realDepth[NAGENTS];
float nextCellProb[NAGENTS][NAGENTS][DIRECTIONS+1];
int mostProbPositionXY[NAGENTS][NAGENTS][100][3]; //Max lookahead 100
int lastMobileCellDist[NAGENTS];
int lastMobileCellX[NAGENTS];
int lastMobileCellY[NAGENTS]; 
int initialCellX[NAGENTS];
int initialCellY[NAGENTS];
// Coste de camino asociado al agente (se compara con formula)
int agentInfo[NAGENTS];
int backtrack[NAGENTS];
int distanceFromStart[NAGENTS];
/* Tipo de conflicto existente entre agentes 
 * si es 0 es un punto de interseccion,
 * si es -1,
 * si es 1 */
int conflictType[NAGENTS][NAGENTS];

float conflictCost[NAGENTS][MAZEWIDTH][MAZEHEIGHT][100];//Max lookahead 100
// Indicador de deadlocks
float deadlock[NAGENTS][MAZEWIDTH][MAZEHEIGHT][100];
 /* 
int agentVelx[NAGENTS], agentVely[NAGENTS];

 
int agentMovingTo[MAZEWIDTH][MAZEHEIGHT][NAGENTS][100][NAGENTS];
 
 
 

 */int canSee[NAGENTS][NAGENTS];
 int role[NAGENTS][NAGENTS];
 float backupH[MAZEWIDTH*MAZEHEIGHT][NAGENTS];
 

#endif
