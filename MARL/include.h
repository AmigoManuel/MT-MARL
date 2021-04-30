/////////////////////////////////////////////////////////////////////
// Xiaoxun Sun & Sven Koenig @ USC 2010
// All rights reserved
/////////////////////////////////////////////////////////////////////

#ifndef INCLUDEH
#define INCLUDEH

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define INFORMEDSEARCH           /* use Manhattan distance  */

#define TESTRTAASTAR         /* test RTAA star */
//#define TESTLSSLRTA         /* test LSS-LRTA star */   

#define GAMEMAP

//#define RANDOMMOVES

#define WITHSOLUTION

//#define EIGHTCONNECTED    // must defined 8-connected grids .


#define RUNS 50
#define RANDOMMAZE            /* must define this, becasue 8-connect maze can not be generated   */

#define MAZEWIDTH  12
#define MAZEHEIGHT  11


#define MAZEDENSITY 0.2     /* percentage of blocked cells if RANDOMMAZE is defined       */

#define MAX_TIME_STEPS 1000
#define NAGENTS 10
// Medida de tiempo que el agente recuerda
#define MEMORY 3

float hvalues[MAZEWIDTH * MAZEHEIGHT][NAGENTS];
int agent_expansions[NAGENTS];
float agent_cost[NAGENTS];
// valores heuristicos A* por agente
float hValueForAgent[NAGENTS];
int **agent_locations;

//#define DISPLAY
#define STATISTICS     /* should disable this for timing, however it is necessay to count the number of propagation */
#define STATISTICS2    /* ACCOUNT the traced number*/
//#define DEBUG        /* whether debugging is on  */
//#define TORUS                /* must define TORUS */
#define REVERSE         /* A* search from the target back to the start    */

#define LARGE  10000000
#define BASE   100000
#define ALLMAXSEARCHES 100000000
#define WALLSTOREMOVE 0           /* number of walls to remove if RANDOMMAZE is NOT defined - infinite loop if too large */
#define RANDOMSTARTGOAL           /* whether the start and goal state are drawn randomly                                 */
#define STARTX 1                  /* x coordinate of the start cell                                                      */
#define STARTY 9                  /* y coordinate of the start cell                                                      */
#define GOALX 5                  /* x coordinate of the goal  cell                                                      */
#define GOALY 9                  /* y coordinate of the goal  cell                                                      */
#define TIEBREAKING 1             /* D* lite 0: no tie breaking, 1: towards smaller g-values; 2: towards larger g-values */
#define ALPHA 0.0/* learning rate for the direction vector*/


#ifdef EIGHTCONNECTED
#define DIRECTIONS 8
    static int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static int dy[8] = {0, 1, 1,  1,    0, -1, -1, -1};
//nuevas    static int dx[8] = {0, -1, -1, -1,  0,  1, 1, 1};
//    static int dy[8] = {1,  1,  0, -1, -1, -1, 0, 1};

static int reverse[8] = {4, 5, 6, 7, 0,  1,  2,  3};
#else // 4-connected
#define DIRECTIONS 5

    // Pares de movimientos en ejes x e y
    static int dx[DIRECTIONS] = {1, 0, -1, 0, 0};
static int dy[DIRECTIONS] = {0, 1, 0, -1, 0};

static int reverse[DIRECTIONS] = {2, 3, 0, 1, 4};
#endif
long int statpercolated2;


#define max(x, y) ( (x) > (y) ? (x) : (y) )
#define min(x, y) ( (x) < (y) ? (x) : (y) )

#endif
