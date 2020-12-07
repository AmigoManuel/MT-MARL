/////////////////////////////////////////////////////////////////////
// Xiaoxun Sun & Sven Koenig @ USC 2009
// All rights reserved
/////////////////////////////////////////////////////////////////////
#include "include.h"
#include "maze.h"

int mazeiteration = 0;
int goaly;// = GOALY;
int goalx;// = GOALX;
int starty;// = STARTY;
int startx;// = STARTX;


#ifdef EIGHTCONNECTED
#define HA(from,to) (  (sqrt(2)) * min(abs((from)->y - (to)->y), abs((from)->x - (to)->x))) + ((1) * ( max(abs((from)->y - (to)->y), abs((from)->x - (to)->x )) - min( abs((from)->y - (to)->y), abs((from)->x - (to)->x )))     )
#else
#define HA(from, to) ((1)* ( abs( (from)->y - (to)->y  ) + abs( (from)->x - (to)->x )   ))
#endif
//#define HA(from, to) ((10) * (min( abs( (from)->y - (to)->y), abs((from)->x - (to)->x ))))

//---------------------------------------------------------------------------------------------------------
void preprocessmaze_astar() {
    int x, y, d;
    int newx, newy;

    if (maze1 == NULL) {
        maze1 = (cell1 **) calloc(MAZEHEIGHT, sizeof(cell1 *));
        for (y = 0; y < MAZEHEIGHT; ++y)
            maze1[y] = (cell1 *) calloc(MAZEWIDTH, sizeof(cell1));
        // for for 1.
        for (y = 0; y < MAZEHEIGHT; ++y)
            for (x = 0; x < MAZEWIDTH; ++x) {
                maze1[y][x].x = x;
                maze1[y][x].y = y;
                for (d = 0; d < DIRECTIONS; ++d) {
                    newy = y + dy[d];
                    newx = x + dx[d];
                    maze1[y][x].succ[d] = (newy >= 0 && newy < MAZEHEIGHT && newx >= 0 && newx < MAZEWIDTH)
                                          ? &maze1[newy][newx] : NULL;

#ifdef EIGHTCONNECTED
                    if(d % 2 == 0)
                        maze1[y][x].cost[d] = 10;   // (2)
                    else
                        maze1[y][x].cost[d] = 14;  //2.
#else
                    maze1[y][x].cost[d] = 10;   // (2)
#endif

                }
            }
    }  // end     if (maze1 == NULL)
}

//------------------------------------------------------------------------------------
void postprocessmaze_astar() {
    int x, y;
    int d1, d2;
    cell1 *tmpcell3;

//2007.04.15
    mazestart1->obstacle = 0;
    mazegoal1->obstacle = 0;

    // for for 2
    for (y = 0; y < MAZEHEIGHT; ++y)
        for (x = 0; x < MAZEWIDTH; ++x) {
            maze1[y][x].iteration = 0;
            maze1[y][x].overexpanded = 0;

            for (d1 = 0; d1 < DIRECTIONS; ++d1) {
                maze1[y][x].move[d1] = (!maze1[y][x].obstacle && maze1[y][x].succ[d1] &&
                                        !maze1[y][x].succ[d1]->obstacle) ? maze1[y][x].succ[d1] : NULL;
            }
        }

#ifdef UNKNOWN
    for (d1 = 0; d1 < DIRECTIONS; ++d1)
    if (mazestart1->move[d1] && mazestart1->move[d1]->obstacle)
    {
        tmpcell3 = mazestart1->move[d1];
        for (d2 = 0; d2 < DIRECTIONS; ++d2)
        if (tmpcell3->move[d2])
        {
            tmpcell3->move[d2] = NULL;
            tmpcell3->succ[d2]->move[reverse[d2]] = NULL;
        }
    }
#endif
}


//-------------------------------------------------------------------------------------------------
void newrandommaze_astar()     // for pure astar
{
    int d, d1, d2;
    int x, y;
    int newx, newy;
    int goaly, goalx;
    int starty, startx;
    cell1 *tmpcell3;

    for (y = 0; y < MAZEHEIGHT; ++y)
        for (x = 0; x < MAZEWIDTH; ++x) {
            maze1[y][x].x = x;
            maze1[y][x].y = y;
            for (d = 0; d < DIRECTIONS; ++d) {
                newy = y + dy[d];
                newx = x + dx[d];
                maze1[y][x].succ[d] = (newy >= 0 && newy < MAZEHEIGHT && newx >= 0 && newx < MAZEWIDTH)
                                      ? &maze1[newy][newx] : NULL;
                maze1[y][x].cost[d] = sqrt(pow((newx - x), 2.0) + pow((newy - y), 2.0));
                //printf("[%d], maze1[y][x].cost[d]:%f\n",d,maze1[y][x].cost[d]);getchar();
            }
        }

    //printf("aqui:%d\n");

    for (y = 0; y < MAZEHEIGHT; ++y) {
        for (x = 0; x < MAZEWIDTH; ++x) {
            maze1[y][x].iteration = 0;
            maze1[y][x].overexpanded = 0;
            maze1[y][x].trace = NULL;
            maze1[y][x].blocked = 0;

            for (d1 = 0; d1 < DIRECTIONS; ++d1) {
                maze1[y][x].move[d1] = (!maze1[y][x].obstacle && maze1[y][x].succ[d1] &&
                                        !maze1[y][x].succ[d1]->obstacle) ? maze1[y][x].succ[d1] : NULL;
            }
        }
    }


    for (d = 0; d < NAGENTS; d++) {
        position[d]->blocked = 1;
        agent_expansions[d] = 0;
        agent_cost[d] = 0;
    }

// H initialitation
    for (d = 0; d < NAGENTS; d++)
        for (y = 0; y < MAZEHEIGHT; ++y)
            for (x = 0; x < MAZEWIDTH; ++x)
                hvalues[MAZEWIDTH * y + x][d] = HA(&maze1[y][x], goal[d]);

}
