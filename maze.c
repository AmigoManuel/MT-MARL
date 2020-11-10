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
#define HA(from, to) ((1)* ( abs( (from)->y - (to)->y  ) + abs( (from)->x - (to)->x )   )) //define HA(from,to) ( sqrtf( abs( (from)->y - (to)->y  )*abs( (from)->y - (to)->y  ) + abs( (from)->x - (to)->x )*abs( (from)->x - (to)->x )   ))  //#define HA(from,to) ((1)* ( abs( (from)->y - (to)->y  ) + abs( (from)->x - (to)->x )   ))
#endif
//#define HA(from, to) ((10) * (min( abs( (from)->y - (to)->y), abs((from)->x - (to)->x ))))

// h debería ser el valor de la heuristica
/* TODO: VERIFICAR
Inicialización de valores heuristicos */
void initialization_h_values_2D() {
    // Apuntador a celdas del laberinto
    cell1 *cellpas;
    int y, x;
    // Por cada agente a
    for (int a = 0; a < NAGENTS; a++) {
        // y por cada casilla en el laberinto
        for (y = 0; y < MAZEHEIGHT; ++y){
            for (x = 0; x < MAZEWIDTH; ++x){
                // Marca el valor h como LARGE
                // LARGE es un valor muy alto ref->include.h
                maze1[y][x].h = LARGE;
            }
        }
        // Apunta a el objetivo del agente actual
        cellpas = &maze1[goal[a]->y][goal[a]->x];
        // Escribe sobre su valor h un 0
        // Escribe sobre hvalues en la posición indicada un 0
        cellpas->h = hvalues[MAZEWIDTH * goal[a]->y + goal[a]->x][a] = 0;
        cellpas->key3 = cellpas->h;
        // Inserta la casilla dentro del heap
        insertheap3(cellpas);
        // Actualiza el heap
        while (topheap3() != NULL) {
            // toma el primer elemento del heap
            cellpas = popheap3();
            // Actualiza el valor del heap
            hvalues[MAZEWIDTH * cellpas->y + cellpas->x][a] = cellpas->h;
            //++statexpanded1_disjktra;
            // Por cada dirección d
            for (int d = 0; d < DIRECTIONS; ++d) {
                // Desde la celda actual
                // Si es posible el movimiento actual
                if (cellpas->move[d]) {
                    // Si el valor h actual es mayor que el ajustado
                    if (cellpas->move[d]->h > cellpas->h + cellpas->cost[d]) {
                        // Actualiza el valor de h
                        cellpas->move[d]->h = cellpas->h + cellpas->cost[d];
                        printf("H of cell [%d %d] is %i\n", cellpas->move[d]->y, cellpas->move[d]->x,cellpas->move[d]->h);
                        // Posiblemente sea un backup
                        cellpas->move[d]->key3 = cellpas->move[d]->h;
                        // Inserta la celda actualizada dentro del heap
                        insertheap3(cellpas->move[d]);
                    }
                }
            }
        }
    }
}

/* Reserva memoria sobre cada celda,
marca las casillas siguientes a las cuales posible llegar
desde una celda y asigna el costo de llegar a dicha casilla*/
void preprocessmaze_astar() {
    int x, y, d;
    int newx, newy;
    // Si el mapa de celdas no se encuentra definido
    if (maze1 == NULL) {
        // Reserva memoria para el mapa de celdas
        maze1 = (cell1 **) calloc(MAZEHEIGHT, sizeof(cell1 *));
        for (y = 0; y < MAZEHEIGHT; ++y){
            maze1[y] = (cell1 *) calloc(MAZEWIDTH, sizeof(cell1));
            // Enumera cada casilla en orden creciente
            for (y = 0; y < MAZEHEIGHT; ++y) {
                for (x = 0; x < MAZEWIDTH; ++x) {
                    maze1[y][x].x = x;
                    maze1[y][x].y = y;
                    // Por cada una de las direcciones posibles
                    for (d = 0; d < DIRECTIONS; ++d) {
                        // Asigna un nuevo x e y en aquella dirección
                        newy = y + dy[d];
                        newx = x + dx[d];
                        // succ es la casilla sucesora
                        // Asigna la posición correspondiente a la casilla siguiente (otra casilla o un null)
                        maze1[y][x].succ[d] = (newy >= 0 && newy < MAZEHEIGHT && newx >= 0 && newx < MAZEWIDTH) ? &maze1[newy][newx] : NULL;
                        #ifdef EIGHTCONNECTED
                        if(d % 2 == 0)
                            maze1[y][x].cost[d] = 10;   // (2)
                        else
                            maze1[y][x].cost[d] = 14;  //2.
                        #else
                        // Costo asociado al moverse en la dirección d
                        maze1[y][x].cost[d] = 10;   // (2)
                        #endif
                    }
                }
            }
        }
    }
}

/* Reinicia las iteraciones sobre las celdas,
 las marca como no expandidas y
 marca los movimientos siguientes para cada casilla */
void postprocessmaze_astar() {
    int x, y;
    int d1, d2;
    cell1 *tmpcell3;
    // Marca las celdas de incio y objetivo sin obstaculo
    mazestart1->obstacle = 0;
    mazegoal1->obstacle = 0;
    // Por cada casilla en el mapa
    for (y = 0; y < MAZEHEIGHT; ++y) {
        for (x = 0; x < MAZEWIDTH; ++x) {
            // Reiniciar su iteración a cero
            maze1[y][x].iteration = 0;
            // Marcar como no expandido
            maze1[y][x] .overexpanded = 0;
            for (d1 = 0; d1 < DIRECTIONS; ++d1) {
                /* Si en x/y no hay obstaculo, existe la casilla sucesora en d1 y
                 la casilla sucesora en d1 no tienes obstaculo, entonces asigna la casilla sucesora 
                 en la ddirección dada al movimiento en aquella dirección.
                */
                maze1[y][x].move[d1] = (!maze1[y][x].obstacle && maze1[y][x].succ[d1] && !maze1[y][x].succ[d1]->obstacle) ? maze1[y][x].succ[d1] : NULL;
            }
        }
    }

    #ifdef UNKNOWN
    for (d1 = 0; d1 < DIRECTIONS; ++d1){
        if (mazestart1->move[d1] && mazestart1->move[d1]->obstacle) {
            tmpcell3 = mazestart1->move[d1];
            for (d2 = 0; d2 < DIRECTIONS; ++d2){
                if (tmpcell3->move[d2]) {
                    tmpcell3->move[d2] = NULL;
                    tmpcell3->succ[d2]->move[reverse[d2]] = NULL;
                }
            }
        }
    }
    #endif
}

/* Determina las celdas siguientes y el costo asociado para maze1,
 Añade el posible movimiento desde una celda considerando bloqueos*/
void newrandommaze_astar() {
    int d, d1, d2;
    int x, y;
    int newx, newy;
    int goaly, goalx;
    int starty, startx;
    cell1 *tmpcell3;

    // Por cada celda en el mapa
    for (y = 0; y < MAZEHEIGHT; ++y) {
        for (x = 0; x < MAZEWIDTH; ++x) {
            // Asigna x e y de manera incremental
            maze1[y][x].x = x;
            maze1[y][x].y = y;
            // Por cada una de las direcciones
            for (d = 0; d < DIRECTIONS; ++d) { // printf("Whats here ");
                // Determina el nuevo valor de x e y, en base al movimiento posible
                newy = y + dy[d];
                newx = x + dx[d];
                /* Si los valores del nuevo x e y se encuentran dentro del mapa, entonces se
                 asigna la casilla siguiente en aquella dirección */
                maze1[y][x].succ[d] = (newy >= 0 && newy < MAZEHEIGHT && newx >= 0 && newx < MAZEWIDTH) ? &maze1[newy][newx] : NULL;
                // Añade el costo asociado a moverse en aquella dirección
                maze1[y][x].cost[d] = sqrt(pow((newx - x), 2.0) + pow((newy - y), 2.0));
            }
        }
    }    
    // Por cada celda en el maze1
    for (y = 0; y < MAZEHEIGHT; ++y) {
        for (x = 0; x < MAZEWIDTH; ++x) {
            // Marca la iteración en cero
            maze1[y][x].iteration = 0;
            // Marca como no expandida
            maze1[y][x].overexpanded = 0;
            // TODO: Posiblemente sea seguimiento del camino
            maze1[y][x].trace = NULL;
            // Una posible celda bloqueada por otro agente.
            maze1[y][x].blocked[0] = 0;
            // Por cada una de las direcciones posibles
            for (d1 = 0; d1 < DIRECTIONS; ++d1) {
                /* Si no es encuentra un obstaculo en x,y,
                 existe una casilla siguiente en el sentido d1 y
                 esa casilla siguiente no contiene obstaculo, 
                 entonces asigna la celda de movimiento en d1 para x,y */
                maze1[y][x].move[d1] = (!maze1[y][x].obstacle && maze1[y][x].succ[d1] && !maze1[y][x].succ[d1]->obstacle) ? maze1[y][x].succ[d1] : NULL;
            }
        }
    }

    // Por cada uno de los agentes
    for (d = 0; d < NAGENTS; d++) {
        // TODO: VERIFICAR - marca la celda del agente como bloqueada
        position[d]->blocked[0] = 1;
        // TODO: VERIFICAR - No expande la celda del agente
        agent_expansions[d] = 0;
        // TODO: VERIFICAR - Asigna costo cero al agente
        agent_cost[d] = 0;
        // TODO: realmente no entiendo cual es el objetivo de este ciclo
        for (int t = 1; t < 100; t++) {
            position[d]->blocked[t] = 0; //If agent believes that the future position is not predictable
            //position[d]->blocked[t]=1; //If agent believes that the position of agents will not change
        }
    }
    printf("INITIALIZE HEURISTICs\n");
    // Llama a incializar las heuristicas
    initialization_h_values_2D();
    printf("DONE WITH HEURISTICs\n");
}
