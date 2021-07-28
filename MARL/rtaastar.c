#include "rtaastar.h"

#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "agent.h"
#include "heap.h"
#include "include.h"
#include "math.h"
#include "maze.h"
#include "stdio.h"
#include "testall.h"

#ifdef TESTRTAASTAR
#define MAZEITERATIONS 1000000
unsigned long int targetdiscount[MAZEITERATIONS];
#define MAXSEARCHES 100000000
unsigned long int pathlengths[MAXSEARCHES];

// Xiaoxun: this is for 8-connected grids, with sqrt(2) as the cost for diagonal
// movements
#ifdef EIGHTCONNECTED
#define HA(from, to)                                                        \
    ((sqrt(2)) * min(abs((from)->y - (to)->y), abs((from)->x - (to)->x))) + \
        ((1) * (max(abs((from)->y - (to)->y), abs((from)->x - (to)->x)) -   \
                min(abs((from)->y - (to)->y), abs((from)->x - (to)->x))))
#else
#define HA(from, to) \
    ((1) * (abs((from)->y - (to)->y) + abs((from)->x - (to)->x)))
#endif
#define euclidian(from, to) \
    (sqrt(pow(((from)->x - (to)->x), 2.0) + pow(((from)->y - (to)->y), 2.0)))

#define QUEUE2_SIZE 100000
#define QUEUE2_PUSH(x)                 \
    {                                  \
        queue[pf2] = (x);              \
        (x)->heapindex = pf2;          \
        pf2 = (pf2 + 1) % QUEUE2_SIZE; \
        n2++;                          \
    }
#define QUEUE2_POP(x)                  \
    {                                  \
        (x) = queue[pi2];              \
        (x)->heapindex = -1;           \
        pi2 = (pi2 + 1) % QUEUE2_SIZE; \
        n2--;                          \
    }

long int max_expansions = 0;
long int counter = 0;
long int callclean = 0;
short flag_success1 = 0;
short d, d1, d2;
int i;
long int RUN1;
int times_of_billion1 = 0;
long long int expansion_persearch1[ALLMAXSEARCHES];
long long int statexpanded1_initial;
int keymodifier1 = 0;
long long int robot_steps1 = 0;
long long int mazeiteration1 = 0;
long long int searches_astar1 = 0;
double solution_cost = 0;
long long int statexpanded_percase1 = 0;
long long int robotmoves_total1 = 0;
unsigned long long int statexpanded1 = 0;
unsigned long long int statexpanded1_prune = 0;
unsigned long long int statexpanded1_first = 0;
static unsigned int InOpen = 0;
long int open_size = 0;
long int memory_count = 0;
int cont_closed = 0;
int cont_connected = 0;
short active_prune =
    0; // active the prune when there is an obstacle/border in the search tree
cell1 *CLOSED[10000];
cell1 *path[NAGENTS][200]; // 200 is a limit for lookahead, change accordingly
cell1 *idealPath[NAGENTS][200];
int learningCutoff[NAGENTS];
double f_value = 0;
double total_cost = 0;
double total_time_cost = 0;

float time_astar_initialize1 = 0;
float time_astar_first1 = 0;
float time_min_computer_astar = 0;
float time_max_computer_astar = 0;
float time_deadline = 0;
float time_searches = 0;
float standar_deviation_computer_astar = 0;
float standar_deviation_array[100000];
int sdIndex = 0;
short finish_all = NAGENTS;
// float avg_finish=0;
int time_step;
int totalpredictions[10];
int badp = 0;
int totp = 0;
int goop = 0;
int badpredictions[10];
short lastfinish = -1000;
short goal_reached[NAGENTS];
int pathlength[NAGENTS];
int pred_agents[NAGENTS][NAGENTS];
int good_pred_agents[NAGENTS][NAGENTS];
int predict[NAGENTS][NAGENTS];
int completion_time[NAGENTS];
int push_over[NAGENTS];
cell1 *last_recently_see[NAGENTS];

cell1 *tmpcell1;
cell1 *tmpcell2;
cell1 *tmpcell3;
cell1 *lastMobileState[NAGENTS];
struct timeval tv11, tv22, tv11a, tv22a, tv11b, tv22b, tv11c, tv22c, tv11d,
    tv22d, tv11e, tv22e;

long numberofexp, hsal;

int enable_print = 0;
int push_out_count = 0;

/* Inicialización de A* */
void initialize_astar() {
    // Setea las iteraciones sobre el mapa en 0
    mazeiteration1 = 0;
    // Setea los pasos realizados por el robot en 0
    robot_steps1 = 0;
    keymodifier1 = 0;
    statpercolated2 = 0;
    // Vacia el heap
    emptyheap2();
    // Por cada agente
    for (int i = 0; i < NAGENTS; i++) {
        // Marca el destino como no logrado
        goal_reached[i] = 0;
        // Por cada otro agente
        for (int a = 0; a < NAGENTS; a++) {
            // Borra el camino almacenado en memoria
            for (int t = 0; t < MEMORY; t++) {
                track[i][a][t][0] = -1;
                track[i][a][t][1] = -1;
            }
            // Quita la referencia a la siguiente celda
            for (int k = 0; k <= DIRECTIONS; k++) {
                obsNextCell[i][a][k] = 0;
            }
        }
    }
}

/* TODO: Acotación - las siguientes dos funciones resultan de cierta manera
ineficientes, de ser posible sería mejor reemplazar el procedimiento por hashing
o utilizar una referencia
desde la celda al agente que se encuentre en ella, de esta forma se evita la
busqueda lineal */

/* Determina si auxcell esta ocupada por un agente y retorna el agente
 * referenciado
 * @param auxcell celda a determinar referencia
 * @return retorna si la celda se encuentra ocupada o no */
int agent_acupation(cell1 *auxcell) {
    // Por cada uno de los agentes
    for (int i = 0; i < NAGENTS; i++)
        // Determina si la celda del agente es igual auxcell
        if (position[i] == auxcell)
            // En tal caso entrega el agente referenciado
            return i + 1;
    // En caso contrario retorna 0
    return 0;
}

/* Determina si auxcell es el goal de otro agente y retornando la referencia al
 * agente
 * @param auxcell celda a determinar referencia
 * @return retorna si la celda se encuentra ocupada o no */
int goal_acupation(cell1 *auxcell) {
    // Por cada uno de los agentes
    for (int i = 0; i < NAGENTS; i++)
        // Determina si auxcell es goal de un agente
        if (goal[i] == auxcell)
            // Retorna la referencia a ese agente
            return i + 1;
    // En caso contrario retorna 0
    return 0;
}

/* Despliega el mapa con los agentes, obstaculos, celdas objetivo y celdas
 * vacías por pantalla */
void multi_print_grid() {
    int x, y, x0, xf, y0, yf, w, aoc, goc;
    long int num1, num2, num;
    double n, dif = 0;
    // Por cada celda en el mapa
    for (y = 0; y < MAZEHEIGHT; ++y) {
        for (x = 0; x < MAZEWIDTH; ++x) {
            // Apunta a la celda x,y
            tmpcell1 = &maze1[y][x];
            // Determina si hay un agente aoc en la celda
            aoc = agent_acupation(tmpcell1);
            // Determina si la celda es goal y es utilizada por otro agente goc
            goc = goal_acupation(tmpcell1);
            // Si es un obstaculo despliega un #
            if (maze1[y][x].obstacle == 1) {
                printf("\e[42m");
                printf("%2s", "#");
                printf("\e[0m");
            } else {
                // Si se cumple aoc despliega el agente
                if (aoc) {
                    printf("\e[44m");
                    printf("%2d", aoc);
                    printf("\e[0m");
                } else {
                    // Si se cumple goc despliega el agente
                    if (goc) {
                        printf("\e[43m");
                        printf("%2d", goc);
                        printf("\e[0m");
                    }
                    // Caso contrario solo despliega un 0
                    else {
                        printf("\e[31m");
                        printf("%2d", 0);
                        printf("\e[0m");
                    }
                }
            }
        }
        printf("\n");
    }
}

/* Inicializa el costo g sobre la celda y setea las iteraciones
 * @param tmpcell celda a inicializar */
void initialize_state(cell1 *tmpcell) {
    // Si no se ha cruzado sobre la celda
    if (tmpcell->iteration == 0) {
        tmpcell->g = LARGE;
    } else
        // Si la iteración sobre la celda difiere a las iteraciones sobre el mapa
        if (tmpcell->iteration != mazeiteration1) {
        tmpcell->g = LARGE;
    }
    // Setea las iteraciones sobre la celda igual a las del mapa
    tmpcell->iteration = mazeiteration1;
    return;
}

/* TODO: Determina las situaciones de conflicto entre agentes,
   define las restricciones sobre estas y
   ejecuta decisiones sobre estas en base al entorno de ambos agentes
 * @param a Agente actual
 * @param lookahead estado de lookhead
 * @param formula Es comparada con agentInfo de todos los vecinos para dar
 preferencia
 * @param currentCell Celda actual sobre la cual itera
 * @param initialState El estado inicial es 1 si esta es la primera llamada por
 el agente
*/
void determine_constraints(int a, int lookahead, int formula,
                           cell1 *currentCell, int initialState) {

    // Inicializa los valores sobre path evitando referencias a null
    for (int a = 0; a < NAGENTS; a++) {
        for (int future = 0; future < 200; future++) {
            path[a][future] = currentCell;
        }
    }

    // int conflictsPosition=0;
    int conflictStepMe, conflictStepOther;
    int cell_role;
    // Verifica otros agentes por posibles conflictos en la celda actual y
    // tambien por celdas futuras. Por cada uno de los agentes
    for (int j = 0; j < NAGENTS; ++j) {
        int distanceFromAgent;
        // Verifica que a sea sobre otro agente y no el mismo
        if (a != j) {
            if (enable_print) printf("Checking constraints between my ideal path and agent %i (me: %i "", other: %i) intended motion:.. \n",j + 1, formula, agentInfo[j]);
            // Determina la distancia entre los agentes a y j
            distanceFromAgent = abs(position[a]->x - position[j]->x) +
                                abs(position[a]->y - position[j]->y);
        }

        // TODO: Ver para que se utilizan estas variables
        conflictType[a][j] = -1;
        conflictStepMe = -1;
        conflictStepOther = -1;

        /* Si el objetivo no ha sido alcanzado por j y
      la distancia entre a y j esta entre 0 y lookahead */
        if ((!goal_reached[j]) &&
            (((abs(position[j]->x - position[a]->x) +
               abs(position[j]->y - position[a]->y)) <= (lookahead)) &&
             ((abs(position[j]->x - position[a]->x) +
               abs(position[j]->y - position[a]->y)) > 0)) &&
            (a != j)) {
            // La linea superior es verdadera si el vecino es visible

            if (((formula < 20000) || (agentInfo[j] < 20000)) &&
                ((formula > 20000) && (agentInfo[j] > 20000) &&
                 (agentInfo[j] < formula))) {
                // Analiza el camino del vecino, compara con el camino ideal del
                // vecino j (shared after the previous iteration)
                for (int l = 1; l <= lookahead; l++) {
                    deadlock[a][currentCell->y][currentCell->x][l] = 0;

                    // Here I can add the computation of teams
                    if ((a != j) && (idealPath[j][l] != NULL)) {
                        if (enable_print) printf("Agent %i: [%d %d] me: [%d %d], H: %.1f, step %i \n",j + 1, idealPath[j][l]->y, idealPath[j][l]->x,idealPath[a][l]->y, idealPath[a][l]->x,hvalues[MAZEWIDTH * position[j]->y + position[j]->x][j],l);

                        // Si mi posición actual esta en el camino ideal de mi vecino
                        // dentro del paso l (how bad is to stay here)
                        if ((position[a]->y == idealPath[j][l]->y) &&
                            (position[a]->x == idealPath[j][l]->x)) {
                            // Incrementa el numero de conflictos para esta posición
                            // por este agente.
                            position[a]->numConflicts[a] =
                                position[a]->numConflicts[a] + 1;
                            if (enable_print) printf("Oops! might need to move from CURRENT pos [%d %d], ""total conflicts: %i \n",position[a]->y, position[a]->x,position[a]->numConflicts[a]);
                            conflictStepMe = 1;
                            conflictStepOther = l;
                        }

                        // Si la siguiente posición en mi camino ideal (desde step=1)
                        // y que ambos no sean nulos para ambos de nosotros
                        if ((idealPath[a][l + 1] != NULL) &&
                            (idealPath[j][l + 1] != NULL) && (l < lookahead)) {
                            // Si existe otro conflicto en el paso siguiente de el plan
                            // para ambos (yo y mi vecino j)
                            if ((idealPath[j][l + 1] != NULL) &&
                                (idealPath[a][l + 1]->y == idealPath[j][l + 1]->y) &&
                                (idealPath[a][l + 1]->x == idealPath[j][l + 1]->x)) {
                                // Mi agente a para la siguiente posición se encuentra
                                // en el camino el camino del vecino j
                                if (enable_print) printf("Oops! might need to wait/backtrack,  FUTURE ""conflict at position [%d %d] \n",idealPath[a][l + 1]->y, idealPath[a][l + 1]->x);
                                if (enable_print) printf("My info %i vs other agent's info %i \n",formula, agentInfo[j]);
                                // Basado en esta realación se necesita tomar una
                                // decisión
                                conflictStepMe = l + 1;
                                conflictStepOther = l + 1;

                                // Si sobre mi vecino j se encuentra el subsiguiente
                                // (l+1 no es NULL)
                                if (idealPath[j][l + 2] != NULL) {
                                    // Si entre a y j se encuentran en conflicto para l+2
                                    if ((idealPath[j][l + 2]->y == idealPath[a][l]->y) &&
                                        (idealPath[j][l + 2]->y == idealPath[a][l]->y)) {
                                        if (enable_print) printf("Oops! PATH CONFLICT!!!\n");
                                        // Marca ambos como en conflicto
                                        conflictType[a][j] = 1;
                                    }
                                }
                            }

                            // Si mi siguiente posicion se encuentra sobre la posicion
                            // del otro agente podria necesitar a esperar si yo
                            // quisiera moverme primero Otro caso el necesitaria
                            // intentar moverse primero y no donde me encuentro ahora
                            if ((idealPath[j][l] != NULL) &&
                                (idealPath[a][l + 1]->y == idealPath[j][l]->y) &&
                                (idealPath[a][l + 1]->x == idealPath[j][l]->x) &&
                                ((!(idealPath[j][l + 1]->y == idealPath[a][l]->y)) ||
                                 (!(idealPath[j][l + 1]->y == idealPath[a][l]->y)))) {
                                // La siguiente posicion de a se encuentra sobre el
                                // camino de j
                                if (enable_print) printf("Oops! might need to WAIT, OTHER AGENT IN MY WAY ""at position [%d %d] \n",idealPath[a][l + 1]->y, idealPath[a][l + 1]->x);
                                if (enable_print) printf("My info %i vs other agent's info %i \n",formula, agentInfo[j]);
                                // Basado en esta relación, es necesario tomar una
                                // desicion
                                conflictStepMe = l + 1;
                                conflictStepOther = l;

                                // Verifica si la interseccion continua repitiendose a
                                // futuro esto significa que ambos se encuentran
                                // siguiendo el mismo camino
                                if (idealPath[a][l + 2] != NULL) {
                                    if ((idealPath[a][l + 1]->y == idealPath[j][l]->y) &&
                                        (idealPath[a][l + 1]->x == idealPath[j][l]->x) &&
                                        (idealPath[j][l + 1]->y ==
                                         idealPath[a][l + 2]->y) &&
                                        (idealPath[j][l + 1]->y ==
                                         idealPath[a][l + 2]->y)) {
                                        // TODO: Agent a is following agent j !!! here we
                                        // could make a team or something TODO
                                    }
                                }
                            }

                            // Si yo y mi vecino j cruzamos caminos (no es factible el
                            // cambio) en el mismo camino pero con direcciones opuestas
                            if ((idealPath[j][l + 1] != NULL) &&
                                (idealPath[a][l + 1]->y == idealPath[j][l]->y) &&
                                (idealPath[a][l + 1]->x == idealPath[j][l]->x) &&
                                (idealPath[j][l + 1]->y == idealPath[a][l]->y) &&
                                (idealPath[j][l + 1]->y == idealPath[a][l]->y)) {
                                // Si la siguiente posicion de a forma parte del camino
                                // de j
                                if (enable_print) printf("Oops! UNFEASIBLE SWAPPING projected at [%d %d] \n",idealPath[a][l + 1]->y, idealPath[a][l + 1]->x);
                                if (enable_print) printf("My info %i vs other agent's info %i \n",formula, agentInfo[j]);
                                // Basado en esta realacion, necesitamos tomar una
                                // desicion Conocemos de inmediato que ellos se
                                // encuetran en celdas opuestas
                                conflictType[a][j] = 1;
                                conflictStepMe = l;
                                conflictStepOther = l + 1;
                            }
                        }// Aquí acaba el lookahead
                    }
                }

                // Si el conflictType no se encuentra seteado (si llega con -1)
                // este no es un swap (ambientes estrechos 1 casilla)
                // lo que significa que es un punto de interseccion
                if ((conflictType[a][j] < 0) && (conflictStepMe > -1)) {
                    conflictType[a][j] = 0;
                }
            } // acaba si tengo que tomar atención sobre mi vecino
        }     // acaba "si mi vecino es visible y no se encuentra en su destino"
    }         // acaba el de agentes

    // En este pnto el conflictType debería encontrarse seteado
    // El paso de conflicto por cada agentes (conflictStepMe, conflictStepOther)
    // NO SERA ALMACENADO!! ¿yo quiero esto?

    // Segunda seccion --> determinar el costo de restricciones
    // Valores heuristicos mas relevantes
    int maxHagent = a; // Agente con el maximo valor heuristico actual
    int maxInfo = a;   // Maximo valor camino actual

    // En la seccion anterior, iteramos por cada uno de los pasos en el plan de
    // movimiento, pero esta comienza desde pathlength[a], mientra la otra desde
    // 0
    for (int future = pathlength[a]; future <= lookahead; ++future) {
        cell_role = -1;
        if (enable_print) printf("*****Checking issues of stayin here [%d %d] at time %i!!: \n",currentCell->y, currentCell->x, future);

        // Verificando si mi celda actual se encuetra bloqueada
        // Si me quedo en esta celda a futuro llevara posiblemente a un conflicto
        if (((maze1[currentCell->y][currentCell->x].blockedAgent[a][future]) &&
             (future > 0)) ||
            (((maze1[currentCell->y][currentCell->x]
                   .blockedAgent[a][future - 1])) &&
             (future > 0))) {
            if (enable_print) printf("*****Staying here will bring me trouble at time %i!!\n",future);

            // TODO: Definir estás variables
            int canmovehere = 1;
            int wasthere = 0;
            // Contador de conflictos entre agentes a - j
            int numConflicts = 0;

            // El primer caso, para el primer paso (DESDE PATHLENGTH) en el plan,
            // donde otro agente planee moverse a mi posicion Este verifica la
            // posicion actual, cuando los cambios actuales dependen del paso sobre
            // el camino
            if (future == pathlength[a]) {
                if (enable_print) printf("\n\n****FUTURE At %i another agent WOULD LIKE TO MOVE to [%d ""%d], but who??\n",future, currentCell->y, currentCell->x); // cont_closed
                // Mayor valor heuristico presente entre los agentes
                if (enable_print) printf("*********************************LEEME*******************\n");
                if (enable_print) printf("position[%d]->y: %d\n", a, position[a]->y);
                if (enable_print) printf("position[%d]->x: %d\n", a, position[a]->x);
                if (enable_print) printf("a: %d\n", a);
                if (enable_print) printf("hvalues: %f\n", hvalues[MAZEWIDTH * (position[a]->y) + (position[a]->x)][a]);
                if (enable_print) printf("backupH: %f\n", backupH[MAZEWIDTH * (position[a]->y) + (position[a]->x)][a]);
                float maxH = backupH[MAZEWIDTH * (position[a]->y) + (position[a]->x)][a];
                // Suma de valores heuristicos
                float sumH = 0;

                // Por cada agente
                for (int j = 0; j < NAGENTS; ++j) {
                    // TODO: verificar - Si en la celda actual, el agente a se mueve
                    // con diferencia a j (existe delta)
                    if ((maze1[currentCell->y][currentCell->x]
                             .agentMovingTo[a][future][j] > 0) &&
                        (j != a)) {
                        // Si el otro agente - sobre el paso actual = 0 (significa que
                        // se encuentra sobre su objetivo)
                        if ((goal_reached[j])) {
                            // El agente se encuentra en su objetivo
                            // no es relevante
                            continue;
                        }

                        // Incrementa el contador de conflictos
                        numConflicts++;

                        if (path[j][future] != NULL) {
                            if (enable_print) printf("\033[1;32m");
                            if (enable_print) printf("No es null\n");
                            if (enable_print) printf("\033[0m");
                        } else {
                            if (enable_print) printf("\033[1;31m");
                            if (enable_print) printf("Es null\n");
                            if (enable_print) printf("\033[0m");
                        }
                        if (enable_print) printf("j: %d\n", j);
                        if (enable_print) printf("future: %d\n", future);
                        if (enable_print) printf("path[%d][%d]->y: %d\n", j, future, path[j][future]->y);
                        if (enable_print) printf("path[%d][%d]->x: %d\n", j, future, path[j][future]->x);

                        if (enable_print) printf("His previous position at [%d %d] had a degree of %i \n",path[j][future]->y, path[j][future]->x,maze1[path[j][future]->y][path[j][future]->x].degree[j]);
                        if (enable_print) printf("My info %i vs other agent's info %i \n", formula,agentInfo[j]);

                        // Si es un punto de conflicto interseccion
                        if (conflictType[a][j] == 0) {
                            if (enable_print) printf(" POINT conflict, my cost: %i vs his :%i \n",(int)(hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]) + 2,(int)(hvalues[MAZEWIDTH * position[j]->y + position[j]->x][j]) + 1);
                            // Former comparison, based only on who has higher
                            // heuristic if((int)(hvalues[MAZEWIDTH*position[j]->y +
                            // position[j]->x][j])+1 >
                            //(int)(hvalues[MAZEWIDTH*position[a]->y +
                            // position[a]->x][a])+2)

                            if (formula < agentInfo[j]) {
                                maxInfo = j;
                            }
                        }

                        // Si es un punto de conflicto 1, uno de ambos debe hacerse a
                        // un lado
                        if (conflictType[a][j] == 1) {
                            if (enable_print) printf(" PATH conflict\n");
                            if (formula < agentInfo[j]) {
                                maxInfo = j;
                            }
                        }

                        // Extra part copied from the last case
                        // If both agents had a very high formula/info
                        if ((formula > 20000) && (agentInfo[j] > 20000)) {
                            // Distancia absoluta entre agentes a y j
                            int distanceFromAgent =
                                abs(position[a]->x - position[j]->x) +
                                abs(position[a]->y - position[j]->y);

                            // Si j quiere moverse a mi posicion actual (En mi actual
                            // valor future/pathleght[i])
                            if (enable_print) printf("(1) We are both SCREWED!!, his H is %i, mine is %i ""anddistanceFromStart my distance from start is %i, dist from agent is ""%i, total for comparison: Other %i vs mine %i \n",agentInfo[j] - 20003, formula - 20003,distanceFromStart[a], distanceFromAgent,agentInfo[j] - 20003 - distanceFromAgent,distanceFromStart[a]);

                            if (agentInfo[j] - 20003 < formula - 20003) {
                                if (enable_print) printf("I COULD AT LEAST HELP HIM GET TO HIS GOAL!!!\n");
                                // j pasa a ser el maxInfo
                                maxInfo = j;
                                // Marca deadlock
                                deadlock[a][currentCell->y][currentCell->x][future] = 1;
                            } else {
                                if (enable_print) printf(" NO WAY, I CANT HELP EVEN IF I WANTED TO\n");
                                // Se conserva a como el maxInfo
                                maxInfo = a;
                            }
                        } else {
                            // Caso contrario
                            if (formula < agentInfo[j]) {
                                // j pasa a ser maxInfo sin marcar deadlock
                                maxInfo = j;
                            }
                        }

                        if (enable_print) printf("And his path is \n");

                        // Itera por las veces que permita el lookahead
                        for (int l = 1; l <= lookahead; l++) {
                            // Si a es distinto de j y mi camino ideal se ecuentra
                            // definido
                            if ((a != j) && (idealPath[j][l] != NULL)) {
                                if (enable_print) printf("Agent %i: [%d %d], H: %.1f, step %i \n", j + 1,idealPath[j][l]->y, idealPath[j][l]->x,hvalues[MAZEWIDTH * position[j]->y + position[j]->x][j], l);

                                // Si encuentra una intersección entre a y j sobre la
                                // iteración l
                                if ((position[a]->y == idealPath[j][l]->y) &&
                                    (position[a]->x == idealPath[j][l]->x)) {
                                    // Asigna el contador de conflictos a la posición
                                    // dada
                                    position[a]->numConflicts[a] = numConflicts;
                                    if (enable_print) printf("BOops! might need to move, total conflicts: %i ""\n",position[a]->numConflicts[a]);
                                }

                                // FIXME: Este if se encuentra vacío
                                /* if (hvalues[MAZEWIDTH * position[j]->y +
                        position[j]->x][j] > hvalues[MAZEWIDTH * position[a]->y
                        + position[a]->x][a]) {
                        } */
                            }
                        }
                        // if((idealPath[j][l+1]!=NULL)&&(idealPath[a][l+1]->y==idealPath[j][l]->y)&&(idealPath[a][l+1]->x==idealPath[j][l]->x)&&
                        //(idealPath[j][l+1]->y==idealPath[a][l]->y)&&(idealPath[j][l+1]->y==idealPath[a][l]->y))
                        // getchar();

                        // Despliega valor heuristico de j
                        if (enable_print) printf(" with H of %.1f, (%.1f)  vs ",hvalues[MAZEWIDTH * (currentCell->y) + (currentCell->x)]       [j],hvalues[MAZEWIDTH * (position[j]->y) + (position[j]->x)]       [j]);

                        // Despliega valor heuristico de a
                        if (enable_print) printf(" my H of %.1f, (%.1f) ",hvalues[MAZEWIDTH * (currentCell->y) + (currentCell->x)]       [a],hvalues[MAZEWIDTH * (position[a]->y) + (position[a]->x)]       [a]); //[MAZEWIDTH*(tmpcell1->y) +//(tmpcell1->x)][a]);

                        // Despliega la suma de los valores heuristicos
                        sumH = sumH + backupH[MAZEWIDTH * (currentCell->y) +
                                              (currentCell->x)][j];
                        if (enable_print) printf(", SumH is %f \n", sumH);

                        // Si el valor heuristico de j es mayor que el actual mayor
                        if ((hvalues[MAZEWIDTH * (position[j]->y) + (position[j]->x)]
                                    [j] > maxH)) {
                            // Marca a j como el nuevo agente con mayor valor
                            // heuristico
                            maxHagent = j;
                            // Actualiza el mayor valor heuristico
                            maxH = hvalues[MAZEWIDTH * (currentCell->y) +
                                           (currentCell->x)][j];
                        }
                        // TODO: por definir
                        determine_role(&role[a][j], maxInfo, a, j, &cell_role);
                    }
                }
                // El agente con la maxima heuristica es
                if (enable_print) printf("The Agent with MAX H, whose H changes by my movement  is %i ",maxHagent + 1);
                // Con un valor heuristico de
                if (enable_print) printf(" with H of %f \n", maxH);
            }

            // Otro agente ya debiera encontrarse aquí
            if (((maze1[currentCell->y][currentCell->x]
                      .blockedAgent[a][future - 1])) &&
                (future > 0)) {
                // El agente future-1 no puede desplazarse a la celda actual, ya que
                // hay un agente en ella
                if (enable_print) printf("****At %i MIGHT NOT BE ABLE to move to [%d %d], there MIGHT ""ALREADY BE an agent\n",future - 1, currentCell->y, currentCell->x);
                // TODO: Marca como agente wasthere
                wasthere = 1;
                // backupH[MAZEWIDTH*(tmpcell1->move[d]->y) +
                // (tmpcell1->move[d]->x)][a]; hvalues[MAZEWIDTH*(position[a]->y) +
                // (position[a]->x)][a]; [MAZEWIDTH*(tmpcell1->y) +
                //(tmpcell1->x)][a];

                // Actualiza el maximo valor heuristico
                float maxH =
                    backupH[MAZEWIDTH * (position[a]->y) + (position[a]->x)][a];
                // Inicializa la suma de valores heuristicos a cero
                float sumH = 0;
                // Itera por cada agente
                for (int j = 0; j < NAGENTS; ++j) {
                    // Si la celda para moverse a a j a travez future-1 esta
                    // bloqueada
                    if (maze1[currentCell->y][currentCell->x]
                            .agentMovingTo[a][future - 1][j] > 0) {
                        // Incrementa el contador de conflictos
                        numConflicts++;
                        // El agente j con una cantidad de numConflicts
                        if (enable_print) printf("This guy -> %i  (total %i)", j + 1, numConflicts);
                        // Un valor heuristico de hvalues
                        if (enable_print) printf(" with H of %.1f, (%.1f)  vs ",hvalues[MAZEWIDTH * (currentCell->y) + (currentCell->x)]       [j],hvalues[MAZEWIDTH * (position[j]->y) + (position[j]->x)]       [j]);
                        // vs el agente a con un valor heuristico de hvalues
                        if (enable_print) printf(" my H of %.1f, (%.1f) ",hvalues[MAZEWIDTH * (currentCell->y) + (currentCell->x)]       [a],hvalues[MAZEWIDTH * (position[a]->y) + (position[a]->x)]       [a]);
                        // Actualiza la suma de valores heuristicos
                        sumH = sumH + backupH[MAZEWIDTH * (currentCell->y) +
                                              (currentCell->x)][j];
                        if (enable_print) printf(", SumH is %f \n", sumH);
                        // Copied from above
                        if (enable_print) printf("My info %i vs other agent's info %i \n", formula,agentInfo[j]);

                        // Si existe un punto de conflicto entre a y j
                        if (conflictType[a][j] == 0) {
                            if (enable_print) printf(" POINT conflict, my cost :  %i vs his :%i \n",(int)(hvalues[MAZEWIDTH * position[a]->y +              position[a]->x][a]) +    2,(int)(hvalues[MAZEWIDTH * position[j]->y +              position[j]->x][j]) +    1);

                            // Si el valor heuristico de j es mayor que el de a
                            if ((int)(hvalues[MAZEWIDTH * position[j]->y +
                                              position[j]->x][j]) +
                                    1 >
                                (int)(hvalues[MAZEWIDTH * position[a]->y +
                                              position[a]->x][a]) +
                                    2) {
                                // Actualiza maxInfo a j
                                maxInfo = j;
                                if (enable_print) printf(" MaxInfo: %i\n", maxInfo);
                            }
                        }

                        // Si existe punto de conflicto
                        if (conflictType[a][j] == 1) {
                            if (enable_print) printf(" PATH conflict\n");
                            if (formula < agentInfo[j]) {
                                // Actualiza maxInfo a j
                                maxInfo = j;
                                if (enable_print) printf(" MaxInfo: %i\n", maxInfo + 1);
                            }
                        }

                        // Si formula y el camino asociado a j superan el umbral
                        if ((formula > 20000) && (agentInfo[j] > 20000)) {
                            // Determina la distancia entre a y j
                            int distanceFromAgent =
                                abs(position[a]->x - position[j]->x) +
                                abs(position[a]->y - position[j]->y);

                            if (enable_print) printf("(2) We are both SCREWED!!, his H is %i, mine is %i ""and my distance from start is %i, dist from agent is ""%i, total for comparison: Other %i vs mine %i \n",agentInfo[j] - 20003, formula - 20003,distanceFromStart[a], distanceFromAgent,agentInfo[j] - 20003 - distanceFromAgent,distanceFromStart[a]);

                            // Si el camino sobre j es menor a formula
                            if (agentInfo[j] - 20003 < formula - 20003) {
                                if (enable_print) printf(" I COULD AT LEAST HELP HIM GET TO HIS GOAL!!!\n");
                                // Asigna j a maxInfo
                                maxInfo = j;
                            } else {
                                if (enable_print) printf(" NO WAY, I CANT HELP EVEN IF I WANTED TO\n");
                                // De otra forma asigna a a
                                maxInfo = a;
                            }
                        } else {
                            // Si el valor de formula es menor que el camino sobre j
                            if (formula < agentInfo[j]) {
                                // Asigna j a maxInfo
                                maxInfo = j;
                            }
                        }

                        // Si el valor heuristico sobre j es mayor que el mayor actual
                        if ((hvalues[MAZEWIDTH * (position[j]->y) + (position[j]->x)]
                                    [j] > maxH)) {
                            // Actualiza el los mayores actuales
                            maxHagent = j;
                            maxH = hvalues[MAZEWIDTH * (currentCell->y) +
                                           (currentCell->x)][j];
                        }

                        // TODO: Queda por definir esto
                        determine_role(&role[a][j], maxInfo, a, j, &cell_role);
                    }
                }

                // Despliega el agente con maximo valor heuristico y su valor
                // heuristico asociado
                if (enable_print) printf("The Agent with MAX H, whose H changes by my movement  is %i ",maxHagent + 1);
                if (enable_print) printf(" with H of %f \n", maxH);
            }

            // Tercer caso - hay un agente en la celda actual

            // Por cada uno de los agentes
            for (int u = 0; u < NAGENTS; ++u) {
                // Si en celda actual hay un agente que quiere llegar
                if ((currentCell->x == position[u]->x) &&
                    (currentCell->y == position[u]->y) && (future == 1) &&
                    (u != a)) {
                    if (enable_print) printf("\nOps, agent %i is at the next position position, is it ""its goal?..",u + 1);

                    // Si el agente actual esta en su goal lo pasa por encima
                    // ignorandolo
                    if ((goal[u]->y == position[u]->y) &&
                        (goal[u]->x == position[u]->x)) {
                        if (enable_print) printf("\nYES, I can move through");
                    } else {
                        if (enable_print) printf("\n No, Cant move here\n");
                        // caso contrario no es posible moverse sobre la celda actual
                        canmovehere = 0;
                    }
                }
            }

            // En el caso que la celda actual se ecuentre bloqueada por un agente
            if (((maze1[currentCell->y][currentCell->x].blockedAgent[a][0])) &&
                (!canmovehere)) {
                // No puedo moverme sobre la celda hay un agente
                if (enable_print) printf("\n****At %i Cant move to [%d %d], there is an agent\n",future, currentCell->y, currentCell->x);
                // TODO: POR DEFINIR
                learningCutoff[a] = future - 1;

                // Por cada uno de los agentes
                for (int j = 0; j < NAGENTS; ++j) {
                    // Si j esta sobre la celda actual
                    if ((currentCell->y == position[j]->y) &&
                        (currentCell->x == position[j]->x)) {
                        // Incremeta el contador de conflictos sobre a
                        (currentCell)->numConflicts[a] =
                            (currentCell)->numConflicts[a] + 1;
                        if (enable_print) printf("This guy -> %i, total conflicts %i \n", j + 1,(currentCell)->numConflicts[a]);
                        // Actualiza el nuevo camino más largo a j
                        maxInfo = j;
                        // TODO: POR DEFINIR
                        determine_role(&role[a][j], maxInfo, a, j, &cell_role);
                    }
                }
            }

            // Caso que el número de conflictos sea cero
            if (numConflicts == 0) {
                // Por cada uno de los agentes
                for (int j = 0; j < NAGENTS; ++j) {
                    // Si sobre la celda actual es posible mover a sobre j en future
                    if (maze1[currentCell->y][currentCell->x]
                                .agentMovingTo[a][future][j] > 0 &&
                        idealPath[j][future] != NULL) {
                        // si el agente j se encuentra en el goal
                        if (goal_reached[j]) {
                            // pasa de j y continua
                            continue;
                        }
                        // Incrementa el contador de conflictos
                        numConflicts++;
                        if (enable_print) printf("This guy -> %i wants to move where I am (planning to ""be)  (total %i) at time %i \n",j + 1, numConflicts, future);

                        if (enable_print) printf("His previous position at [%d %d] had a degree of %i \n",idealPath[j][future]->y, idealPath[j][future]->x,maze1[idealPath[j][future]->y][idealPath[j][future]->x]    .degree[j]);
                        if (enable_print) printf("My info %i vs other agent's info %i \n", formula,agentInfo[j]);
                        // Si formula y el camino de j superan el umbral
                        if ((formula > 20000) && (agentInfo[j] > 20000)) {
                            // Determina la distancia entre a y j
                            int distanceFromAgent =
                                abs(position[a]->x - position[j]->x) +
                                abs(position[a]->y - position[j]->y);
                            if (enable_print) printf("(3) We are both SCREWED!!, his H is %i, mine is %i ""and my distance from start is %i, dist from agent is ""%i, total for comparison: Other %i vs mine %i \n",agentInfo[j] - 20003, formula - 20003,distanceFromStart[a], distanceFromAgent,agentInfo[j] - 20003 - distanceFromAgent,distanceFromStart[a]);

                            // Si el camino sobre j es menor que el determinado
                            // mediante formula
                            if (agentInfo[j] < formula) {
                                if (enable_print) printf(" I COULD AT LEAST HELP HIM GET TO HIS GOAL!!!\n");
                                // Marca j como el mayor camino
                                maxInfo = j;
                                // Marca deadlock en a sobre la celda actual en el paso
                                // future
                                deadlock[a][currentCell->y][currentCell->x][future] = 1;
                            } else {
                                if (enable_print) printf(" NO WAY, I CANT HELP EVEN IF I WANTED TO\n");
                                maxInfo = a;
                            }
                        } else {
                            if (formula < agentInfo[j]) {
                                maxInfo = j;
                                // if (enable_print) printf("MAX INFO: %i, role(%i, %i): %i %i\n",j+1,a+1,
                                // j+1,role[a][j],role[1][0]);
                            } else {
                                maxInfo = a;
                                // if (enable_print) printf("MAX INFO: %i, role(%i, %i): %i %i\n",a+1,a+1,
                                // j+1,role[a][j],role[1][0]);
                            }
                        }
                        // TODO:
                        determine_role(&role[a][j], maxInfo, a, j, &cell_role);
                    }
                }
            }
            if (enable_print) printf(" ªªªª****ªªªªª*****A THE AGENT WITH MAX INFO IS %i\n",maxInfo + 1);
            // if(maxInfo!=a)
            // if(role[i][j]<0.9)
            if (cell_role == 0) {
                int step = 0;
                if (initialState == 0) {
                    step = pathlength[a];
                }
                if (enable_print) printf(" My step (of pathlength) is now %i\n", step);
                if (conflictCost[a][currentCell->y][currentCell->x][step] ==
                    0) //<  (float)1/(float)(future-pathlength[a]+1))
                {
                    if (enable_print) printf("CCost of current cell is %.1f\n", conflictCost[a][currentCell->y][currentCell->x][step]);
                    // If the neighbor tried to move to my "current" cell
                    if (!wasthere) {
                        if (enable_print) printf(" %f  + %f= %f\n",(deadlock[a][currentCell->y][currentCell->x][step]),(float)1 / (float)(future - step + 2),(deadlock[a][currentCell->y][currentCell->x][step]) +(float)1 / (float)(future - step + 2));
                        conflictCost[a][currentCell->y][currentCell->x][step] =
                            (deadlock[a][currentCell->y][currentCell->x][step]) +
                            (float)1 / (float)(future - step + 2);
                        if (conflictCost[a][currentCell->y][currentCell->x][step] > 1) {
                            conflictCost[a][currentCell->y][currentCell->x][step] = 1;
                        }
                        if (enable_print) printf("CASE A!!\n");
                        if (enable_print) printf("New CCost of current cell is %.1f\n",conflictCost[a][currentCell->y][currentCell->x][step]);
                    } else // if the neighbor was there when the agent tried to move to the cell
                    {
                        conflictCost[a][currentCell->y][currentCell->x][future] =
                            (deadlock[a][currentCell->y][currentCell->x][step]) +
                            (float)1 / (float)((future - 1) - step + 2);
                        if (conflictCost[a][currentCell->y][currentCell->x][step] >
                            1) {
                            conflictCost[a][currentCell->y][currentCell->x][step] = 1;
                        }
                        if (enable_print) printf("CASE B!!\n");
                    }
                }
                if (enable_print) printf(" ªªªª****ªªªªª***** My time %i vs conflict time %i, ""ConflictCost at time %i is: %f, deadlock at %i is: %f  \n",step, future, step,conflictCost[a][currentCell->y][currentCell->x][step], step,deadlock[a][currentCell->y][currentCell->x][step]);
            }
        }
    }
}

void determine_role(int *roleij, int maxInfo, int a, int j, int *cell_role) {
    // if (enable_print) printf("Inside, role is %i", *roleij);
    // meaning that it has not been set yet
    if (*roleij == -1) {
        if (maxInfo == a) {
            // if (enable_print) printf("This is me\n");
            *roleij = 1;
        } else {
            *roleij = 0;
            // if (enable_print) printf("I defer\n");
        }
    }
    // Meaning that there is already a relation between the agents a and j
    else {
    }

    if (*roleij == 0) {
        *cell_role = 0;
        // if (enable_print) printf("I defer2\n");
    }

    if ((*roleij == 1) && (*cell_role != 0)) {
        *cell_role = 1;
        // if (enable_print) printf("This is me2\n");
    }
    if (enable_print) printf("Role between %i and %i is %i, cell_role is %i\n", a + 1, j + 1,*roleij, *cell_role);
}

void push_over_func(int i, cell1* previous) {
    // Me conservo en el lugar, entonces incrementa contador
    push_over[i] = push_over[i] + 1;
    if (push_over[i] > PUSH_OVER_THRESHOLD)
    {
        d = rand() % DIRECTIONS;
        for (int _d = 0; _d < DIRECTIONS; _d++)
        {
            if (!position[i]->succ[d]->obstacle &&
                !position[i]->succ[d]->blocked[0] &&
                (int)position[i]->succ[d]->g)
            {
                push_over[i] = 0;
                position[i] = position[i]->succ[d];
                agent_cost[i] += euclidian(previous, position[i]);
                robot_steps1++;
                previous->blocked[0] = 0;
                position[i]->blocked[0] = 1;
                position[i]->parent[i] = previous;
                push_out_count = push_out_count + 1;
                break;
            }
            d = (d + 1) % DIRECTIONS;
        }
    }
}

int compute_shortestpath_astar(int a, int lookahead) {
    long valuehpath, j, minimo;
    int variant1, sal, i;
    cell1 *cellminimo, *cellpas, *parent, *tempcellpas;

    int pi = 0, pf = 0;
    double ch, oh, difh, mindif, f;
    time_deadline = 0;

    float totalPenalty = 0;
    learningCutoff[a] = lookahead;

    // States with degree >= 3 can be used for swapping, store this
    if (maze1[position[a]->y][position[a]->x].degree[a] >= 3) {
        // HEEY IF IT IS THE SECOND TIME IT STAYS IN THE STATE; IT COUNTS MORE
        // DEGREES!!!! Need to save the state so that it can be used for swapping
        if (enable_print) printf("Awesome! can use this state for swapping!!!\n");
        lastMobileCellDist[a] = 0;
        lastMobileState[a] = &maze1[position[a]->y][position[a]->x];

    } else {
        if (lastMobileCellDist[a] == 0) {
            // lastMobileCellDist[a]=	lastMobileCellDist[a]+1;
        }
    }

    if (enable_print) printf(" COMPUTING FOR %i (%i), at [%d %d] w degree %i, lastMobile at %i!!!\n",a + 1, pathlength[a], position[a]->y, position[a]->x,maze1[position[a]->y][position[a]->x].degree[a], lastMobileCellDist[a]);

    // Show previous cell
    if (position[a]->parent[a] != NULL) {
        if (enable_print) printf("parent: [%d %d]!!!\n", position[a]->parent[a]->y,position[a]->parent[a]->x);
        // position[a]->parent[a]=position[a]->searchtree;
    }

    // Show last mobile state
    if (lastMobileState[a] != NULL) {
        if (enable_print) printf(" lastMobile at [%d %d] ", lastMobileState[a]->y,lastMobileState[a]->x);
    }

    // For each cell in the map
    if (enable_print) printf("Checking predicted occupied states up to %i..", lookahead);
    for (int y = 0; y < MAZEHEIGHT; ++y) {
        for (int x = 0; x < MAZEWIDTH; ++x) {
            maze1[y][x].numConflicts[a] = 0;

            // for each step in the plan of the agent
            for (int z = 0; z < lookahead + 1; ++z) {
                maze1[y][x].marked[a][z] = 0;

                // Set initial conflict cost to 0
                conflictCost[a][y][x][z] = 0;

                // if the cell is blocked at time z
                if (maze1[y][x].blockedAgent[a][z] > 0) {
                    if (enable_print) printf(" BUSY [%d %d] at T %i\n", y, x, z);
                    if (maze1[y][x].toTransition[a][z] >
                        0) //(toTransition[x][y][a][z]>0)
                    {
                        if (enable_print) printf(" TO [%d %d] at T %i: %i by agents: ", y, x, z,maze1[y][x].toTransition[a][z]);

                        for (int j = 0; j < NAGENTS; ++j) {
                            if (maze1[y][x].agentMovingTo
                                    [a][z][j]) //(agentMovingTo[x][y][a][z][j]>0)
                            {
                                if (enable_print) printf(" %i, ", j + 1);
                            }
                        }
                        if (enable_print) printf("\n");
                    }
                }

                // Double loop through the maze, looking for transitions between
                // cells
                for (int q = 0; q < MAZEHEIGHT; ++q) {
                    for (int e = 0; e < MAZEWIDTH; ++e) {
                        if (z > 0) {
                            if (((maze1[y][x].fromTransition[a][z - 1] > 0) &&
                                 (maze1[q][e].toTransition[a][z])) &&
                                (abs(q - y) + abs(e - x) <=
                                 1)) //(betweenTransition[x][y][e][q][a][z]>0)
                            {
                                for (int j = 0; j < NAGENTS; ++j) {
                                    if ((z == 1) && (position[j]->x == x) &&
                                        (position[j]->y == y) &&
                                        (maze1[q][e].agentMovingTo[a][z][j] == 1)) {
                                        if (enable_print) printf(" TRANSITION BETWEEN [%d %d] and [%d %d] at ""T %i, so To should be %i\n",y, x, q, e, z,maze1[q][e].toTransition[a][z]);
                                    }

                                    if ((z > 1) &&
                                        (maze1[y][x].agentMovingTo[a][z - 1][j] == 1) &&
                                        (maze1[q][e].agentMovingTo[a][z][j] == 1)) {
                                        if (enable_print) printf(" TRANSITION BETWEEN [%d %d] and [%d %d] at ""T %i, so To should be %i\n",y, x, q, e, z,maze1[q][e].toTransition[a][z]);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // End fo going through the maze

    // Recap statistics: H, M and H+(2M+3)
    if (enable_print) printf("My H currently is %.1f \n",hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]);
    // Checking mobility:

    if (enable_print) printf("CHECKING SURROUNDINGS FOR OBSTACLES: %f\n",hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]);
    if (enable_print) printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y, position[a]->x,maze1[position[a]->y][position[a]->x].obstacle);
    if (enable_print) printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y + 1,position[a]->x, maze1[(position[a]->y) + 1][position[a]->x].obstacle);
    if (enable_print) printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y,position[a]->x + 1,maze1[position[a]->y][(position[a]->x) + 1].obstacle);
    if (enable_print) printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y,position[a]->x - 1,maze1[position[a]->y][(position[a]->x) - 1].obstacle);
    if (enable_print) printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y - 1,position[a]->x,maze1[(position[a]->y) - 1][(position[a]->x)].obstacle);

    int mobility = 4 - (maze1[(position[a]->y) + 1][position[a]->x].obstacle +
                        maze1[position[a]->y][(position[a]->x) + 1].obstacle +
                        maze1[position[a]->y][(position[a]->x) - 1].obstacle +
                        maze1[(position[a]->y) - 1][position[a]->x].obstacle);
    // if (enable_print) printf("Mobility : %i \n",mobility);
    if (enable_print) printf("CHECKING SURROUNDINGS FOR AGENTS: %f\n",hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]);

    // Should compute here which of the neighboring cells are occupied by other
    // agents

    int agentNext = 0;

    if (maze1[(position[a]->y) + 1][position[a]->x].blockedAgent[a][0]) {
        if (enable_print) printf("Cell [%d %d] has AGENT \n", position[a]->y + 1, position[a]->x);
        agentNext++;
    }

    if (maze1[(position[a]->y) - 1][position[a]->x].blockedAgent[a][0]) {
        if (enable_print) printf("Cell [%d %d] has AGENT \n", position[a]->y - 1, position[a]->x);

        agentNext++;
    }

    // Esta en una casilla borde e intenta leer una posición fuera del maze
    if (enable_print) printf("sigo con vida %d en (x,y)=(%d,%d)", a, position[a]->x, position[a]->y);

    if (maze1[(position[a]->y)][(position[a]->x) + 1].blockedAgent[a][0]) {
        if (enable_print) printf("Cell [%d %d] has AGENT \n", position[a]->y, position[a]->x + 1);
        agentNext++;
    }

    if (maze1[(position[a]->y)][(position[a]->x) - 1].blockedAgent[a][0]) {
        if (enable_print) printf("Cell [%d %d] has AGENT \n", position[a]->y, position[a]->x - 1);
        agentNext++;
    }

    int agentMobility = 4 - agentNext;
    if (enable_print) printf("Agent Mobility : %i \n", agentMobility);

    int somethingNext = 0;

    if ((maze1[(position[a]->y) + 1][position[a]->x].blockedAgent[a][0]) ||
        (maze1[(position[a]->y) + 1][position[a]->x].obstacle)) {
        if (enable_print) printf("Cell [%d %d] has SOMETHING \n", position[a]->y + 1,position[a]->x);
        somethingNext++;
    }

    if ((maze1[(position[a]->y) - 1][position[a]->x].blockedAgent[a][0]) ||
        (maze1[(position[a]->y) - 1][position[a]->x].obstacle)) {
        if (enable_print) printf("Cell [%d %d] has SOMETHING \n", position[a]->y - 1,position[a]->x);
        somethingNext++;
    }

    if ((maze1[(position[a]->y)][(position[a]->x) + 1].blockedAgent[a][0]) ||
        (maze1[(position[a]->y)][(position[a]->x) + 1].obstacle)) {
        if (enable_print) printf("Cell [%d %d] has SOMETHING \n", position[a]->y,position[a]->x + 1);
        somethingNext++;
    }

    if ((maze1[(position[a]->y)][(position[a]->x) - 1].blockedAgent[a][0]) ||
        (maze1[(position[a]->y)][(position[a]->x) - 1].obstacle)) {
        if (enable_print) printf("Cell [%d %d] has SOMETHING \n", position[a]->y,position[a]->x - 1);
        somethingNext++;
    }

    int netMobility = 4 - somethingNext;

    if (enable_print) printf("NET Mobility : %i \n", netMobility);

    distanceFromStart[a] = abs(position[a]->x - initialCellX[a]) + 
                           abs(position[a]->y - initialCellY[a]);

    if (enable_print) printf("Distance from Start Cell: %i \n", distanceFromStart[a]);

    if (mobility > 2) // If the agent can step out of the other's way
    {
        lastMobileCellDist[a] = 0;
        // need to store this cell
        lastMobileCellY[a] = position[a]->y;
        lastMobileCellX[a] = position[a]->x;

    } else // Agent cannot step out of the other's way
    {
        if (lastMobileCellDist[a] < 900) // If there is a last mobile cell somewhere
        {
            lastMobileCellDist[a] = lastMobileCellDist[a] + 1; // increase the distance to it
        }
    }

    if (enable_print) printf("My M currently is %i \n", lastMobileCellDist[a]);

    // Compute my formula..
    int formula =
        (int)(hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]) +
        (2 * (lastMobileCellDist[a]) + 3);

    if (enable_print) printf("My formula is %i and my pathlenght is %i ((H:) %i + 2* ""(lastmobilecell) %i +3)\n",formula, pathlength[a],(int)(hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]),lastMobileCellDist[a]);

    agentInfo[a] = formula;

    // Review path while there is a desired cell
    for (int l = 1; l <= lookahead; l++) {
        if (idealPath[a][l] != NULL) {
            if (enable_print) printf("My current IDEAL path at pos %i is [%d %d]\n", l,idealPath[a][l]->y, idealPath[a][l]->x);
        }

        if (idealPath[1][l] != NULL) {
            if (enable_print) printf("Agent 2 IDEAL path at pos %i is [%d %d]\n", l,idealPath[1][l]->y, idealPath[1][l]->x);
        }
    }

    // As a new path will be computed, need to reset path to NULL
    for (int l = 0; l < lookahead; ++l) {
        path[a][l] = NULL;
    }

    mazestart1 = position[a]; // Current position
    mazegoal1 = goal[a];      // New position

    mazeiteration1++;
    emptyheap2();
    int newdepth = 0;
    cont_closed = 0; // Initialized the number of "steps" into the future

    #ifdef STATISTICS
    searches_astar1++;
    statexpanded1_initial = statexpanded1;
    #endif

    initialize_state(mazestart1);
    mazestart1->g = 0; // Setting start node cost to zero
    mazestart1->key = 0;
    mazestart1->searchtree = NULL;
    mazestart1->trace = NULL;
    insertheap2(mazestart1);
    flag_success1 = 0;
    float lastStepDepth = 0;

    while (topheap2() != NULL) {
        // If in the middle of the search...
        if (cont_closed > 0) {
            if (enable_print) printf("\n****GETTING NEW NODE FROM THE STACK!! ");
            if (enable_print) printf("\nThe previously analyzed node has degree %i ",tmpcell1->degree[a]);
        }
        tmpcell3 = tmpcell1;
        tmpcell1 = topheap2();

        // If in the middle of the search...
        if (cont_closed > 0) {
            if (enable_print) printf("\nThe next node to expand is [%d %d] with depth %i (or %i) from ""parent [%d %d] (cont %i)",tmpcell1->y, tmpcell1->x, tmpcell1->tmpdepth[a], newdepth,tmpcell3->y, tmpcell3->x, cont_closed);

            // Update search settings
            if (cont_closed == lookahead) {
                lastStepDepth = tmpcell1->tmpdepth[a];
                // last step of path is at depth tmpcell1->tmpdepth[a]
            }

            if (newdepth != tmpcell1->depth[a]) {
                if (enable_print) printf("Strange..");
            }
            tmpcell1->depth[a] = tmpcell1->tmpdepth[a];
            tmpcell1->searchtree =
                tmpcell1->tmpsearchtree[tmpcell1->depth[a]]; // tmpcell3;
        }

        // WHEN AT THE END OF THE SEARCH:
        if ((tmpcell1 == mazegoal1) ||
            (cont_closed == lookahead)) { // open_size = opensize2() + open_size;
            // Se calcul f para actualizar h

            f_value = hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a] + tmpcell1->g;
            if (enable_print) printf(" \n\nH value of [%d %d]: %f,", tmpcell1->y, tmpcell1->x,hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a]);
            if (enable_print) printf(" G : %f\n", tmpcell1->g);

            if (enable_print) printf("\nLEARNING NEW HEURISTICS OF FOUND PATH.. %f\n", tmpcell1->g);
            cellpas = tmpcell1;
            for (d = 0; d < cont_closed; d++) {
                hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a] =
                    max(hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a], f_value - CLOSED[d]->g);

                if (enable_print) printf("Updating H of [%d %d] = %.1f, d: %i, size %i \n",CLOSED[d]->y, CLOSED[d]->x,hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a], d,sizeheap2());
            }

            if (enable_print) printf(" Final destination : [%d %d]", cellpas->y, cellpas->x);

            flag_success1 = 1;
            tmpcell1 = popheap2();
            if (enable_print) printf("\n(BEST MOVE WIHTOUT ACCOUNTING FOR AGENTS) A* top a:%d [%d,%d] ""\n",a, tmpcell1->y, tmpcell1->x);
            break;
        }

        // Obtaining new cell from the stack
        tmpcell1 = popheap2();
        tmpcell1->degree[a] = 0;
        CLOSED[cont_closed] = tmpcell1;
        // Increase the number of steps
        cont_closed++;

        if (cont_closed == 1) // In first iteration
        {
            pathlength[a] = 1;
            tmpcell1->depth[a] = 0;
            tmpcell1->penalty = 0;
        }

        tmpcell1->overexpanded = mazeiteration1;

        statexpanded1++;

        if (cont_closed > 1) // Second to last step
        {
            if (enable_print) printf("\n\n****** FROM CELL [%d %d] at time %i, H: %.1f, degree: %i\n ",tmpcell1->y, tmpcell1->x, tmpcell1->depth[a],hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a],tmpcell1->degree[a]);
        }

        if (cont_closed == 1) { // First state
            if (enable_print) printf("\n\n******FROM CELL [%d %d] at time 0, H: %.1f, degree: %i\n ",tmpcell1->y, tmpcell1->x,hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a],tmpcell1->degree[a]);
            // First time the agent computes next cell, determines constraints:
            determine_constraints(a, lookahead, formula, tmpcell1, 1);
        }

        newdepth = tmpcell1->depth[a] + 1;
        ++agent_expansions[a];

        d = rand() % DIRECTIONS;
        for (i = 0; i < DIRECTIONS; ++i) {
            if (tmpcell1->move[d]) {
                if (enable_print) printf("\n********************************************************");

                if (!((tmpcell1->x == tmpcell1->move[d]->x) &&
                      (tmpcell1->y == tmpcell1->move[d]->y))) {
                    tmpcell1->degree[a] = tmpcell1->degree[a] + 1;
                }
                if (enable_print) printf("\nThinking about moving to [%d %d]..(degree of parent %i) \n",tmpcell1->move[d]->y, tmpcell1->move[d]->x,tmpcell1->degree[a]);

            } else {
                // if (enable_print) printf("\nDirection..%i\n", i);
                // if (enable_print) printf("It REALLY is blocked by obstacle\n" );
            }

            if (tmpcell1->move[d] && (!tmpcell1->move[d]->obstacle))
            {
                initialize_state(tmpcell1->move[d]);
                if (enable_print) printf("\nA* generation a:%d [%d,%d] from [%d %d], %.1f >= %.1f   ",a, tmpcell1->move[d]->y, tmpcell1->move[d]->x, tmpcell1->y,tmpcell1->x, tmpcell1->move[d]->g,tmpcell1->g + tmpcell1->cost[d]);

                float goaldirX = ((float)goal[a]->x - (tmpcell1->x));
                float goaldirY = ((float)goal[a]->y - (tmpcell1->y));

                float maggoaldir = sqrtf(((float)goal[a]->x - (tmpcell1->x)) * ((float)goal[a]->x - (tmpcell1->x)) +
                                         ((float)goal[a]->y - (tmpcell1->y)) * ((float)goal[a]->y - (tmpcell1->y)));

                float magdir = sqrtf(((float)(tmpcell1->move[d])->x - (tmpcell1->x)) * ((float)(tmpcell1->move[d])->x - (tmpcell1->x)) +
                                    ((float)(tmpcell1->move[d])->y - (tmpcell1->y)) * ((float)(tmpcell1->move[d])->y - (tmpcell1->y)));

                if ((tmpcell1->move[d]->g >= tmpcell1->g + tmpcell1->cost[d])) ////way to check if state has been visited before
                {
                    tmpcell1->move[d]->tmpdepth[a] = newdepth;
                    pathlength[a] = tmpcell1->move[d]->tmpdepth[a];
                    if (enable_print) printf("***INCREASING PATHLENGTH to %i !!\n", pathlength[a]);
                    if (enable_print) printf("\nThinking about moving to [%d %d] in my pathlength %i..\n",tmpcell1->move[d]->y, tmpcell1->move[d]->x, pathlength[a]);
                    tmpcell1->move[d]->g_backup = (tmpcell1->g + tmpcell1->cost[d]);
                    tmpcell1->move[d]->g = (tmpcell1->g + tmpcell1->cost[d]);

                    determine_constraints(a, lookahead, formula, tmpcell1->move[d],
                                          0);

                    // Check that, if another agent wants to move here in a later
                    // moment (greater or equal to pathlenght[a]) and it has the
                    // preference, I should be able to move out of the way, that is,
                    // to get to a position where I do not have evidence of another
                    // agent using this cell

                    // After these three checks, we can see if agent can consider
                    // this move or not

                    if (0)
                    {
                        if (enable_print) printf(" I DONT have the max H, I defer to the other agent, ""cutoff at %i \n",pathlength[a] - 1);
                        learningCutoff[a] = pathlength[a] - 1;

                    } else {
                        if (enable_print) printf("[%d %d] G es %.1f +", tmpcell1->move[d]->y,tmpcell1->move[d]->x, tmpcell1->move[d]->g);
                        if (enable_print) printf(" H  es %f, ",hvalues[MAZEWIDTH * tmpcell1->move[d]->y + tmpcell1->move[d]->x][a]);
                        if (enable_print) printf(" F es %f\n",tmpcell1->move[d]->g + hvalues[MAZEWIDTH * tmpcell1->move[d]->y + tmpcell1->move[d]->x][a]);

                        tmpcell1->move[d]->tmpsearchtree[tmpcell1->move[d]->tmpdepth[a]] = tmpcell1;

                        if (enable_print) printf("Mi parent is [%d %d] at depth %i..", tmpcell1->y,tmpcell1->x, tmpcell1->move[d]->tmpdepth[a]);
                        tmpcell1->move[d]->pathlength = tmpcell1->pathlength + 1;

                        float tempG = (tmpcell1->g + tmpcell1->cost[d]);
                        tmpcell1->move[d]->key = (tempG + hvalues[MAZEWIDTH * tmpcell1->move[d]->y + tmpcell1->move[d]->x][a]) * BASE - (tempG);

                        if (enable_print) printf(" Adding [%d %d] with f %f to the heap ...\n",tmpcell1->move[d]->y, tmpcell1->move[d]->x,tempG + hvalues[MAZEWIDTH * tmpcell1->move[d]->y + tmpcell1->move[d]->x][a]);

                        insertheap2(tmpcell1->move[d]);
                    }
                }
            }
            d = (d + 1) % DIRECTIONS;
        } /* end for */
    } /* end while */
    int co = 0;
    int pasada = 0;
    if (flag_success1 == 1) {
        idealPath[a][0] = position[a];

        do {
            pasada = pasada + 1;
            if (enable_print) printf("Pasada numero %i ", pasada);
            pathlength[a] =
                abs(cellpas->x - mazestart1->x) + abs(cellpas->y - mazestart1->y);

            pathlength[a] = cellpas->depth[a];
            if (enable_print) printf("\n\nBACKTRACKING PATH of lenght %i for agent %d", pathlength[a], a + 1);
            realDepth[a] = pathlength[a];

            if (pathlength[a] < lookahead) {
                for (int i = pathlength[a] + 1; i <= lookahead; i++) {
                    path[a][i] = NULL;
                    if (pasada == 1) {
                        idealPath[a][i] = path[a][i];
                    }
                    if (enable_print) printf("\nMarking path[%i][%i] to NULL", a, i);
                }
            }

            lastStepDepth = pathlength[a];

            if (enable_print) printf("\nTO Cell [%d %d] %i", cellpas->y, cellpas->x, co);

            cellpas->trace = NULL; // tracing back a path from the goal back to the start

            if ((pathlength[a] > 0) && (co == 0)) {
                path[a][pathlength[a]] = cellpas;
                if (pasada == 1) {
                    idealPath[a][pathlength[a]] = path[a][pathlength[a]];
                    if (enable_print) printf("IDEAL PATH [%d %d] at time %i, degree %i \n",idealPath[a][pathlength[a]]->y,idealPath[a][pathlength[a]]->x, pathlength[a],maze1[idealPath[a][pathlength[a]]->y][idealPath[a][pathlength[a]]->x].degree[a]);
                }
            }

            if (cellpas == mazestart1) {
                if (enable_print) printf("Did I finish backtracking? %i ", pathlength[a]);
                if (pathlength[a] == 1) {
                    for (int b = 2; b <= lookahead; b++) {
                        path[a][b] = path[a][1];
                        if (enable_print) printf("MY %i nd step is same as before\n", b);
                    }
                }
                cellpas->trace = cellpas;
            }

            while ((cellpas != mazestart1) ||
                   ((cellpas == mazestart1) &&
                    (pathlength[a] >= 1))) {
                pathlength[a] = pathlength[a] - 1;

                tempcellpas = cellpas;

                if ((cellpas->searchtree->y ==
                     cellpas->tmpsearchtree[pathlength[a] + 1]->y) &&
                    (cellpas->searchtree->x ==
                     cellpas->tmpsearchtree[pathlength[a] + 1]->x)) {
                    parent = cellpas->searchtree;
                } else {
                    parent = cellpas->tmpsearchtree[pathlength[a] + 1];
                }

                parent->trace = cellpas;
                cellpas = parent;

                if ((pathlength[a] > 0) && (co == 0)) {
                    path[a][pathlength[a]] = cellpas;

                    if (pasada == 1) {
                        idealPath[a][pathlength[a]] = path[a][pathlength[a]];
                        if (enable_print) printf("IDEAL PATH [%d %d] at time %i, degree %i \n",idealPath[a][pathlength[a]]->y,idealPath[a][pathlength[a]]->x, pathlength[a],maze1[idealPath[a][pathlength[a]]->y][idealPath[a][pathlength[a]]->x].degree[a]);
                    }
                }
            }
            if (enable_print) printf("Nope..");

            if (enable_print) printf(" Got to the start:  [%d %d]. \n First move [%d %d]\n",cellpas->y, cellpas->x, mazestart1->trace->y,mazestart1->trace->x);
            if (path[a][2] != NULL) {
                if (enable_print) printf("Second move [%d %d] ..and  %i\n", path[a][2]->y,path[a][2]->x, pathlength[a]);
            }
            if (enable_print) printf("Checking cell [%d %d]...and [%d %d]", cellpas->y, cellpas->x,mazestart1->trace->y, mazestart1->trace->x);

            if (mazestart1->trace != NULL) {
                if ((mazestart1->trace->blocked[pathlength[a]] != 1) ||
                    ((mazestart1->trace->y == cellpas->y) &&
                     (mazestart1->trace->x == cellpas->x) &&
                     (mazestart1->trace->blocked[pathlength[a]] == 1)))
                {
                    if (enable_print) printf(" GOT IT %i \n", flag_success1);
                    break;
                }
            } else {
                if (enable_print) printf(" GOT IT %i???? [%d %d]\n", flag_success1, cellpas->y,cellpas->x);

                if ((cellpas->blocked[pathlength[a]] != 1) ||
                    ((cellpas->blocked[pathlength[a]] == 1) &&
                     (cellpas == mazestart1)))
                {
                    if (enable_print) printf(" GOT IT %i \n", flag_success1);
                    if (enable_print) printf("PATHLENGHT; %i \n", pathlength[a]);
                    break;
                }
            }

            if (sizeheap2() == 1) {
                if (enable_print) printf(" QUEDA UNOOOOOOOOOOOOO size %i\n", sizeheap2());
            }
            if (sizeheap2() == 0) {
                if (enable_print) printf(" VACIOOOOO size %i\n", sizeheap2());
                return (0);
            }
            if (enable_print) printf(" EN [%d %d] and start is [%d %d]\n", cellpas->y, cellpas->x,mazestart1->y, mazestart1->x);
            cellpas = popheap2();
            if (enable_print) printf(" PARECE QUE ESTOY BLOQUEADO?? NEXT [%d %d] with depth %i anda ""parent [%d %d]\n",cellpas->y, cellpas->x, cellpas->tmpdepth[a],cellpas->tmpsearchtree[cellpas->tmpdepth[a]]->y,cellpas->tmpsearchtree[cellpas->tmpdepth[a]]->x);

            cellpas->depth[a] = cellpas->tmpdepth[a];
            pathlength[a] = cellpas->depth[a];
            cellpas->searchtree = cellpas->tmpsearchtree[cellpas->depth[a]];
        } while (cellpas != NULL);
    }
    return (flag_success1);
}

int compute_constraintpath(int a, int lookahead) {
    long valuehpath, j, minimo;
    int variant1, sal, i;
    cell1 *cellminimo, *cellpas, *parent, *tempcellpas;

    int pi = 0, pf = 0;
    double ch, oh, difh, mindif, f;
    time_deadline = 0;

    float totalPenalty = 0;
    learningCutoff[a] = lookahead;
    if (enable_print) printf(" COMPUTING FOR %i (%i)!!!\n", a + 1, pathlength[a]);

    int formula;

    if (enable_print) printf("\n\n**********************\n *********************\n Path lenght of ""this agent...%i now in [%d %d]\n",pathlength[a], position[a]->y, position[a]->x);

    if (position[a]->parent[a] != NULL) {
        if (enable_print) printf("Whose parent is [%d %d]\n", position[a]->parent[a]->y,position[a]->parent[a]->x);
    }
    int limitcell = pathlength[a];
    for (int l = pathlength[a]; l >= 0; --l) {
        if (enable_print) printf("Checking the path at position %i, [%d %d], conflictCost: %.1f, ""degree %i, current limit: %i, mark: %i\n",l, idealPath[a][l]->y, idealPath[a][l]->x,conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l],maze1[idealPath[a][l]->y][idealPath[a][l]->x].degree[a], limitcell,maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l]);

        if (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] > 0.49) {
            if (enable_print) printf("Cannot move here, will violate deference constraints\n");
            limitcell = l - 1;

            maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l] = 1;
        }

        if ((pathlength[a] > l) &&
            (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] <= 0.49) &&
            (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] > 0.01) &&
            (limitcell == l) &&
            (maze1[idealPath[a][l]->y][idealPath[a][l]->x].degree[a] < 3)) {
            // Unless it is the initial state, the agent does not have to move here
            // as it will have to backtrack. If it is the initial state: backtrack!
            if (l != 0) {
                limitcell = l - 1;
                if (enable_print) printf("Cannot move here, will NOT violate deference constraints but ""the agent does not have enough mobility, limitcell is %i \n",limitcell);
                // need to check the degree of this node, If it is less than 3 we
                // are in trouble and need to go even further back

                maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l] = 1;
            } else {
                limitcell = l;
                if (enable_print) printf("It is here, but will need to BACKTRACK for the step %i\n",l + 1);
                // need to check the degree of this node, If it is less than 3 we
                // are in trouble and need to go even further back

                maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l] = 0;
                maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l + 1] = 1;
            }
        }

        if ((pathlength[a] > l) &&
            (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] <= 0.49) &&
            (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] > 0.201) &&
            (limitcell == l) &&
            (maze1[idealPath[a][l]->y][idealPath[a][l]->x].degree[a] >= 3)) {
            if (l != 0) {
                limitcell = l - 1;
            } else {
                limitcell = l;
            }
            if (enable_print) printf("Can move here (maybe it is here), but will need to move ""somewhere else for the step %i\n",l + 1);
            // need to check the degree of this node, If it is less than 3 we are
            // in trouble and need to go even further back

            maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l] = 0;
            maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l + 1] = 1;
        }
    }

    if (limitcell < 0) {
        if (enable_print) printf(" If it gets here it means the agent cannot find a solution\n");
        // in this case, it should act as a movable object. (IMPLEMENT!!!)

        if (enable_print) printf("It may need to bactrack %i moves ", lastMobileCellDist[a]);

        if (lastMobileState[a] != NULL) {
            if (enable_print) printf(" to [%d %d] \n", lastMobileState[a]->y, lastMobileState[a]->x);
        }

        return (0);
    }

    if (enable_print) printf(" After exploring the ideal path, agent can move max until step %i ""which is cell [%d %d] with degree %i\n",limitcell, idealPath[a][limitcell]->y, idealPath[a][limitcell]->x,maze1[idealPath[a][limitcell]->y][idealPath[a][limitcell]->x].degree[a]);

    if (limitcell > 0) {
        if (enable_print) printf(" For backtracking purposes, I would need to backtrack to [%d %d]\n",idealPath[a][0]->y, idealPath[a][0]->x);
    }

    for (int l = 0; l < lookahead; ++l) {
        path[a][l] = NULL;
    }

    if (enable_print) printf("\nNOTHING HERE...first position in plan: [%d %d] vs [%d %d], second ""[%d %d] ",mazestart1->y, mazestart1->x, position[a]->y, position[a]->x,mazestart1->trace->y, mazestart1->trace->x);

    int new_lookahead = pathlength[a] - limitcell;

    if (enable_print) printf("\n New lookahead %i, real Depth %i ", new_lookahead, realDepth[a]);

    if (enable_print) printf("\n Copying unmodified part of the path:\n");
    for (int l = 0; l <= limitcell; ++l) {
        path[a][l] = idealPath[a][l];
        if (enable_print) printf("Path at step %i is [%d %d] ", l, idealPath[a][l]->y,idealPath[a][l]->x);
        if (enable_print) printf(" and its got trace: ");
        if (mazestart1->trace != NULL) {
            if (enable_print) printf("YES!!!!\n");
        } else {
            if (enable_print) printf(" NO :( \n");
        }
    }

    if (enable_print) printf("\n1 NOTHING HERE...first position in plan: [%d %d] vs [%d %d], second ""[%d %d] ",mazestart1->y, mazestart1->x, position[a]->y, position[a]->x,mazestart1->trace->y, mazestart1->trace->x);

    if (enable_print) printf("\n2 NOTHING HERE...first position in plan: [%d %d] vs [%d %d], second ""[%d %d] ",mazestart1->y, mazestart1->x, position[a]->y, position[a]->x,mazestart1->trace->y, mazestart1->trace->x);

    if (new_lookahead != 0)
    {
        mazestart1 = idealPath[a][limitcell];
        mazegoal1 = goal[a]; // New position
        if (enable_print) printf("HOla\n");

        if (enable_print) printf("\n3 NOTHING HERE...first position in plan: [%d %d] vs [%d %d] ",mazestart1->y, mazestart1->x, position[a]->y, position[a]->x);

        mazeiteration1++;

        emptyheap2();

        cont_closed = 0;

        #ifdef STATISTICS
        searches_astar1++;
        statexpanded1_initial = statexpanded1;
        #endif

        initialize_state(mazestart1);
        mazestart1->g = 0; // Setting start node cost to zero
        mazestart1->key = 0;
        mazestart1->searchtree = NULL;
        mazestart1->trace = NULL;
        insertheap2(mazestart1);
        flag_success1 = 0;
        // cell1 *tmpcell3
    }
    int newdepth = 0;
    float lastStepDepth = 0;

    if (new_lookahead == 0) {
        flag_success1 = 1;
    }

    while ((topheap2() != NULL) && (new_lookahead != 0)) {
        if (cont_closed > 0) {
            if (enable_print) printf("\n****GETTING NEW NODE FROM THE STACK!! ");
            if (enable_print) printf("\nThe previously analyzed node has degree %i ",tmpcell1->degree[a]);
        }

        tmpcell3 = tmpcell1;
        tmpcell1 = topheap2();

        if (cont_closed > 0) {
            if (enable_print) printf("\nThe next node to expand is [%d %d] with depth %i (or %i) from ""parent [%d %d] (cont %i)",tmpcell1->y, tmpcell1->x, tmpcell1->tmpdepth[a], newdepth,tmpcell3->y, tmpcell3->x, cont_closed);

            if (cont_closed == lookahead) {
                lastStepDepth = tmpcell1->tmpdepth[a];
            }

            if (newdepth != tmpcell1->depth[a]) {
                if (enable_print) printf("Strange..");
            }
            tmpcell1->depth[a] = tmpcell1->tmpdepth[a];
            tmpcell1->searchtree =
                tmpcell1->tmpsearchtree[tmpcell1->depth[a]]; // tmpcell3;

        }

        if (enable_print) printf("A* top a:%d [%d,%d] It:%lld LA:%d\n",a, tmpcell1->y, tmpcell1->x,mazeiteration1, lookahead);

        // This next block of code (if statement) should not be executed in this
        // path planning, as we do not want to overwrite previously found
        // heuristic values

        if ((tmpcell1 == mazegoal1) ||
            (cont_closed == lookahead)) { // open_size = opensize2() + open_size;
            // Se calcul f para actualizar h
            f_value =
                hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a] + tmpcell1->g;
            if (enable_print) printf(" \n\nH value of [%d %d]: %f,", tmpcell1->y, tmpcell1->x,hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a]);
            if (enable_print) printf(" G : %f\n", tmpcell1->g);

            if (enable_print) printf("\nLEARNING NEW HEURISTICS OF FOUND PATH.. %f\n", tmpcell1->g);
            cellpas = tmpcell1;
            for (d = 0; d < cont_closed; d++) {
                if (enable_print) printf("Updating H of [%d %d] = %.1f, d: %i, size %i \n",CLOSED[d]->y, CLOSED[d]->x,hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a], d,sizeheap2());
            }

            if (enable_print) printf(" Final destination : [%d %d]", cellpas->y, cellpas->x);
            // getchar();

            flag_success1 = 1;
            tmpcell1 = popheap2();
            if (enable_print) printf("\n(BEST MOVE WIHTOUT ACCOUNTING FOR AGENTS) A* top a:%d [%d,%d] ""\n",a, tmpcell1->y, tmpcell1->x);
            break;
        }

        tmpcell1 = popheap2();
        tmpcell1->degree[a] = 0;
        CLOSED[cont_closed] = tmpcell1;
        cont_closed++;

        if (cont_closed == 1) {
            pathlength[a] = 1;
            tmpcell1->depth[a] = 0;
        }
        if (cont_closed == 1) {
            tmpcell1->penalty = 0;
        }

        tmpcell1->overexpanded = mazeiteration1;

        statexpanded1++;
        if (cont_closed > 1) {
            if (enable_print) printf("\n\n******From Cell [%d %d] at time %i, H: %.1f, degree: %i\n ",tmpcell1->y, tmpcell1->x, tmpcell1->depth[a],hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a],tmpcell1->degree[a]);
        }

        if (cont_closed == 1) {
            if (enable_print) printf("\n\n******From Cell [%d %d] at time 0, H: %.1f, degree: %i\n ",tmpcell1->y, tmpcell1->x,hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a],tmpcell1->degree[a]);
        }
        newdepth = tmpcell1->depth[a] + 1;
        ++agent_expansions[a];

        d = rand() % DIRECTIONS;
        for (i = 0; i < DIRECTIONS; ++i) {
            if (tmpcell1->move[d]) {
                if (enable_print) printf("\n********************************************************");
                if (enable_print) printf("\nThinking about moving to [%d %d]..whose ConflictCost is ""%.1f\n",tmpcell1->move[d]->y, tmpcell1->move[d]->x,conflictCost[a][tmpcell1->move[d]->y][tmpcell1->move[d]->x][pathlength[a]]);
                if (enable_print) printf("****\nPROSPECTIVE NODE [%d %d] is at time/depth %i..is it ""MARKED? %i\n ******",tmpcell1->move[d]->y, tmpcell1->move[d]->x, newdepth,maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x].marked[a][newdepth]);
                // tmpcell1->move[d]->depth[a],
                // maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x].marked[a][tmpcell1->move[d]->depth[a]])
                // ;

                // tmpcell1->degree[a]=tmpcell1->degree[a]+1;

                if (tmpcell1->degree[a] >= 3) {
                    if (enable_print) printf("\nThis node has degree greater than 3!!! it can help me ""swap with other agents\n");
                    // Need to store this cell somewhere as a failsafe
                }

            } else {
                // if (enable_print) printf("\nDirection..%i\n", i);
                // if (enable_print) printf("It REALLY is blocked by obstacle\n" );
            }

            if (tmpcell1->move[d] && (!tmpcell1->move[d]->obstacle) &&
                (!maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                      .marked[a][newdepth])) // tmpcell1->move[d]->depth[a]&&
                                             // tmpcell1->move[d]->overexpanded !=
                                             // mazeiteration1)
            {
                initialize_state(tmpcell1->move[d]);
                if (enable_print) printf("\nA* generation a:%d [%d,%d] from [%d %d], %.1f >= %.1f   ",a, tmpcell1->move[d]->y, tmpcell1->move[d]->x, tmpcell1->y,tmpcell1->x, tmpcell1->move[d]->g,tmpcell1->g + tmpcell1->cost[d]);

                float goaldirX = ((float)goal[a]->x - (tmpcell1->x));
                float goaldirY = ((float)goal[a]->y - (tmpcell1->y));

                float maggoaldir = sqrtf(((float)goal[a]->x - (tmpcell1->x)) * ((float)goal[a]->x - (tmpcell1->x)) +
                                         ((float)goal[a]->y - (tmpcell1->y)) * ((float)goal[a]->y - (tmpcell1->y)));

                float magdir =
                    sqrtf(((float)(tmpcell1->move[d])->x - (tmpcell1->x)) * ((float)(tmpcell1->move[d])->x - (tmpcell1->x)) +
                          ((float)(tmpcell1->move[d])->y - (tmpcell1->y)) * ((float)(tmpcell1->move[d])->y - (tmpcell1->y)));

                if ((tmpcell1->move[d]->g >= tmpcell1->g + tmpcell1->cost[d])) //||((d==4)&&(tmpcell1->move[d]->g ==
                                             // tmpcell1->g + tmpcell1->cost[d])))
                                             ////way to check if state has been
                                             // visited before
                {
                    tmpcell1->move[d]->tmpdepth[a] = newdepth;
                    pathlength[a] = tmpcell1->move[d]->tmpdepth[a];
                    // if (enable_print) printf("\nThinking about moving to [%d %d]..\n",
                    // tmpcell1->move[d]->y,tmpcell1->move[d]->x);
                    if (enable_print) printf("***INCREASING PATHLENGTH to %i !!\n", pathlength[a]);
                    if (enable_print) printf("\nThinking about moving to [%d %d] in my pathlength %i..\n",tmpcell1->move[d]->y, tmpcell1->move[d]->x, pathlength[a]);
                    tmpcell1->move[d]->g_backup = (tmpcell1->g + tmpcell1->cost[d]);
                    tmpcell1->move[d]->g = (tmpcell1->g + tmpcell1->cost[d]);

                    // Check that, if another agent wants to move here in a later
                    // moment (greater or equal to pathlenght[a]) and it has the
                    // preference, I should be able to move out of the way, that is,
                    // to get to a position where I do not have evidence of another
                    // agent using this cell

                    for (int future = pathlength[a]; future < lookahead; ++future) {
                        if (maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                                .blockedAgent[a][future]) {
                            if (enable_print) printf("Staying here will bring me trouble at t %i!!\n",future);
                            // conflictCost[a][currentCell->y][currentCell->x][step]=(float)1/(float)(future-step+2);
                            if (enable_print) printf("QUICK SCORE: %f",conflictCost[a][tmpcell1->move[d]->y][tmpcell1->move[d]->x][pathlength[a]]); //(float)1/(float)(future-pathlength[a]+2));
                        }
                    }

                    int maxHagent = a;
                    int maxInfo = a;
                    if ((maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                             .blockedAgent[a][pathlength[a]]) &&
                        (pathlength[a] > 0)) // Another agent wants to move here
                    {
                        if (enable_print) printf("\n\n****FUTURE At %i another agent WOULD LIKE TO MOVE ""to [%d %d], but who??\n",pathlength[a], tmpcell1->move[d]->y,tmpcell1->move[d]->x); // cont_closed
                        int numConflicts = 0;
                        float maxH = backupH[MAZEWIDTH * (position[a]->y) + (position[a]->x)][a];
                        float sumH = 0;

                        for (int j = 0; j < NAGENTS; ++j) {
                            if (maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                                    .agentMovingTo[a][pathlength[a]][j] > 0) {
                                if (enable_print) printf("This guy -> %i  (total %i) inGoal?: %i or ""(%f-%i)=%f \n",j + 1, numConflicts, goal_reached[j],hvalues[MAZEWIDTH * position[j]->y + position[j]->x][j],pathlength[a],(hvalues[MAZEWIDTH * position[j]->y +position[j]->x][j] -pathlength[a]));
                                if ((goal_reached[j]) ||
                                    ((hvalues[MAZEWIDTH * position[j]->y +
                                              position[j]->x][j] -
                                      pathlength[a]) < 0.1)) {
                                    // agent is on goal, never mind
                                    continue;
                                }
                                numConflicts++;
                                if (enable_print) printf("This guy -> %i  (total %i) inGoal?: %i \n",j + 1, numConflicts, goal_reached[j]);
                                if (enable_print) printf("His previous position at [%d %d] had a degree of ""%i \n",path[j][pathlength[a]]->y,path[j][pathlength[a]]->x,maze1[path[j][pathlength[a]]->y][path[j][pathlength[a]]->x].degree[j]);
                                if (enable_print) printf("My info %i vs other agent's info %i \n",formula, agentInfo[j]);
                                if (conflictType[a][j] == 0) {
                                    if (enable_print) printf(" POINT conflict, my cost: %i vs his :%i \n",(int)(hvalues[MAZEWIDTH * position[a]->y +position[a]->x][a]) +2,(int)(hvalues[MAZEWIDTH * position[j]->y +position[j]->x][j]) +1);

                                    if ((int)(hvalues[MAZEWIDTH * position[j]->y +
                                                      position[j]->x][j]) +
                                            1 >
                                        (int)(hvalues[MAZEWIDTH * position[a]->y +
                                                      position[a]->x][a]) +
                                            2) {
                                        maxInfo = j;
                                    }
                                }

                                if (conflictType[a][j] == 1) {
                                    if (enable_print) printf(" PATH conflict\n");
                                    if (formula < agentInfo[j]) {
                                        maxInfo = j;
                                    }
                                }
                                if (enable_print) printf("And his path is \n");

                                for (int l = 1; l <= lookahead; l++) {
                                    if ((a != j) && (path[j][l] != NULL)) {
                                        if (enable_print) printf("Agent %i: [%d %d], H: %.1f, step %i \n",j + 1, path[j][l]->y, path[j][l]->x,hvalues[MAZEWIDTH * position[j]->y +position[j]->x][j],l);

                                        if ((position[a]->y == path[j][l]->y) &&
                                            (position[a]->x == path[j][l]->x)) {
                                            // conflictsPosition++;
                                            position[a]->numConflicts[a] =
                                                numConflicts; // position[a]->numConflicts[a]+1;
                                            if (enable_print) printf("COops! might need to move, total ""conflicts: %i \n",position[a]->numConflicts[a]);
                                        }

                                        if (hvalues[MAZEWIDTH * position[j]->y +
                                                    position[j]->x][j] >
                                            hvalues[MAZEWIDTH * position[a]->y +
                                                    position[a]->x][a]) {
                                        }
                                    }
                                }

                                // getchar();

                                if (enable_print) printf(" with H of %.1f, (%.1f)  vs ",hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +(tmpcell1->move[d]->x)][j],hvalues[MAZEWIDTH * (position[j]->y) +(position[j]->x)][j]);

                                if (enable_print) printf(" my H of %.1f, (%.1f) ",hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +(tmpcell1->move[d]->x)][a],hvalues[MAZEWIDTH * (position[a]->y) +(position[a]->x)][a]); //[MAZEWIDTH*(tmpcell1->y) +//(tmpcell1->x)][a]);
                                sumH = sumH + backupH[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                                   (tmpcell1->move[d]->x)][j];
                                if (enable_print) printf(", SumH is %f \n", sumH);

                                if ((hvalues[MAZEWIDTH * (position[j]->y) +
                                             (position[j]->x)][j] > maxH)) {
                                    maxHagent = j;
                                    maxH = hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                                   (tmpcell1->move[d]->x)][j];
                                }
                            }
                        }
                        if (enable_print) printf("The Agent with MAX H, whose H changes by my movement  ""is %i ",maxHagent + 1);
                        if (enable_print) printf(" with H of %f \n", maxH);
                    }

                    // if (enable_print) printf(" Bloqueado? %i pathlength %i goal reached %i (agent
                    // %i) ",
                    // maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x].blockedAgent[a][0],pathlength[a],goal_reached[i],i+1);
                    int canmovehere = 1;
                    for (int u = 0; u < NAGENTS; ++u) {
                        if ((tmpcell1->move[d]->x == position[u]->x) &&
                            (tmpcell1->move[d]->y == position[u]->y) &&
                            (pathlength[a] == 1) && (u != a)) {
                            if (enable_print) printf("\nOps, agent %i is at the next position position, is ""it its goal?..",u + 1);

                            if ((goal[u]->y == position[u]->y) &&
                                (goal[u]->x == position[u]->x)) {
                                if (enable_print) printf("\nYES, I can move through");

                            } else {
                                if (enable_print) printf("\n No, Cant move here\n");
                                canmovehere = 0;
                            }
                        }
                    }

                    if ((((maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x].blockedAgent[a][0])) &&
                         (pathlength[a] == 1)) &&
                        (!canmovehere)) // Another agent IS here (first step)
                    {
                        if (enable_print) printf("\n****At %i Cant move to [%d %d], there is an agent\n",pathlength[a], tmpcell1->move[d]->y,tmpcell1->move[d]->x);

                        learningCutoff[a] = pathlength[a] - 1;

                        for (int j = 0; j < NAGENTS; ++j) {
                            if ((tmpcell1->move[d]->y == position[j]->y) &&
                                (tmpcell1->move[d]->x == position[j]->x)) {
                                (tmpcell1->move[d])->numConflicts[a] =
                                    (tmpcell1->move[d])->numConflicts[a] + 1;
                                if (enable_print) printf("This guy -> %i, total conflicts %i \n", j + 1,(tmpcell1->move[d])->numConflicts[a]);
                                maxInfo = j;
                            }
                        }
                    }

                    maxHagent = a;
                    if (((maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x].blockedAgent[a][pathlength[a] - 1])) &&
                        (pathlength[a] > 1)) // Another agent might ALREADY be here (> first step)
                    {
                        if (enable_print) printf("****At %i MIGHT NOT BE ABLE to move to [%d %d], there ""MIGHT ALREADY BE an agent\n",pathlength[a] - 1, tmpcell1->move[d]->y,tmpcell1->move[d]->x);

                        int numConflicts = 0;
                        float maxH = backupH[MAZEWIDTH * (position[a]->y) + (position[a]->x)][a];
                        float sumH = 0;

                        for (int j = 0; j < NAGENTS; ++j) {
                            if (maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                                    .agentMovingTo[a][pathlength[a] - 1][j] > 0) {
                                numConflicts++;
                                if (enable_print) printf("This guy -> %i  (total %i)", j + 1,numConflicts);

                                if (enable_print) printf(" with H of %.1f, (%.1f)  vs ",hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +(tmpcell1->move[d]->x)][j],hvalues[MAZEWIDTH * (position[j]->y) +(position[j]->x)][j]);

                                if (enable_print) printf(" my H of %.1f, (%.1f) ",hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +(tmpcell1->move[d]->x)][a],hvalues[MAZEWIDTH * (position[a]->y) +(position[a]->x)][a]); //[MAZEWIDTH*(tmpcell1->y) +
                                sumH = sumH + backupH[MAZEWIDTH * (tmpcell1->move[d]->y) + (tmpcell1->move[d]->x)][j];
                                if (enable_print) printf(", SumH is %f \n", sumH);

                                // Copied from above
                                if (enable_print) printf("My info %i vs other agent's info %i \n",formula, agentInfo[j]);

                                if (conflictType[a][j] == 0) {
                                    if (enable_print) printf(" POINT conflict, my cost :  %i vs his :%i \n",(int)(hvalues[MAZEWIDTH * position[a]->y +position[a]->x][a]) +2,(int)(hvalues[MAZEWIDTH * position[j]->y +position[j]->x][j]) +1);
                                    if ((int)(hvalues[MAZEWIDTH * position[j]->y + position[j]->x][j]) + 1 > 
                                        (int)(hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]) + 2) {
                                        maxInfo = j;
                                        if (enable_print) printf(" MaxInfo: %i\n", maxInfo);
                                    }
                                }

                                if (conflictType[a][j] == 1) {
                                    if (enable_print) printf(" PATH conflict\n");
                                    if (formula < agentInfo[j]) {
                                        maxInfo = j;
                                        if (enable_print) printf(" MaxInfo: %i\n", maxInfo + 1);
                                    }
                                }

                                // End of copy from above
                                if ((hvalues[MAZEWIDTH * (position[j]->y) + (position[j]->x)][j] > maxH)) {
                                    maxHagent = j;
                                    maxH = hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) + (tmpcell1->move[d]->x)][j];
                                }
                            }
                        }
                        if (enable_print) printf("The Agent with MAX H, whose H changes by my movement  ""is %i ",maxHagent + 1);
                        if (enable_print) printf(" with H of %f \n", maxH);
                    }

                    // After these three checks, we can see if agent can consider
                    // this move or not
                    if (0)
                    {
                        if (enable_print) printf(" I DONT have the max H, I defer to the other agent, ""cutoff at %i \n",pathlength[a] - 1);
                        learningCutoff[a] = pathlength[a] - 1;
                    } else {
                        if (enable_print) printf("[%d %d] G es %.1f +", tmpcell1->move[d]->y,tmpcell1->move[d]->x, tmpcell1->move[d]->g);
                        if (enable_print) printf(" H  es %f, ",hvalues[MAZEWIDTH * tmpcell1->move[d]->y +tmpcell1->move[d]->x][a]);
                        if (enable_print) printf(" F es %f\n",tmpcell1->move[d]->g +hvalues[MAZEWIDTH * tmpcell1->move[d]->y +tmpcell1->move[d]->x][a]);
                        for (int j = 0; j < NAGENTS; ++j) {
                            if (canSee[a][j] > 0) {
                                if (hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a] <
                                    hvalues[MAZEWIDTH * position[j]->y + position[j]->x]
                                           [j]) {
                                }
                            }
                        }
                        tmpcell1->move[d]->tmpsearchtree[tmpcell1->move[d]->tmpdepth[a]] = tmpcell1;

                        if (enable_print) printf("Mi parent is [%d %d] at depth %i..", tmpcell1->y,tmpcell1->x, tmpcell1->move[d]->tmpdepth[a]);
                        tmpcell1->move[d]->pathlength = tmpcell1->pathlength + 1;

                        float tempG = (tmpcell1->g + tmpcell1->cost[d]) + 3 * (conflictCost[a][tmpcell1->move[d]->y][tmpcell1->move[d]->x][pathlength[a]]);
                        tmpcell1->move[d]->key = (tempG + hvalues[MAZEWIDTH * tmpcell1->move[d]->y + tmpcell1->move[d]->x][a]) * BASE - (tempG);

                        if (enable_print) printf(" Adding [%d %d] with f %f to the heap ...\n",tmpcell1->move[d]->y, tmpcell1->move[d]->x,tempG + hvalues[MAZEWIDTH * tmpcell1->move[d]->y +tmpcell1->move[d]->x][a]);
                        insertheap2(tmpcell1->move[d]);
                    }
                }
            }
            d = (d + 1) % DIRECTIONS;
        } /* end for */

    } /* end while */
    int co = 0;
    int pasada = 0;
    if ((flag_success1 == 1) && (new_lookahead != 0)) {
        do {
            pasada = pasada + 1;
            if (enable_print) printf("Pasada numero %i ", pasada);
            pathlength[a] =
                abs(cellpas->x - mazestart1->x) + abs(cellpas->y - mazestart1->y);

            pathlength[a] = cellpas->depth[a];
            if (enable_print) printf("\n\nBACKTRACKING PATH of lenght %i for agent %d",pathlength[a], a + 1);
            realDepth[a] = pathlength[a];

            if (pathlength[a] < lookahead) {
                for (int i = pathlength[a] + 1; i <= lookahead; i++) {
                    path[a][i] = NULL;
                    if (enable_print) printf("\nMarking path[%i][%i] to NULL", a, i);
                }
            }

            lastStepDepth = pathlength[a];
            if (enable_print) printf("\nTO Cell [%d %d] %i", cellpas->y, cellpas->x, co);
            cellpas->trace = NULL; // tracing back a path from the goal back to the start

            if ((pathlength[a] > 0) && (co == 0)) {
                path[a][pathlength[a]] = cellpas;
                if (enable_print) printf("At [%d %d] at time %i \n", path[a][pathlength[a]]->y,path[a][pathlength[a]]->x, pathlength[a]);
            }

            if (cellpas == mazestart1) {
                if (enable_print) printf("Did I finish backtracking? %i ", pathlength[a]);
                if (pathlength[a] == 1) {
                    for (int b = 2; b <= lookahead; b++) {
                        path[a][b] = path[a][1];
                        if (enable_print) printf("MY %i nd step is same as before\n", b);
                    }
                }
                cellpas->trace = cellpas;
            }

            while ((cellpas != mazestart1) ||
                   ((cellpas == mazestart1) && (pathlength[a] >= 1))) {
                pathlength[a] = pathlength[a] - 1;
                tempcellpas = cellpas;
                if ((cellpas->searchtree->y == cellpas->tmpsearchtree[pathlength[a] + 1]->y) &&
                    (cellpas->searchtree->x == cellpas->tmpsearchtree[pathlength[a] + 1]->x)) {
                    parent = cellpas->searchtree;
                } else {
                    parent = cellpas->tmpsearchtree[pathlength[a] + 1];
                }
                parent->trace = cellpas;
                cellpas = parent;

                if ((pathlength[a] > 0) && (co == 0)) {
                    path[a][pathlength[a]] = cellpas;
                    if (enable_print) printf("Att [%d %d] at time %i \n", path[a][pathlength[a]]->y,path[a][pathlength[a]]->x, pathlength[a]);
                }
            }
            if (enable_print) printf("Nope..");
            if (enable_print) printf(" Got to the start:  [%d %d]. \n First move [%d %d]\n",cellpas->y, cellpas->x, mazestart1->trace->y,mazestart1->trace->x);
            if (path[a][2] != NULL) {
                if (enable_print) printf("Second move [%d %d] ..and  %i\n", path[a][2]->y,path[a][2]->x, pathlength[a]);
            }
            if (enable_print) printf("Checking cell [%d %d]...and [%d %d]", cellpas->y, cellpas->x,mazestart1->trace->y, mazestart1->trace->x);

            if (mazestart1->trace != NULL) {
                if ((mazestart1->trace->blocked[pathlength[a]] != 1) ||
                    ((mazestart1->trace->y == cellpas->y) && (mazestart1->trace->x == cellpas->x) && (mazestart1->trace->blocked[pathlength[a]] == 1)))
                {
                    if (enable_print) printf(" GOT IT %i \n", flag_success1);
                    break;
                }
            } else {
                if (enable_print) printf(" GOT IT %i???? [%d %d]\n", flag_success1, cellpas->y,cellpas->x);
                if ((cellpas->blocked[pathlength[a]] != 1) ||
                    ((cellpas->blocked[pathlength[a]] == 1) && (cellpas == mazestart1)))
                {
                    if (enable_print) printf(" GOT IT %i \n", flag_success1);
                    if (enable_print) printf("PATHLENGHT; %i \n", pathlength[a]);
                    break;
                }
            }
            if (sizeheap2() == 1) {
                if (enable_print) printf(" QUEDA UNOOOOOOOOOOOOO size %i\n", sizeheap2());
            }
            if (sizeheap2() == 0) {
                if (enable_print) printf(" VACIOOOOO size %i\n", sizeheap2());
                return (0);
            }
            if (enable_print) printf(" EN [%d %d] and start is [%d %d]\n", cellpas->y, cellpas->x,mazestart1->y, mazestart1->x);
            cellpas = popheap2();
            if (enable_print) printf(" PARECE QUE ESTOY BLOQUEADO?? NEXT [%d %d] with depth %i anda ""parent [%d %d]\n",cellpas->y, cellpas->x, cellpas->tmpdepth[a],cellpas->tmpsearchtree[cellpas->tmpdepth[a]]->y,cellpas->tmpsearchtree[cellpas->tmpdepth[a]]->x);
            cellpas->depth[a] = cellpas->tmpdepth[a];
            pathlength[a] = cellpas->depth[a];
            cellpas->searchtree = cellpas->tmpsearchtree[cellpas->depth[a]];
        } while (cellpas != NULL); //(cellpas != NULL); //
    }                              // end if (flag_success1 == 1)
    return (flag_success1);
}

void observe_new_agents(
    int a, int i, int lookahead) { // if (enable_print) printf(" seen at [%d %d] ",
                                   // track[a][i][0][1],track[a][i][0][0]);

    if (track[a][i][0][0] == -1) // if a hasn't seen agent i before
    {
        if ((!goal_reached[i]) &&
            (((abs(position[i]->x - position[a]->x) +
               abs(position[i]->y - position[a]->y)) <= (lookahead)) &&
             ((abs(position[i]->x - position[a]->x) +
               abs(position[i]->y - position[a]->y)) > 0))) {
            if (!canSee[a][i]) {
                // DIDNT SEE AGENT i IN THE PREVIOUS STEP
            }

            // canSee[a][i]=1;
            if (enable_print) printf("Agent %i appeared AGAIN?? [%d %d] can see agent %i at [%d %d]\n",a + 1, position[a]->y, position[a]->x, i + 1, position[i]->y,position[i]->x);

            if (track[a][i][0][0] ==
                -1) // There is no previous record of the agent
            {
                track[a][i][0][0] = position[i]->x;
                track[a][i][0][1] = position[i]->y;
            } else // at least the agent was seen once before
            {
                track[a][i][1][0] = track[a][i][0][0];
                track[a][i][1][1] = track[a][i][0][1];

                track[a][i][0][0] = position[i]->x;
                track[a][i][0][1] = position[i]->y;
            }

            if (enable_print) printf("Now Computing PRediction......\n");
            computePrediction2(a, i, lookahead);
            // updateHistory(a,i,lookahead,position);
            // getchar();
        }

    } else {
        if (((abs(position[i]->x - position[a]->x) + abs(position[i]->y - position[a]->y)) > (lookahead)) &&
            ((abs(position[i]->x - position[a]->x) + abs(position[i]->y - position[a]->y)) > 0)) {
            canSee[a][i] = 0;
            track[a][i][1][0] = track[a][i][0][0];
            track[a][i][1][1] = track[a][i][0][1];

            track[a][i][0][0] = -1;
            track[a][i][0][1] = -1;

            for (int z = 0; z < (lookahead); z++) {
                if (enable_print) printf("Checking previous seen position of agent %i at [%d %d] \n",i, mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0]);
                if (maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] > 0)
                {
                    maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] =
                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .blockedAgent[a][z] -
                        1;

                    if (enable_print) printf("(%i) SAW the guy before at [%d %d], not anymore, making ""blocked back to  %i\n",z, mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0],maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].blockedAgent[a][z]);

                    if (z > 0) {
                        maze1[mostProbPositionXY[a][i][z - 1][1]]
                             [mostProbPositionXY[a][i][z - 1][0]]
                                 .fromTransition[a][z]--;
                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .agentMovingTo[a][z][i] = 0;
                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .toTransition[a][z]--;

                        if (enable_print) printf("(%i) ANNNd transition between [%d %d] and  [%d %d] back ""to  %i\n",z, mostProbPositionXY[a][i][z - 1][1],mostProbPositionXY[a][i][z - 1][0],mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0],maze1[mostProbPositionXY[a][i][z - 1][1]][mostProbPositionXY[a][i][z - 1][0]].fromTransition[a][z]);
                    }
                }
            }
        }
        if (enable_print) printf(" [4 4] at T 0: %i and mostProb [%d %d]\n",maze1[4][4].blockedAgent[0][0], mostProbPositionXY[a][i][0][1],mostProbPositionXY[a][i][0][0]);
    }
}

void observe_agent2(int a, int i, int lookahead, cell1 *previous) {
    if ((!goal_reached[i]) &&
        (((abs(position[i]->x - position[a]->x) + abs(position[i]->y - position[a]->y)) <= (lookahead)) &&
         ((abs(position[i]->x - position[a]->x) + abs(position[i]->y - position[a]->y)) > 0))) {
        canSee[a][i] = 1;
        if (enable_print) printf("Agent %i at [%d %d] can see agent %i at [%d %d]\n", a + 1,position[a]->y, position[a]->x, i + 1, position[i]->y,position[i]->x);
        totp++;
        if ((mostProbPositionXY[a][i][1][0] == position[i]->x) &&
            (mostProbPositionXY[a][i][1][1] == position[i]->y)) {
            if (enable_print) printf("good prediction!!!");
            goop++;
            evalPrevPrediction(a, i, 1);
        } else {
            if (enable_print) printf("BAD prediction");
            badp++;
            evalPrevPrediction(a, i, 0);
        }

        if (track[a][i][0][0] == -1) // There is no previous record of the agent
        {
            track[a][i][0][0] = position[i]->x;
            track[a][i][0][1] = position[i]->y;
        } else // at least the agent was seen once before
        {
            track[a][i][1][0] = track[a][i][0][0];
            track[a][i][1][1] = track[a][i][0][1];

            track[a][i][0][0] = position[i]->x;
            track[a][i][0][1] = position[i]->y;
        }

        if (enable_print) printf("Now Computing PRediction......\n");
        computePrediction2(a, i, lookahead);

    } else {
        canSee[a][i] = 0;
        track[a][i][1][0] = track[a][i][0][0];
        track[a][i][1][1] = track[a][i][0][1];
        if (track[a][i][1][0] != -1) {
            if (enable_print) printf("****But I just saw this guy at [%d %d]...%i.\n",mostProbPositionXY[a][i][0][1], mostProbPositionXY[a][i][0][0],maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]].blockedAgent[a][0]);
        }

        track[a][i][0][0] = -1;
        track[a][i][0][1] = -1;

        for (int z = 0; z < (lookahead); z++) {
            if ((mostProbPositionXY[a][i][z][0] > -1) &&
                (mostProbPositionXY[a][i][z][1] > -1)) {
                if (maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] > 0) {
                    maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] =
                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .blockedAgent[a][z] -
                        1;

                    if (enable_print) printf("(%i) SAW the guy (%i) before at [%d %d], not anymore, ""making blocked back to  %i\n",z, i, mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0],maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].blockedAgent[a][z]);
                    if (z > 0) {
                        maze1[mostProbPositionXY[a][i][z - 1][1]]
                             [mostProbPositionXY[a][i][z - 1][0]]
                                 .fromTransition[a][z - 1]--;

                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .toTransition[a][z]--;
                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .agentMovingTo[a][z][i] = 0;
                        if (enable_print) printf("(%i) ANd transition between [%d %d] and  [%d %d] back ""to  %i\n",z, mostProbPositionXY[a][i][z - 1][1],mostProbPositionXY[a][i][z - 1][0],mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0],maze1[mostProbPositionXY[a][i][z - 1][1]][mostProbPositionXY[a][i][z - 1][0]].fromTransition[a][z - 1]);
                    }
                }
            }
        }
    }
}

void observe_agent(int a, int i, int lookahead, cell1 *previous) {
    for (int n = 0; n < 100; n++) {
        for (int m = 0; m < 3; m++) {
            mostProbPositionXY[a][i][n][m] = 0;
        }
    }

    for (int n = 0; n < 3; n++) {
        for (int m = 0; m < 3; m++) {
            track[a][i][n][m] = -1;
        }
    }
    
    
    if ((!goal_reached[i]) &&
        (((abs(position[i]->x - position[a]->x) +
           abs(position[i]->y - position[a]->y)) <= (lookahead)) &&
         ((abs(position[i]->x - position[a]->x) +
           abs(position[i]->y - position[a]->y)) > 0))) {
        canSee[a][i] = 1;
        if (enable_print) printf("Agent %i at [%d %d] can see agent %i at [%d %d]\n", a + 1,position[a]->y, position[a]->x, i + 1, position[i]->y,position[i]->x);
        totp++;
        
        if ((mostProbPositionXY[a][i][1][0] == position[i]->x) &&
            (mostProbPositionXY[a][i][1][1] == position[i]->y)) {
            if (enable_print) printf("good prediction!!!");
            goop++;
            evalPrevPrediction(a, i, 1);
        } else {
            if (enable_print) printf("BAD prediction");
            badp++;
            evalPrevPrediction(a, i, 0);
        }

        // totalpredictions[lookahead]++;

        updateHistory(a, i, lookahead, previous);

        if (enable_print) printf("Now Computing PRediction......\n");
        // computePrediction2(a,i, lookahead);

    } else {
        canSee[a][i] = 0;
        track[a][i][1][0] = track[a][i][0][0];
        track[a][i][1][1] = track[a][i][0][1];
        if (track[a][i][1][0] != -1) {
            if (enable_print) printf("****But I just saw this guy at [%d %d]...%i.\n",mostProbPositionXY[a][i][0][1], mostProbPositionXY[a][i][0][0],maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]].blockedAgent[a][0]);
        }

        track[a][i][0][0] = -1;
        track[a][i][0][1] = -1;

        for (int z = 0; z < (lookahead); z++) {
            if ((mostProbPositionXY[a][i][z][0] > -1) &&
                (mostProbPositionXY[a][i][z][1] > -1)) {
                // Nunca se cumple sobre la primera observación.
                if (maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] > 0) {
                    maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] =
                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .blockedAgent[a][z] - 1;

                    if (enable_print) printf("(%i) SAW the guy (%i) before at [%d %d], not anymore, ""making blocked back to  %i\n",z, i, mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0],maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].blockedAgent[a][z]);
                    if (z > 0) {
                        maze1[mostProbPositionXY[a][i][z - 1][1]]
                             [mostProbPositionXY[a][i][z - 1][0]]
                                 .fromTransition[a][z - 1]--;
                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .toTransition[a][z]--;
                        maze1[mostProbPositionXY[a][i][z][1]]
                             [mostProbPositionXY[a][i][z][0]]
                                 .agentMovingTo[a][z][i] = 0;
                        if (enable_print) printf("(%i) ANd transition between [%d %d] and  [%d %d] back ""to  %i\n",z, mostProbPositionXY[a][i][z - 1][1],mostProbPositionXY[a][i][z - 1][0],mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0],maze1[mostProbPositionXY[a][i][z - 1][1]][mostProbPositionXY[a][i][z - 1][0]].fromTransition[a][z - 1]);
                    }
                }
            }
        }
    }
}

void updateHistory(int a, int i, int lookahead, cell1 *previous) {
    if (track[a][i][0][0] == -1) // There is no previous record of the agent
    {
        track[a][i][0][0] = position[i]->x;
        track[a][i][0][1] = position[i]->y;
    } else // at least the agent was seen once before
    {
        track[a][i][1][0] = track[a][i][0][0];
        track[a][i][1][1] = track[a][i][0][1];

        track[a][i][0][0] = position[i]->x;
        track[a][i][0][1] = position[i]->y;
    }

    if (enable_print) printf("Agent %i model of agent %i is: \n", a + 1, i + 1);

    for (int t = 0; t < MEMORY; t++) {
        if (enable_print) printf("Time %i: [%i %i] \n", t, track[a][i][t][1], track[a][i][t][0]);
    }

    if (track[a][i][1][0] != -1) // The agent has at least two observations of its neighbor i
    {
        updateProbabilities(a, i);
    }

    if (track[a][i][1][0] != -1) // The agent has at least two observations of its neighbor i
    {
        if ((track[a][i][1][1] == track[a][i][0][1]) &&
            (track[a][i][1][0] == track[a][i][0][0])) {
        } else {
            // blockedAgent[track[a][i][1][0]][track[a][i][1][1]][a][0]=
            // blockedAgent[track[a][i][1][0]][track[a][i][1][1]][a][0]-1;
        }

        computePrediction(a, i, lookahead);
    } else {
        for (int z = 0; z < (lookahead); z++) {
            if (enable_print) printf("blocking OBSERVED position [%d %d] at t %i, so now %i \n",position[i]->y, position[i]->x, z,maze1[5][3].blockedAgent[3][0]);
            maze1[position[i]->y][position[i]->x].blockedAgent[a][z] =
                1; // blockedAgent[position[i]->x][position[i]->y][a][z]=1;
            maze1[position[i]->y][position[i]->x].fromTransition[a][z]++;
            //	blockedAgent[position[i]->x][position[i]->y][a][1]=1;
            if (z > 0) {
                // betweenTransition[position[i]->x][position[i]->y][position[i]->x][position[i]->y][a][z]++;

                maze1[position[i]->y][position[i]->x].blockedAgent[a][z] = 1;

                // toTransition[position[i]->x][position[i]->y][a][z]++;
                maze1[position[i]->y][position[i]->x].toTransition[a][z]++;
                maze1[position[i]->y][position[i]->x].agentMovingTo[a][z][i] = 1;

                //	 agentMovingTo[position[i]->x][position[i]->y][a][z][i]=1;
                if (enable_print) printf("Observing AGNET %i moving FROM [%d %d] at t %i TO [%d %d] at ""t %i, total: %i\n",i, position[i]->y, position[i]->x, z - 1, position[i]->y,position[i]->x, z,maze1[position[i]->y][position[i]->x].toTransition[a][z]);

                //		 toTransition[position[i]->x][position[i]->y][a][z]);
            }

            mostProbPositionXY[a][i][z][0] = position[i]->x;
            mostProbPositionXY[a][i][z][1] = position[i]->y;
        }
    }
}

void evalPrevPrediction(int a, int i, int correct) {
    pred_agents[a][i] = pred_agents[a][i] + 1;
    if (correct == 1) {
        good_pred_agents[a][i] = good_pred_agents[a][i] + 1;
    }
    if (enable_print) printf(" - So far, %.2f of accuracy, ", (float)goop / (float)totp);

    if (enable_print) printf(" and with neighbor %i, it is %.2f, or %i of total %i\n", i + 1,(float)good_pred_agents[a][i] / (float)pred_agents[a][i],good_pred_agents[a][i], pred_agents[a][i]);

    // getchar();
}

void updateProbabilities(int a, int i) {
    // 4 cases (assuming 4-connected grid)
    if (track[a][i][1][0] > track[a][i][0][0]) {
        obsNextCell[a][i][2]++; // Left
        lastMove[a][i] = 2;

        if (enable_print) printf("Agent %i moved LEFT %i\n", i + 1, obsNextCell[a][i][2]);
    }

    if (track[a][i][1][0] < track[a][i][0][0]) {
        obsNextCell[a][i][0]++; // Right
        lastMove[a][i] = 0;
        if (enable_print) printf("Agent %i moved RIGHT %i\n", i + 1, obsNextCell[a][i][0]);
    }

    if (track[a][i][1][0] == track[a][i][0][0]) {
        if (track[a][i][1][1] == track[a][i][0][1]) {
            obsNextCell[a][i][4]++; // NoOp
            lastMove[a][i] = 4;
            if (enable_print) printf("Agent %i DIDNT MOVE \n", i + 1);
        }
    }

    if (track[a][i][1][1] > track[a][i][0][1]) {
        obsNextCell[a][i][3]++; // Up
        lastMove[a][i] = 3;
        if (enable_print) printf("Agent %i moved UP %i\n", i + 1, obsNextCell[a][i][3]);
    }

    if (track[a][i][1][1] < track[a][i][0][1]) {
        obsNextCell[a][i][1]++; // Down
        lastMove[a][i] = 1;
        if (enable_print) printf("Agent %i moved DOWN %i\n", i + 1, obsNextCell[a][i][1]);
    }

    int total_nextCell = obsNextCell[a][i][2] + obsNextCell[a][i][0] +
                         obsNextCell[a][i][1] + obsNextCell[a][i][3] +
                         obsNextCell[a][i][4];

    nextCellProb[a][i][0] = 100 * (float)obsNextCell[a][i][0] / (float)total_nextCell;
    nextCellProb[a][i][1] = 100 * (float)obsNextCell[a][i][1] / (float)total_nextCell;
    nextCellProb[a][i][2] = 100 * (float)obsNextCell[a][i][2] / (float)total_nextCell;
    nextCellProb[a][i][3] = 100 * (float)obsNextCell[a][i][3] / (float)total_nextCell;
    nextCellProb[a][i][4] = 100 * (float)obsNextCell[a][i][4] / (float)total_nextCell;

    if (enable_print) printf("Observed Probabilities: DOWN %.1f, UP %.1f, LEFT %.1f , RIGHT %.1f\n",nextCellProb[a][i][1], nextCellProb[a][i][3], nextCellProb[a][i][2],nextCellProb[a][i][0]);
}

void computePrediction2(int a, int i, int lookahead) {
    if (enable_print) printf("AGENT %i EEERASING PREVIOUS OBSERVATIONS of %i as in [%d %d] REALDEPTH ""%i, lookahead %i\n",a + 1, i + 1, mostProbPositionXY[a][i][0][1],mostProbPositionXY[a][i][0][0], realDepth[i], lookahead);

    for (int z = 0; z < realDepth[i]; z++)
    {
        if (mostProbPositionXY[a][i][z][0] > -2)
        {
            if (z == 0) {
                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .blockedAgent[a][z] = 0;
                if (enable_print) printf("\nERASING blocking at [%d %d] to %i  ", mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0], maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].blockedAgent[a][z]);
                if (enable_print) printf("Now z is %i and %i\n", mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0]);
            }

            if ((z > 0) &&
                (maze1[mostProbPositionXY[a][i][z - 1][1]]
                      [mostProbPositionXY[a][i][z - 1][0]]
                          .fromTransition[a][z - 1] > 0) &&
                (maze1[mostProbPositionXY[a][i][z][1]]
                      [mostProbPositionXY[a][i][z][0]]
                          .toTransition[a][z] > 0)) {
                if (enable_print) printf("**** [4 4] at T 0: %i \n", maze1[4][4].blockedAgent[0][0]);
                maze1[mostProbPositionXY[a][i][z - 1][1]]
                     [mostProbPositionXY[a][i][z - 1][0]]
                         .fromTransition[a][z - 1]--;
                if (enable_print) printf("\nERASING TRANSITION BETWEEN [%d %d] and [%d %d] to %i  ",mostProbPositionXY[a][i][z - 1][1],mostProbPositionXY[a][i][z - 1][0],mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0],maze1[mostProbPositionXY[a][i][z - 1][1]][mostProbPositionXY[a][i][z - 1][0]].fromTransition[a][z]);
                if (enable_print) printf(" ***[4 4] at T 0: %i \n", maze1[4][4].blockedAgent[0][0]);

                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .toTransition[a][z]--;

                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .agentMovingTo[a][z][i] = 0;

                if (enable_print) printf("\nERASING TRANSITION TO [%d %d] to %i (at %i)", mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0], maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].toTransition[a][z], z);
            }
            if (maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .toTransition[a][z] == 0)
            {
                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .blockedAgent[a][z] = maze1[mostProbPositionXY[a][i][z][1]]
                                                    [mostProbPositionXY[a][i][z][0]]
                                                        .blockedAgent[a][z] - 1;
                if (maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] < 0) {
                    maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] = 0;
                }

                if (enable_print) printf("\n1 No more ToTrans, ERASING BLOCKED [%d %d] at t %i to %i  ",mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0],z,maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].blockedAgent[a][z]);
            }
        }

        if (z > 0) {
            mostProbPositionXY[a][i][z - 1][0] = -10;
            mostProbPositionXY[a][i][z - 1][1] = -10;
        }
    }

    mostProbPositionXY[a][i][0][0] = (position[i]->x);

    mostProbPositionXY[a][i][0][1] = (position[i]->y);

    maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]]
        .blockedAgent[a][0] = 1;
    if (enable_print) printf("\nblocking NEW OBSERVED position [%d %d] [%d %d] : %i at T 0\n",position[i]->y, position[i]->x, mostProbPositionXY[a][i][0][1],mostProbPositionXY[a][i][0][0],maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]].blockedAgent[a][0]);

    for (int t = 1; t < realDepth[i]; t++) //(lookahead);t++)
    {
        if (enable_print) printf(" NOW T is %i\n", t);
        // Using comm instead of prediction
        if (path[i][t + 1] != NULL) {
            if (canSee[a][i]) {
                mostProbPositionXY[a][i][t][0] = path[i][t + 1]->x;
                mostProbPositionXY[a][i][t][1] = path[i][t + 1]->y;
            } else {
                mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
            }

        } else {
            mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
            mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
        }

        maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
            .blockedAgent[a][t] =
            maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
                .blockedAgent[a][t] +
            1;

        if (enable_print) printf("BBLOCKING: [%i %i] at time %i: %i \n",mostProbPositionXY[a][i][t][1], mostProbPositionXY[a][i][t][0], t,maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].blockedAgent[a][t]);
        if (maze1[mostProbPositionXY[a][i][t - 1][1]]
                 [mostProbPositionXY[a][i][t - 1][0]]
                     .fromTransition[a][t - 1] < 0) {
            maze1[mostProbPositionXY[a][i][t - 1][1]]
                 [mostProbPositionXY[a][i][t - 1][0]]
                     .fromTransition[a][t - 1] = 0;
        }
        maze1[mostProbPositionXY[a][i][t - 1][1]]
             [mostProbPositionXY[a][i][t - 1][0]]
                 .fromTransition[a][t - 1]++;
        maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
            .toTransition[a][t]++;
        maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
            .agentMovingTo[a][t][i] = 1;
        if (enable_print) printf("TO: [%i %i] at time %i: %i \n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0], t,maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].toTransition[a][t]);
        if (enable_print) printf("TRANSITION: [%i %i] to [%i %i] at time %i: %i \n",mostProbPositionXY[a][i][t - 1][1],mostProbPositionXY[a][i][t - 1][0], mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0], t,maze1[mostProbPositionXY[a][i][t - 1][1]][mostProbPositionXY[a][i][t - 1][0]].fromTransition[a][t - 1]);
    }

    canSee[a][i] = 1;
}

void computePrediction(int a, int i, int lookahead) {
    if (enable_print) printf("AGENT %i ERASING PREVIOUS OBSERVATIONS of %i as in [%d %d]\n", a + 1,i + 1, mostProbPositionXY[a][i][0][1],mostProbPositionXY[a][i][0][0]);

    for (int z = 0; z < (lookahead); z++) {
        if (mostProbPositionXY[a][i][z][0] > -2) {
            if (z == 0) {
                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .blockedAgent[a][z] = 0;
                // blockedAgent[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]=0;
                if (enable_print) printf("\nERASING blocking at [%d %d] to %i  ", mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0], maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].blockedAgent[a][z]);
                if (enable_print) printf("how about this [%d %d] %i at 0 and this [%d %d] %i at 1 ", mostProbPositionXY[a][i][0][1], mostProbPositionXY[a][i][0][0], maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]].blockedAgent[a][0], mostProbPositionXY[a][i][1][1], mostProbPositionXY[a][i][1][0], maze1[mostProbPositionXY[a][i][1][1]][mostProbPositionXY[a][i][1][0]].blockedAgent[a][1]);
            }

            if ((z > 0) &&
                (maze1[mostProbPositionXY[a][i][z - 1][1]]
                      [mostProbPositionXY[a][i][z - 1][0]]
                          .fromTransition[a][z - 1] > 0) &&
                (maze1[mostProbPositionXY[a][i][z][1]]
                      [mostProbPositionXY[a][i][z][0]]
                          .toTransition[a][z] > 0)) {
                //(betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]>0))

                maze1[mostProbPositionXY[a][i][z - 1][1]]
                     [mostProbPositionXY[a][i][z - 1][0]]
                         .fromTransition[a][z - 1]--;
                // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;
                if (enable_print) printf("\nERASING TRANSITION BETWEEN [%d %d] and [%d %d] to %i  ", mostProbPositionXY[a][i][z - 1][1], mostProbPositionXY[a][i][z - 1][0], mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0], maze1[mostProbPositionXY[a][i][z - 1][1]][mostProbPositionXY[a][i][z - 1][0]].fromTransition[a][z]);

                // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]);
                //	toTransition[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;

                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .toTransition[a][z]--;

                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .agentMovingTo[a][z][i] = 0;
                // agentMovingTo[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z][i]=0;

                if (enable_print) printf("\nERASING TRANSITION TO [%d %d] to %i (at %i)", mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0], maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].toTransition[a][z], z);
            }
            if (maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .toTransition[a][z] ==
                0) //    (toTransition[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]==0)//(blockedAgent[mostProbPositionX[a][i][z]][mostProbPositionY[a][i][z]][a][z]>0)
            {
                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .blockedAgent[a][z] = maze1[mostProbPositionXY[a][i][z][1]]
                                                    [mostProbPositionXY[a][i][z][0]]
                                                        .blockedAgent[a][z] -
                                               1;
                // blockedAgent[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]=blockedAgent[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]-1;
                if (maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] < 0) {
                    maze1[mostProbPositionXY[a][i][z][1]]
                         [mostProbPositionXY[a][i][z][0]]
                             .blockedAgent[a][z] = 0;
                }

                if (enable_print) printf( "\n2 No more ToTrans, ERASING BLOCKED [%d %d] at t %i to %i  ", mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0], z, maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].blockedAgent[a][z]);
            }
        }

        if (z > 0) {
            mostProbPositionXY[a][i][z - 1][0] = -10;
            mostProbPositionXY[a][i][z - 1][1] = -10;
        }
    }

    mostProbPositionXY[a][i][0][0] = (position[i]->x);
    mostProbPositionXY[a][i][0][1] = (position[i]->y);
    if (enable_print) printf("\nACCORDING TO AGENT %i, MOST LIKELY DIRECTION FOR AGENT %i, currently ""at [%d %d], at time 0 IS: ",a + 1, i + 1, position[i]->y, position[i]->x);
    if (enable_print) printf("SAME!!: [%i %i]\n", mostProbPositionXY[a][i][0][1],mostProbPositionXY[a][i][0][0]);

    maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]].blockedAgent[a][0] = 1;
    if (enable_print) printf("blocking NEW OBSERVED position [%d %d] : %i at T 0\n", position[i]->y, position[i]->x, maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]].blockedAgent[a][0] = 1);

    // if (enable_print) printf("how about at 4?: %i",
    // blockedAgent[mostProbPositionX[a][i][4]][mostProbPositionY[a][i][4]][a][4]);
    if (pred_agents[a][i] > 0) {
        // If prediction accuracy is too low (for now <0.5) don't make prediction,
        // assume it will not bother the agent
        if (((float)good_pred_agents[a][i] / (float)pred_agents[a][i]) > 0.9) predict[a][i] = 1;
        else predict[a][i] = 0;
    } else {
        predict[a][i] = 0;
    }

    for (int t = 1; t < (lookahead); t++) {
        float maxProb = -10;
        int mostProb = -1;
        for (int j = 0; j <= DIRECTIONS; j++) {
            if (nextCellProb[a][i][j] > maxProb) {
                mostProb = j;
                maxProb = nextCellProb[a][i][j];
            }
        }

        mostProb = lastMove[a][i];

        if (mostProb == 0) {
            if (t == 1) {
                if (((position[i]->x) + 1) < MAZEWIDTH) {
                    if (maze1[position[i]->y][position[i]->x + 1].obstacle == 0) {
                        mostProbPositionXY[a][i][t][0] = (position[i]->x) + 1;
                        mostProbPositionXY[a][i][t][1] = (position[i]->y);

                    } else {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                        if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    }
                }
            } else {
                if ((mostProbPositionXY[a][i][t - 1][0] + 1) < MAZEWIDTH) {
                    if (maze1[mostProbPositionXY[a][i][t - 1][1]]
                             [mostProbPositionXY[a][i][t - 1][0] + 1]
                                 .obstacle == 0) {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0] + 1;
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                    } else {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                        if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    }
                }
            }
            if ((((position[i]->x) + 1) >= MAZEWIDTH) ||
                (mostProbPositionXY[a][i][t - 1][0] + 1 >= MAZEWIDTH)) {
                mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
            } else {
                if (enable_print) printf("RIGHT!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
            }
        }

        if (mostProb == 1) {
            if (t == 1) {
                if (((position[i]->y) + 1) < MAZEHEIGHT) {
                    if (maze1[position[i]->y + 1][position[i]->x].obstacle == 0) {
                        mostProbPositionXY[a][i][t][0] = (position[i]->x);
                        mostProbPositionXY[a][i][t][1] = (position[i]->y) + 1;

                    } else {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                        if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    }
                }

            } else {
                if ((mostProbPositionXY[a][i][t - 1][1] + 1) < MAZEHEIGHT) {
                    if (maze1[mostProbPositionXY[a][i][t - 1][1] + 1][mostProbPositionXY[a][i][t - 1][0]].obstacle == 0) {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1] + 1;
                        if (enable_print) printf("DOWN!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    } else {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                        if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    }
                }
            }

            if ((((position[i]->y + 1)) >= MAZEHEIGHT) ||
                (mostProbPositionXY[a][i][t - 1][1] + 1 >= MAZEHEIGHT)) {
                mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
            } else {
                if (enable_print) printf("DOWN!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
            }
        }
        if (mostProb == 2) {
            if (t == 1) {
                if (((position[i]->x) - 1) >= 0) {
                    if (maze1[position[i]->y][position[i]->x - 1].obstacle == 0) {
                        mostProbPositionXY[a][i][t][0] = (position[i]->x) - 1;
                        mostProbPositionXY[a][i][t][1] = (position[i]->y);
                    } else {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                        if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    }
                }
            } else {
                if ((mostProbPositionXY[a][i][t - 1][0] - 1) >= 0) {
                    if (maze1[mostProbPositionXY[a][i][t - 1][1]][mostProbPositionXY[a][i][t - 1][0] - 1].obstacle == 0) {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0] - 1;
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                    } else {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                        if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    }
                }
            }

            if ((((position[i]->x) - 1) < 0) || (mostProbPositionXY[a][i][t - 1][0] - 1 < 0)) {
                mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
            } else {
                if (enable_print) printf("LEFT!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
            }
        }

        if (mostProb == 3) {
            if (t == 1) {
                if (((position[i]->y) - 1) >= 0) {
                    if (maze1[position[i]->y - 1][position[i]->x].obstacle == 0) {
                        mostProbPositionXY[a][i][t][0] = (position[i]->x);
                        mostProbPositionXY[a][i][t][1] = (position[i]->y) - 1;
                    } else {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                        if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    }
                }
            } else {
                if ((mostProbPositionXY[a][i][t - 1][1] - 1) >= 0) {
                    if (maze1[mostProbPositionXY[a][i][t - 1][1] - 1][mostProbPositionXY[a][i][t - 1][0]].obstacle == 0) {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1] - 1;
                        if (enable_print) printf("UP!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    } else {
                        mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                        mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                        if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
                    }
                }
            }

            if ((((position[i]->y - 1)) < 0) ||
                (mostProbPositionXY[a][i][t - 1][1] - 1 < 0)) {
                mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
                if (enable_print) printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
            } else {
                if (enable_print) printf("UP!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
            }
        }

        if (mostProb == 4) {
            // if (enable_print) printf("NOTHING !!\n");
            if (t == 1) {
                mostProbPositionXY[a][i][t][0] = (position[i]->x);
                mostProbPositionXY[a][i][t][1] = (position[i]->y);
            } else {
                mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
                mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
            }
            if (enable_print) printf("NOTHING!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],mostProbPositionXY[a][i][t][0]);
        }

        // Using comm instead of prediction

        mostProbPositionXY[a][i][t][0] = path[i][t]->x;
        mostProbPositionXY[a][i][t][1] = path[i][t]->y;

        maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].blockedAgent[a][t] = maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].blockedAgent[a][t] + 1;

        if (enable_print) printf("BLOCKING: [%i %i] at time %i: %i \n", mostProbPositionXY[a][i][t][1], mostProbPositionXY[a][i][t][0], t, maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].blockedAgent[a][t]);
        maze1[mostProbPositionXY[a][i][t - 1][1]][mostProbPositionXY[a][i][t - 1][0]].fromTransition[a][t - 1]++;
        maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].toTransition[a][t]++;
        maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].agentMovingTo[a][t][i] = 1;
        if (enable_print) printf("TO: [%i %i] at time %i: %i \n", mostProbPositionXY[a][i][t][1], mostProbPositionXY[a][i][t][0], t, maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].toTransition[a][t]);
        if (enable_print) printf("TRANSITION: [%i %i] to [%i %i] at time %i: %i \n", mostProbPositionXY[a][i][t - 1][1], mostProbPositionXY[a][i][t - 1][0], mostProbPositionXY[a][i][t][1], mostProbPositionXY[a][i][t][0], t, maze1[mostProbPositionXY[a][i][t - 1][1]][mostProbPositionXY[a][i][t - 1][0]].fromTransition[a][t]);
    }
}

void randommove(int a) {
    cell1 *tmpcell1 = position[a];
    ++agent_expansions[a];
    d = rand() % DIRECTIONS;

    int i;
    for (i = 0; i < DIRECTIONS; ++i) {
        if (tmpcell1->move[d] && (!tmpcell1->move[d]->obstacle) &&
            (!tmpcell1->move[d]->blocked[0])) {
            tmpcell1->blocked[0] = 0;
            position[a] = tmpcell1->move[d];
            position[a]->blocked[0] = 1;
            return;
        }
        d = (d + 1) % DIRECTIONS;
    }
}

void test_rtaastar(int lookahead, int prunning) {
    cell1 *tempcell, *previous, *current;
    int y, x, i, j, k;
    long int m;

    if (enable_print) printf("\nGENERATING RANDOMMAZE\n");
    newrandommaze_astar();
    if (enable_print) printf("\nDONE\n");
    time_astar_initialize1 += 1.0 * (tv22c.tv_sec - tv11c.tv_sec) +
                              1.0 * (tv22c.tv_usec - tv11c.tv_usec) / 1000000.0;
    time_astar_initialize1 = 0;
    for (int d = 0; d < NAGENTS; d++) {
        for (y = 0; y < MAZEHEIGHT; ++y) {
            for (x = 0; x < MAZEWIDTH; ++x) {
                backupH[MAZEWIDTH * y + x][d] = hvalues[MAZEWIDTH * y + x][d];
            }
        }
    }

    initialize_astar();
    current = mazestart1;
    finish_all = NAGENTS;
    time_step = 1;

    // Set up values for the maze
    for (i = 0; i < NAGENTS; i++) {
        lastMobileState[i] = NULL;
        backtrack[i] = 0;
        initialCellX[i] = position[i]->x;
        initialCellY[i] = position[i]->y;
        for (int y = 0; y < MAZEHEIGHT; ++y) {
            for (int x = 0; x < MAZEWIDTH; ++x) {
                for (int z = 0; z < (lookahead); ++z) {
                    maze1[y][x].blockedAgent[i][z] = 0;
                    maze1[y][x].fromTransition[i][z] = 0;
                    // if (enable_print) printf(" AHA!!!\n");
                    // blockedAgent[x][y][i][z]=0;
                    for (j = 0; j < NAGENTS; j++) {
                        maze1[y][x].agentMovingTo[i][z][j] = 0;
                    }
                }
            }
        }
    }

    // First observation
    for (i = 0; i < NAGENTS; i++) {
        for (j = 0; j < NAGENTS; j++) {
            if (i != j) {
                canSee[i][j] = 0;
                role[i][j] = -1;
                // Role=1 means I am not deferent to agent j
                // Role=0 means taht I am deferent to agent i
                if (enable_print) printf("Watching %i from %i.. \n", i + 1, j + 1);
                // if (enable_print) printf("OBSERVING AGENTS\n");
                observe_agent(j, i, lookahead, position[i]);
                //	if (enable_print) printf("\n now %i\n ", maze1[5][3].blockedAgent[3][0]);
            }
        }
    }

    // Loop until all agents finish
    while (finish_all) {
        if (enable_print) printf("OBSERVING AGENTS\n");
        // i = random() % NAGENTS;
        // For each agent in the problem..
        for (i = 0; i < NAGENTS; i++) {
            /* if (RUN1 >= 0 && robot_steps1 >= 0) {
                // if (enable_print) printf("Antes Agent[%d] A* Start [%d,%d] Goal [%d,%d] h:%f
                // step:%d time_step:%d
                // terminado:%d\n",i+1,position[i]->y,position[i]->x,goal[i]->y,goal[i]->x,position[i]->h,robot_steps1,time_step,NAGENTS-finish_all);
                // print the grid
                // if (i==0) printf("[%d]", time_step);
                if (i==0) multi_print_grid();
                for (k = 0; k < NAGENTS; k++) {
                    if (enable_print) printf("(%d)[%d,%d]....(%i and %i) ", k + 1, position[k]->y,position[k]->x, role[0][1], role[1][0]);
                }
                if (enable_print) printf("\n");
                if (i==0) getchar();
            } */

            if (position[i] != goal[i]) { // While it is not at its goal...
                // First, compute the shortest path, ignoring other agents...
                if (!compute_shortestpath_astar(i, lookahead)) {
                    // if (enable_print) printf(" OOOPPS,AGENT %i NEED TO BACKTRACK!!!\n",i);
                    // if (enable_print) printf("*   A*  when mazeiteration1 = %d,    No path possible
                    // return;

                } else {
                    // For each step in the lookahed
                    for (int l = 0; l <= lookahead; l++) {
                        // If the agent has a desired current/future position
                        if (idealPath[i][l] != NULL) {
                            // And if this is not the current position
                            if ((l > 0) && (idealPath[i][l - 1] != NULL)) {
                                // If the conflict cost is too high, make it 1 to
                                // emphasize the problem of visiting this cell
                                if (conflictCost[i][idealPath[i][l - 1]->y][idealPath[i][l - 1]->x][l - 1] >= 0.5) {
                                    conflictCost[i][idealPath[i][l]->y][idealPath[i][l]->x][l] = 1;
                                }
                            }

                            if (enable_print) printf("IDEAL PATH AT POS %i: [%d %d] - CCost %.2f\n", l, idealPath[i][l]->y, idealPath[i][l]->x, conflictCost[i][idealPath[i][l]->y][idealPath[i][l]->x][l]);
                            pathlength[i] = l;
                        }
                    }

                    if (enable_print) printf("0 SO FAR SO GOOD AGENT %i!!!\n", i);
                    previous = position[i];
                    if (enable_print) printf("0 Me QUIERO MOVER a [%d %d]\n", (position[i]->trace)->y,(position[i]->trace)->x);

                    if (position[i]->parent[i] != NULL) {
                        if (enable_print) printf(" desde [%d %d] \n", position[i]->parent[i]->y,position[i]->parent[i]->x);
                    }

                    if (enable_print) printf(" THE PATH LENGHT OF AGENT %i is %i \n", i + 1,pathlength[i]);

                    // SECOND SEARCH, BASED ON CONSTRAINTS/CONFLICTS:
                    if (!compute_constraintpath(i, lookahead)) {
                        if (enable_print) printf("No solution???? Might need to backtrack %i steps, \n",lastMobileCellDist[i]);
                        // NEED TO CHANGE MODE TO BACKTRACK!!!
                        backtrack[i] = 1;


                        if (position[i]->parent[i] != NULL) {
                            if (enable_print) printf(" BACKTRACKING TO postiion [%d %d]!!!\n",position[i]->parent[i]->y,position[i]->parent[i]->x);
                            previous = position[i];

                            if (enable_print) printf("Estoy vivo");
                            // getchar();
                            if (enable_print) printf(" Me QUIERO MOVER a %d %d \n",position[i]->parent[i]->y,position[i]->parent[i]->x);
                            if ((position[i]->parent[i]->blocked[0]) &&
                                (position[i]->parent[i]->x != position[i]->x) &&
                                ((position[i]->parent[i]->y != position[i]->y))) {
                                if (enable_print) printf(" PERO ESTOY BLOQUEADO (look: %i)\n", lookahead);

                                continue;
                            }
                            position[i] = position[i]->parent[i];
                            agent_cost[i] += euclidian(previous, position[i]);
                            robot_steps1++;
                            previous->trace = NULL;
                            previous->blocked[0] = 0;
                        }
                        if (position[i]->parent[i] == NULL) if (enable_print) printf("SOY NULL");
                        else if (enable_print) printf("NO SOY NULL");
                    } else {
                        if (enable_print) printf(" SO FAR SO GOOD AGENT %i, at postiion [%d %d]!!!\n",i, position[i]->y, position[i]->x);
                        previous = position[i];

                        if (position[i]->parent[i] != NULL) {
                            if (enable_print) printf("Whose parent is [%d %d]\n",position[i]->parent[i]->y,position[i]->parent[i]->x);
                        }
                        if (enable_print) printf(" Me QUIERO MOVER a %d %d \n", (position[i]->trace)->y,(position[i]->trace)->x);

                        for (int l = 1; l <= lookahead; l++) {
                            if (path[i][l] != NULL) {
                                if (enable_print) printf("PATH AT POS %i: [%d %d]\n", l, path[i][l]->y,path[i][l]->x);
                            }
                        }

                        if (enable_print) printf("REAL DEPTH %i", realDepth[i]);

                        if ((position[i]->trace->blocked[0]) &&
                            (position[i]->trace->x != position[i]->x) &&
                            ((position[i]->trace->y != position[i]->y))) {
                            if (enable_print) printf(" PERO ESTOY BLOQUEADO (look: %i)\n", lookahead);

                            continue;
                        }
                        // Asigna nueva posición al agente
                        position[i] = position[i]->trace;
                        // Determina el costo del movimiento y lo agrega al costo total 
                        agent_cost[i] += euclidian(previous, position[i]);
                        /* if (enable_print) printf("\n%d -> %f", i, agent_cost[i]);
                        getchar(); */
                        robot_steps1++;
                        previous->trace = NULL;
                        previous->blocked[0] = 0;

                        position[i]->blocked[0] = 1;
                        if (enable_print) printf(" Me movi a %d %d", position[i]->y, position[i]->x);
                        if (enable_print) printf(" con H %.1f \n",hvalues[MAZEWIDTH * position[i]->y + position[i]->x][i]);
                        position[i]->parent[i] = previous;
                        if (enable_print) printf(" My new parent is [%d %d]\n",position[i]->parent[i]->y, position[i]->parent[i]->x);

                        if (enable_print) printf(" -------------------------------------------------------""---------------------------\n");
                        if (enable_print) printf(" -------------------------------------------------------""---------------------------\n");
                        //	agentVelx[i]=(float)(position[i]->x - previous->x);
                        //	agentVely[i]=(float)(position[i]->y - previous->y);

                        for (j = 0; j < NAGENTS; j++) {
                            // if (enable_print) printf(" [4 4] at T 0: %i \n",
                            // maze1[4][4].blockedAgent[0][0]);
                            if (i != j) {
                                if (enable_print) printf("\nWatching %i from %i..\n", i + 1, j + 1);
                                if (enable_print) printf("OBSERVING MOVING AGENT with lookahaead %i and ""previous [%d %d]\n",lookahead, previous->y, previous->x);

                                observe_agent2(j, i, lookahead, previous);

                                if (enable_print) printf("\nWatching %i from %i.. \n", j + 1, i + 1);
                                if (enable_print) printf("MOVING AGENT OBSERVING\n");
                                observe_new_agents(i, j, lookahead); // Previous not used anymore
                            }
                        }


                        
                        // Esta rutina permite empujar los agentes fuera de su celda luego de PUSH_OVER_THRESHOLD tiempo en ella
                        // Si no me encuentro en mi goal
                        if (position[i] != goal[i]) {
                            // No me he movido en el paso anterior
                            if (position[i] == previous) {
                                // No me he movido desde la ultima vez
                                if (last_recently_see[i] != NULL &&
                                    last_recently_see[i] == position[i]) {
                                    push_over_func(i, previous);
                                } else {
                                    // Me moví desde la ultima vez, entonces reinicio
                                    push_over[i] = 0;
                                }
                                // Ultima vez visto
                                last_recently_see[i] = previous;
                                //for (int a = 0; a < NAGENTS; a++) {
                                //    printf("[%d]", push_over[a]);
                                //}
                                //printf("\n");
                            }
                        }



                        if (position[i] == goal[i]) {
                            if (goal_reached[i] == 0) {
                                lastfinish = time_step;
                                completion_time[i] = time_step;
                            }
                            goal_reached[i] = 1;
                            solution_cost += agent_cost[i];
                            finish_all--;
                            position[i]->obstacle = 0;
                            for (int j = 0; j < 100; j++) {
                                position[i]->blocked[j] = 0;
                                position[i]->blockedAgent[i][j];
                            }
                            #ifndef RANDOMMOVES
                            // position[i]->obstacle = 1;
                            // position[i]->x=1;
                            // position[i]->y=1;
                            if (enable_print) printf("** LLEGO time_step:%d** %d finish:%d cost:%f total ""cost:%f, now at [%d %d]\n",time_step, i, NAGENTS - finish_all, agent_cost[i],total_cost, position[i]->y, position[i]->x);
                            // getchar();
                            #endif

                        }
                        if (finish_all == 0 || time_step >= MAX_TIME_STEPS) {
                            // multi_print_grid();

                            enable_print = 1;
                            total_cost = 0;

                            char buf[0x100];
                            snprintf(buf, sizeof(buf), "log-resultados-%d.txt", lookahead);
                            FILE *fp;

                            fp = fopen(buf, "a+");

                            fprintf(fp, "RUN %ld\n", RUN1);
                            
                            fprintf(fp, "valores_ideales ");
                            fprintf(fp, "[");
                            for (int a = 0; a < NAGENTS; a++)
                            {
                                if (a == NAGENTS - 1) fprintf(fp, "%.0lf", hValueForAgent[a]);
                                else fprintf(fp, "%.0lf,", hValueForAgent[a]);
                            }
                            fprintf(fp, "]\n");

                            fprintf(fp, "costo_por_agente ");
                            fprintf(fp, "[");
                            for (int a = 0; a < NAGENTS; a++)
                            {
                                if (goal_reached[a]) total_cost += agent_cost[a];
                                if (a == NAGENTS - 1) fprintf(fp, "%.0lf", agent_cost[a]);
                                else fprintf(fp, "%.0lf,", agent_cost[a]);
                            }
                            float total_time_cost = 0;
                            fprintf(fp, "]\n");
                            
                            
                            fprintf(fp, "completion_time_por_agente ");
                            fprintf(fp, "[");
                            for (int a = 0; a < NAGENTS; a++)
                            {
                                total_time_cost += completion_time[a];
                                if (a == NAGENTS - 1) fprintf(fp, "%d", completion_time[a]);
                                else fprintf(fp, "%d,", completion_time[a]);
                            }
                            ("costo_promedio %f\n", total_cost / NAGENTS);
                            fprintf(fp, "]\n");

                            fprintf(fp, "tiempo_ultimo_agente_goal %d\n", lastfinish);
                            fprintf(fp, "tiempo_en_acabar %d\n", time_step);
                            fprintf(fp, "tiempo_promedio %f\n", total_time_cost / NAGENTS);
                            fprintf(fp, "agentes_en_goal %d\n", NAGENTS - finish_all);
                            fprintf(fp, "bad_good_total_rate_pred [%i,%i,%i,%.1f]\n", badp, goop, totp, (float)goop / (float)totp);
                            fprintf(fp, "push_out_count %d\n", push_out_count);

                            fclose(fp);
                            // getchar();

                            /* total_cost = 0;
                            printf("\nValores ideales:\n");
                            for (int a = 0; a < NAGENTS; a++) {
                                if (enable_print) printf("agent [%d] -> valor H: %f\n", a+1, hValueForAgent[a]);
                            }
                            if (enable_print) printf("Costo por agente\n");
                            for (int a=0; a < NAGENTS; a++){
                                total_cost += agent_cost[a];
                                if (enable_print) printf("agent [%d] -> costo total: %f\n", a+1, agent_cost[a]);
                            }
                            total_time_cost = 0;
                            if (enable_print) printf("Completion time por agente\n");
                            for (int a; a < NAGENTS; a++) {
                                total_time_cost += completion_time[a];
                                if (enable_print) printf("agent [%d] -> tiempo total: %d\n", a + 1, completion_time[a]);
                            }
                                ("Costo promedio: %f\n", total_cost / NAGENTS);
                            if (enable_print) printf("Tiempo en acabar: %d\n", time_step);
                            if (enable_print) printf("Tiempo promedio: %f\n", total_time_cost /  NAGENTS);

                            float hSuma = 0;
                            for (int a = 0; a < NAGENTS; a++) hSuma = hSuma + hValueForAgent[a];
                            float hMean = hSuma / NAGENTS;

                            if (enable_print) printf("Promedio A*: %f\n", hMean);

                            getchar(); */
                            
                            enable_print = 0;
                            return;
                        }
                    } // From computenewpath
                } // from computeshortestpath
            }
            // i = (i+1) % NAGENTS;
        }
        time_step++;
        // updatemaze1(previous,mazestart1);

        // getchar();
    } // end  while(finish_all)
    if (finish_all != 0) {
        if (enable_print) printf("\nNOT ALL AGENTS WERE ABLE TO REACH THEIR GOALS!!! :( :(  (look: ""%i)\n",lookahead);
        if (enable_print) printf("\nNOT ALL AGENTS WERE ABLE TO REACH THEIR GOALS!!! :( :( \n");
        if (enable_print) printf("\nNOT ALL AGENTS WERE ABLE TO REACH THEIR GOALS!!! :( :( \n");

        // getchar();
    } else {
        if (enable_print) printf("\nGREAT!!!  ALL AGENTS WERE ABLE TO REACH THEIR GOALS in %i ""TIMESTEPS!!! :) :) \n",time_step);
        if (enable_print) printf("\nGREAT!!!  ALL AGENTS WERE ABLE TO REACH THEIR GOALS in %i ""TIMESTEPS!!! :) :) \n",time_step);
        if (enable_print) printf("\nGREAT!!!  ALL AGENTS WERE ABLE TO REACH THEIR GOALS in %i ""TIMESTEPS!!! :) :) \n",time_step);
        // getchar();
    }

    return;
}

// 23677273--------------------------------------------------------------------------------------
void call_rtaastar() {
    FILE *salida;
    long long int j;
    float time_astar = 0;
    float average_expansion_persearch = 0;
    float average_trace_persearch = 0;
    float variance_expansion_persearch = 0;
    float SDOM = 0;
    int lookahead;
    int prunning, i;
    int look[9] = {6,9,12,15,18,21,24,27,30}; // 3,4,5,8,14};//{1,8,16,32,64,128,256,512,1024};
    float total_score[(int)(sizeof(look) / (float)sizeof(int))],
        avg_score[(int)(sizeof(look) / (float)sizeof(int))],
        total_time[(int)(sizeof(look) / (float)sizeof(int))];
    float avg_finish[(int)(sizeof(look) / (float)sizeof(int))],
        last_finish[(int)(sizeof(look) / (float)sizeof(int))];
    srand(time(NULL));
    float ftimes[RUNS];
    for (i = 0; i < (int)(sizeof(look) / (float)sizeof(int)); i++) {
        avg_finish[i] = 0;
        total_time[i] = 0;
        avg_score[i] = 0;
        lookahead = look[i];
        last_finish[i] = 0;
        if (enable_print) printf("Now I is %i \n", i);
        int RUN1_agents = 0;
        if (enable_print) printf("lookahead == [%d] ___________________________________\n",lookahead);

        badpredictions[i] = 0;

        totalpredictions[i] = 0;

        // For each iteration
        for (RUN1 = 0; RUN1 < RUNS; ++RUN1) {
            badp = 0;
            totp = 0;
            goop = 0;

            // For each agent
            for (int a = 0; a < NAGENTS; a++) {
                // and each other agent
                push_over[a] = 0;
                for (int j = 0; j < NAGENTS; j++) {
                    lastMobileCellDist[a] =
                        10000; // big number to emphasize that at the beginning he
                               // does not have memory of previously mobile states
                    pred_agents[a][j] = 0;
                    agentInfo[a] = -1;
                    good_pred_agents[a][j] = 0;
                    predict[a][j] = 0;
                    conflictType[a][j] = -1;
                }
            }
            if (enable_print) printf("case == [%ld] ___________________________________\n", RUN1);
            generate_maze(RUN1);
            if (enable_print) printf("NOW TEST RTA!!! \n");
            
            // Call to method, one per iteration
            test_rtaastar(lookahead, prunning);

            if (enable_print) printf("Agents Remaining: %i at RUN %ld \n", finish_all, RUN1);
            badpredictions[i] = badpredictions[i] + badp;
            totalpredictions[i] = totalpredictions[i] + totp;
            avg_finish[i] =
                (avg_finish[i] * RUN1 + (float)(NAGENTS - finish_all)) /
                (float)(RUN1 + 1);

            if ((NAGENTS - finish_all) > 0) {
                last_finish[i] =
                    (last_finish[i] * RUN1_agents + (float)lastfinish) /
                    (float)(RUN1_agents + 1);
                RUN1_agents = RUN1_agents + 1;
            }
            if (enable_print) printf("AVG # agents FINISH %f runagents %i\n", avg_finish[i],RUN1_agents);
            if (enable_print) printf("AVG COST %f, ", total_cost / (float)NAGENTS);
            avg_score[i] = avg_score[i] + total_cost / (float)NAGENTS;
            total_time[i] = total_time[i] + time_step;
            if (enable_print) printf("FINISH TIME %i in avg %f TIMESTEPS\n", lastfinish,last_finish[i]);
            if (enable_print) printf(" Bad predictions: %i, Good: %i and total: %i, rate: %.1f \n",badp, goop, totp, (float)goop / (float)totp);
            ftimes[RUN1] = lastfinish;
            for (int a = 0; a < NAGENTS; a++) {
                for (int j = 0; j < NAGENTS; j++) {
                    if (enable_print) printf("\nGoodPred for %i with %i: %.2f (%i/%i)", a + 1, j + 1,(float)good_pred_agents[a][j] / (float)pred_agents[a][j],good_pred_agents[a][j], pred_agents[a][j]);
                }
            }

            time_astar += 1.0 * (tv22.tv_sec - tv11.tv_sec) +
                          1.0 * (tv22.tv_usec - tv11.tv_usec) / 1000000.0;
            robotmoves_total1 += robot_steps1;
            lastfinish = -1000;
            #ifdef STATISTICS
            if (times_of_billion1 > 0)
                average_expansion_persearch =
                    ((float)1000000000 / (float)searches_astar1 *
                     (float)times_of_billion1) +
                    ((float)statexpanded1 / (float)searches_astar1);
            else
                average_expansion_persearch =
                    ((float)statexpanded1 / (float)searches_astar1);
            #endif

            if ((salida = fopen("Output-mrtaa-1-step", "a")) == NULL) {
                if (enable_print) printf("No se puede abrir el archivo de salida");
            }
            fprintf(salida, "%d %f %d %d %lld %f %d %ld %lld %ld", lookahead,
                    solution_cost, NAGENTS, NAGENTS - finish_all, searches_astar1,
                    (time_astar - time_astar_initialize1) * 1000, time_step - 1,
                    RUN1, statexpanded1, statpercolated2);

            fputs("\n", salida);
            fclose(salida);

            statexpanded1_first = 0;
            statexpanded1 = 0;
            robotmoves_total1 = 0;
            time_astar = 0;
            time_searches = 0;
            time_astar_initialize1 = 0;
            searches_astar1 = 0;
            average_expansion_persearch = 0;
            average_trace_persearch = 0;
            variance_expansion_persearch = 0;
            SDOM = 0;
            solution_cost = 0;
            max_expansions = 0;
            open_size = 0;
            memory_count = 0;
            statexpanded1_prune = 0;

            for (int a = 0; a < NAGENTS; a++) {
                for (int j = 0; j < NAGENTS; j++) {
                    pred_agents[a][j] = 0;
                    good_pred_agents[a][j] = 0;
                }
            }

        }
    }

    for (int i = 0; i < (int)(sizeof(look) / (float)sizeof(int)); i++) {
        lookahead = look[i];
        float op = 0;
        for (int r = 0; r < RUNS; r++) op = op + fabs(ftimes[r] - last_finish[i]);
        float stdv = sqrtf(op / (float)(RUNS - 1));
        if (enable_print) printf("\naccuracy of pred: %.1f",1 - ((float)badpredictions[i] / (float)totalpredictions[i]));
        if (enable_print) printf(" of %i predictions\n", totalpredictions[i]);
        if (enable_print) printf("AVG TIME FOR LOOK %i: ", lookahead);
        if (enable_print) printf("%f\n", last_finish[i]);
        if (enable_print) printf("AVG AGENTS FINISHIN FOR LOOK %i: ", lookahead);
        if (enable_print) printf("%f  std %f \n", avg_finish[i], stdv);
    }
    return;
}

#endif
