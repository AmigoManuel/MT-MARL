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
#define HA(from, to)                                                       \
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
#define QUEUE2_PUSH(x)               \
   {                                 \
      queue[pf2] = (x);              \
      (x)->heapindex = pf2;          \
      pf2 = (pf2 + 1) % QUEUE2_SIZE; \
      n2++;                          \
   }
#define QUEUE2_POP(x)                \
   {                                 \
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
    0;  // active the prune when there is an obstacle/border in the search tree
cell1 *CLOSED[10000];
cell1 *path[NAGENTS][200];  // 200 is a limit for lookahead, change accordingly
cell1 *idealPath[NAGENTS][200];
int learningCutoff[NAGENTS];
double f_value = 0;
double total_cost = 0;

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

cell1 *tmpcell1;
cell1 *tmpcell2;
cell1 *tmpcell3;
cell1 *lastMobileState[NAGENTS];
struct timeval tv11, tv22, tv11a, tv22a, tv11b, tv22b, tv11c, tv22c, tv11d,
    tv22d, tv11e, tv22e;

long numberofexp, hsal;

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
void Multi_print_grid() {
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

/* FIXME: Esta función no es llamada desde ningun lugar */
int compare_path(int a, int j, int myConflictStep, int otherConflictStep,
                 int lookahead) {
   float neigh_x, neigh_y, mine_x, mine_y;
   printf("MyConflictStep: %i and Other: %i, real Depth: %i \n", myConflictStep,
          otherConflictStep, realDepth[a]);
   // En el mejor de los casos es posible ver los siguientes pasos previstos
   if ((myConflictStep < realDepth[a]) && (otherConflictStep < realDepth[j])) {
      printf("Case 1");
      neigh_x = idealPath[j][otherConflictStep + 1]->x -
                idealPath[j][otherConflictStep]->x;
      neigh_y = idealPath[j][otherConflictStep + 1]->y -
                idealPath[j][otherConflictStep]->y;
      mine_x =
          idealPath[a][myConflictStep + 1]->x - idealPath[a][myConflictStep]->x;
      mine_y =
          idealPath[a][myConflictStep + 1]->y - idealPath[a][myConflictStep]->y;
   }

   if ((myConflictStep < realDepth[a]) && (otherConflictStep == realDepth[i])) {
      printf("Case 2");
   }

   if ((myConflictStep == realDepth[a]) && (otherConflictStep < realDepth[i])) {
      printf("Case 3");
   }

   if ((myConflictStep == realDepth[a]) &&
       (otherConflictStep == realDepth[i])) {
      printf("Case 4");
      neigh_x = idealPath[j][otherConflictStep]->x -
                idealPath[j][otherConflictStep - 1]->x;
      neigh_y = idealPath[j][otherConflictStep]->y -
                idealPath[j][otherConflictStep - 1]->y;
      mine_x =
          idealPath[a][myConflictStep]->x - idealPath[a][myConflictStep - 1]->x;
      mine_y =
          idealPath[a][myConflictStep]->y - idealPath[a][myConflictStep - 1]->y;
   }

   float dot_prod = mine_x * neigh_x + mine_y * neigh_y;
   float abs_value_mine = sqrtf(mine_x * mine_x + mine_y * mine_y);
   float abs_value_neigh = sqrtf(neigh_x * neigh_x + neigh_y * neigh_y);
   float anglerad = acosf(dot_prod / (float)(abs_value_mine * abs_value_neigh));
   printf("\nANGLERAD is %f, neigh is [%.1f %.1f] mine is [%.1f %.1f]\n",
          anglerad, neigh_x, neigh_y, mine_x, mine_y);

   //(anglerad<(3.14159265358979323846f/(float)2)) between 1.5 and 1.6
   if ((anglerad < (3.2f / (float)2)) && (anglerad > (3.0f / (float)2))) {
      // we are going in the same (or at least similar) direction
      printf("ALERTA de POINT intersection entre agentes %i y %i \n", a + 1,
             j + 1);
      conflictType[a][j] = 0;
   }  // between 3 and 3.2
   else if ((anglerad < 3.2) && (anglerad > 3)) {
      // we are going in the same (or at least similar) direction
      printf(
          "ALERTA de PATH INTERSECTION entre agentes %i y %i: van hacia lados "
          "contrarios \n",
          a + 1, j + 1);
      conflictType[a][j] = 1;

   }  // less than 1
   else if (anglerad < 1) {
      // we are going in the same (or at least similar) direction
      printf(
          "ALERTA DE SIMILARIDAD (PATH INTERSECTION)  ENTRE AGENTES %i y %i : "
          "van hacia el mismo lado \n",
          a + 1, j + 1);
      conflictType[a][j] = 1;
   } else {
      printf(
          " NON determined type of conflict, assumoing POINT intersection "
          "entre agentes %i y %i \n",
          a + 1, j + 1);
      conflictType[a][j] = 0;
   }
   return 1;
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
   for (int a = 0; a < NAGENTS; a++){
      for (int future = 0; future < 120; future++){
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
         printf(
             "Checking constraints between my ideal path and agent %i (me: %i "
             ", other: %i) intended motion:.. \n",
             j + 1, formula, agentInfo[j]);
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
                  printf(
                      "Agent %i: [%d %d] me: [%d %d], H: %.1f, step %i \n",
                      j + 1, idealPath[j][l]->y, idealPath[j][l]->x,
                      idealPath[a][l]->y, idealPath[a][l]->x,
                      hvalues[MAZEWIDTH * position[j]->y + position[j]->x][j],
                      l);

                  // Si mi posición actual esta en el camino ideal de mi vecino
                  // dentro del paso l (how bad is to stay here)
                  if ((position[a]->y == idealPath[j][l]->y) &&
                      (position[a]->x == idealPath[j][l]->x)) {
                     // Incrementa el numero de conflictos para esta posición
                     // por este agente.
                     position[a]->numConflicts[a] =
                         position[a]->numConflicts[a] + 1;
                     printf(
                         "Oops! might need to move from CURRENT pos [%d %d], "
                         "total conflicts: %i \n",
                         position[a]->y, position[a]->x,
                         position[a]->numConflicts[a]);
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
                        printf(
                            "Oops! might need to wait/backtrack,  FUTURE "
                            "conflict at position [%d %d] \n",
                            idealPath[a][l + 1]->y, idealPath[a][l + 1]->x);
                        printf("My info %i vs other agent's info %i \n",
                               formula, agentInfo[j]);
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
                              printf("Oops! PATH CONFLICT!!!\n");
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
                        printf(
                            "Oops! might need to WAIT, OTHER AGENT IN MY WAY "
                            "at position [%d %d] \n",
                            idealPath[a][l + 1]->y, idealPath[a][l + 1]->x);
                        printf("My info %i vs other agent's info %i \n",
                               formula, agentInfo[j]);
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
                        printf(
                            "Oops! UNFEASIBLE SWAPPING projected at [%d %d] \n",
                            idealPath[a][l + 1]->y, idealPath[a][l + 1]->x);
                        printf("My info %i vs other agent's info %i \n",
                               formula, agentInfo[j]);
                        // Basado en esta realacion, necesitamos tomar una
                        // desicion Conocemos de inmediato que ellos se
                        // encuetran en celdas opuestas
                        conflictType[a][j] = 1;
                        conflictStepMe = l;
                        conflictStepOther = l + 1;
                     }
                  }

                  // TODO: este if no tiene nada
                  /* if (hvalues[MAZEWIDTH * position[j]->y + position[j]->x][j]
                  > hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]) {

                  } */

                  // Aquí acaba el lookahead
               }
            }

            // Si el conflictType no se encuentra seteado (si llega con -1)
            // este no es un swap (ambientes estrechos 1 casilla)
            // lo que significa que es un punto de interseccion
            if ((conflictType[a][j] < 0) && (conflictStepMe > -1)) {
               conflictType[a][j] = 0;
            }

            // if((path[j][2]!=NULL)&&(path[a][2]!=NULL)&&(conflictStepMe>-1)&&(conflictStepOther>-1))
            // { printf("Comparing paths....\n"); compare_path(a,j,
            // conflictStepMe, conflictStepOther, lookahead);
            // }

         }  // acaba si tengo que tomar atención sobre mi vecino
      }     // acaba "si mi vecino es visible y no se encuentra en su destino"
   }        // acaba el de agentes

   // En este pnto el conflictType debería encontrarse seteado
   // El paso de conflicto por cada agentes (conflictStepMe, conflictStepOther)
   // NO SERA ALMACENADO!! ¿yo quiero esto?

   // Segunda seccion --> determinar el costo de restricciones
   // Valores heuristicos mas relevantes
   int maxHagent = a;  // Agente con el maximo valor heuristico actual
   int maxInfo = a;    // Maximo valor camino actual

   // En la seccion anterior, iteramos por cada uno de los pasos en el plan de
   // movimiento, pero esta comienza desde pathlength[a], mientra la otra desde
   // 0
   for (int future = pathlength[a]; future <= lookahead; ++future) {
      cell_role = -1;
      printf("*****Checking issues of stayin here [%d %d] at time %i!!: \n",
             currentCell->y, currentCell->x, future);

      // Verificando si mi celda actual se encuetra bloqueada
      // Si me quedo en esta celda a futuro llevara posiblemente a un conflicto
      if (((maze1[currentCell->y][currentCell->x].blockedAgent[a][future]) &&
           (future > 0)) ||
          (((maze1[currentCell->y][currentCell->x]
                 .blockedAgent[a][future - 1])) &&
           (future > 0))) {
         printf("*****Staying here will bring me trouble at time %i!!\n",
                future);

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
            printf(
                "\n\n****FUTURE At %i another agent WOULD LIKE TO MOVE to [%d "
                "%d], but who??\n",
                future, currentCell->y, currentCell->x);  // cont_closed
            // Mayor valor heuristico presente entre los agentes
            printf("*********************************LEEME*******************\n");
            printf("position[%d]->y: %d\n", a, position[a]->y);
            printf("position[%d]->x: %d\n", a, position[a]->x);
            printf("a: %d\n", a);
            printf("hvalues: %f\n",hvalues[MAZEWIDTH * (position[a]->y) + (position[a]->x)][a]);
            printf("backupH: %f\n", backupH[MAZEWIDTH * (position[a]->y) + (position[a]->x)][a]);
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

                  if (path[j][future] != NULL){
                     printf("\033[1;32m");
                     printf("No es null\n");
                     printf("\033[0m");
                  }else{
                     printf("\033[1;31m");
                     printf("Es null\n");
                     printf("\033[0m");
                  }
                  printf("j: %d\n", j);
                  printf("future: %d\n", future);
                  printf("path[%d][%d]->y: %d\n", j, future, path[j][future]->y);
                  printf("path[%d][%d]->x: %d\n", j, future, path[j][future]->x);

                  printf("His previous position at [%d %d] had a degree of %i \n",
                      path[j][future]->y, path[j][future]->x,
                      maze1[path[j][future]->y][path[j][future]->x].degree[j]);
                  printf("My info %i vs other agent's info %i \n", formula,
                         agentInfo[j]);

                  // Si es un punto de conflicto interseccion
                  if (conflictType[a][j] == 0) {
                     printf(" POINT conflict, my cost: %i vs his :%i \n",
                            (int)(hvalues[MAZEWIDTH * position[a]->y +
                                          position[a]->x][a]) +
                                2,
                            (int)(hvalues[MAZEWIDTH * position[j]->y +
                                          position[j]->x][j]) +
                                1);

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
                     printf(" PATH conflict\n");
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
                     printf(
                         "(1) We are both SCREWED!!, his H is %i, mine is %i "
                         "and my distance from start is %i, dist from agent is "
                         "%i, total for comparison: Other %i vs mine %i \n",
                         agentInfo[j] - 20003, formula - 20003,
                         distanceFromStart[a], distanceFromAgent,
                         agentInfo[j] - 20003 - distanceFromAgent,
                         distanceFromStart[a]);

                     if (agentInfo[j] - 20003 < formula - 20003) {
                        printf(
                            "I COULD AT LEAST HELP HIM GET TO HIS GOAL!!!\n");
                        // j pasa a ser el maxInfo
                        maxInfo = j;
                        // Marca deadlock
                        deadlock[a][currentCell->y][currentCell->x][future] = 1;
                     } else {
                        printf(" NO WAY, I CANT HELP EVEN IF I WANTED TO\n");
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

                  printf("And his path is \n");

                  // Itera por las veces que permita el lookahead
                  for (int l = 1; l <= lookahead; l++) {
                     // Si a es distinto de j y mi camino ideal se ecuentra
                     // definido
                     if ((a != j) && (idealPath[j][l] != NULL)) {
                        printf("Agent %i: [%d %d], H: %.1f, step %i \n", j + 1,
                               idealPath[j][l]->y, idealPath[j][l]->x,
                               hvalues[MAZEWIDTH * position[j]->y + position[j]->x][j],l);

                        // Si encuentra una intersección entre a y j sobre la
                        // iteración l
                        if ((position[a]->y == idealPath[j][l]->y) &&
                            (position[a]->x == idealPath[j][l]->x)) {
                           // Asigna el contador de conflictos a la posición
                           // dada
                           position[a]->numConflicts[a] = numConflicts;
                           printf(
                               "BOops! might need to move, total conflicts: %i "
                               "\n",
                               position[a]->numConflicts[a]);
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
                  printf(
                      " with H of %.1f, (%.1f)  vs ",
                      hvalues[MAZEWIDTH * (currentCell->y) + (currentCell->x)]
                             [j],
                      hvalues[MAZEWIDTH * (position[j]->y) + (position[j]->x)]
                             [j]);

                  // Despliega valor heuristico de a
                  printf(
                      " my H of %.1f, (%.1f) ",
                      hvalues[MAZEWIDTH * (currentCell->y) + (currentCell->x)]
                             [a],
                      hvalues[MAZEWIDTH * (position[a]->y) + (position[a]->x)]
                             [a]);  //[MAZEWIDTH*(tmpcell1->y) +
                                    //(tmpcell1->x)][a]);

                  // Despliega la suma de los valores heuristicos
                  sumH = sumH + backupH[MAZEWIDTH * (currentCell->y) +
                                        (currentCell->x)][j];
                  printf(", SumH is %f \n", sumH);

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
            printf(
                "The Agent with MAX H, whose H changes by my movement  is %i ",
                maxHagent + 1);
            // Con un valor heuristico de
            printf(" with H of %f \n", maxH);
         }

         // Otro agente ya debiera encontrarse aquí
         if (((maze1[currentCell->y][currentCell->x]
                   .blockedAgent[a][future - 1])) &&
             (future > 0)) {
            // El agente future-1 no puede desplazarse a la celda actual, ya que
            // hay un agente en ella
            printf(
                "****At %i MIGHT NOT BE ABLE to move to [%d %d], there MIGHT "
                "ALREADY BE an agent\n",
                future - 1, currentCell->y, currentCell->x);
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
                  printf("This guy -> %i  (total %i)", j + 1, numConflicts);
                  // Un valor heuristico de hvalues
                  printf(
                      " with H of %.1f, (%.1f)  vs ",
                      hvalues[MAZEWIDTH * (currentCell->y) + (currentCell->x)]
                             [j],
                      hvalues[MAZEWIDTH * (position[j]->y) + (position[j]->x)]
                             [j]);
                  // vs el agente a con un valor heuristico de hvalues
                  printf(
                      " my H of %.1f, (%.1f) ",
                      hvalues[MAZEWIDTH * (currentCell->y) + (currentCell->x)]
                             [a],
                      hvalues[MAZEWIDTH * (position[a]->y) + (position[a]->x)]
                             [a]);
                  // Actualiza la suma de valores heuristicos
                  sumH = sumH + backupH[MAZEWIDTH * (currentCell->y) +
                                        (currentCell->x)][j];
                  printf(", SumH is %f \n", sumH);
                  // Copied from above
                  printf("My info %i vs other agent's info %i \n", formula,
                         agentInfo[j]);

                  // Si existe un punto de conflicto entre a y j
                  if (conflictType[a][j] == 0) {
                     printf(" POINT conflict, my cost :  %i vs his :%i \n",
                            (int)(hvalues[MAZEWIDTH * position[a]->y +
                                          position[a]->x][a]) +
                                2,
                            (int)(hvalues[MAZEWIDTH * position[j]->y +
                                          position[j]->x][j]) +
                                1);

                     // Si el valor heuristico de j es mayor que el de a
                     if ((int)(hvalues[MAZEWIDTH * position[j]->y +
                                       position[j]->x][j]) +
                             1 >
                         (int)(hvalues[MAZEWIDTH * position[a]->y +
                                       position[a]->x][a]) +
                             2) {
                        // Actualiza maxInfo a j
                        maxInfo = j;
                        printf(" MaxInfo: %i\n", maxInfo);
                     }
                  }

                  // Si existe punto de conflicto
                  if (conflictType[a][j] == 1) {
                     printf(" PATH conflict\n");
                     if (formula < agentInfo[j]) {
                        // Actualiza maxInfo a j
                        maxInfo = j;
                        printf(" MaxInfo: %i\n", maxInfo + 1);
                     }
                  }

                  // Si formula y el camino asociado a j superan el umbral
                  if ((formula > 20000) && (agentInfo[j] > 20000)) {
                     // Determina la distancia entre a y j
                     int distanceFromAgent =
                         abs(position[a]->x - position[j]->x) +
                         abs(position[a]->y - position[j]->y);

                     printf(
                         "(2) We are both SCREWED!!, his H is %i, mine is %i "
                         "and my distance from start is %i, dist from agent is "
                         "%i, total for comparison: Other %i vs mine %i \n",
                         agentInfo[j] - 20003, formula - 20003,
                         distanceFromStart[a], distanceFromAgent,
                         agentInfo[j] - 20003 - distanceFromAgent,
                         distanceFromStart[a]);

                     // Si el camino sobre j es menor a formula
                     if (agentInfo[j] - 20003 < formula - 20003) {
                        printf(
                            " I COULD AT LEAST HELP HIM GET TO HIS GOAL!!!\n");
                        // Asigna j a maxInfo
                        maxInfo = j;
                     } else {
                        printf(" NO WAY, I CANT HELP EVEN IF I WANTED TO\n");
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
            printf(
                "The Agent with MAX H, whose H changes by my movement  is %i ",
                maxHagent + 1);
            printf(" with H of %f \n", maxH);
         }

         // Tercer caso - hay un agente en la celda actual

         // Por cada uno de los agentes
         for (int u = 0; u < NAGENTS; ++u) {
            // Si en celda actual hay un agente que quiere llegar
            if ((currentCell->x == position[u]->x) &&
                (currentCell->y == position[u]->y) && (future == 1) &&
                (u != a)) {
               printf(
                   "\nOps, agent %i is at the next position position, is it "
                   "its goal?..",
                   u + 1);

               // Si el agente actual esta en su goal lo pasa por encima
               // ignorandolo
               if ((goal[u]->y == position[u]->y) &&
                   (goal[u]->x == position[u]->x)) {
                  printf("\nYES, I can move through");
               } else {
                  printf("\n No, Cant move here\n");
                  // caso contrario no es posible moverse sobre la celda actual
                  canmovehere = 0;
               }
            }
         }

         // En el caso que la celda actual se ecuentre bloqueada por un agente
         if (((maze1[currentCell->y][currentCell->x].blockedAgent[a][0])) &&
             (!canmovehere)) {
            // No puedo moverme sobre la celda hay un agente
            printf("\n****At %i Cant move to [%d %d], there is an agent\n",
                   future, currentCell->y, currentCell->x);
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
                  printf("This guy -> %i, total conflicts %i \n", j + 1,
                         (currentCell)->numConflicts[a]);
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
                       .agentMovingTo[a][future][j] > 0) {
                  // si el agente j se encuentra en el goal
                  if (goal_reached[j]) {
                     // pasa de j y continua
                     continue;
                  }
                  // Incrementa el contador de conflictos
                  numConflicts++;
                  printf(
                      "This guy -> %i wants to move where I am (planning to "
                      "be)  (total %i) at time %i \n",
                      j + 1, numConflicts, future);

                  printf(
                      "His previous position at [%d %d] had a degree of %i \n",
                      idealPath[j][future]->y, idealPath[j][future]->x,
                      maze1[idealPath[j][future]->y][idealPath[j][future]->x]
                          .degree[j]);
                  printf("My info %i vs other agent's info %i \n", formula,
                         agentInfo[j]);
                  // Si formula y el camino de j superan el umbral
                  if ((formula > 20000) && (agentInfo[j] > 20000)) {
                     // Determina la distancia entre a y j
                     int distanceFromAgent =
                         abs(position[a]->x - position[j]->x) +
                         abs(position[a]->y - position[j]->y);
                     printf(
                         "(3) We are both SCREWED!!, his H is %i, mine is %i "
                         "and my distance from start is %i, dist from agent is "
                         "%i, total for comparison: Other %i vs mine %i \n",
                         agentInfo[j] - 20003, formula - 20003,
                         distanceFromStart[a], distanceFromAgent,
                         agentInfo[j] - 20003 - distanceFromAgent,
                         distanceFromStart[a]);

                     // Si el camino sobre j es menor que el determinado
                     // mediante formula
                     if (agentInfo[j] < formula) {
                        printf(
                            " I COULD AT LEAST HELP HIM GET TO HIS GOAL!!!\n");
                        // Marca j como el mayor camino
                        maxInfo = j;
                        // Marca deadlock en a sobre la celda actual en el paso
                        // future
                        deadlock[a][currentCell->y][currentCell->x][future] = 1;
                     } else {
                        printf(" NO WAY, I CANT HELP EVEN IF I WANTED TO\n");
                        maxInfo = a;
                     }
                  } else {
                     if (formula < agentInfo[j]) {
                        maxInfo = j;
                        // printf("MAX INFO: %i, role(%i, %i): %i %i\n",j+1,a+1,
                        // j+1,role[a][j],role[1][0]);
                     } else {
                        maxInfo = a;
                        // printf("MAX INFO: %i, role(%i, %i): %i %i\n",a+1,a+1,
                        // j+1,role[a][j],role[1][0]);
                     }
                  }
                  // TODO:
                  determine_role(&role[a][j], maxInfo, a, j, &cell_role);
               }
            }
         }
         printf(" ªªªª****ªªªªª*****A THE AGENT WITH MAX INFO IS %i\n",
                maxInfo + 1);
         // if(maxInfo!=a)
         // if(role[i][j]<0.9)
         if (cell_role == 0) {
            int step = 0;
            if (initialState == 0) {
               step = pathlength[a];
            }
            printf(" My step (of pathlength) is now %i\n", step);
            if (conflictCost[a][currentCell->y][currentCell->x][step] ==
                0)  //<  (float)1/(float)(future-pathlength[a]+1))
            {
               printf("CCost of current cell is %.1f\n",
                      conflictCost[a][currentCell->y][currentCell->x][step]);
               // If the neighbor tried to move to my "current" cell
               if (!wasthere) {
                  printf(" %f  + %f= %f\n",
                         (deadlock[a][currentCell->y][currentCell->x][step]),
                         (float)1 / (float)(future - step + 2),
                         (deadlock[a][currentCell->y][currentCell->x][step]) +
                             (float)1 / (float)(future - step + 2));
                  conflictCost[a][currentCell->y][currentCell->x][step] =
                      (deadlock[a][currentCell->y][currentCell->x][step]) +
                      (float)1 / (float)(future - step + 2);
                  if (conflictCost[a][currentCell->y][currentCell->x][step] >
                      1) {
                     conflictCost[a][currentCell->y][currentCell->x][step] = 1;
                  }
                  printf("CASE A!!\n");
                  printf("New CCost of current cell is %.1f\n",
                         conflictCost[a][currentCell->y][currentCell->x][step]);
               } else  // if the neighbor was there when the agent tried to move
                       // to the cell
               {
                  conflictCost[a][currentCell->y][currentCell->x][future] =
                      (deadlock[a][currentCell->y][currentCell->x][step]) +
                      (float)1 / (float)((future - 1) - step + 2);
                  if (conflictCost[a][currentCell->y][currentCell->x][step] >
                      1) {
                     conflictCost[a][currentCell->y][currentCell->x][step] = 1;
                  }
                  printf("CASE B!!\n");
               }
               // printf(" ªªªª****ªªªªª***** My time %i vs conflict time %i,
               // ConflictCost at time %i is: %f \n", pathlength[a], future,
               // pathlength[a],
               // conflictCost[a][currentCell->y][currentCell->x][pathlength[a]]);
            }
            printf(
                " ªªªª****ªªªªª***** My time %i vs conflict time %i, "
                "ConflictCost at time %i is: %f, deadlock at %i is: %f  \n",
                step, future, step,
                conflictCost[a][currentCell->y][currentCell->x][step], step,
                deadlock[a][currentCell->y][currentCell->x][step]);
         }
      }
   }
}

void determine_role(int *roleij, int maxInfo, int a, int j, int *cell_role) {
   // printf("Inside, role is %i", *roleij);
   // meaning that it has not been set yet
   if (*roleij == -1) {
      if (maxInfo == a) {
         // printf("This is me\n");
         *roleij = 1;
      } else {
         *roleij = 0;
         // printf("I defer\n");
      }
   }
   // Meaning that there is already a relation between the agents a and j
   else {
   }

   if (*roleij == 0) {
      *cell_role = 0;
      // printf("I defer2\n");
   }

   if ((*roleij == 1) && (*cell_role != 0)) {
      *cell_role = 1;
      // printf("This is me2\n");
   }
   printf("Role between %i and %i is %i, cell_role is %i\n", a + 1, j + 1,
          *roleij, *cell_role);
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
      printf("Awesome! can use this state for swapping!!!\n");
      lastMobileCellDist[a] = 0;
      lastMobileState[a] = &maze1[position[a]->y][position[a]->x];

   } else {
      if (lastMobileCellDist[a] == 0) {
         // lastMobileCellDist[a]=	lastMobileCellDist[a]+1;
      }
   }

   printf(
       " COMPUTING FOR %i (%i), at [%d %d] w degree %i, lastMobile at %i!!!\n",
       a + 1, pathlength[a], position[a]->y, position[a]->x,
       maze1[position[a]->y][position[a]->x].degree[a], lastMobileCellDist[a]);

   // Show previous cell
   if (position[a]->parent[a] != NULL) {
      printf("parent: [%d %d]!!!\n", position[a]->parent[a]->y,
             position[a]->parent[a]->x);
      // position[a]->parent[a]=position[a]->searchtree;
   }

   // Show last mobile state
   if (lastMobileState[a] != NULL) {
      printf(" lastMobile at [%d %d] ", lastMobileState[a]->y,
             lastMobileState[a]->x);
   }

   // For each cell in the map
   printf("Checking predicted occupied states up to %i..", lookahead);
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
               printf(" BUSY [%d %d] at T %i\n", y, x, z);
               if (maze1[y][x].toTransition[a][z] >
                   0)  //(toTransition[x][y][a][z]>0)
               {
                  printf(" TO [%d %d] at T %i: %i by agents: ", y, x, z,
                         maze1[y][x].toTransition[a][z]);

                  for (int j = 0; j < NAGENTS; ++j) {
                     if (maze1[y][x].agentMovingTo
                             [a][z][j])  //(agentMovingTo[x][y][a][z][j]>0)
                     {
                        printf(" %i, ", j + 1);
                     }
                  }
                  printf("\n");
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
                          1))  //(betweenTransition[x][y][e][q][a][z]>0)
                     {
                        for (int j = 0; j < NAGENTS; ++j) {
                           if ((z == 1) && (position[j]->x == x) &&
                               (position[j]->y == y) &&
                               (maze1[q][e].agentMovingTo[a][z][j] == 1)) {
                              printf(
                                  " TRANSITION BETWEEN [%d %d] and [%d %d] at "
                                  "T %i, so To should be %i\n",
                                  y, x, q, e, z,
                                  maze1[q][e].toTransition[a][z]);
                           }

                           if ((z > 1) &&
                               (maze1[y][x].agentMovingTo[a][z - 1][j] == 1) &&
                               (maze1[q][e].agentMovingTo[a][z][j] == 1)) {
                              printf(
                                  " TRANSITION BETWEEN [%d %d] and [%d %d] at "
                                  "T %i, so To should be %i\n",
                                  y, x, q, e, z,
                                  maze1[q][e].toTransition[a][z]);
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
   printf("My H currently is %.1f \n",
          hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]);
   // Checking mobility:

   printf("CHECKING SURROUNDINGS FOR OBSTACLES: %f\n",
          hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]);
   printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y, position[a]->x,
          maze1[position[a]->y][position[a]->x].obstacle);
   printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y + 1,
          position[a]->x, maze1[(position[a]->y) + 1][position[a]->x].obstacle);
   printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y,
          position[a]->x + 1,
          maze1[position[a]->y][(position[a]->x) + 1].obstacle);
   printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y,
          position[a]->x - 1,
          maze1[position[a]->y][(position[a]->x) - 1].obstacle);
   printf("Cell [%d %d] has obstacle? : %i \n", position[a]->y - 1,
          position[a]->x,
          maze1[(position[a]->y) - 1][(position[a]->x)].obstacle);

   int mobility = 4 - (maze1[(position[a]->y) + 1][position[a]->x].obstacle +
                       maze1[position[a]->y][(position[a]->x) + 1].obstacle +
                       maze1[position[a]->y][(position[a]->x) - 1].obstacle +
                       maze1[(position[a]->y) - 1][position[a]->x].obstacle);
   // printf("Mobility : %i \n",mobility);
   printf("CHECKING SURROUNDINGS FOR AGENTS: %f\n",
          hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]);
   // Should compute here which of the neighboring cells are occupied by other
   // agents

   int agentNext = 0;

   if (maze1[(position[a]->y) + 1][position[a]->x].blockedAgent[a][0]) {
      printf("Cell [%d %d] has AGENT \n", position[a]->y + 1, position[a]->x);
      agentNext++;
   }

   if (maze1[(position[a]->y) - 1][position[a]->x].blockedAgent[a][0]) {
      printf("Cell [%d %d] has AGENT \n", position[a]->y - 1, position[a]->x);

      agentNext++;
   }

   if (maze1[(position[a]->y)][(position[a]->x) + 1].blockedAgent[a][0]) {
      printf("Cell [%d %d] has AGENT \n", position[a]->y, position[a]->x + 1);
      agentNext++;
   }

   if (maze1[(position[a]->y)][(position[a]->x) - 1].blockedAgent[a][0]) {
      printf("Cell [%d %d] has AGENT \n", position[a]->y, position[a]->x - 1);
      agentNext++;
   }

   int agentMobility = 4 - agentNext;
   printf("Agent Mobility : %i \n", agentMobility);

   int somethingNext = 0;

   if ((maze1[(position[a]->y) + 1][position[a]->x].blockedAgent[a][0]) ||
       (maze1[(position[a]->y) + 1][position[a]->x].obstacle)) {
      printf("Cell [%d %d] has SOMETHING \n", position[a]->y + 1,
             position[a]->x);
      somethingNext++;
   }

   if ((maze1[(position[a]->y) - 1][position[a]->x].blockedAgent[a][0]) ||
       (maze1[(position[a]->y) - 1][position[a]->x].obstacle)) {
      printf("Cell [%d %d] has SOMETHING \n", position[a]->y - 1,
             position[a]->x);

      somethingNext++;
   }

   if ((maze1[(position[a]->y)][(position[a]->x) + 1].blockedAgent[a][0]) ||
       (maze1[(position[a]->y)][(position[a]->x) + 1].obstacle)) {
      printf("Cell [%d %d] has SOMETHING \n", position[a]->y,
             position[a]->x + 1);
      somethingNext++;
   }

   if ((maze1[(position[a]->y)][(position[a]->x) - 1].blockedAgent[a][0]) ||
       (maze1[(position[a]->y)][(position[a]->x) - 1].obstacle)) {
      printf("Cell [%d %d] has SOMETHING \n", position[a]->y,
             position[a]->x - 1);
      somethingNext++;
   }

   int netMobility = 4 - somethingNext;

   printf("NET Mobility : %i \n", netMobility);

   distanceFromStart[a] = abs(position[a]->x - initialCellX[a]) +
                          abs(position[a]->y - initialCellY[a]);

   printf("Distance from Start Cell: %i \n", distanceFromStart[a]);

   if (mobility > 2)  // If the agent can step out of the other's way
   {
      lastMobileCellDist[a] = 0;
      // need to store this cell
      lastMobileCellY[a] = position[a]->y;
      lastMobileCellX[a] = position[a]->x;

   } else  // Agent cannot step out of the other's way
   {
      if (lastMobileCellDist[a] <
          900)  // If there is a last mobile cell somewhere
      {
         lastMobileCellDist[a] =
             lastMobileCellDist[a] + 1;  // increase the distance to it
      }
   }

   printf("My M currently is %i \n", lastMobileCellDist[a]);

   // Compute my formula..
   int formula =
       (int)(hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]) +
       (2 * (lastMobileCellDist[a]) + 3);

   printf(
       "My formula is %i and my pathlenght is %i ((H:) %i + 2* "
       "(lastmobilecell) %i +3)\n",
       formula, pathlength[a],
       (int)(hvalues[MAZEWIDTH * position[a]->y + position[a]->x][a]),
       lastMobileCellDist[a]);

   agentInfo[a] = formula;

   // Review path while there is a desired cell
   for (int l = 1; l <= lookahead; l++) {
      if (idealPath[a][l] != NULL) {
         printf("My current IDEAL path at pos %i is [%d %d]\n", l,
                idealPath[a][l]->y, idealPath[a][l]->x);
      }

      if (idealPath[1][l] != NULL) {
         printf("Agent 2 IDEAL path at pos %i is [%d %d]\n", l,
                idealPath[1][l]->y, idealPath[1][l]->x);
      }
   }

   // As a new path will be computed, need to reset path to NULL
   for (int l = 0; l < lookahead; ++l) {
      path[a][l] = NULL;
   }
   // getchar();

   mazestart1 = position[a];  // Current position
   mazegoal1 = goal[a];       // New position
   //	printf("a:%d [%d,%d]\n",a, position[a]->y,position[a]->x);

   mazeiteration1++;
   emptyheap2();
   int newdepth = 0;
   cont_closed = 0;  // Initialized the number of "steps" into the future

#ifdef STATISTICS
   searches_astar1++;
   statexpanded1_initial = statexpanded1;
#endif

   initialize_state(mazestart1);
   mazestart1->g = 0;  // Setting start node cost to zero
   mazestart1->key = 0;
   mazestart1->searchtree = NULL;
   mazestart1->trace = NULL;
   insertheap2(mazestart1);
   flag_success1 = 0;
   // cell1 *tmpcell3
   float lastStepDepth = 0;

   while (topheap2() != NULL) {
      // If in the middle of the search...
      if (cont_closed > 0) {
         printf("\n****GETTING NEW NODE FROM THE STACK!! ");
         printf("\nThe previously analyzed node has degree %i ",
                tmpcell1->degree[a]);
      }
      tmpcell3 = tmpcell1;
      tmpcell1 = topheap2();

      // If in the middle of the search...
      if (cont_closed > 0) {
         printf(
             "\nThe next node to expand is [%d %d] with depth %i (or %i) from "
             "parent [%d %d] (cont %i)",
             tmpcell1->y, tmpcell1->x, tmpcell1->tmpdepth[a], newdepth,
             tmpcell3->y, tmpcell3->x, cont_closed);

         // Update search settings
         if (cont_closed == lookahead) {
            lastStepDepth = tmpcell1->tmpdepth[a];
            // last step of path is at depth tmpcell1->tmpdepth[a]
         }

         if (newdepth != tmpcell1->depth[a]) {
            printf("Strange..");
            // getchar();
         }
         tmpcell1->depth[a] = tmpcell1->tmpdepth[a];
         tmpcell1->searchtree =
             tmpcell1->tmpsearchtree[tmpcell1->depth[a]];  // tmpcell3;

         // printf("The parent of [%d %d] is [%d %d] at depth %i..",
         // tmpcell1->y, tmpcell1->x,tmpcell1->searchtree->y,
         // tmpcell1->searchtree->x, tmpcell1->move[d]->depth[a]);
         // getchar();
         // tmpcell1->searchtree = tmpcell1->tmpsearchtree;
      }

      //   printf("A* top a:%d [%d,%d] It:%d LA:%d\n",a,
      //   tmpcell1->y,tmpcell1->x, mazeiteration1,lookahead);

      // WHEN AT THE END OF THE SEARCH:
      if ((tmpcell1 == mazegoal1) ||
          (cont_closed == lookahead)) {  // open_size = opensize2() + open_size;
         // Se calcul f para actualizar h

         f_value =
             hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a] + tmpcell1->g;
         printf(" \n\nH value of [%d %d]: %f,", tmpcell1->y, tmpcell1->x,
                hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a]);
         printf(" G : %f\n", tmpcell1->g);

         printf("\nLEARNING NEW HEURISTICS OF FOUND PATH.. %f\n", tmpcell1->g);
         cellpas = tmpcell1;
         for (d = 0; d < cont_closed; d++) {
            hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a] =
                max(hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a],
                    f_value - CLOSED[d]->g);

            // max(CLOSED[d]->h,f_value - CLOSED[d]->g);
            printf("Updating H of [%d %d] = %.1f, d: %i, size %i \n",
                   CLOSED[d]->y, CLOSED[d]->x,
                   hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a], d,
                   sizeheap2());
         }

         printf(" Final destination : [%d %d]", cellpas->y, cellpas->x);
         // getchar();

         flag_success1 = 1;
         tmpcell1 = popheap2();
         printf(
             "\n(BEST MOVE WIHTOUT ACCOUNTING FOR AGENTS) A* top a:%d [%d,%d] "
             "\n",
             a, tmpcell1->y, tmpcell1->x);
         break;
      }

      // Obtaining new cell from the stack
      tmpcell1 = popheap2();
      tmpcell1->degree[a] = 0;
      CLOSED[cont_closed] = tmpcell1;
      // Increase the number of steps
      cont_closed++;

      if (cont_closed == 1)  // In first iteration
      {
         pathlength[a] = 1;
         tmpcell1->depth[a] = 0;

         //}
         //   if(cont_closed==1)
         //		{
         tmpcell1->penalty = 0;
      }

      tmpcell1->overexpanded = mazeiteration1;

      statexpanded1++;

      if (cont_closed > 1)  // Second to last step
      {
         printf(
             "\n\n****** FROM CELL [%d %d] at time %i, H: %.1f, degree: %i\n ",
             tmpcell1->y, tmpcell1->x, tmpcell1->depth[a],
             hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a],
             tmpcell1->degree[a]);
      }

      if (cont_closed == 1) {  // First state
         printf("\n\n******FROM CELL [%d %d] at time 0, H: %.1f, degree: %i\n ",
                tmpcell1->y, tmpcell1->x,
                hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a],
                tmpcell1->degree[a]);

         // First time the agent computes next cell, determines constraints:
         determine_constraints(a, lookahead, formula, tmpcell1, 1);
      }

      newdepth = tmpcell1->depth[a] + 1;
      ++agent_expansions[a];

      // printf("MaxSim: %.1f ", maxSimil);

      // Previous code
      /*  d = random() % DIRECTIONS;
      for (i = 0; i < DIRECTIONS; ++i)
      {

         if (tmpcell1->move[d] && (!tmpcell1->move[d]->obstacle)  &&
      tmpcell1->move[d]->overexpanded != mazeiteration1)
         {
            initialize_state(tmpcell1->move[d]);
      //        printf("A* generation a:%d [%d,%d] %d\n",a,
      tmpcell1->move[d]->y,tmpcell1->move[d]->x, d); if(tmpcell1->move[d]->g >
      tmpcell1->g + tmpcell1->cost[d])
            {
               tmpcell1->move[d]->g = tmpcell1->g + tmpcell1->cost[d];
               tmpcell1->move[d]->searchtree = tmpcell1;
               tmpcell1->move[d]->key = (tmpcell1->move[d]->g +
      hvalues[MAZEWIDTH*tmpcell1->move[d]->y + tmpcell1->move[d]->x][a]) * BASE
      - tmpcell1->move[d]->g; insertheap2(tmpcell1->move[d]);

            }
         }
         d = (d+1) % DIRECTIONS;
      } /*end for */

      // End Previous code

      d = rand() % DIRECTIONS;
      for (i = 0; i < DIRECTIONS; ++i) {
         if (tmpcell1->move[d]) {
            printf(
                "\n********************************************************");

            if (!((tmpcell1->x == tmpcell1->move[d]->x) &&
                  (tmpcell1->y == tmpcell1->move[d]->y))) {
               tmpcell1->degree[a] = tmpcell1->degree[a] + 1;
            }
            printf(
                "\nThinking about moving to [%d %d]..(degree of parent %i) \n",
                tmpcell1->move[d]->y, tmpcell1->move[d]->x,
                tmpcell1->degree[a]);

         } else {
            // printf("\nDirection..%i\n", i);
            // printf("It REALLY is blocked by obstacle\n" );
         }

         if (tmpcell1->move[d] &&
             (!tmpcell1->move[d]
                   ->obstacle))  //&& tmpcell1->move[d]->overexpanded !=
                                 // mazeiteration1)
         {
            initialize_state(tmpcell1->move[d]);
            printf("\nA* generation a:%d [%d,%d] from [%d %d], %.1f >= %.1f   ",
                   a, tmpcell1->move[d]->y, tmpcell1->move[d]->x, tmpcell1->y,
                   tmpcell1->x, tmpcell1->move[d]->g,
                   tmpcell1->g + tmpcell1->cost[d]);

            float goaldirX = ((float)goal[a]->x - (tmpcell1->x));
            float goaldirY = ((float)goal[a]->y - (tmpcell1->y));

            float maggoaldir = sqrtf(((float)goal[a]->x - (tmpcell1->x)) *
                                         ((float)goal[a]->x - (tmpcell1->x)) +
                                     ((float)goal[a]->y - (tmpcell1->y)) *
                                         ((float)goal[a]->y - (tmpcell1->y)));

            float magdir =
                sqrtf(((float)(tmpcell1->move[d])->x - (tmpcell1->x)) *
                          ((float)(tmpcell1->move[d])->x - (tmpcell1->x)) +
                      ((float)(tmpcell1->move[d])->y - (tmpcell1->y)) *
                          ((float)(tmpcell1->move[d])->y - (tmpcell1->y)));

            if ((tmpcell1->move[d]->g >=
                 tmpcell1->g +
                     tmpcell1->cost[d]))  //||((d==4)&&(tmpcell1->move[d]->g ==
                                          // tmpcell1->g + tmpcell1->cost[d])))
                                          ////way to check if state has been
                                          // visited before
            {
               tmpcell1->move[d]->tmpdepth[a] = newdepth;
               pathlength[a] = tmpcell1->move[d]->tmpdepth[a];
               // printf("\nThinking about moving to [%d %d]..\n",
               // tmpcell1->move[d]->y,tmpcell1->move[d]->x);
               printf("***INCREASING PATHLENGTH to %i !!\n", pathlength[a]);
               printf(
                   "\nThinking about moving to [%d %d] in my pathlength %i..\n",
                   tmpcell1->move[d]->y, tmpcell1->move[d]->x, pathlength[a]);
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

               if (0)  //(maxInfo!=a)//(maxHagent!=a)
               {
                  printf(
                      " I DONT have the max H, I defer to the other agent, "
                      "cutoff at %i \n",
                      pathlength[a] - 1);
                  learningCutoff[a] = pathlength[a] - 1;

                  // getchar();
               } else {
                  printf("[%d %d] G es %.1f +", tmpcell1->move[d]->y,
                         tmpcell1->move[d]->x, tmpcell1->move[d]->g);

                  printf(" H  es %f, ",
                         hvalues[MAZEWIDTH * tmpcell1->move[d]->y +
                                 tmpcell1->move[d]->x][a]);

                  printf(" F es %f\n",
                         tmpcell1->move[d]->g +
                             hvalues[MAZEWIDTH * tmpcell1->move[d]->y +
                                     tmpcell1->move[d]->x][a]);

                  //   getchar();

                  for (int j = 0; j < NAGENTS; ++j) {
                     if (canSee[a][j] > 0) {
                        /*   printf("CAN SEEE %i \n", j+1);
               printf("The heuristic of my neighbor %i is %f " ,
               j+1,hvalues[MAZEWIDTH*position[j]->y + position[j]->x][j]);
               printf(" and mine at [%d %d] is ",tmpcell1->y,tmpcell1->x);
               printf(" %f \n" ,hvalues[MAZEWIDTH*tmpcell1->y + tmpcell1->x][a]
               );
              */
                        if (hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a] <
                            hvalues[MAZEWIDTH * position[j]->y + position[j]->x]
                                   [j]) {
                        }
                     }
                  }
                  tmpcell1->move[d]
                      ->tmpsearchtree[tmpcell1->move[d]->tmpdepth[a]] =
                      tmpcell1;

                  // tmpcell1->move[d]->tmpsearchtree = tmpcell1;
                  printf("Mi parent is [%d %d] at depth %i..", tmpcell1->y,
                         tmpcell1->x, tmpcell1->move[d]->tmpdepth[a]);
                  // getchar();
                  tmpcell1->move[d]->pathlength = tmpcell1->pathlength + 1;

                  //   pathlength[a]= abs(tmpcell1->move[d]->x-mazestart1->x)+
                  //   abs(tmpcell1->move[d]->y-mazestart1->y);

                  //     printf("PARENT OF [%d %d] is [%d %d]
                  //     ...\n",tmpcell1->move[d]->y,tmpcell1->move[d]->x,
                  //     tmpcell1->y, tmpcell1->x);

                  //      Computing     key
                  // tmpcell1->move[d]->key = (tmpcell1->move[d]->g +
                  // hvalues[MAZEWIDTH*tmpcell1->move[d]->y +
                  // tmpcell1->move[d]->x][a]) * BASE - tmpcell1->move[d]->g;

                  float tempG =
                      (tmpcell1->g +
                       tmpcell1->cost
                           [d]);  //*(tmpcell1->move[d]->penalty)/(float)tmpcell1->pathlength;

                  tmpcell1->move[d]->key =
                      (tempG + hvalues[MAZEWIDTH * tmpcell1->move[d]->y +
                                       tmpcell1->move[d]->x][a]) *
                          BASE -
                      (tempG);
                  printf(" Adding [%d %d] with f %f to the heap ...\n",
                         tmpcell1->move[d]->y, tmpcell1->move[d]->x,
                         tempG + hvalues[MAZEWIDTH * tmpcell1->move[d]->y +
                                         tmpcell1->move[d]->x][a]);
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
         printf("Pasada numero %i ", pasada);
         pathlength[a] =
             abs(cellpas->x - mazestart1->x) + abs(cellpas->y - mazestart1->y);

         pathlength[a] = cellpas->depth[a];
         printf("\n\nBACKTRACKING PATH of lenght %i for agent %d",
                pathlength[a], a + 1);
         realDepth[a] = pathlength[a];

         if (pathlength[a] < lookahead) {
            for (int i = pathlength[a] + 1; i <= lookahead; i++) {
               path[a][i] = NULL;
               if (pasada == 1) {
                  idealPath[a][i] = path[a][i];
               }
               printf("\nMarking path[%i][%i] to NULL", a, i);
            }
         }

         lastStepDepth = pathlength[a];

         //  path[a][pathlength[a]+1]=NULL;
         printf("\nTO Cell [%d %d] %i", cellpas->y, cellpas->x, co);

         cellpas->trace =
             NULL;  // tracing back a path from the goal back to the start

         if ((pathlength[a] > 0) && (co == 0)) {
            path[a][pathlength[a]] = cellpas;
            // printf("At [%d %d] at time %i \n", path[a][pathlength[a]]->y,
            // path[a][pathlength[a]]->x, pathlength[a]);
            if (pasada == 1) {
               idealPath[a][pathlength[a]] = path[a][pathlength[a]];
               printf("IDEAL PATH [%d %d] at time %i, degree %i \n",
                      idealPath[a][pathlength[a]]->y,
                      idealPath[a][pathlength[a]]->x, pathlength[a],
                      maze1[idealPath[a][pathlength[a]]->y]
                           [idealPath[a][pathlength[a]]->x]
                               .degree[a]);
            }
         }
         // path[a][pathlength[a]]=cellpas;

         if (cellpas == mazestart1) {
            printf("Did I finish backtracking? %i ", pathlength[a]);
            if (pathlength[a] == 1) {
               for (int b = 2; b <= lookahead; b++) {
                  path[a][b] = path[a][1];  // NULL; //should enter here when
                                            // agent wants to stay here
                  printf("MY %i nd step is same as before\n", b);  // NULL!!\n",
                                                                // b);
               }
            }
            cellpas->trace = cellpas;
         }

         while ((cellpas != mazestart1) ||
                ((cellpas == mazestart1) && (pathlength[a] >= 1))) {
            // getchar();
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

            // getchar();

            parent->trace = cellpas;
            cellpas = parent;

            if ((pathlength[a] > 0) && (co == 0)) {
               path[a][pathlength[a]] = cellpas;

               // printf("Att [%d %d] at time %i \n", path[a][pathlength[a]]->y,
               // path[a][pathlength[a]]->x, pathlength[a]);
               if (pasada == 1) {
                  idealPath[a][pathlength[a]] = path[a][pathlength[a]];
                  printf("IDEAL PATH [%d %d] at time %i, degree %i \n",
                         idealPath[a][pathlength[a]]->y,
                         idealPath[a][pathlength[a]]->x, pathlength[a],
                         maze1[idealPath[a][pathlength[a]]->y]
                              [idealPath[a][pathlength[a]]->x]
                                  .degree[a]);
               }
            }
         }
         printf("Nope..");

         printf(" Got to the start:  [%d %d]. \n First move [%d %d]\n",
                cellpas->y, cellpas->x, mazestart1->trace->y,
                mazestart1->trace->x);
         // path[a][1]=mazestart1->trace;
         // getchar();
         if (path[a][2] != NULL) {
            printf("Second move [%d %d] ..and  %i\n", path[a][2]->y,
                   path[a][2]->x, pathlength[a]);
         }
         printf("Checking cell [%d %d]...and [%d %d]", cellpas->y, cellpas->x,
                mazestart1->trace->y, mazestart1->trace->x);

         if (mazestart1->trace != NULL) {
            if ((mazestart1->trace->blocked[pathlength[a]] != 1) ||
                ((mazestart1->trace->y == cellpas->y) &&
                 (mazestart1->trace->x == cellpas->x) &&
                 (mazestart1->trace->blocked[pathlength[a]] ==
                  1)))  //&&(pathlength[a]>0))||((mazestart1->trace->blocked[pathlength[a]]
                        //== 1)&&(pathlength[a]==0)))
            {
               printf(" GOT IT %i \n", flag_success1);
               break;
            }
         } else {
            printf(" GOT IT %i???? [%d %d]\n", flag_success1, cellpas->y,
                   cellpas->x);

            if ((cellpas->blocked[pathlength[a]] != 1) ||
                ((cellpas->blocked[pathlength[a]] == 1) &&
                 (cellpas ==
                  mazestart1)))  //&&(pathlength[a]>0))||((mazestart1->trace->blocked[pathlength[a]]
                                 //== 1)&&(pathlength[a]==0)))
            {
               printf(" GOT IT %i \n", flag_success1);
               printf("PATHLENGHT; %i \n", pathlength[a]);
               break;
            }
         }

         /*
          * if (flag_success1 == 1)
   {

      do{
      //	printf("construyendo path :%d agente %d\n",++co,a+1);
         cellpas->trace = NULL;   // tracing back a path from the goal back to
   the start while(cellpas != mazestart1)
         {
            parent = cellpas->searchtree;
            parent->trace = cellpas;
            cellpas = parent;
         }
         if (mazestart1->trace->blocked != 1) break;
         cellpas = popheap2();
      }while (topheap2() != NULL);
   } */

         if (sizeheap2() == 1) {
            printf(" QUEDA UNOOOOOOOOOOOOO size %i\n", sizeheap2());
            // getchar();
         }
         if (sizeheap2() == 0) {
            printf(" VACIOOOOO size %i\n", sizeheap2());
            // getchar();
            return (0);
         }
         printf(" EN [%d %d] and start is [%d %d]\n", cellpas->y, cellpas->x,
                mazestart1->y, mazestart1->x);
         cellpas = popheap2();
         printf(
             " PARECE QUE ESTOY BLOQUEADO?? NEXT [%d %d] with depth %i anda "
             "parent [%d %d]\n",
             cellpas->y, cellpas->x, cellpas->tmpdepth[a],
             cellpas->tmpsearchtree[cellpas->tmpdepth[a]]->y,
             cellpas->tmpsearchtree[cellpas->tmpdepth[a]]->x);
         // co=co+1;
         // tmpcell1->searchtree =
         // tmpcell1->tmpsearchtree[tmpcell1->depth[a]];//tmpcell3;

         cellpas->depth[a] = cellpas->tmpdepth[a];
         pathlength[a] = cellpas->depth[a];
         cellpas->searchtree = cellpas->tmpsearchtree[cellpas->depth[a]];
      } while (cellpas != NULL);  //(cellpas != NULL); //
   }                              // end if (flag_success1 == 1)

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
   printf(" COMPUTING FOR %i (%i)!!!\n", a + 1, pathlength[a]);

   int formula;

   printf(
       "\n\n**********************\n *********************\n Path lenght of "
       "this agent...%i now in [%d %d]\n",
       pathlength[a], position[a]->y, position[a]->x);

   if (position[a]->parent[a] != NULL) {
      printf("Whose parent is [%d %d]\n", position[a]->parent[a]->y,
             position[a]->parent[a]->x);
   }
   int limitcell = pathlength[a];
   for (int l = pathlength[a]; l >= 0; --l) {
      printf(
          "Checking the path at position %i, [%d %d], conflictCost: %.1f, "
          "degree %i, current limit: %i, mark: %i\n",
          l, idealPath[a][l]->y, idealPath[a][l]->x,
          conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l],
          maze1[idealPath[a][l]->y][idealPath[a][l]->x].degree[a], limitcell,
          maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l]);

      if (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] > 0.49) {
         printf("Cannot move here, will violate deference constraints\n");
         limitcell = l - 1;

         maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l] = 1;
      }

      if ((pathlength[a] > l) &&
          (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] <=
           0.49) &&
          (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] > 0.01) &&
          (limitcell == l) &&
          (maze1[idealPath[a][l]->y][idealPath[a][l]->x].degree[a] < 3)) {
         // Unless it is the initial state, the agent does not have to move here
         // as it will have to backtrack. If it is the initial state: backtrack!
         if (l != 0) {
            limitcell = l - 1;
            printf(
                "Cannot move here, will NOT violate deference constraints but "
                "the agent does not have enough mobility, limitcell is %i \n",
                limitcell);
            // need to check the degree of this node, If it is less than 3 we
            // are in trouble and need to go even further back

            maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l] = 1;
         } else {
            limitcell = l;
            printf("It is here, but will need to BACKTRACK for the step %i\n",
                   l + 1);
            // need to check the degree of this node, If it is less than 3 we
            // are in trouble and need to go even further back

            maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l] = 0;
            maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l + 1] = 1;
         }
      }

      if ((pathlength[a] > l) &&
          (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] <=
           0.49) &&
          (conflictCost[a][idealPath[a][l]->y][idealPath[a][l]->x][l] >
           0.201) &&
          (limitcell == l) &&
          (maze1[idealPath[a][l]->y][idealPath[a][l]->x].degree[a] >= 3)) {
         if (l != 0) {
            limitcell = l - 1;
         } else {
            limitcell = l;
         }
         printf(
             "Can move here (maybe it is here), but will need to move "
             "somewhere else for the step %i\n",
             l + 1);
         // need to check the degree of this node, If it is less than 3 we are
         // in trouble and need to go even further back

         maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l] = 0;
         maze1[idealPath[a][l]->y][idealPath[a][l]->x].marked[a][l + 1] = 1;
      }
   }

   if (limitcell < 0) {
      printf(" If it gets here it means the agent cannot find a solution\n");
      // in this case, it should act as a movable object. (IMPLEMENT!!!)

      printf("It may need to bactrack %i moves ", lastMobileCellDist[a]);

      if (lastMobileState[a] != NULL) {
         printf(" to [%d %d] \n", lastMobileState[a]->y, lastMobileState[a]->x);
      }

      return (0);
   }

   printf(
       " After exploring the ideal path, agent can move max until step %i "
       "which is cell [%d %d] with degree %i\n",
       limitcell, idealPath[a][limitcell]->y, idealPath[a][limitcell]->x,
       maze1[idealPath[a][limitcell]->y][idealPath[a][limitcell]->x].degree[a]);

   if (limitcell > 0) {
      printf(
          " For backtracking purposes, I would need to backtrack to [%d %d]\n",
          idealPath[a][0]->y, idealPath[a][0]->x);
   }

   for (int l = 0; l < lookahead; ++l) {
      path[a][l] = NULL;
   }
   // getchar();

   // mazestart1 = position[a]; //Current position

   printf(
       "\nNOTHING HERE...first position in plan: [%d %d] vs [%d %d], second "
       "[%d %d] ",
       mazestart1->y, mazestart1->x, position[a]->y, position[a]->x,
       mazestart1->trace->y, mazestart1->trace->x);

   // New lookahead: lookahead -limitcell, represents how much should the agent
   // compute If new_lookahead = 0, means that the agent is free to move in its
   // own idealPath.
   // int new_lookahead=lookahead-limitcell;
   int new_lookahead = pathlength[a] - limitcell;

   printf("\n New lookahead %i, real Depth %i ", new_lookahead, realDepth[a]);

   printf("\n Copying unmodified part of the path:\n");
   for (int l = 0; l <= limitcell; ++l) {
      path[a][l] = idealPath[a][l];
      printf("Path at step %i is [%d %d] ", l, idealPath[a][l]->y,
             idealPath[a][l]->x);
      printf(" and its got trace: ");
      if (mazestart1->trace != NULL) {
         printf("YES!!!!\n");
      } else {
         printf(" NO :( \n");
      }
   }

   printf(
       "\n1 NOTHING HERE...first position in plan: [%d %d] vs [%d %d], second "
       "[%d %d] ",
       mazestart1->y, mazestart1->x, position[a]->y, position[a]->x,
       mazestart1->trace->y, mazestart1->trace->x);

   printf(
       "\n2 NOTHING HERE...first position in plan: [%d %d] vs [%d %d], second "
       "[%d %d] ",
       mazestart1->y, mazestart1->x, position[a]->y, position[a]->x,
       mazestart1->trace->y, mazestart1->trace->x);

   if (new_lookahead != 0)  // || idealPath[a][limitcell]==goal[a])
   {
      // mazestart1 = position[a]; //Current position
      mazestart1 = idealPath[a][limitcell];
      mazegoal1 = goal[a];  // New position
      //	printf("a:%d [%d,%d]\n",a, position[a]->y,position[a]->x);
      printf("HOla\n");

      printf("\n3 NOTHING HERE...first position in plan: [%d %d] vs [%d %d] ",
             mazestart1->y, mazestart1->x, position[a]->y, position[a]->x);

      mazeiteration1++;

      // printf("\n4 NOTHING HERE...first position in plan: [%d %d] vs [%d %d],
      // second [%d %d] ",mazestart1->y, mazestart1->x, position[a]->y,
      // position[a]->x,mazestart1->trace->y,mazestart1->trace->x );

      emptyheap2();

      // printf("\n5 NOTHING HERE...first position in plan: [%d %d] vs [%d %d],
      // second [%d %d] ",mazestart1->y, mazestart1->x, position[a]->y,
      // position[a]->x,mazestart1->trace->y,mazestart1->trace->x );

      cont_closed = 0;

#ifdef STATISTICS
      searches_astar1++;
      statexpanded1_initial = statexpanded1;
#endif

      initialize_state(mazestart1);
      mazestart1->g = 0;  // Setting start node cost to zero
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
         printf("\n****GETTING NEW NODE FROM THE STACK!! ");
         printf("\nThe previously analyzed node has degree %i ",
                tmpcell1->degree[a]);
      }

      tmpcell3 = tmpcell1;
      tmpcell1 = topheap2();

      if (cont_closed > 0) {
         printf(
             "\nThe next node to expand is [%d %d] with depth %i (or %i) from "
             "parent [%d %d] (cont %i)",
             tmpcell1->y, tmpcell1->x, tmpcell1->tmpdepth[a], newdepth,
             tmpcell3->y, tmpcell3->x, cont_closed);

         if (cont_closed == lookahead) {
            lastStepDepth = tmpcell1->tmpdepth[a];
            // last step of path is at depth tmpcell1->tmpdepth[a]
         }

         if (newdepth != tmpcell1->depth[a]) {
            printf("Strange..");
            // getchar();
         }
         tmpcell1->depth[a] = tmpcell1->tmpdepth[a];
         tmpcell1->searchtree =
             tmpcell1->tmpsearchtree[tmpcell1->depth[a]];  // tmpcell3;

         // printf("The parent of [%d %d] is [%d %d] at depth %i..",
         // tmpcell1->y, tmpcell1->x,tmpcell1->searchtree->y,
         // tmpcell1->searchtree->x, tmpcell1->move[d]->depth[a]);
         // getchar();
         // tmpcell1->searchtree = tmpcell1->tmpsearchtree;
      }

      printf("A* top a:%d [%d,%d] It:%lld LA:%d\n", 
               a, tmpcell1->y, tmpcell1->x,
             mazeiteration1, lookahead);

      // This next block of code (if statement) should not be executed in this
      // path planning, as we do not want to overwrite previously found
      // heuristic values

      if ((tmpcell1 == mazegoal1) ||
          (cont_closed == lookahead)) {  // open_size = opensize2() + open_size;
         // Se calcul f para actualizar h

         f_value =
             hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a] + tmpcell1->g;
         printf(" \n\nH value of [%d %d]: %f,", tmpcell1->y, tmpcell1->x,
                hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a]);
         printf(" G : %f\n", tmpcell1->g);

         printf("\nLEARNING NEW HEURISTICS OF FOUND PATH.. %f\n", tmpcell1->g);
         cellpas = tmpcell1;
         for (d = 0; d < cont_closed; d++) {
            // hvalues[MAZEWIDTH*CLOSED[d]->y + CLOSED[d]->x][a] =
            // max(hvalues[MAZEWIDTH*CLOSED[d]->y + CLOSED[d]->x][a],f_value -
            // CLOSED[d]->g);

            // max(CLOSED[d]->h,f_value - CLOSED[d]->g);
            printf("Updating H of [%d %d] = %.1f, d: %i, size %i \n",
                   CLOSED[d]->y, CLOSED[d]->x,
                   hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a], d,
                   sizeheap2());
         }

         printf(" Final destination : [%d %d]", cellpas->y, cellpas->x);
         // getchar();

         flag_success1 = 1;
         tmpcell1 = popheap2();
         printf(
             "\n(BEST MOVE WIHTOUT ACCOUNTING FOR AGENTS) A* top a:%d [%d,%d] "
             "\n",
             a, tmpcell1->y, tmpcell1->x);
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
         printf(
             "\n\n******From Cell [%d %d] at time %i, H: %.1f, degree: %i\n ",
             tmpcell1->y, tmpcell1->x, tmpcell1->depth[a],
             hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a],
             tmpcell1->degree[a]);
      }

      if (cont_closed == 1) {
         printf("\n\n******From Cell [%d %d] at time 0, H: %.1f, degree: %i\n ",
                tmpcell1->y, tmpcell1->x,
                hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a],
                tmpcell1->degree[a]);
      }
      newdepth = tmpcell1->depth[a] + 1;
      ++agent_expansions[a];

      // printf("MaxSim: %.1f ", maxSimil);

      // Previous code
      /*  d = random() % DIRECTIONS;
      for (i = 0; i < DIRECTIONS; ++i)
      {

         if (tmpcell1->move[d] && (!tmpcell1->move[d]->obstacle)  &&
      tmpcell1->move[d]->overexpanded != mazeiteration1)
         {
            initialize_state(tmpcell1->move[d]);
      //        printf("A* generation a:%d [%d,%d] %d\n",a,
      tmpcell1->move[d]->y,tmpcell1->move[d]->x, d); if(tmpcell1->move[d]->g >
      tmpcell1->g + tmpcell1->cost[d])
            {
               tmpcell1->move[d]->g = tmpcell1->g + tmpcell1->cost[d];
               tmpcell1->move[d]->searchtree = tmpcell1;
               tmpcell1->move[d]->key = (tmpcell1->move[d]->g +
      hvalues[MAZEWIDTH*tmpcell1->move[d]->y + tmpcell1->move[d]->x][a]) * BASE
      - tmpcell1->move[d]->g; insertheap2(tmpcell1->move[d]);

            }
         }
         d = (d+1) % DIRECTIONS;
      } /*end for */

      // End Previous code

      d = rand() % DIRECTIONS;
      for (i = 0; i < DIRECTIONS; ++i) {
         if (tmpcell1->move[d]) {
            printf(
                "\n********************************************************");
            printf(
                "\nThinking about moving to [%d %d]..whose ConflictCost is "
                "%.1f\n",
                tmpcell1->move[d]->y, tmpcell1->move[d]->x,
                conflictCost[a][tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                            [pathlength[a]]);

            printf(
                "****\nPROSPECTIVE NODE [%d %d] is at time/depth %i..is it "
                "MARKED? %i\n ******",
                tmpcell1->move[d]->y, tmpcell1->move[d]->x, newdepth,
                maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                    .marked[a][newdepth]);
            // tmpcell1->move[d]->depth[a],
            // maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x].marked[a][tmpcell1->move[d]->depth[a]])
            // ;

            // tmpcell1->degree[a]=tmpcell1->degree[a]+1;

            if (tmpcell1->degree[a] >= 3) {
               printf(
                   "\nThis node has degree greater than 3!!! it can help me "
                   "swap with other agents\n");
               // Need to store this cell somewhere as a failsafe
            }

         } else {
            // printf("\nDirection..%i\n", i);
            // printf("It REALLY is blocked by obstacle\n" );
         }

         if (tmpcell1->move[d] && (!tmpcell1->move[d]->obstacle) &&
             (!maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                   .marked[a][newdepth]))  // tmpcell1->move[d]->depth[a]&&
                                           // tmpcell1->move[d]->overexpanded !=
                                           // mazeiteration1)
         {
            initialize_state(tmpcell1->move[d]);
            printf("\nA* generation a:%d [%d,%d] from [%d %d], %.1f >= %.1f   ",
                   a, tmpcell1->move[d]->y, tmpcell1->move[d]->x, tmpcell1->y,
                   tmpcell1->x, tmpcell1->move[d]->g,
                   tmpcell1->g + tmpcell1->cost[d]);

            float goaldirX = ((float)goal[a]->x - (tmpcell1->x));
            float goaldirY = ((float)goal[a]->y - (tmpcell1->y));

            float maggoaldir = sqrtf(((float)goal[a]->x - (tmpcell1->x)) *
                                         ((float)goal[a]->x - (tmpcell1->x)) +
                                     ((float)goal[a]->y - (tmpcell1->y)) *
                                         ((float)goal[a]->y - (tmpcell1->y)));

            float magdir =
                sqrtf(((float)(tmpcell1->move[d])->x - (tmpcell1->x)) *
                          ((float)(tmpcell1->move[d])->x - (tmpcell1->x)) +
                      ((float)(tmpcell1->move[d])->y - (tmpcell1->y)) *
                          ((float)(tmpcell1->move[d])->y - (tmpcell1->y)));

            if ((tmpcell1->move[d]->g >=
                 tmpcell1->g +
                     tmpcell1->cost[d]))  //||((d==4)&&(tmpcell1->move[d]->g ==
                                          // tmpcell1->g + tmpcell1->cost[d])))
                                          ////way to check if state has been
                                          // visited before
            {
               tmpcell1->move[d]->tmpdepth[a] = newdepth;
               pathlength[a] = tmpcell1->move[d]->tmpdepth[a];
               // printf("\nThinking about moving to [%d %d]..\n",
               // tmpcell1->move[d]->y,tmpcell1->move[d]->x);
               printf("***INCREASING PATHLENGTH to %i !!\n", pathlength[a]);
               printf(
                   "\nThinking about moving to [%d %d] in my pathlength %i..\n",
                   tmpcell1->move[d]->y, tmpcell1->move[d]->x, pathlength[a]);
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
                     printf("Staying here will bring me trouble at t %i!!\n",
                            future);
                     // conflictCost[a][currentCell->y][currentCell->x][step]=(float)1/(float)(future-step+2);
                     printf(
                         "QUICK SCORE: %f",
                         conflictCost
                             [a][tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                             [pathlength
                                  [a]]);  //(float)1/(float)(future-pathlength[a]+2));
                  }
               }

               int maxHagent = a;
               int maxInfo = a;
               if ((maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                        .blockedAgent[a][pathlength[a]]) &&
                   (pathlength[a] > 0))  // Another agent wants to move here
               {
                  printf(
                      "\n\n****FUTURE At %i another agent WOULD LIKE TO MOVE "
                      "to [%d %d], but who??\n",
                      pathlength[a], tmpcell1->move[d]->y,
                      tmpcell1->move[d]->x);  // cont_closed

                  int numConflicts = 0;
                  float maxH = backupH
                      [MAZEWIDTH * (position[a]->y) + (position[a]->x)]
                      [a];  // backupH[MAZEWIDTH*(tmpcell1->move[d]->y) +
                            // (tmpcell1->move[d]->x)][a];//hvalues[MAZEWIDTH*(position[a]->y)
                            // +
                            // (position[a]->x)][a];//[MAZEWIDTH*(tmpcell1->y)
                            // + (tmpcell1->x)][a];
                  float sumH = 0;

                  for (int j = 0; j < NAGENTS; ++j) {
                     if (maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                             .agentMovingTo[a][pathlength[a]][j] > 0) {
                        printf(
                            "This guy -> %i  (total %i) inGoal?: %i or "
                            "(%f-%i)=%f \n",
                            j + 1, numConflicts, goal_reached[j],
                            hvalues[MAZEWIDTH * position[j]->y + position[j]->x]
                                   [j],
                            pathlength[a],
                            (hvalues[MAZEWIDTH * position[j]->y +
                                     position[j]->x][j] -
                             pathlength[a]));

                        if ((goal_reached[j]) ||
                            ((hvalues[MAZEWIDTH * position[j]->y +
                                      position[j]->x][j] -
                              pathlength[a]) < 0.1)) {
                           // agent is on goal, never mind
                           continue;
                        }
                        numConflicts++;
                        printf("This guy -> %i  (total %i) inGoal?: %i \n",
                               j + 1, numConflicts, goal_reached[j]);

                        printf(
                            "His previous position at [%d %d] had a degree of "
                            "%i \n",
                            path[j][pathlength[a]]->y,
                            path[j][pathlength[a]]->x,
                            maze1[path[j][pathlength[a]]->y]
                                 [path[j][pathlength[a]]->x]
                                     .degree[j]);

                        printf("My info %i vs other agent's info %i \n",
                               formula, agentInfo[j]);

                        if (conflictType[a][j] == 0) {
                           printf(" POINT conflict, my cost: %i vs his :%i \n",
                                  (int)(hvalues[MAZEWIDTH * position[a]->y +
                                                position[a]->x][a]) +
                                      2,
                                  (int)(hvalues[MAZEWIDTH * position[j]->y +
                                                position[j]->x][j]) +
                                      1);

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
                           printf(" PATH conflict\n");
                           if (formula < agentInfo[j]) {
                              maxInfo = j;
                           }
                        }
                        printf("And his path is \n");

                        for (int l = 1; l <= lookahead; l++) {
                           if ((a != j) && (path[j][l] != NULL)) {
                              printf("Agent %i: [%d %d], H: %.1f, step %i \n",
                                     j + 1, path[j][l]->y, path[j][l]->x,
                                     hvalues[MAZEWIDTH * position[j]->y +
                                             position[j]->x][j],
                                     l);

                              if ((position[a]->y == path[j][l]->y) &&
                                  (position[a]->x == path[j][l]->x)) {
                                 // conflictsPosition++;
                                 position[a]->numConflicts[a] =
                                     numConflicts;  // position[a]->numConflicts[a]+1;
                                 printf(
                                     "COops! might need to move, total "
                                     "conflicts: %i \n",
                                     position[a]->numConflicts[a]);
                              }

                              if (hvalues[MAZEWIDTH * position[j]->y +
                                          position[j]->x][j] >
                                  hvalues[MAZEWIDTH * position[a]->y +
                                          position[a]->x][a]) {
                              }
                           }
                        }

                        // getchar();

                        printf(" with H of %.1f, (%.1f)  vs ",
                               hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                       (tmpcell1->move[d]->x)][j],
                               hvalues[MAZEWIDTH * (position[j]->y) +
                                       (position[j]->x)][j]);

                        printf(" my H of %.1f, (%.1f) ",
                               hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                       (tmpcell1->move[d]->x)][a],
                               hvalues[MAZEWIDTH * (position[a]->y) +
                                       (position[a]->x)]
                                      [a]);  //[MAZEWIDTH*(tmpcell1->y) +
                                             //(tmpcell1->x)][a]);
                        sumH =
                            sumH + backupH[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                           (tmpcell1->move[d]->x)][j];
                        printf(", SumH is %f \n", sumH);

                        if ((hvalues[MAZEWIDTH * (position[j]->y) +
                                     (position[j]->x)][j] > maxH)) {
                           maxHagent = j;
                           maxH = hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                          (tmpcell1->move[d]->x)][j];
                        }
                     }
                  }
                  printf(
                      "The Agent with MAX H, whose H changes by my movement  "
                      "is %i ",
                      maxHagent + 1);
                  printf(" with H of %f \n", maxH);
               }

               // printf(" Bloqueado? %i pathlength %i goal reached %i (agent
               // %i) ",
               // maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x].blockedAgent[a][0],pathlength[a],goal_reached[i],i+1);
               int canmovehere = 1;
               for (int u = 0; u < NAGENTS; ++u) {
                  if ((tmpcell1->move[d]->x == position[u]->x) &&
                      (tmpcell1->move[d]->y == position[u]->y) &&
                      (pathlength[a] == 1) && (u != a)) {
                     printf(
                         "\nOps, agent %i is at the next position position, is "
                         "it its goal?..",
                         u + 1);

                     if ((goal[u]->y == position[u]->y) &&
                         (goal[u]->x == position[u]->x)) {
                        printf("\nYES, I can move through");

                     } else {
                        printf("\n No, Cant move here\n");
                        canmovehere = 0;
                     }
                  }
               }

               if ((((maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                          .blockedAgent[a][0])) &&
                    (pathlength[a] == 1)) &&
                   (!canmovehere))  // Another agent IS here (first step)
               {
                  printf(
                      "\n****At %i Cant move to [%d %d], there is an agent\n",
                      pathlength[a], tmpcell1->move[d]->y,
                      tmpcell1->move[d]->x);

                  learningCutoff[a] = pathlength[a] - 1;

                  for (int j = 0; j < NAGENTS; ++j) {
                     if ((tmpcell1->move[d]->y == position[j]->y) &&
                         (tmpcell1->move[d]->x == position[j]->x)) {
                        (tmpcell1->move[d])->numConflicts[a] =
                            (tmpcell1->move[d])->numConflicts[a] + 1;
                        printf("This guy -> %i, total conflicts %i \n", j + 1,
                               (tmpcell1->move[d])->numConflicts[a]);
                        maxInfo = j;
                     }
                  }

                  //	getchar();
               }

               maxHagent = a;
               if (((maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                         .blockedAgent[a][pathlength[a] - 1])) &&
                   (pathlength[a] >
                    1))  // Another agent might ALREADY be here (> first step)
               {
                  printf(
                      "****At %i MIGHT NOT BE ABLE to move to [%d %d], there "
                      "MIGHT ALREADY BE an agent\n",
                      pathlength[a] - 1, tmpcell1->move[d]->y,
                      tmpcell1->move[d]->x);

                  int numConflicts = 0;
                  float maxH = backupH
                      [MAZEWIDTH * (position[a]->y) + (position[a]->x)]
                      [a];  // backupH[MAZEWIDTH*(tmpcell1->move[d]->y) +
                            // (tmpcell1->move[d]->x)][a];//hvalues[MAZEWIDTH*(position[a]->y)
                            // +
                            // (position[a]->x)][a];//[MAZEWIDTH*(tmpcell1->y)
                            // + (tmpcell1->x)][a];
                  float sumH = 0;

                  for (int j = 0; j < NAGENTS; ++j) {
                     if (maze1[tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                             .agentMovingTo[a][pathlength[a] - 1][j] > 0) {
                        numConflicts++;
                        printf("This guy -> %i  (total %i)", j + 1,
                               numConflicts);

                        // getchar();

                        printf(" with H of %.1f, (%.1f)  vs ",
                               hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                       (tmpcell1->move[d]->x)][j],
                               hvalues[MAZEWIDTH * (position[j]->y) +
                                       (position[j]->x)][j]);

                        printf(" my H of %.1f, (%.1f) ",
                               hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                       (tmpcell1->move[d]->x)][a],
                               hvalues[MAZEWIDTH * (position[a]->y) +
                                       (position[a]->x)]
                                      [a]);  //[MAZEWIDTH*(tmpcell1->y) +
                                             //(tmpcell1->x)][a]);
                        sumH =
                            sumH + backupH[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                           (tmpcell1->move[d]->x)][j];
                        printf(", SumH is %f \n", sumH);

                        // Copied from above
                        printf("My info %i vs other agent's info %i \n",
                               formula, agentInfo[j]);

                        if (conflictType[a][j] == 0) {
                           printf(
                               " POINT conflict, my cost :  %i vs his :%i \n",
                               (int)(hvalues[MAZEWIDTH * position[a]->y +
                                             position[a]->x][a]) +
                                   2,
                               (int)(hvalues[MAZEWIDTH * position[j]->y +
                                             position[j]->x][j]) +
                                   1);

                           if ((int)(hvalues[MAZEWIDTH * position[j]->y +
                                             position[j]->x][j]) +
                                   1 >
                               (int)(hvalues[MAZEWIDTH * position[a]->y +
                                             position[a]->x][a]) +
                                   2) {
                              maxInfo = j;
                              printf(" MaxInfo: %i\n", maxInfo);
                           }
                        }

                        if (conflictType[a][j] == 1) {
                           printf(" PATH conflict\n");
                           if (formula < agentInfo[j]) {
                              maxInfo = j;
                              printf(" MaxInfo: %i\n", maxInfo + 1);
                           }
                        }

                        // End of copy from above

                        if ((hvalues[MAZEWIDTH * (position[j]->y) +
                                     (position[j]->x)][j] > maxH)) {
                           maxHagent = j;
                           maxH = hvalues[MAZEWIDTH * (tmpcell1->move[d]->y) +
                                          (tmpcell1->move[d]->x)][j];
                        }
                     }
                  }
                  printf(
                      "The Agent with MAX H, whose H changes by my movement  "
                      "is %i ",
                      maxHagent + 1);
                  printf(" with H of %f \n", maxH);
               }

               // After these three checks, we can see if agent can consider
               // this move or not

               if (0)  //(maxInfo!=a)//(maxHagent!=a)
               {
                  printf(
                      " I DONT have the max H, I defer to the other agent, "
                      "cutoff at %i \n",
                      pathlength[a] - 1);
                  learningCutoff[a] = pathlength[a] - 1;

                  // getchar();
               } else {
                  printf("[%d %d] G es %.1f +", tmpcell1->move[d]->y,
                         tmpcell1->move[d]->x, tmpcell1->move[d]->g);

                  printf(" H  es %f, ",
                         hvalues[MAZEWIDTH * tmpcell1->move[d]->y +
                                 tmpcell1->move[d]->x][a]);

                  printf(" F es %f\n",
                         tmpcell1->move[d]->g +
                             hvalues[MAZEWIDTH * tmpcell1->move[d]->y +
                                     tmpcell1->move[d]->x][a]);

                  //   getchar();

                  for (int j = 0; j < NAGENTS; ++j) {
                     if (canSee[a][j] > 0) {
                        /*   printf("CAN SEEE %i \n", j+1);
               printf("The heuristic of my neighbor %i is %f " ,
               j+1,hvalues[MAZEWIDTH*position[j]->y + position[j]->x][j]);
               printf(" and mine at [%d %d] is ",tmpcell1->y,tmpcell1->x);
               printf(" %f \n" ,hvalues[MAZEWIDTH*tmpcell1->y + tmpcell1->x][a]
               );
              */
                        if (hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a] <
                            hvalues[MAZEWIDTH * position[j]->y + position[j]->x]
                                   [j]) {
                        }
                     }
                  }
                  tmpcell1->move[d]
                      ->tmpsearchtree[tmpcell1->move[d]->tmpdepth[a]] =
                      tmpcell1;

                  // tmpcell1->move[d]->tmpsearchtree = tmpcell1;
                  printf("Mi parent is [%d %d] at depth %i..", tmpcell1->y,
                         tmpcell1->x, tmpcell1->move[d]->tmpdepth[a]);
                  // getchar();
                  tmpcell1->move[d]->pathlength = tmpcell1->pathlength + 1;

                  //   pathlength[a]= abs(tmpcell1->move[d]->x-mazestart1->x)+
                  //   abs(tmpcell1->move[d]->y-mazestart1->y);

                  //     printf("PARENT OF [%d %d] is [%d %d]
                  //     ...\n",tmpcell1->move[d]->y,tmpcell1->move[d]->x,
                  //     tmpcell1->y, tmpcell1->x);

                  //      Computing     key
                  // tmpcell1->move[d]->key = (tmpcell1->move[d]->g +
                  // hvalues[MAZEWIDTH*tmpcell1->move[d]->y +
                  // tmpcell1->move[d]->x][a]) * BASE - tmpcell1->move[d]->g;

                  float tempG =
                      (tmpcell1->g + tmpcell1->cost[d]) +
                      3 * (conflictCost
                               [a][tmpcell1->move[d]->y][tmpcell1->move[d]->x]
                               [pathlength
                                    [a]]);  //*(tmpcell1->move[d]->penalty)/(float)tmpcell1->pathlength;

                  // tmpcell1->move[d]->g = tmpcell1->g + tmpcell1->cost[d];
                  // tmpcell1->move[d]->searchtree = tmpcell1;
                  // tmpcell1->move[d]->key = (tmpcell1->move[d]->g +
                  // hvalues[MAZEWIDTH*tmpcell1->move[d]->y +
                  // tmpcell1->move[d]->x][a]) * BASE - tmpcell1->move[d]->g;

                  tmpcell1->move[d]->key =
                      (tempG + hvalues[MAZEWIDTH * tmpcell1->move[d]->y +
                                       tmpcell1->move[d]->x][a]) *
                          BASE -
                      (tempG);
                  printf(" Adding [%d %d] with f %f to the heap ...\n",
                         tmpcell1->move[d]->y, tmpcell1->move[d]->x,
                         tempG + hvalues[MAZEWIDTH * tmpcell1->move[d]->y +
                                         tmpcell1->move[d]->x][a]);
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
         printf("Pasada numero %i ", pasada);
         pathlength[a] =
             abs(cellpas->x - mazestart1->x) + abs(cellpas->y - mazestart1->y);

         pathlength[a] = cellpas->depth[a];
         printf("\n\nBACKTRACKING PATH of lenght %i for agent %d",
                pathlength[a], a + 1);
         realDepth[a] = pathlength[a];

         if (pathlength[a] < lookahead) {
            for (int i = pathlength[a] + 1; i <= lookahead; i++) {
               path[a][i] = NULL;
               if (pasada == 1) {
                  // idealPath[a][i]=path[a][i];
               }

               printf("\nMarking path[%i][%i] to NULL", a, i);
            }
         }

         lastStepDepth = pathlength[a];

         //  path[a][pathlength[a]+1]=NULL;
         printf("\nTO Cell [%d %d] %i", cellpas->y, cellpas->x, co);

         cellpas->trace =
             NULL;  // tracing back a path from the goal back to the start

         if ((pathlength[a] > 0) && (co == 0)) {
            path[a][pathlength[a]] = cellpas;
            printf("At [%d %d] at time %i \n", path[a][pathlength[a]]->y,
                   path[a][pathlength[a]]->x, pathlength[a]);
            if (pasada == 1) {
               // idealPath[a][pathlength[a]]=path[a][pathlength[a]];
            }
         }
         // path[a][pathlength[a]]=cellpas;

         if (cellpas == mazestart1) {
            printf("Did I finish backtracking? %i ", pathlength[a]);
            if (pathlength[a] == 1) {
               for (int b = 2; b <= lookahead; b++) {
                  path[a][b] = path[a][1];  // NULL; //should enter here when
                                            // agent wants to stay here
                  printf("MY %i nd step is same as before\n", b);  // NULL!!\n",
                                                                // b);
               }
            }
            cellpas->trace = cellpas;
         }

         while ((cellpas != mazestart1) ||
                ((cellpas == mazestart1) && (pathlength[a] >= 1))) {
            // getchar();
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

            // getchar();

            parent->trace = cellpas;
            cellpas = parent;

            if ((pathlength[a] > 0) && (co == 0)) {
               path[a][pathlength[a]] = cellpas;

               printf("Att [%d %d] at time %i \n", path[a][pathlength[a]]->y,
                      path[a][pathlength[a]]->x, pathlength[a]);
               if (pasada == 1) {
                  // idealPath[a][pathlength[a]]=path[a][pathlength[a]];
               }
            }
         }
         printf("Nope..");

         printf(" Got to the start:  [%d %d]. \n First move [%d %d]\n",
                cellpas->y, cellpas->x, mazestart1->trace->y,
                mazestart1->trace->x);
         // path[a][1]=mazestart1->trace;
         // getchar();
         if (path[a][2] != NULL) {
            printf("Second move [%d %d] ..and  %i\n", path[a][2]->y,
                   path[a][2]->x, pathlength[a]);
         }
         printf("Checking cell [%d %d]...and [%d %d]", cellpas->y, cellpas->x,
                mazestart1->trace->y, mazestart1->trace->x);

         if (mazestart1->trace != NULL) {
            if ((mazestart1->trace->blocked[pathlength[a]] != 1) ||
                ((mazestart1->trace->y == cellpas->y) &&
                 (mazestart1->trace->x == cellpas->x) &&
                 (mazestart1->trace->blocked[pathlength[a]] ==
                  1)))  //&&(pathlength[a]>0))||((mazestart1->trace->blocked[pathlength[a]]
                        //== 1)&&(pathlength[a]==0)))
            {
               printf(" GOT IT %i \n", flag_success1);
               break;
            }
         } else {
            printf(" GOT IT %i???? [%d %d]\n", flag_success1, cellpas->y,
                   cellpas->x);

            if ((cellpas->blocked[pathlength[a]] != 1) ||
                ((cellpas->blocked[pathlength[a]] == 1) &&
                 (cellpas ==
                  mazestart1)))  //&&(pathlength[a]>0))||((mazestart1->trace->blocked[pathlength[a]]
                                 //== 1)&&(pathlength[a]==0)))
            {
               printf(" GOT IT %i \n", flag_success1);
               printf("PATHLENGHT; %i \n", pathlength[a]);
               break;
            }
         }

         /*
          * if (flag_success1 == 1)
   {

      do{
      //	printf("construyendo path :%d agente %d\n",++co,a+1);
         cellpas->trace = NULL;   // tracing back a path from the goal back to
   the start while(cellpas != mazestart1)
         {
            parent = cellpas->searchtree;
            parent->trace = cellpas;
            cellpas = parent;
         }
         if (mazestart1->trace->blocked != 1) break;
         cellpas = popheap2();
      }while (topheap2() != NULL);
   } */

         if (sizeheap2() == 1) {
            printf(" QUEDA UNOOOOOOOOOOOOO size %i\n", sizeheap2());
            // getchar();
         }
         if (sizeheap2() == 0) {
            printf(" VACIOOOOO size %i\n", sizeheap2());
            // getchar();
            return (0);
         }
         printf(" EN [%d %d] and start is [%d %d]\n", cellpas->y, cellpas->x,
                mazestart1->y, mazestart1->x);
         cellpas = popheap2();
         printf(
             " PARECE QUE ESTOY BLOQUEADO?? NEXT [%d %d] with depth %i anda "
             "parent [%d %d]\n",
             cellpas->y, cellpas->x, cellpas->tmpdepth[a],
             cellpas->tmpsearchtree[cellpas->tmpdepth[a]]->y,
             cellpas->tmpsearchtree[cellpas->tmpdepth[a]]->x);
         // co=co+1;
         // tmpcell1->searchtree =
         // tmpcell1->tmpsearchtree[tmpcell1->depth[a]];//tmpcell3;

         cellpas->depth[a] = cellpas->tmpdepth[a];
         pathlength[a] = cellpas->depth[a];
         cellpas->searchtree = cellpas->tmpsearchtree[cellpas->depth[a]];
      } while (cellpas != NULL);  //(cellpas != NULL); //
   }                              // end if (flag_success1 == 1)

   return (flag_success1);
}

void observe_new_agents(
    int a, int i, int lookahead) {  // printf(" seen at [%d %d] ",
                                    // track[a][i][0][1],track[a][i][0][0]);

   if (track[a][i][0][0] == -1)  // if a hasn't seen agent i before
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
         printf(
             "Agent %i appeared AGAIN?? [%d %d] can see agent %i at [%d %d]\n",
             a + 1, position[a]->y, position[a]->x, i + 1, position[i]->y,
             position[i]->x);

         if (track[a][i][0][0] ==
             -1)  // There is no previous record of the agent
         {
            track[a][i][0][0] = position[i]->x;
            track[a][i][0][1] = position[i]->y;
         } else  // at least the agent was seen once before
         {
            track[a][i][1][0] = track[a][i][0][0];
            track[a][i][1][1] = track[a][i][0][1];

            track[a][i][0][0] = position[i]->x;
            track[a][i][0][1] = position[i]->y;
         }

         printf("Now Computing PRediction......\n");
         computePrediction2(a, i, lookahead);
         // updateHistory(a,i,lookahead,position);
         // getchar();
      }

   } else {
      if (((abs(position[i]->x - position[a]->x) +
            abs(position[i]->y - position[a]->y)) > (lookahead)) &&
          ((abs(position[i]->x - position[a]->x) +
            abs(position[i]->y - position[a]->y)) > 0)) {
         canSee[a][i] = 0;
         track[a][i][1][0] = track[a][i][0][0];
         track[a][i][1][1] = track[a][i][0][1];

         track[a][i][0][0] = -1;
         track[a][i][0][1] = -1;

         for (int z = 0; z < (lookahead); z++) {
            printf("Checking previous seen position of agent %i at [%d %d] \n",
                   i, mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0]);
            if (maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .blockedAgent[a][z] >
                0)  // (blockedAgent[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]>0)
            {
               maze1[mostProbPositionXY[a][i][z][1]]
                    [mostProbPositionXY[a][i][z][0]]
                        .blockedAgent[a][z] =
                   maze1[mostProbPositionXY[a][i][z][1]]
                        [mostProbPositionXY[a][i][z][0]]
                            .blockedAgent[a][z] -
                   1;

               printf(
                   "(%i) SAW the guy before at [%d %d], not anymore, making "
                   "blocked back to  %i\n",
                   z, mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z][1]]
                        [mostProbPositionXY[a][i][z][0]]
                            .blockedAgent[a][z]);

               if (z > 0) {
                  maze1[mostProbPositionXY[a][i][z - 1][1]]
                       [mostProbPositionXY[a][i][z - 1][0]]
                           .fromTransition[a][z]--;
                  // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;
   
                  maze1[mostProbPositionXY[a][i][z][1]]
                       [mostProbPositionXY[a][i][z][0]]
                           .agentMovingTo[a][z][i] = 0;
                  // agentMovingTo[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z][i]=0;
                  maze1[mostProbPositionXY[a][i][z][1]]
                       [mostProbPositionXY[a][i][z][0]]
                           .toTransition[a][z]--;

                  printf(
                      "(%i) ANNNd transition between [%d %d] and  [%d %d] back "
                      "to  %i\n",
                      z, mostProbPositionXY[a][i][z - 1][1],
                      mostProbPositionXY[a][i][z - 1][0],
                      mostProbPositionXY[a][i][z][1],
                      mostProbPositionXY[a][i][z][0],
                      maze1[mostProbPositionXY[a][i][z - 1][1]]
                           [mostProbPositionXY[a][i][z - 1][0]]
                               .fromTransition[a][z]);

                  // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]
                  // );
               }
            }
         }

         /* for (int z = 0; z < lookahead; z++) {
            mostProbPositionXY[a][i][z][1] = -2;
            mostProbPositionXY[a][i][z][0] = -2;
         } */
      }
      printf(" [4 4] at T 0: %i and mostProb [%d %d]\n",
             maze1[4][4].blockedAgent[0][0], mostProbPositionXY[a][i][0][1],
             mostProbPositionXY[a][i][0][0]);
   }
}

void observe_agent2(int a, int i, int lookahead, cell1 *previous) {
   if ((!goal_reached[i]) &&
       (((abs(position[i]->x - position[a]->x) +
          abs(position[i]->y - position[a]->y)) <= (lookahead)) &&
        ((abs(position[i]->x - position[a]->x) +
          abs(position[i]->y - position[a]->y)) > 0))) {
      canSee[a][i] = 1;
      printf("Agent %i at [%d %d] can see agent %i at [%d %d]\n", a + 1,
             position[a]->y, position[a]->x, i + 1, position[i]->y,
             position[i]->x);
      totp++;
      if ((mostProbPositionXY[a][i][1][0] == position[i]->x) &&
          (mostProbPositionXY[a][i][1][1] == position[i]->y)) {
         printf("good prediction!!!");
         goop++;
         evalPrevPrediction(a, i, 1);
      } else {
         printf("BAD prediction");  // neeed a counter here
         badp++;
         evalPrevPrediction(a, i, 0);
         // printf("increasing one to bad predictions, now is %i with lookahead
         // %i", badpredictions[lookahead], lookahead);
      }

      if (track[a][i][0][0] == -1)  // There is no previous record of the agent
      {
         track[a][i][0][0] = position[i]->x;
         track[a][i][0][1] = position[i]->y;
      } else  // at least the agent was seen once before
      {
         track[a][i][1][0] = track[a][i][0][0];
         track[a][i][1][1] = track[a][i][0][1];

         track[a][i][0][0] = position[i]->x;
         track[a][i][0][1] = position[i]->y;
      }

      printf("Now Computing PRediction......\n");
      computePrediction2(a, i, lookahead);

   } else {
      canSee[a][i] = 0;
      track[a][i][1][0] = track[a][i][0][0];
      track[a][i][1][1] = track[a][i][0][1];
      if (track[a][i][1][0] != -1) {
         printf("****But I just saw this guy at [%d %d]...%i.\n",
                mostProbPositionXY[a][i][0][1], mostProbPositionXY[a][i][0][0],
                maze1[mostProbPositionXY[a][i][0][1]]
                     [mostProbPositionXY[a][i][0][0]]
                         .blockedAgent[a][0]);
      }

      track[a][i][0][0] = -1;
      track[a][i][0][1] = -1;

      for (int z = 0; z < (lookahead); z++) {
         // printf(" HERE I AM %i %d
         // %d",!goal_reached[i],mostProbPositionXY[a][i][z][1],mostProbPositionXY[a][i][z][0]);//,maze1[mostProbPositionXY[a][i][z][1]][mostProbPositionXY[a][i][z][0]].blockedAgent[a][z]
         // );

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

               printf(
                   "(%i) SAW the guy (%i) before at [%d %d], not anymore, "
                   "making blocked back to  %i\n",
                   z, i, mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z][1]]
                        [mostProbPositionXY[a][i][z][0]]
                            .blockedAgent[a][z]);
               if (z > 0) {
                  //    betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;
                  maze1[mostProbPositionXY[a][i][z - 1][1]]
                       [mostProbPositionXY[a][i][z - 1][0]]
                           .fromTransition[a][z - 1]--;

                  //		toTransition[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;
                  maze1[mostProbPositionXY[a][i][z][1]]
                       [mostProbPositionXY[a][i][z][0]]
                           .toTransition[a][z]--;
                  maze1[mostProbPositionXY[a][i][z][1]]
                       [mostProbPositionXY[a][i][z][0]]
                           .agentMovingTo[a][z][i] = 0;

                  // agentMovingTo[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z][i]=0;

                  printf(
                      "(%i) ANd transition between [%d %d] and  [%d %d] back "
                      "to  %i\n",
                      z, mostProbPositionXY[a][i][z - 1][1],
                      mostProbPositionXY[a][i][z - 1][0],
                      mostProbPositionXY[a][i][z][1],
                      mostProbPositionXY[a][i][z][0],
                      maze1[mostProbPositionXY[a][i][z - 1][1]]
                           [mostProbPositionXY[a][i][z - 1][0]]
                               .fromTransition[a][z - 1]);

                  // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]
                  // );
               }
            }
         }
      }

      /* for (int z = 0; z < lookahead; z++) {
         mostProbPositionXY[a][i][z][1] = -2;
         mostProbPositionXY[a][i][z][0] = -2;
      } */
   }
}

void observe_agent(int a, int i, int lookahead, cell1 *previous) {
   if ((!goal_reached[i]) &&
       (((abs(position[i]->x - position[a]->x) +
          abs(position[i]->y - position[a]->y)) <= (lookahead)) &&
        ((abs(position[i]->x - position[a]->x) +
          abs(position[i]->y - position[a]->y)) > 0))) {
      canSee[a][i] = 1;
      printf("Agent %i at [%d %d] can see agent %i at [%d %d]\n", a + 1,
             position[a]->y, position[a]->x, i + 1, position[i]->y,
             position[i]->x);
      totp++;
      if ((mostProbPositionXY[a][i][1][0] == position[i]->x) &&
          (mostProbPositionXY[a][i][1][1] == position[i]->y)) {
         printf("good prediction!!!");
         goop++;
         evalPrevPrediction(a, i, 1);
      } else {
         printf("BAD prediction");  // neeed a counter here
         badp++;
         evalPrevPrediction(a, i, 0);
         // printf("increasing one to bad predictions, now is %i with lookahead
         // %i", badpredictions[lookahead], lookahead);
      }

      // totalpredictions[lookahead]++;

      updateHistory(a, i, lookahead, previous);

      printf("Now Computing PRediction......\n");
      // computePrediction2(a,i, lookahead);

   } else {
      canSee[a][i] = 0;
      track[a][i][1][0] = track[a][i][0][0];
      track[a][i][1][1] = track[a][i][0][1];
      if (track[a][i][1][0] != -1) {
         printf("****But I just saw this guy at [%d %d]...%i.\n",
                mostProbPositionXY[a][i][0][1], mostProbPositionXY[a][i][0][0],
                maze1[mostProbPositionXY[a][i][0][1]]
                     [mostProbPositionXY[a][i][0][0]]
                         .blockedAgent[a][0]);
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

               printf(
                   "(%i) SAW the guy (%i) before at [%d %d], not anymore, "
                   "making blocked back to  %i\n",
                   z, i, mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z][1]]
                        [mostProbPositionXY[a][i][z][0]]
                            .blockedAgent[a][z]);
               if (z > 0) {
                  //    betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;
                  maze1[mostProbPositionXY[a][i][z - 1][1]]
                       [mostProbPositionXY[a][i][z - 1][0]]
                           .fromTransition[a][z - 1]--;

                  //		toTransition[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;
                  maze1[mostProbPositionXY[a][i][z][1]]
                       [mostProbPositionXY[a][i][z][0]]
                           .toTransition[a][z]--;
                  maze1[mostProbPositionXY[a][i][z][1]]
                       [mostProbPositionXY[a][i][z][0]]
                           .agentMovingTo[a][z][i] = 0;

                  // agentMovingTo[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z][i]=0;

                  printf(
                      "(%i) ANd transition between [%d %d] and  [%d %d] back "
                      "to  %i\n",
                      z, mostProbPositionXY[a][i][z - 1][1],
                      mostProbPositionXY[a][i][z - 1][0],
                      mostProbPositionXY[a][i][z][1],
                      mostProbPositionXY[a][i][z][0],
                      maze1[mostProbPositionXY[a][i][z - 1][1]]
                           [mostProbPositionXY[a][i][z - 1][0]]
                               .fromTransition[a][z - 1]);

                  // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]
                  // );
               }
            }
         }
      }

      /* for (int z = 0; z < lookahead; z++) {
         mostProbPositionXY[a][i][z][1] = -2;
         mostProbPositionXY[a][i][z][0] = -2;
      } */
   }
}

void updateHistory(int a, int i, int lookahead, cell1 *previous) {
   if (track[a][i][0][0] == -1)  // There is no previous record of the agent
   {
      track[a][i][0][0] = position[i]->x;
      track[a][i][0][1] = position[i]->y;
   } else  // at least the agent was seen once before
   {
      track[a][i][1][0] = track[a][i][0][0];
      track[a][i][1][1] = track[a][i][0][1];

      track[a][i][0][0] = position[i]->x;
      track[a][i][0][1] = position[i]->y;
   }

   printf("Agent %i model of agent %i is: \n", a + 1, i + 1);

   for (int t = 0; t < MEMORY; t++) {
      printf("Time %i: [%i %i] \n", t, track[a][i][t][1], track[a][i][t][0]);
   }

   if (track[a][i][1][0] !=
       -1)  // The agent has at least two observations of its neighbor i
   {
      updateProbabilities(a, i);
   }

   if (track[a][i][1][0] !=
       -1)  // The agent has at least two observations of its neighbor i
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
         printf("blocking OBSERVED position [%d %d] at t %i, so now %i \n",
                position[i]->y, position[i]->x, z,
                maze1[5][3].blockedAgent[3][0]);
         maze1[position[i]->y][position[i]->x].blockedAgent[a][z] =
             1;  // blockedAgent[position[i]->x][position[i]->y][a][z]=1;
         maze1[position[i]->y][position[i]->x].fromTransition[a][z]++;
         //	blockedAgent[position[i]->x][position[i]->y][a][1]=1;
         if (z > 0) {
            // betweenTransition[position[i]->x][position[i]->y][position[i]->x][position[i]->y][a][z]++;

            maze1[position[i]->y][position[i]->x].blockedAgent[a][z] = 1;

            // toTransition[position[i]->x][position[i]->y][a][z]++;
            maze1[position[i]->y][position[i]->x].toTransition[a][z]++;
            maze1[position[i]->y][position[i]->x].agentMovingTo[a][z][i] = 1;

            //	 agentMovingTo[position[i]->x][position[i]->y][a][z][i]=1;
            printf(
                "Observing AGNET %i moving FROM [%d %d] at t %i TO [%d %d] at "
                "t %i, total: %i\n",
                i, position[i]->y, position[i]->x, z - 1, position[i]->y,
                position[i]->x, z,
                maze1[position[i]->y][position[i]->x].toTransition[a][z]);

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
   printf(" - So far, %.2f of accuracy, ", (float)goop / (float)totp);

   printf(" and with neighbor %i, it is %.2f, or %i of total %i\n", i + 1,
          (float)good_pred_agents[a][i] / (float)pred_agents[a][i],
          good_pred_agents[a][i], pred_agents[a][i]);

   // getchar();
}

void updateProbabilities(int a, int i) {
   // 4 cases (assuming 4-connected grid)
   if (track[a][i][1][0] > track[a][i][0][0]) {
      obsNextCell[a][i][2]++;  // Left
      lastMove[a][i] = 2;

      printf("Agent %i moved LEFT %i\n", i + 1, obsNextCell[a][i][2]);
   }

   if (track[a][i][1][0] < track[a][i][0][0]) {
      obsNextCell[a][i][0]++;  // Right
      lastMove[a][i] = 0;
      printf("Agent %i moved RIGHT %i\n", i + 1, obsNextCell[a][i][0]);
   }

   if (track[a][i][1][0] == track[a][i][0][0]) {
      if (track[a][i][1][1] == track[a][i][0][1]) {
         obsNextCell[a][i][4]++;  // NoOp
         lastMove[a][i] = 4;
         printf("Agent %i DIDNT MOVE \n", i + 1);
      }
   }

   if (track[a][i][1][1] > track[a][i][0][1]) {
      obsNextCell[a][i][3]++;  // Up
      lastMove[a][i] = 3;
      printf("Agent %i moved UP %i\n", i + 1, obsNextCell[a][i][3]);
   }

   if (track[a][i][1][1] < track[a][i][0][1]) {
      obsNextCell[a][i][1]++;  // Down
      lastMove[a][i] = 1;
      printf("Agent %i moved DOWN %i\n", i + 1, obsNextCell[a][i][1]);
   }

   int total_nextCell = obsNextCell[a][i][2] + obsNextCell[a][i][0] +
                        obsNextCell[a][i][1] + obsNextCell[a][i][3] +
                        obsNextCell[a][i][4];

   nextCellProb[a][i][0] =
       100 * (float)obsNextCell[a][i][0] / (float)total_nextCell;
   nextCellProb[a][i][1] =
       100 * (float)obsNextCell[a][i][1] / (float)total_nextCell;
   nextCellProb[a][i][2] =
       100 * (float)obsNextCell[a][i][2] / (float)total_nextCell;
   nextCellProb[a][i][3] =
       100 * (float)obsNextCell[a][i][3] / (float)total_nextCell;
   nextCellProb[a][i][4] =
       100 * (float)obsNextCell[a][i][4] / (float)total_nextCell;

   printf(
       "Observed Probabilities: DOWN %.1f, UP %.1f, LEFT %.1f , RIGHT %.1f\n",
       nextCellProb[a][i][1], nextCellProb[a][i][3], nextCellProb[a][i][2],
       nextCellProb[a][i][0]);
}

void computePrediction2(int a, int i, int lookahead) {
   printf(
       "AGENT %i EEERASING PREVIOUS OBSERVATIONS of %i as in [%d %d] REALDEPTH "
       "%i, lookahead %i\n",
       a + 1, i + 1, mostProbPositionXY[a][i][0][1],
       mostProbPositionXY[a][i][0][0], realDepth[i], lookahead);

   for (int z = 0; z < realDepth[i]; z++)  //(lookahead); z++)
   {  // printf("Now z is %i and %i\n", z,mostProbPositionXY[a][i][z][0]);
      if (mostProbPositionXY[a][i][z][0] > -2) {
         if (z == 0) {
            maze1[mostProbPositionXY[a][i][z][1]]
                 [mostProbPositionXY[a][i][z][0]]
                     .blockedAgent[a][z] = 0;
            // blockedAgent[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]=0;
            printf("\nERASING blocking at [%d %d] to %i  ",
                   mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z][1]]
                        [mostProbPositionXY[a][i][z][0]]
                            .blockedAgent[a][z]);

            // printf("how about this [%d %d] %i at 0 and this [%d %d] %i at 1
            // ",mostProbPositionXY[a][i][0][1],mostProbPositionXY[a][i][0][0],
            // maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]].blockedAgent[a][0],
            // mostProbPositionXY[a][i][1][1],mostProbPositionXY[a][i][1][0]
            // ,maze1[mostProbPositionXY[a][i][1][1]][mostProbPositionXY[a][i][1][0]].blockedAgent[a][1]);

            printf("Now z is %i and %i\n", mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0]);
         }

         if ((z > 0) &&
             (maze1[mostProbPositionXY[a][i][z - 1][1]]
                   [mostProbPositionXY[a][i][z - 1][0]]
                       .fromTransition[a][z - 1] > 0) &&
             (maze1[mostProbPositionXY[a][i][z][1]]
                   [mostProbPositionXY[a][i][z][0]]
                       .toTransition[a][z] > 0)) {
            //(betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]>0))
            printf("**** [4 4] at T 0: %i \n", maze1[4][4].blockedAgent[0][0]);
            maze1[mostProbPositionXY[a][i][z - 1][1]]
                 [mostProbPositionXY[a][i][z - 1][0]]
                     .fromTransition[a][z - 1]--;
            // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;
            printf("\nERASING TRANSITION BETWEEN [%d %d] and [%d %d] to %i  ",
                   mostProbPositionXY[a][i][z - 1][1],
                   mostProbPositionXY[a][i][z - 1][0],
                   mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z - 1][1]]
                        [mostProbPositionXY[a][i][z - 1][0]]
                            .fromTransition[a][z]);
            printf(" ***[4 4] at T 0: %i \n", maze1[4][4].blockedAgent[0][0]);
            // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]);
            //	toTransition[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;

            maze1[mostProbPositionXY[a][i][z][1]]
                 [mostProbPositionXY[a][i][z][0]]
                     .toTransition[a][z]--;

            maze1[mostProbPositionXY[a][i][z][1]]
                 [mostProbPositionXY[a][i][z][0]]
                     .agentMovingTo[a][z][i] = 0;
            // agentMovingTo[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z][i]=0;

            printf("\nERASING TRANSITION TO [%d %d] to %i (at %i)",
                   mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z][1]]
                        [mostProbPositionXY[a][i][z][0]]
                            .toTransition[a][z],
                   z);
         }
         if (maze1[mostProbPositionXY[a][i][z][1]]
                  [mostProbPositionXY[a][i][z][0]]
                      .toTransition[a][z] ==
             0)  //    (toTransition[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]==0)//(blockedAgent[mostProbPositionX[a][i][z]][mostProbPositionY[a][i][z]][a][z]>0)
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

            printf(
                "\n1 No more ToTrans, ERASING BLOCKED [%d %d] at t %i to %i  ",
                mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0],
                z,
                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .blockedAgent[a][z]);
         }
      }

      if (z > 0) {
         mostProbPositionXY[a][i][z - 1][0] = -10;
         mostProbPositionXY[a][i][z - 1][1] = -10;
      }
   }

   mostProbPositionXY[a][i][0][0] = (position[i]->x);

   mostProbPositionXY[a][i][0][1] = (position[i]->y);

   // printf("\nACCORDING TO AGENT %i, MOST LIKELY DIRECTION FOR AGENT %i,
   // currently at [%d %d], at time 1 IS: ", a+1, i+1, position[i]->y,
   // position[i]->x); printf(" [%i %i]\n",path[i][2]->y,path[i][2]->x);

   //	blockedAgent[mostProbPositionXY[a][i][0][0]][mostProbPositionXY[a][i][0][1]][a][0]=1;
   maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]]
       .blockedAgent[a][0] = 1;
   printf("\nblocking NEW OBSERVED position [%d %d] [%d %d] : %i at T 0\n",
          position[i]->y, position[i]->x, mostProbPositionXY[a][i][0][1],
          mostProbPositionXY[a][i][0][0],
          maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]]
              .blockedAgent[a][0]);

   for (int t = 1; t < realDepth[i]; t++)  //(lookahead);t++)
   {
      printf(" NOW T is %i\n", t);
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

      // blockedAgent[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t][1]][a][t-1]=blockedAgent[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t][1]][a][t-1]-1;
      maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
          .blockedAgent[a][t] =
          maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
              .blockedAgent[a][t] +
          1;

      //	blockedAgent[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]=blockedAgent[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]+1;
      printf(
          "BBLOCKING: [%i %i] at time %i: %i \n",
          mostProbPositionXY[a][i][t][1], mostProbPositionXY[a][i][t][0], t,
          maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
              .blockedAgent[a][t]);

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
      //	betweenTransition[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t-1][1]][mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]++;

      maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
          .toTransition[a][t]++;
      // toTransition[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]++;

      maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
          .agentMovingTo[a][t][i] = 1;
      // agentMovingTo[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t][i]=1;
      printf(
          "TO: [%i %i] at time %i: %i \n", mostProbPositionXY[a][i][t][1],
          mostProbPositionXY[a][i][t][0], t,
          maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
              .toTransition[a][t]);

      //	 toTransition[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]);

      printf("TRANSITION: [%i %i] to [%i %i] at time %i: %i \n",
             mostProbPositionXY[a][i][t - 1][1],
             mostProbPositionXY[a][i][t - 1][0], mostProbPositionXY[a][i][t][1],
             mostProbPositionXY[a][i][t][0], t,
             maze1[mostProbPositionXY[a][i][t - 1][1]]
                  [mostProbPositionXY[a][i][t - 1][0]]
                      .fromTransition[a][t - 1]);
      // betweenTransition[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t-1][1]][mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]);

      // printf("REDUCING POS: [%i %i] at time %i to %i
      // \n",mostProbPositionXY[a][i][t-1][1],
      // mostProbPositionXY[a][i][t-1][0],t-1,blockedAgent[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t][1]][a][t-1]);
      //	blockedAgent[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t][1]][a][t-1]=0;
   }

   canSee[a][i] = 1;
}

void computePrediction(int a, int i, int lookahead) {
   printf("AGENT %i ERASING PREVIOUS OBSERVATIONS of %i as in [%d %d]\n", a + 1,
          i + 1, mostProbPositionXY[a][i][0][1],
          mostProbPositionXY[a][i][0][0]);

   for (int z = 0; z < (lookahead); z++) {
      if (mostProbPositionXY[a][i][z][0] > -2) {
         if (z == 0) {
            maze1[mostProbPositionXY[a][i][z][1]]
                 [mostProbPositionXY[a][i][z][0]]
                     .blockedAgent[a][z] = 0;
            // blockedAgent[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]=0;
            printf("\nERASING blocking at [%d %d] to %i  ",
                   mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z][1]]
                        [mostProbPositionXY[a][i][z][0]]
                            .blockedAgent[a][z]);
            printf(
                "how about this [%d %d] %i at 0 and this [%d %d] %i at 1 ",
                mostProbPositionXY[a][i][0][1], mostProbPositionXY[a][i][0][0],
                maze1[mostProbPositionXY[a][i][0][1]]
                     [mostProbPositionXY[a][i][0][0]]
                         .blockedAgent[a][0],
                mostProbPositionXY[a][i][1][1], mostProbPositionXY[a][i][1][0],
                maze1[mostProbPositionXY[a][i][1][1]]
                     [mostProbPositionXY[a][i][1][0]]
                         .blockedAgent[a][1]);
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
            printf("\nERASING TRANSITION BETWEEN [%d %d] and [%d %d] to %i  ",
                   mostProbPositionXY[a][i][z - 1][1],
                   mostProbPositionXY[a][i][z - 1][0],
                   mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z - 1][1]]
                        [mostProbPositionXY[a][i][z - 1][0]]
                            .fromTransition[a][z]);

            // betweenTransition[mostProbPositionXY[a][i][z-1][0]][mostProbPositionXY[a][i][z-1][1]][mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]);
            //	toTransition[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]--;

            maze1[mostProbPositionXY[a][i][z][1]]
                 [mostProbPositionXY[a][i][z][0]]
                     .toTransition[a][z]--;

            maze1[mostProbPositionXY[a][i][z][1]]
                 [mostProbPositionXY[a][i][z][0]]
                     .agentMovingTo[a][z][i] = 0;
            // agentMovingTo[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z][i]=0;

            printf("\nERASING TRANSITION TO [%d %d] to %i (at %i)",
                   mostProbPositionXY[a][i][z][1],
                   mostProbPositionXY[a][i][z][0],
                   maze1[mostProbPositionXY[a][i][z][1]]
                        [mostProbPositionXY[a][i][z][0]]
                            .toTransition[a][z],
                   z);
         }
         if (maze1[mostProbPositionXY[a][i][z][1]]
                  [mostProbPositionXY[a][i][z][0]]
                      .toTransition[a][z] ==
             0)  //    (toTransition[mostProbPositionXY[a][i][z][0]][mostProbPositionXY[a][i][z][1]][a][z]==0)//(blockedAgent[mostProbPositionX[a][i][z]][mostProbPositionY[a][i][z]][a][z]>0)
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

            printf(
                "\n2 No more ToTrans, ERASING BLOCKED [%d %d] at t %i to %i  ",
                mostProbPositionXY[a][i][z][1], mostProbPositionXY[a][i][z][0],
                z,
                maze1[mostProbPositionXY[a][i][z][1]]
                     [mostProbPositionXY[a][i][z][0]]
                         .blockedAgent[a][z]);
         }
      }

      if (z > 0) {
         mostProbPositionXY[a][i][z - 1][0] = -10;
         mostProbPositionXY[a][i][z - 1][1] = -10;
      }
   }

   mostProbPositionXY[a][i][0][0] = (position[i]->x);
   mostProbPositionXY[a][i][0][1] = (position[i]->y);
   printf(
       "\nACCORDING TO AGENT %i, MOST LIKELY DIRECTION FOR AGENT %i, currently "
       "at [%d %d], at time 0 IS: ",
       a + 1, i + 1, position[i]->y, position[i]->x);
   printf("SAME!!: [%i %i]\n", mostProbPositionXY[a][i][0][1],
          mostProbPositionXY[a][i][0][0]);

   //	blockedAgent[mostProbPositionXY[a][i][0][0]][mostProbPositionXY[a][i][0][1]][a][0]=1;
   maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]]
       .blockedAgent[a][0] = 1;
   printf("blocking NEW OBSERVED position [%d %d] : %i at T 0\n",
          position[i]->y, position[i]->x,
          maze1[mostProbPositionXY[a][i][0][1]][mostProbPositionXY[a][i][0][0]]
              .blockedAgent[a][0] = 1);

   // printf("how about at 4?: %i",
   // blockedAgent[mostProbPositionX[a][i][4]][mostProbPositionY[a][i][4]][a][4]);

   if (pred_agents[a][i] > 0) {
      // If prediction accuracy is too low (for now <0.5) don't make prediction,
      // assume it will not bother the agent

      if (((float)good_pred_agents[a][i] / (float)pred_agents[a][i]) > 0.9) {
         predict[a][i] = 1;
         //	printf("NICE!! good prediction rate, %.1f from %i and %i \n",
         //(float)good_pred_agents[a][i]/(float)pred_agents[a][i],good_pred_agents[a][i],
         // pred_agents[a][i]); getchar();

      } else {
         predict[a][i] = 0;
      }
   } else {
      predict[a][i] = 0;
   }

   // printf("SO: rate, %.1f from %i and %i and predict is %i \n",
   // (float)good_pred_agents[a][i]/(float)pred_agents[a][i],good_pred_agents[a][i],
   // pred_agents[a][i],predict[a][i]); 	getchar();

   for (int t = 1; t < (lookahead); t++) {
      float maxProb = -10;
      int mostProb = -1;
      // printf("Most prob is %.1f", nextCellProb[a][i][0]);
      for (int j = 0; j <= DIRECTIONS; j++) {
         if (nextCellProb[a][i][j] > maxProb) {
            mostProb = j;
            maxProb = nextCellProb[a][i][j];
         }
      }

      // printf("Most prob is %i ", lastMove[a][i]);
      mostProb = lastMove[a][i];

      // printf("ACCORDING TO AGENT %i, MOST LIKELY DIRECTION FOR AGENT %i,
      // currently at [%d %d], at time %i IS: ", a+1, i+1, position[i]->y,
      // position[i]->x,t); printf("Most prob is %i ", mostProb);

      if (mostProb == 0) {
         if (t == 1) {
            if (((position[i]->x) + 1) < MAZEWIDTH) {
               if (maze1[position[i]->y][position[i]->x + 1].obstacle == 0) {
                  mostProbPositionXY[a][i][t][0] = (position[i]->x) + 1;
                  mostProbPositionXY[a][i][t][1] = (position[i]->y);

               } else {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
                  printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               }
            }
         } else {
            if ((mostProbPositionXY[a][i][t - 1][0] + 1) < MAZEWIDTH) {
               if (maze1[mostProbPositionXY[a][i][t - 1][1]]
                        [mostProbPositionXY[a][i][t - 1][0] + 1]
                            .obstacle == 0) {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0] + 1;
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
               } else {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
                  printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               }
            }
         }

         // if((blockedObstacle[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]]==1)||(((position[i]->x)+1)>=MAZEWIDTH)||(mostProbPositionXY[a][i][t-1][0]+1>=MAZEWIDTH))
         if ((((position[i]->x) + 1) >= MAZEWIDTH) ||
             (mostProbPositionXY[a][i][t - 1][0] + 1 >= MAZEWIDTH)) {
            mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
            mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
            printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                   mostProbPositionXY[a][i][t][0]);
         } else {
            printf("RIGHT!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                   mostProbPositionXY[a][i][t][0]);
         }
      }

      if (mostProb == 1) {
         if (t == 1) {
            if (((position[i]->y) + 1) < MAZEHEIGHT) {
               if (maze1[position[i]->y + 1][position[i]->x].obstacle == 0) {
                  mostProbPositionXY[a][i][t][0] = (position[i]->x);
                  mostProbPositionXY[a][i][t][1] = (position[i]->y) + 1;

               } else {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
                  printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               }
            }

         } else {
            if ((mostProbPositionXY[a][i][t - 1][1] + 1) < MAZEHEIGHT) {
               if (maze1[mostProbPositionXY[a][i][t - 1][1] + 1]
                        [mostProbPositionXY[a][i][t - 1][0]]
                            .obstacle == 0) {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1] + 1;
                  printf("DOWN!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               } else {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
                  printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               }
            }
         }

         if ((((position[i]->y + 1)) >= MAZEHEIGHT) ||
             (mostProbPositionXY[a][i][t - 1][1] + 1 >= MAZEHEIGHT)) {
            mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
            mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
            printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                   mostProbPositionXY[a][i][t][0]);
         } else {
            printf("DOWN!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                   mostProbPositionXY[a][i][t][0]);
         }
      }

      /*
         if(t==1)
   {
      if(((position[i]->y)+1)<MAZEHEIGHT)
      {
      mostProbPositionXY[a][i][t][0]= (position[i]->x);
      mostProbPositionXY[a][i][t][1]=  (position[i]->y)+1;
   }
   }
   else
   {if((mostProbPositionXY[a][i][t-1][1]+1)<MAZEHEIGHT)
      {
      mostProbPositionXY[a][i][t][0]= mostProbPositionXY[a][i][t-1][0];
      mostProbPositionXY[a][i][t][1]= mostProbPositionXY[a][i][t-1][1]+1;
   }
   }
   if((maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].obstacle==1)||(((position[i]->y)+1)>=MAZEHEIGHT)||((mostProbPositionXY[a][i][t-1][1]+1)>=MAZEHEIGHT))
   {
      mostProbPositionXY[a][i][t][0]= mostProbPositionXY[a][i][t-1][0];
      mostProbPositionXY[a][i][t][1]= mostProbPositionXY[a][i][t-1][1];
      printf("STUCKK!!: [%i %i]\n",mostProbPositionXY[a][i][t][1],
   mostProbPositionXY[a][i][t][0]);
   }
   else
   {

   printf("DOWN!!: [%i %i]\n",mostProbPositionXY[a][i][t][1],
   mostProbPositionXY[a][i][t][0]);}
   }
   */
      if (mostProb == 2) {
         if (t == 1) {
            if (((position[i]->x) - 1) >= 0) {
               if (maze1[position[i]->y][position[i]->x - 1].obstacle == 0) {
                  mostProbPositionXY[a][i][t][0] = (position[i]->x) - 1;
                  mostProbPositionXY[a][i][t][1] = (position[i]->y);

               } else {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
                  printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               }
            }
         } else {
            if ((mostProbPositionXY[a][i][t - 1][0] - 1) >= 0) {
               if (maze1[mostProbPositionXY[a][i][t - 1][1]]
                        [mostProbPositionXY[a][i][t - 1][0] - 1]
                            .obstacle == 0) {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0] - 1;
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
               } else {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
                  printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               }
            }
         }

         // if((blockedObstacle[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]]==1)||(((position[i]->x)+1)>=MAZEWIDTH)||(mostProbPositionXY[a][i][t-1][0]+1>=MAZEWIDTH))
         if ((((position[i]->x) - 1) < 0) ||
             (mostProbPositionXY[a][i][t - 1][0] - 1 < 0)) {
            mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
            mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
            printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                   mostProbPositionXY[a][i][t][0]);
         } else {
            printf("LEFT!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                   mostProbPositionXY[a][i][t][0]);
         }

         /*
      if(t==1)
   {

      if(((position[i]->x)-1)>=0)
      {
      mostProbPositionXY[a][i][t][0]= (position[i]->x)-1;
      mostProbPositionXY[a][i][t][1]=  (position[i]->y);
   }
   }
   else
   {   if(mostProbPositionXY[a][i][t-1][0]-1>=0)
      {
      mostProbPositionXY[a][i][t][0]= mostProbPositionXY[a][i][t-1][0]-1;
      mostProbPositionXY[a][i][t][1]= mostProbPositionXY[a][i][t-1][1];
      }
   }
   if((mostProbPositionXY[a][i][t-1][0]-1<0)||(((position[i]->x)-1)<0))
   {
   //(maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].obstacle==1)||
      mostProbPositionXY[a][i][t][0]= mostProbPositionXY[a][i][t-1][0];
      mostProbPositionXY[a][i][t][1]= mostProbPositionXY[a][i][t-1][1];
      printf("STUCKK!!: [%i %i]\n",mostProbPositionXY[a][i][t][1],
   mostProbPositionXY[a][i][t][0]);
   }
   else
   {

   printf("LEFT!!: [%i %i]\n",mostProbPositionXY[a][i][t][1],
   mostProbPositionXY[a][i][t][0]);}
   * */
      }

      if (mostProb == 3) {
         if (t == 1) {
            if (((position[i]->y) - 1) >= 0) {
               if (maze1[position[i]->y - 1][position[i]->x].obstacle == 0) {
                  mostProbPositionXY[a][i][t][0] = (position[i]->x);
                  mostProbPositionXY[a][i][t][1] = (position[i]->y) - 1;

               } else {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
                  printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               }
            }
         } else {
            if ((mostProbPositionXY[a][i][t - 1][1] - 1) >= 0) {
               if (maze1[mostProbPositionXY[a][i][t - 1][1] - 1]
                        [mostProbPositionXY[a][i][t - 1][0]]
                            .obstacle == 0) {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1] - 1;
                  printf("UP!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               } else {
                  mostProbPositionXY[a][i][t][0] =
                      mostProbPositionXY[a][i][t - 1][0];
                  mostProbPositionXY[a][i][t][1] =
                      mostProbPositionXY[a][i][t - 1][1];
                  printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                         mostProbPositionXY[a][i][t][0]);
               }
            }
         }

         if ((((position[i]->y - 1)) < 0) ||
             (mostProbPositionXY[a][i][t - 1][1] - 1 < 0)) {
            mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
            mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
            printf("STUCKK!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                   mostProbPositionXY[a][i][t][0]);
         } else {
            printf("UP!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                   mostProbPositionXY[a][i][t][0]);
         }

         /*
   if(t==1)
   {
      if(((position[i]->y)-1)>=0)
      {
      mostProbPositionXY[a][i][t][0]= (position[i]->x);
      mostProbPositionXY[a][i][t][1]=  (position[i]->y)-1;
   }
   }
   else
   {
      if(mostProbPositionXY[a][i][t-1][1]-1>=0)
      {

      mostProbPositionXY[a][i][t][0]= mostProbPositionXY[a][i][t-1][0];
      mostProbPositionXY[a][i][t][1]= mostProbPositionXY[a][i][t-1][1]-1;
   }
   }
   if((maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]].obstacle==1)||(mostProbPositionXY[a][i][t-1][1]-1<0)||(((position[i]->y)-1)<0))
   {
      mostProbPositionXY[a][i][t][0]= mostProbPositionXY[a][i][t-1][0];
      mostProbPositionXY[a][i][t][1]= mostProbPositionXY[a][i][t-1][1];
      printf("STUCKK!!: [%i %i]\n",mostProbPositionXY[a][i][t][1],
   mostProbPositionXY[a][i][t][0]);
   }
   else
   {

   printf("UP!!: [%i %i]\n",mostProbPositionXY[a][i][t][1],
   mostProbPositionXY[a][i][t][0]);}
    */
      }

      if (mostProb == 4) {
         // printf("NOTHING !!\n");
         if (t == 1) {
            mostProbPositionXY[a][i][t][0] = (position[i]->x);
            mostProbPositionXY[a][i][t][1] = (position[i]->y);
         } else {
            mostProbPositionXY[a][i][t][0] = mostProbPositionXY[a][i][t - 1][0];
            mostProbPositionXY[a][i][t][1] = mostProbPositionXY[a][i][t - 1][1];
         }
         printf("NOTHING!!: [%i %i]\n", mostProbPositionXY[a][i][t][1],
                mostProbPositionXY[a][i][t][0]);
      }

      // Using comm instead of prediction

      mostProbPositionXY[a][i][t][0] = path[i][t]->x;
      mostProbPositionXY[a][i][t][1] = path[i][t]->y;

      // blockedAgent[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t][1]][a][t-1]=blockedAgent[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t][1]][a][t-1]-1;
      maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
          .blockedAgent[a][t] =
          maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
              .blockedAgent[a][t] +
          1;

      //	blockedAgent[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]=blockedAgent[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]+1;
      printf(
          "BLOCKING: [%i %i] at time %i: %i \n", mostProbPositionXY[a][i][t][1],
          mostProbPositionXY[a][i][t][0], t,
          maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
              .blockedAgent[a][t]);

      maze1[mostProbPositionXY[a][i][t - 1][1]]
           [mostProbPositionXY[a][i][t - 1][0]]
               .fromTransition[a][t - 1]++;
      //	betweenTransition[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t-1][1]][mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]++;

      maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
          .toTransition[a][t]++;
      // toTransition[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]++;

      maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
          .agentMovingTo[a][t][i] = 1;
      // agentMovingTo[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t][i]=1;
      printf(
          "TO: [%i %i] at time %i: %i \n", mostProbPositionXY[a][i][t][1],
          mostProbPositionXY[a][i][t][0], t,
          maze1[mostProbPositionXY[a][i][t][1]][mostProbPositionXY[a][i][t][0]]
              .toTransition[a][t]);

      //	 toTransition[mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]);

      printf("TRANSITION: [%i %i] to [%i %i] at time %i: %i \n",
             mostProbPositionXY[a][i][t - 1][1],
             mostProbPositionXY[a][i][t - 1][0], mostProbPositionXY[a][i][t][1],
             mostProbPositionXY[a][i][t][0], t,
             maze1[mostProbPositionXY[a][i][t - 1][1]]
                  [mostProbPositionXY[a][i][t - 1][0]]
                      .fromTransition[a][t]);
      // betweenTransition[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t-1][1]][mostProbPositionXY[a][i][t][0]][mostProbPositionXY[a][i][t][1]][a][t]);

      // printf("REDUCING POS: [%i %i] at time %i to %i
      // \n",mostProbPositionXY[a][i][t-1][1],
      // mostProbPositionXY[a][i][t-1][0],t-1,blockedAgent[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t][1]][a][t-1]);
      //	blockedAgent[mostProbPositionXY[a][i][t-1][0]][mostProbPositionXY[a][i][t][1]][a][t-1]=0;
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
   } /* end for */
}

/* ------------------------------------------------------------------------------*/
void test_rtaastar(int lookahead, int prunning) {
   cell1 *tempcell, *previous, *current;
   int y, x, i, j, k;
   long int m;

   gettimeofday(&tv11c, NULL);
   printf("\nGENERATING RANDOMMAZE\n");
   newrandommaze_astar();
   printf("\nDONE\n");
   gettimeofday(&tv22c, NULL);
   time_astar_initialize1 += 1.0 * (tv22c.tv_sec - tv11c.tv_sec) +
                             1.0 * (tv22c.tv_usec - tv11c.tv_usec) / 1000000.0;
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
               // printf(" AHA!!!\n");
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
            canSee[i][k] = 0;
            role[i][j] = -1;
            // Role=1 means I am not deferent to agent j
            // Role=0 means taht I am deferent to agent i

            printf("\nWatching %i from %i.. \n", i + 1, j + 1);
            // printf("OBSERVING AGENTS\n");

            observe_agent(j, i, lookahead, position[i]);

            //	printf("\n now %i\n ", maze1[5][3].blockedAgent[3][0]);
         }
      }
   }

   // Loop until all agents finish
   while (finish_all && time_step <= MAX_TIME_STEPS) {
      printf("OBSERVING AGENTS\n");
      //			i = random() % NAGENTS;
      // For each agent in the problem..
      for (i = 0; i < NAGENTS; i++) {
         if (RUN1 >= 0 && robot_steps1 >= 0) {
            // printf("Antes Agent[%d] A* Start [%d,%d] Goal [%d,%d] h:%f
            // step:%d time_step:%d
            // terminado:%d\n",i+1,position[i]->y,position[i]->x,goal[i]->y,goal[i]->x,position[i]->h,robot_steps1,time_step,NAGENTS-finish_all);
            // print the grid
            Multi_print_grid();
            for (k = 0; k < NAGENTS; k++) {
               printf("(%d)[%d,%d]....(%i and %i) ", k + 1, position[k]->y,
                      position[k]->x, role[0][1], role[1][0]);
            }
            printf("\n");
            getchar();
         }

#ifdef RANDOMMOVES
         if (goal_reached[i])
            randommove(i);
         else {
#else
         if (position[i] != goal[i]) {  // While it is not at its goal...
#endif

            // First, compute the shortest path, ignoring other agents...
            if (!compute_shortestpath_astar(i, lookahead)) {
               //	printf(" OOOPPS,AGENT %i NEED TO BACKTRACK!!!\n",i);
               // printf("***********************************************************************\n");
               // printf("*   A*  when mazeiteration1 = %d,    No path possible
               // !!!    *\n", mazeiteration1);
               // printf("***********************************************************************\n");
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
                        if (conflictCost[i][idealPath[i][l - 1]->y]
                                        [idealPath[i][l - 1]->x][l - 1] >=
                            0.5) {
                           // printf("FIRST CASE IS %.1f\n",
                           // conflictCost[i][idealPath[i][l-1]->y][idealPath[i][l-1]->x][l-1]);
                           conflictCost[i][idealPath[i][l]->y]
                                       [idealPath[i][l]->x][l] = 1;
                        }
                     }

                     printf("IDEAL PATH AT POS %i: [%d %d] - CCost %.2f\n", l,
                            idealPath[i][l]->y, idealPath[i][l]->x,
                            conflictCost[i][idealPath[i][l]->y]
                                        [idealPath[i][l]->x][l]);
                     pathlength[i] = l;
                  }
               }

               printf("0 SO FAR SO GOOD AGENT %i!!!\n", i);
               previous = position[i];
               printf("0 Me QUIERO MOVER a [%d %d]\n", (position[i]->trace)->y,
                      (position[i]->trace)->x);

               if (position[i]->parent[i] != NULL) {
                  printf(" desde [%d %d] \n", position[i]->parent[i]->y,
                         position[i]->parent[i]->x);
               }

               printf(" THE PATH LENGHT OF AGENT %i is %i \n", i + 1,
                      pathlength[i]);

               // getchar();

               // SECOND SEARCH, BASED ON CONSTRAINTS/CONFLICTS:
               if (!compute_constraintpath(i, lookahead)) {
                  printf("No solution???? Might need to backtrack %i steps, \n",
                         lastMobileCellDist[i]);
                  // NEED TO CHANGE MODE TO BACKTRACK!!!
                  backtrack[i] = 1;

                  if (position[i]->parent[i] != NULL) {
                     printf(" BACKTRACKING TO postiion [%d %d]!!!\n",
                            position[i]->parent[i]->y,
                            position[i]->parent[i]->x);
                  }

                  previous = position[i];
                  printf(" Me QUIERO MOVER a %d %d \n",
                         (position[i]->parent[i])->y,
                         (position[i]->parent[i])->x);

                  /*for(int l=1;l<=lookahead;l++) WHAT TO Do INSTEAD OF THIS?
                  {
                     if(path[i][l]!=NULL)
                  {
                  printf("PATH AT POS %i: [%d %d]\n", l, path[i][l]->y,
                  path[i][l]->x);
                  }

                  }*/

                  // printf("REAL DEPTH %i", realDepth[i]);

                  if ((position[i]->parent[i]->blocked[0]) &&
                      (position[i]->parent[i]->x != position[i]->x) &&
                      ((position[i]->parent[i]->y != position[i]->y))) {
                     printf(" PERO ESTOY BLOQUEADO (look: %i)\n", lookahead);

                     continue;
                  }
                  position[i] = position[i]->parent[i];
                  agent_cost[i] += euclidian(previous, position[i]);
                  robot_steps1++;
                  previous->trace = NULL;
                  previous->blocked[0] = 0;

               } else {
                  printf(" SO FAR SO GOOD AGENT %i, at postiion [%d %d]!!!\n",
                         i, position[i]->y, position[i]->x);
                  previous = position[i];

                  if (position[i]->parent[i] != NULL) {
                     printf("Whose parent is [%d %d]\n",
                            position[i]->parent[i]->y,
                            position[i]->parent[i]->x);
                  }
                  printf(" Me QUIERO MOVER a %d %d \n", (position[i]->trace)->y,
                         (position[i]->trace)->x);

                  for (int l = 1; l <= lookahead; l++) {
                     if (path[i][l] != NULL) {
                        printf("PATH AT POS %i: [%d %d]\n", l, path[i][l]->y,
                               path[i][l]->x);
                     }
                  }

                  printf("REAL DEPTH %i", realDepth[i]);

                  if ((position[i]->trace->blocked[0]) &&
                      (position[i]->trace->x != position[i]->x) &&
                      ((position[i]->trace->y != position[i]->y))) {
                     printf(" PERO ESTOY BLOQUEADO (look: %i)\n", lookahead);

                     continue;
                  }
                  position[i] = position[i]->trace;
                  agent_cost[i] += euclidian(previous, position[i]);
                  robot_steps1++;
                  previous->trace = NULL;
                  previous->blocked[0] = 0;

                  for (j = 0; j < NAGENTS; j++) {
                  }
                  position[i]->blocked[0] = 1;
                  printf(" Me movi a %d %d", position[i]->y, position[i]->x);
                  printf(
                      " con H %.1f \n",
                      hvalues[MAZEWIDTH * position[i]->y + position[i]->x][i]);
                  position[i]->parent[i] = previous;
                  printf(" My new parent is [%d %d]\n",
                         position[i]->parent[i]->y, position[i]->parent[i]->x);

                  printf(
                      " -------------------------------------------------------"
                      "---------------------------\n");
                  printf(
                      " -------------------------------------------------------"
                      "---------------------------\n");
                  //	agentVelx[i]=(float)(position[i]->x - previous->x);
                  //	agentVely[i]=(float)(position[i]->y - previous->y);

                  for (j = 0; j < NAGENTS; j++) {
                     // printf(" [4 4] at T 0: %i \n",
                     // maze1[4][4].blockedAgent[0][0]);
                     if (i != j) {
                        printf("\nWatching %i from %i..\n", i + 1, j + 1);
                        printf(
                            "OBSERVING MOVING AGENT with lookahaead %i and "
                            "previous [%d %d]\n",
                            lookahead, previous->y, previous->x);

                        observe_agent2(j, i, lookahead, previous);

                        printf("\nWatching %i from %i.. \n", j + 1, i + 1);
                        printf("MOVING AGENT OBSERVING\n");
                        observe_new_agents(
                            i, j, lookahead);  // Previous not used anymore
                     }
                  }

                  // Agent updates DV of its previous position
                  /*	if((previous->velx[i]<0.0001)&&(previous->vely[i]<0.0001))
                  {
                     previous->velx[i]=(float)(position[i]->x - previous->x);
                     previous->vely[i]=(float)(position[i]->y - previous->y);

                  }
                  else
                  {
                     previous->velx[i]=(float)(position[i]->x -
                  previous->x)*ALPHA + (float)previous->velx[i]*(1-ALPHA);
                     previous->vely[i]=(float)(position[i]->y -
                  previous->y)*ALPHA + (float)previous->vely[i]*(1-ALPHA);

                  }


                  printf("**DV of cell [%d %d]: [%.1f %.1f] for ag.
                  %i\n",previous->y,previous->x,previous->vely[i],previous->velx[i],
                  i+1);
                   */
                  //	getchar();

                  //	if (RUN1 >= 2 && robot_steps1 >= 0){printf("Angent[%d]
                  // A* Start [%d,%d] Goal [%d,%d] h:%f step:%d
                  // nei:%d\n",i,position[i]->y,position[i]->x,goal[i]->y,goal[i]->x,position[i]->h,robot_steps1,count_nei(position[i]));print_grid(position[i]->x,position[i]->y,position[i],goal[i]->x,goal[i]->y);getchar();}
                  if (position[i] == goal[i]) {
                     if (goal_reached[i] == 0) {
                        lastfinish = time_step;
                     }
                     goal_reached[i] = 1;
                     solution_cost += agent_cost[i];
                     finish_all--;
#ifndef RANDOMMOVES
                     // position[i]->obstacle = 1;
                     position[i]->obstacle = 0;
                     position[i]->blocked[0] = 0;
                     // position[i]->x=1;
                     // position[i]->y=1;
                     printf(
                         "** LLEGO time_step:%d** %d finish:%d cost:%f total "
                         "cost:%f, now at [%d %d]\n",
                         time_step, i, NAGENTS - finish_all, agent_cost[i],
                         total_cost, position[i]->y, position[i]->x);
                     // getchar();
#endif
                     if (finish_all == 0) {
                        Multi_print_grid();

                        total_cost = 0;
                        for (int ag = 0; ag < NAGENTS; ag++) {
                           total_cost = total_cost + agent_cost[ag];
                           printf("Agent %i COST: %f \n", ag + 1,
                                  agent_cost[ag]);
                        }
                        // printf("\nTOTAL COST %f, ",total_cost);
                        printf("AVG COST %f, ", total_cost / (float)NAGENTS);
                        printf("FINISH TIME %i TIMESTEPS\n", time_step);

                        return;
                     }
                  }

               }  // From computenewpath

            }  // from computeshortestpath
         }

         //		  i = (i+1) % NAGENTS;
      }
      time_step++;
      // updatemaze1(previous,mazestart1);

      // getchar();
   }  // end  while(finish_all)
   if (finish_all != 0) {
      printf(
          "\nNOT ALL AGENTS WERE ABLE TO REACH THEIR GOALS!!! :( :(  (look: "
          "%i)\n",
          lookahead);
      printf("\nNOT ALL AGENTS WERE ABLE TO REACH THEIR GOALS!!! :( :( \n");
      printf("\nNOT ALL AGENTS WERE ABLE TO REACH THEIR GOALS!!! :( :( \n");

      // getchar();
   } else {
      printf(
          "\nGREAT!!!  ALL AGENTS WERE ABLE TO REACH THEIR GOALS in %i "
          "TIMESTEPS!!! :) :) \n",
          time_step);
      printf(
          "\nGREAT!!!  ALL AGENTS WERE ABLE TO REACH THEIR GOALS in %i "
          "TIMESTEPS!!! :) :) \n",
          time_step);
      printf(
          "\nGREAT!!!  ALL AGENTS WERE ABLE TO REACH THEIR GOALS in %i "
          "TIMESTEPS!!! :) :) \n",
          time_step);
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
   // Valor limite para lookahead
   int lookahead;
   int prunning, i;
   int look[1] = {4};  // 3,4,5,8,14};//{1,8,16,32,64,128,256,512,1024};
   float total_score[(int)(sizeof(look) / (float)sizeof(int))],
       avg_score[(int)(sizeof(look) / (float)sizeof(int))],
       total_time[(int)(sizeof(look) / (float)sizeof(int))];
   float avg_finish[(int)(sizeof(look) / (float)sizeof(int))],
       last_finish[(int)(sizeof(look) / (float)sizeof(int))];
   srand(time(NULL));
   float ftimes[RUNS];
   // for (prunning = 0; prunning <1; prunning++)
   for (i = 0; i < (int)(sizeof(look) / (float)sizeof(int)); i++) {
      avg_finish[i] = 0;
      total_time[i] = 0;
      avg_score[i] = 0;
      lookahead = look[i];
      last_finish[i] = 0;
      printf("Now I is %i \n", i);
      int RUN1_agents = 0;
      printf("lookahead == [%d] ___________________________________\n",
             lookahead);

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
            for (int j = 0; j < NAGENTS; j++) {
               lastMobileCellDist[a] =
                   10000;  // big number to emphasize that at the beginning he
                           // does not have memory of previously mobile states
               pred_agents[a][j] = 0;
               agentInfo[a] = -1;
               good_pred_agents[a][j] = 0;
               predict[a][j] = 0;
               conflictType[a][j] = -1;
            }
         }
         printf("case == [%ld] ___________________________________\n", RUN1);
         srand(5 * RUN1 + 100);
         generate_maze(RUN1);
         gettimeofday(&tv11, NULL);
         printf("NOW TEST RTA!!! \n");
         // getchar();

         // Call to method, one per iteration
         test_rtaastar(lookahead, prunning);

         gettimeofday(&tv22, NULL);
         // printf("Agents Remaining: %i at RUN %i \n", finish_all,RUN1);
         printf("Agents Remaining: %i at RUN %ld \n", finish_all, RUN1);
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
         printf("AVG # agents FINISH %f runagents %i\n", avg_finish[i],
                RUN1_agents);
         printf("AVG COST %f, ", total_cost / (float)NAGENTS);
         avg_score[i] = avg_score[i] + total_cost / (float)NAGENTS;
         total_time[i] = total_time[i] + time_step;
         printf("FINISH TIME %i in avg %f TIMESTEPS\n", lastfinish,
                last_finish[i]);
         printf(" Bad predictions: %i, Good: %i and total: %i, rate: %.1f \n",
                badp, goop, totp, (float)goop / (float)totp);
         ftimes[RUN1] = lastfinish;
         for (int a = 0; a < NAGENTS; a++) {
            for (int j = 0; j < NAGENTS; j++) {
               printf("\nGoodPred for %i with %i: %.2f (%i/%i)", a + 1, j + 1,
                      (float)good_pred_agents[a][j] / (float)pred_agents[a][j],
                      good_pred_agents[a][j], pred_agents[a][j]);
            }
         }

         time_astar += 1.0 * (tv22.tv_sec - tv11.tv_sec) +
                       1.0 * (tv22.tv_usec - tv11.tv_usec) / 1000000.0;
         robotmoves_total1 += robot_steps1;
         // printf("TOTAL %i FINISH TIME %f o %i TIMESTEPS\n",i,total_time[i],
         // robotmoves_total1);
         // getchar();
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
            printf("No se puede abrir el archivo de salida");
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

               if (j != 0) {
                  // printf("Acc to %i, AGENT %i moved UP: %i, DOWN: %i, LEFT:
                  // %i, RIGHT:%i \n", a+1, j+1, obsNextCell[a][j][1],
                  // obsNextCell[a][j][3], obsNextCell[a][j][2],
                  // obsNextCell[a][j][0]);
               }
            }
         }

      }  // end for RUN1S

      // getchar();

   }  // end lookahead

   for (int i = 0; i < (int)(sizeof(look) / (float)sizeof(int)); i++) {
      lookahead = look[i];
      float op = 0;
      for (int r = 0; r < RUNS; r++) {
         // printf(" %f , ", ftimes[r]);
         op = op + fabs(ftimes[r] - last_finish[i]);
         // printf("%f\n", op);
      }

      float stdv = sqrtf(op / (float)(RUNS - 1));

      // printf("AVG COST FOR LOOK %i: ", lookahead);
      //  printf("%f\n",  avg_score[i]/(float)RUNS);

      printf("\naccuracy of pred: %.1f",
             1 - ((float)badpredictions[i] / (float)totalpredictions[i]));
      printf(" of %i predictions\n", totalpredictions[i]);

      printf("AVG TIME FOR LOOK %i: ", lookahead);
      //  printf("RUNS %i ", RUNS);
      printf("%f\n", last_finish[i]);
      printf("AVG AGENTS FINISHIN FOR LOOK %i: ", lookahead);
      printf("%f  std %f \n", avg_finish[i], stdv);

      for (int a = 0; a < NAGENTS; a++) {
         for (int j = 0; j < NAGENTS; j++) {
            if (j != 0) {
               //	printf("Acc to %i, AGENT %i moved UP: %i, DOWN: %i,
               // LEFT: %i, RIGHT:%i \n", a, j, obsNextCell[a][j][1],
               // obsNextCell[a][j][3], obsNextCell[a][j][2],
               // obsNextCell[a][j][0]);
            }
         }
      }
   }

   return;
}

#endif  // end #
