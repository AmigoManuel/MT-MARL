/////////////////////////////////////////////////////////////////////
// Carlos Hernandez & Xiaoxun Sun & Sven Koenig @ USC 2011
// All rights reserved
/////////////////////////////////////////////////////////////////////

#include "heap.h"
#include "maze.h"
#include "stdio.h"
#include "include.h"
#include "math.h"
#include "rtaastar.h"
#include "testall.h"

#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#ifdef TESTRTAASTAR
#define MAZEITERATIONS 1000000
unsigned long int targetdiscount[MAZEITERATIONS];
#define MAXSEARCHES 100000000
// unsigned long int travel_distance[MAXSEARCHES];
int travel_distance[NAGENTS];
int completion_time[NAGENTS];

// Xiaoxun: this is for 8-connected grids, with sqrt(2) as the cost for diagonal movements
#ifdef EIGHTCONNECTED
#define HA(from, to) ((sqrt(2)) * min(abs((from)->y - (to)->y), abs((from)->x - (to)->x))) + ((1) * (max(abs((from)->y - (to)->y), abs((from)->x - (to)->x)) - min(abs((from)->y - (to)->y), abs((from)->x - (to)->x))))
#else
#define HA(from, to) ((1) * (abs((from)->y - (to)->y) + abs((from)->x - (to)->x)))
#endif
#define euclidian(from, to) (sqrt(pow(((from)->x - (to)->x), 2.0) + pow(((from)->y - (to)->y), 2.0)))

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
short active_prune = 0; //active the prune when there is an obstacle/border in the search tree
cell1 *CLOSED[10000];
double f_value = 0;
double total_cost = 0;
double total_time = 0;
short lastfinish = -1000;
cell1 *last_recently_see[NAGENTS];
int push_over[NAGENTS];

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
int time_step;
short goal_reached[NAGENTS];

int push_out_count = 0;
int enable_print = 0;

cell1 *tmpcell1;
cell1 *tmpcell2;
struct timeval tv11, tv22, tv11a, tv22a, tv11b, tv22b, tv11c, tv22c, tv11d, tv22d, tv11e, tv22e;

long numberofexp, hsal;

/* Inicializa A* */
void initialize_astar()
{
    // Iteración sobre el mapa
    mazeiteration1 = 0;
    // Pasos del agente
    robot_steps1 = 0;
    // TODO: por averiguar estas 2
    keymodifier1 = 0;
    statpercolated2 = 0;
    // Vacía el heap2
    emptyheap2();
    // Marca los objetivos como no alcanzados
    for (int i = 0; i < NAGENTS; i++)
        goal_reached[i] = 0;
}

int agent_acupation(cell1 *auxcell)
{
    for (int i = 0; i < NAGENTS; i++)
    {
        if (position[i] == auxcell)
            return i + 1;
    }
    return 0;
}

int goal_acupation(cell1 *auxcell)
{
    for (int i = 0; i < NAGENTS; i++)
    {
        if (goal[i] == auxcell)
            return i + 1;
    }
    return 0;
}

//------------------------------------------------------------------------------------------------------
void Multi_print_grid()
{
    int x, y, x0, xf, y0, yf, w, aoc, goc;
    long int num1, num2, num;
    double n, dif = 0;

    for (y = 0; y < MAZEHEIGHT; ++y)
    {
        for (x = 0; x < MAZEWIDTH; ++x)
        {
            tmpcell1 = &maze1[y][x];
            aoc = agent_acupation(tmpcell1);
            goc = goal_acupation(tmpcell1);
            if (maze1[y][x].obstacle == 1)
            {
                printf("\e[42m");
                printf("%2s", "#");
                printf("\e[0m");
            }
            else
            {
                if (aoc)
                {
                    printf("\e[44m");
                    printf("%2d", aoc);
                    printf("\e[0m");
                }
                else
                {
                    if (goc)
                    {
                        printf("\e[43m");
                        printf("%2d", goc);
                        printf("\e[0m");
                    }
                    else
                    {
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
//------------------------------------------------------------------------------------------------------

void initialize_state(cell1 *tmpcell)
{
    if (tmpcell->iteration == 0)
    {
        tmpcell->g = LARGE; /////
    }
    else if (tmpcell->iteration != mazeiteration1)
    {
        tmpcell->g = LARGE;
    }
    tmpcell->iteration = mazeiteration1;
    return;
}

//-------------------------------------------------------------------------------------------------------

int computeshortestpath_astar(int a, int lookahead)
{
    long valuehpath, j, minimo;
    int variant1, sal, i;
    cell1 *cellminimo, *cellpas, *parent;
    int pi = 0, pf = 0;
    double ch, oh, difh, mindif, f;
    time_deadline = 0;

    mazestart1 = position[a];
    mazegoal1 = goal[a];
    //	if (enable_print) printf("a:%d [%d,%d]\n",a, position[a]->y,position[a]->x);

    mazeiteration1++;
    emptyheap2();
    cont_closed = 0;

#ifdef STATISTICS
    searches_astar1++;
    statexpanded1_initial = statexpanded1;
#endif

    initialize_state(mazestart1);
    mazestart1->g = 0;
    mazestart1->key = 0;
    mazestart1->searchtree = NULL;
    mazestart1->trace = NULL;
    insertheap2(mazestart1);
    flag_success1 = 0;

    while (topheap2() != NULL)
    {
        tmpcell1 = topheap2();
        //  if (enable_print) printf("A* top a:%d [%d,%d] %d\n",a, tmpcell1->y,tmpcell1->x, mazeiteration1);
        if ((tmpcell1 == mazegoal1) || (cont_closed == lookahead))
        {
            //  open_size = opensize2() + open_size;

            //Se calcul f para actualizar h
            f_value = hvalues[MAZEWIDTH * tmpcell1->y + tmpcell1->x][a] + tmpcell1->g;
            cellpas = tmpcell1;
            for (d = 0; d < cont_closed; d++)
            {
                hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a] = max(
                    hvalues[MAZEWIDTH * CLOSED[d]->y + CLOSED[d]->x][a],
                    f_value - CLOSED[d]->g); //max(CLOSED[d]->h,f_value - CLOSED[d]->g);
            }
            flag_success1 = 1;
            tmpcell1 = popheap2();
            // if (enable_print) printf("A* top a:%d [%d,%d] %d\n",a, tmpcell1->y,tmpcell1->x, mazeiteration1);
            break;
        }
        tmpcell1 = popheap2();
        CLOSED[cont_closed] = tmpcell1;
        cont_closed++;
        tmpcell1->overexpanded = mazeiteration1;
        //tmpcell1->h = LARGE;
        statexpanded1++;

        ++agent_expansions[a];
        d = random() % DIRECTIONS;
        for (i = 0; i < DIRECTIONS; ++i)
        {
            //	if (enable_print) printf("d:%d\n",d);
            //	if (tmpcell1->move[d]) if (enable_print) printf("d:%d, %d %d %d\n",d, tmpcell1->move[d]->obstacle,tmpcell1->move[d]->blocked,tmpcell1->move[d]->overexpanded);
            if (tmpcell1->move[d] && (!tmpcell1->move[d]->obstacle) /*&& (!tmpcell1->move[d]->blocked)*/ &&
                tmpcell1->move[d]->overexpanded != mazeiteration1)
            {
                initialize_state(tmpcell1->move[d]);
                //        if (enable_print) printf("A* generation a:%d [%d,%d] %d\n",a, tmpcell1->move[d]->y,tmpcell1->move[d]->x, d);
                if (tmpcell1->move[d]->g > tmpcell1->g + tmpcell1->cost[d])
                {
                    tmpcell1->move[d]->g = tmpcell1->g + tmpcell1->cost[d];
                    tmpcell1->move[d]->searchtree = tmpcell1;
                    tmpcell1->move[d]->key = (tmpcell1->move[d]->g +
                                              hvalues[MAZEWIDTH * tmpcell1->move[d]->y + tmpcell1->move[d]->x][a]) *
                                                 BASE -
                                             tmpcell1->move[d]->g;
                    insertheap2(tmpcell1->move[d]);
                }
            }
            d = (d + 1) % DIRECTIONS;
        } /* end for */
    }     /* end while */
    int co = 0;
    if (flag_success1 == 1)
    {

        do
        {
            //	if (enable_print) printf("construyendo path :%d agente %d\n",++co,a+1);
            cellpas->trace = NULL; // tracing back a path from the goal back to the start
            while (cellpas != mazestart1)
            {
                parent = cellpas->searchtree;
                parent->trace = cellpas;
                cellpas = parent;
            }
            if (mazestart1->trace->blocked != 1)
                break;
            cellpas = popheap2();
        } while (topheap2() != NULL);
    } // end if (flag_success1 == 1)

    return (flag_success1);
}

void randommove(int a)
{
    cell1 *tmpcell1 = position[a];
    ++agent_expansions[a];
    d = random() % DIRECTIONS;

    int i;
    for (i = 0; i < DIRECTIONS; ++i)
    {
        if (tmpcell1->move[d] && (!tmpcell1->move[d]->obstacle) && (!tmpcell1->move[d]->blocked))
        {
            tmpcell1->blocked = 0;
            position[a] = tmpcell1->move[d];
            travel_distance[a]++;
            position[a]->blocked = 1;
            return;
        }
        d = (d + 1) % DIRECTIONS;
    } /* end for */
}

// Conserva al agente en su ubicación actual
void stay_in_place(int a)
{
    cell1 *tmpcell = position[a];
    tmpcell->blocked = 0;
    tmpcell->obstacle = 0;
}

/* ------------------------------------------------------------------------------*/
void test_rtaastar(int lookahead, int prunning)
{
    cell1 *tempcell, *previous, *current;
    int y, x, i, j, k;
    long int m;
    srand(time(NULL));

    // gettimeofday(&tv11c, NULL);
    newrandommaze_astar();
    // gettimeofday(&tv22c, NULL);
    // time_astar_initialize1 += 1.0 * (tv22c.tv_sec - tv11c.tv_sec) + 1.0 * (tv22c.tv_usec - tv11c.tv_usec) / 1000000.0;

    initialize_astar();
    current = mazestart1;
    finish_all = NAGENTS;
    time_step = 1;
    // Inicialización de metricas
    for (int a = 0; a < NAGENTS; a++)
    {
        travel_distance[a] = 0;
        completion_time[a] = 0;
    }
    while (1)
    {
        time_step++;
        for (i = 0; i < NAGENTS; i++)
        {
            if (RUN1 >= 0 && robot_steps1 >= 0) {
                if (enable_print) printf("Antes Agent[%d] A* Start [%d,%d] Goal [%d,%d] h:%f step:%lld time_step:%d terminado:%d\n", i + 1,
                       position[i]->y, position[i]->x, goal[i]->y, goal[i]->x, position[i]->h, robot_steps1, time_step,
                       NAGENTS - finish_all);
                if (i == NAGENTS - 1){
                    Multi_print_grid();
                    //getchar();
                    printf("\n");
                    sleep(1);
                }

                for (k = 0; k < NAGENTS; k++)
                    if (enable_print) printf("(%d)[%d,%d]", k + 1, position[k]->y, position[k]->x);
                if (enable_print) printf("\n");
                // getchar();
            }

            if (goal_reached[i])
            {
                //randommove(i);
                stay_in_place(i);
                // queda el primer time_step con el que logre llegar
                // if (completion_time[i] == 0) completion_time[i] = time_step;
                continue;
            } else {
                completion_time[i] = time_step;
            }
            

            if (!computeshortestpath_astar(i, lookahead))
            {
                //if (enable_print) printf("***********************************************************************\n");
                //if (enable_print) printf("*   A*  when mazeiteration1 = %d,    No path possible   !!!    *\n", mazeiteration1);
                //if (enable_print) printf("***********************************************************************\n");
                //return;
            }
            else
            {
                previous = position[i];

                if (position[i]->trace->blocked)
                {
                    // Esta rutina permite empujar los agentes fuera de su celda luego de PUSH_OVER_THRESHOLD tiempo en ella
                    // Si no me encuentro en mi goal
                    /* if (position[i] != goal[i])
                    {
                        // No me he movido desde la ultima vez
                        if (last_recently_see[i] != NULL &&
                            last_recently_see[i] == position[i])
                        {
                            // Me conservo en el lugar, entonces incrementa contador
                            push_over[i] = push_over[i] + 1;
                            if (push_over[i] > PUSH_OVER_THRESHOLD)
                            {
                                d = rand() % DIRECTIONS;
                                for (int _d = 0; _d < DIRECTIONS; _d++)
                                {
                                    if (!position[i]->succ[d]->obstacle &&
                                        !position[i]->succ[d]->blocked &&
                                        (int)position[i]->succ[d]->g)
                                    {
                                        printf("la celda [%d %d] esta libre desde [%d %d] para %d en dirección %d\n",
                                                    position[i]->succ[d]->y,
                                                    position[i]->succ[d]->x,
                                                    position[i]->y,
                                                    position[i]->x,
                                                    i + 1, d);

                                        push_over[i] = 0;
                                        position[i] = position[i]->succ[d];
                                        agent_cost[i] += euclidian(previous, position[i]);
                                        robot_steps1++;
                                        previous->blocked = 0;
                                        position[i]->blocked = 1;
                                        // position[i]->parent[i] = previous;
                                        push_out_count = push_out_count + 1;
                                        break;
                                    }
                                    d = (d + 1) % DIRECTIONS;
                                }
                            }
                        }
                        else
                        {
                            // Me moví desde la ultima vez, entonces reinicio
                            push_over[i] = 0;
                        }
                        // Ultima vez visto
                        last_recently_see[i] = previous;
                        for (int a = 0; a < NAGENTS; a++) {
                                printf("[%d]", push_over[a]);
                            }
                            printf("\n");
                    } */
                    continue;
                }

                position[i] = position[i]->trace;
                agent_cost[i] += euclidian(previous, position[i]);
                robot_steps1++;
                previous->trace = NULL;
                previous->blocked = 0;
                position[i]->blocked = 1;
                travel_distance[i]++;
                last_recently_see[i] = previous;

                //	if (RUN1 >= 2 && robot_steps1 >= 0){if (enable_print) printf("Angent[%d] A* Start [%d,%d] Goal [%d,%d] h:%f step:%d nei:%d\n",i,position[i]->y,position[i]->x,goal[i]->y,goal[i]->x,position[i]->h,robot_steps1,count_nei(position[i]));print_grid(position[i]->x,position[i]->y,position[i],goal[i]->x,goal[i]->y);getchar();}

                if (position[i] == goal[i])
                {
                    if (goal_reached[i] == 0)
                    {
                        lastfinish = time_step;
                        completion_time[i] = time_step;
                    }
                    goal_reached[i] = 1;
                    solution_cost += agent_cost[i];
                    // El ultimo no alcanza a escribirse dentro del if(goal_rached[i])
                    // por eso se escribe al acabar la ultima iteración.
                    finish_all--;
                    //	if (enable_print) printf("** LLEGO time_step:%d** %d finish:%d cost:%f total cost:%f\n",time_step,i,NAGENTS-finish_all,agent_cost[i],total_cost);
                }
                
                if (finish_all == 0 || time_step >= MAX_TIME_STEPS)
                {
                    enable_print = 1;
                    total_cost = 0;

                    FILE *fp;

                    fp = fopen("log-resultados", "a+");

                    fprintf(fp, "RUN %ld\n", RUN1);

                    fprintf(fp, "valores_ideales ");
                    fprintf(fp, "[");
                    for (int a = 0; a < NAGENTS; a++)
                    {
                        if (a == NAGENTS - 1)
                            fprintf(fp, "%d", 0);
                        else
                            fprintf(fp, "%d,", 0);
                    }
                    fprintf(fp, "]\n");

                    fprintf(fp, "costo_por_agente ");
                    fprintf(fp, "[");
                    for (int a = 0; a < NAGENTS; a++)
                    {
                        if (goal_reached[a])
                            total_cost += agent_cost[a];
                        if (a == NAGENTS - 1)
                            fprintf(fp, "%.0lf", agent_cost[a]);
                        else
                            fprintf(fp, "%.0lf,", agent_cost[a]);
                    }
                    float total_time_cost = 0;
                    fprintf(fp, "]\n");

                    fprintf(fp, "completion_time_por_agente ");
                    fprintf(fp, "[");
                    for (int a = 0; a < NAGENTS; a++)
                    {
                        total_time_cost += completion_time[a];
                        if (a == NAGENTS - 1)
                            fprintf(fp, "%d", completion_time[a]);
                        else
                            fprintf(fp, "%d,", completion_time[a]);
                    }
                    ("costo_promedio %f\n", total_cost / NAGENTS);
                    fprintf(fp, "]\n");

                    fprintf(fp, "tiempo_ultimo_agente_goal %d\n", lastfinish);
                    fprintf(fp, "tiempo_en_acabar %d\n", time_step);
                    fprintf(fp, "tiempo_promedio %f\n", total_time_cost / NAGENTS);
                    fprintf(fp, "agentes_en_goal %d\n", NAGENTS - finish_all);
                    fprintf(fp, "push_out_count %d\n", push_out_count);

                    fclose(fp);

                    /* total_cost = 0;
                        if (enable_print) printf("Costo por agente\n");
                        for (int a; a < NAGENTS; a++) {
                            total_cost += travel_distance[a];
                            if (enable_print) printf("agent [%d] -> costo total: %d\n", a + 1, travel_distance[a]);
                        }
                        total_time = 0;
                        if (enable_print) printf("Completion time por agente\n");
                        for (int a; a < NAGENTS; a++) {
                            total_time += completion_time[a];
                            if (enable_print) printf("agent [%d] -> tiempo total: %d\n", a + 1, completion_time[a]);
                        }
                        if (enable_print) printf("Costo promedio: %f\n", total_cost / NAGENTS);
                        if (enable_print) printf("Tiempo en acabar: %d\n", time_step);
                        if (enable_print) printf("Tiempo promedio: %f\n", total_time / NAGENTS);

                        getchar(); */

                    enable_print = 0;

                    return;
                }
            }
        }
        //		  i = (i+1) % NAGENTS;
    }
    //updatemaze1(previous,mazestart1);
    // end  while(finish_all)
    return;
}

//23677273--------------------------------------------------------------------------------------
void call_rtaastar()
{
    FILE *salida;
    long long int j;
    float time_astar = 0;
    float average_expansion_persearch = 0;
    float average_trace_persearch = 0;
    float variance_expansion_persearch = 0;
    float SDOM = 0;
    int lookahead;
    int prunning, i;
    //int look[9] = {1, 8, 16, 32, 64, 128, 256, 512, 1024};
    int look[1] = {3};

    for (prunning = 0; prunning < 1; prunning++)
    {
        lookahead = look[0];
        if (enable_print)
            printf("lookahead == [%d] ___________________________________\n", lookahead);
        for (RUN1 = 0; RUN1 < RUNS; ++RUN1)
        {
            if (enable_print)
                printf("case == [%ld] ___________________________________\n", RUN1);
            srand(time(NULL));
            generate_maze(RUN1);
            for (int i = 0; i < NAGENTS; i++)
                goal_reached[i] = 0;
            // gettimeofday(&tv11, NULL);
            test_rtaastar(lookahead, prunning);
            emptyheap3();
            // gettimeofday(&tv22, NULL);
            time_astar += 1.0 * (tv22.tv_sec - tv11.tv_sec) + 1.0 * (tv22.tv_usec - tv11.tv_usec) / 1000000.0;
            robotmoves_total1 += robot_steps1;
            lastfinish = -1000;
#ifdef STATISTICS
            if (times_of_billion1 > 0)
                average_expansion_persearch =
                    ((float)1000000000 / (float)searches_astar1 * (float)times_of_billion1) +
                    ((float)statexpanded1 / (float)searches_astar1);
            else
                average_expansion_persearch = ((float)statexpanded1 / (float)searches_astar1);
#endif

            if ((salida = fopen("Output-mrtaa-1-step", "a")) == NULL)
            {
                if (enable_print)
                    printf("No se puede abrir el archivo de salida");
            }
            fprintf(salida, "%d %f %d %d %lld %f %d %ld %lld %ld", lookahead, solution_cost, NAGENTS,
                    NAGENTS - finish_all, searches_astar1, (time_astar - time_astar_initialize1) * 1000,
                    time_step - 1, RUN1, statexpanded1, statpercolated2);

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

            //delay
            sleep(1);

        } // end for RUN1S
    }
    return;
}

#endif // end #
