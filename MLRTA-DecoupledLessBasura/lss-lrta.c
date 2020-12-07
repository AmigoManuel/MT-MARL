/////////////////////////////////////////////////////////////////////
// Carlos Hernandez & Xiaoxun Sun & Sven Koenig @ USC 2011
// All rights reserved
/////////////////////////////////////////////////////////////////////


#include "heap.h"
#include "maze.h"
#include "stdio.h"
#include "include.h"
#include "math.h"
#include "lss-lrta.h"


#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#ifdef TESTLSSLRTA
#define MAZEITERATIONS 1000000
unsigned long int targetdiscount[MAZEITERATIONS];
#define MAXSEARCHES 100000000
unsigned long int pathlengths[MAXSEARCHES];

// Xiaoxun: this is for 8-connected grids, with sqrt(2) as the cost for diagonal movements
#ifdef EIGHTCONNECTED
#define HA(from,to) (  (sqrt(2)) * min(abs((from)->y - (to)->y), abs((from)->x - (to)->x))) + ((1) * ( max(abs((from)->y - (to)->y), abs((from)->x - (to)->x )) - min( abs((from)->y - (to)->y), abs((from)->x - (to)->x )))     )
#else
#define HA(from,to) ((1)* ( abs( (from)->y - (to)->y  ) + abs( (from)->x - (to)->x )   ))
#endif
#define euclidian(from,to) (sqrt(pow(((from)->x-(to)->x),2.0) + pow(((from)->y-(to)->y),2.0)))

#define QUEUE2_SIZE 100000
#define QUEUE2_PUSH(x) {queue[pf2] = (x); (x)->heapindex = pf2; pf2 = (pf2+1)%QUEUE2_SIZE; n2++; }
#define QUEUE2_POP(x) {(x) = queue[pi2];(x)->heapindex = -1; pi2 = (pi2+1)%QUEUE2_SIZE; n2--; }


long int max_expansions = 0;
long int counter=0;
long int callclean=0;
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
int cont_total_open_connected = 0;
cell1 *CLOSED[100000];
double f_value = 0;

float time_astar_initialize1 = 0;
float time_astar_first1 = 0;
float  time_min_computer_astar=0;
float  time_max_computer_astar=0;
float  time_deadline=0;
float  time_searches=0;
float  standar_deviation_computer_astar=0;
float  standar_deviation_array[100000];
int sdIndex=0;


cell1 *tmpcell1;
cell1 *tmpcell2;
struct timeval   tv11, tv22 , tv11a, tv22a , tv11b, tv22b, tv11c, tv22c, tv11d, tv22d,tv11e, tv22e;

long numberofexp,hsal;




/*------------------------------f1------------------------------------------*/
void initialize_astar()
{
mazeiteration1 = 0;
robot_steps1 = 0;
keymodifier1 = 0;
statpercolated2 = 0;
emptyheap2();

}
/* -------------------------------f2-----------------------------------------------*/
void updatemaze1(cell1 *robot) /*  Update the obstacles in the maze1 , this is for A* search    */
{
cell1 *tmpcell;
int prevd, postd;

for (d1 = 0; d1 < DIRECTIONS; ++d1)
{
    if (robot->move[d1])
    {
        if(robot->move[d1]->obstacle)
        {
            tmpcell = robot->move[d1];
            for (d2 = 0; d2 < DIRECTIONS; ++d2)
            {
                if (tmpcell->move[d2])
                {
                    tmpcell->move[d2] = NULL;
                    tmpcell->succ[d2]->move[reverse[d2]] = NULL;   /* Block the both directions */
                }
            }
        }else
        {
            if(d1 % 2 != 0)
            {
                prevd = d1-1;
                if (d1 == 7) postd = 0; else postd = d1+1;
                if(robot->succ[prevd]->obstacle || robot->succ[postd]->obstacle)
                {
                    //printf("  Antes eliminar   !!!    *\n");
                    robot->move[d1] = NULL;
                    robot->succ[d1]->move[reverse[d1]] = NULL;   /* Block the both directions */
                    //printf("  Despues eliminar   !!!    *\n");

                }
            }
        }
    }
}// end for (d1)
return;
}



//------------------------------------------------------------------------------------------------------

int count_nei(cell1 *auxcell)
{
int cont = 0;
for (d2 = 0; d2 < DIRECTIONS; ++d2)
    if (auxcell->move[d2] != NULL)
        cont++;
return cont;
}


void print_grid(int posx, int posy, cell1  *rtastart)
{
int x, y,x0,xf,y0,yf,w;
long int num1,num2,num;
w=12;
if (posx - w > 0) x0 =  posx - w; else x0 =  0;
if (posy - w > 0) y0 =  posy - w; else y0 =  0;
if (posx + w < MAZEWIDTH) xf =  posx + w; else xf =  MAZEWIDTH;
if (posy + w < MAZEHEIGHT) yf =  posy + w; else yf =  MAZEHEIGHT;

    for (y = y0; y < yf; ++y)
     {
       for (x = x0; x < xf; ++x)
             {
              if(maze1[y][x].obstacle==1) {printf("\e[42m");printf("%8s","#");printf("\e[0m");}
              else if((maze1[y][x].obstacle==0)&&(!((x==mazegoal1->x)&&(y==mazegoal1->y)))&&(!((x==rtastart->x)&&(y==rtastart->y))))
              {
                  tmpcell1 = &maze1[y][x];
                  if (tmpcell1->overexpanded == mazeiteration1)
                    //{printf("\e[33m"); printf("%8d",count_nei(tmpcell1));printf("\e[0m");}
                    {printf("\e[33m"); printf("%8.1f",/*count_nei(tmpcell1)*/tmpcell1->h);printf("\e[0m");}
                  else
                    //{printf("\e[31m"); printf("%8d",count_nei(tmpcell1));printf("\e[0m");}
                    {printf("\e[31m"); printf("%8.1f",/*count_nei(tmpcell1)*/tmpcell1->h);printf("\e[0m");}
              }
              else{
               if((x==mazegoal1->x)&&(y==mazegoal1->y)){printf("\e[43m");printf("%8s","G");printf("\e[0m");}
               if((x==rtastart->x)&&(y==rtastart->y)){printf("\e[44m"); printf("%8s","S");printf("\e[0m");}
               }
               }
    printf("\n");
         }
}
//------------------------------------------------------------------------------------------------------

/*
void initialize_state(cell1 *tmpcell)
{
if(tmpcell->iteration == 0)
    {
        tmpcell->g = LARGE;   /////
        tmpcell->h = HA(tmpcell, mazegoal1);
    }
    else if(tmpcell->iteration != mazeiteration1)
    {
        if(tmpcell->g + tmpcell->h < pathlengths[tmpcell->iteration])
             tmpcell->h = pathlengths[tmpcell->iteration] - tmpcell->g;
        tmpcell->g = LARGE;
    }
tmpcell->iteration = mazeiteration1;
return;
}
*/

void initialize_state(cell1 *tmpcell)
{
if(tmpcell->iteration == 0)
 {
     tmpcell->g = LARGE;   /////
     tmpcell->h = HA(tmpcell, mazegoal1);
 }
else if(tmpcell->iteration != mazeiteration1)
 {
     tmpcell->g = LARGE;
 }
tmpcell->iteration = mazeiteration1;
return;
}

void clean_edges(cell1 *parent, cell1 *child)
{
for (d2 = 0; d2 < DIRECTIONS; ++d2)
    if (parent->move[d2] && ((parent->move[d2] != child) == (parent->move[d2] != parent->searchtree)))
    {
        //printf("(%d,%d)[%d]\n",parent->y,parent->x,d2);
        parent->move[d2] = NULL;
        parent->succ[d2]->move[reverse[d2]] = NULL;
    }
}

void connect_edge(cell1 *parent, cell1 *child)
{
for (d2 = 0; d2 < DIRECTIONS; ++d2)
    if (parent->succ[d2] == child)
    {
        parent->move[d2] = child;
        break;
    }
}

void update_cell_connectivity(cell1 *auxcell)
{
while(auxcell != mazestart1)
{
   /* if(auxcell->searchtree->belong_tree == mazeiteration1)
    {
        connect_edge(auxcell->searchtree,auxcell);
        break;
    }
    clean_edges(auxcell->searchtree,auxcell);*/
    //if (auxcell->searchtree->h > auxcell->h + euclidian(auxcell->searchtree,auxcell)) auxcell->searchtree->h = auxcell->h + euclidian(auxcell->searchtree,auxcell);
    auxcell = auxcell->searchtree;
    auxcell->belong_tree = mazeiteration1;
    auxcell->revised = mazeiteration1;
}
}


void update_connectivity()
{
cell1 *auxcell;
while (topheap2() != NULL)
{
    auxcell = popheap2();
    print_grid(auxcell->x,auxcell->y,auxcell);getchar();
    update_cell_connectivity(auxcell);
}

}


void update_cells(double w)
{
cell1 *cellpas;

//OPEN to OPEN-Dijsktra
emptyheap3();
for (i = 1; i <= sizeheap2(); ++i)
{
    cellpas = posheap2(i);
    cellpas->key3 = cellpas->h;
    insertheap3(cellpas);
}

//Update h
while (topheap3() != NULL)
{
    cellpas = popheap3();
    ++statexpanded1;

    for (d = 0; d < DIRECTIONS; ++d)
    {
        if ((cellpas->move[d]) && (cellpas->move[d]->overexpanded == mazeiteration1) && (cellpas->move[d]->h > cellpas->h + cellpas->cost[d])){
            cellpas->move[d]->h = cellpas->h + 8 * cellpas->cost[d];
            cellpas->move[d]->key3 = cellpas->move[d]->h;
            insertheap3(cellpas->move[d]);
        }
    }
}

}

void remove_cell(cell1 *auxcell)
{
/*for (d2 = 0; d2 < DIRECTIONS; ++d2)
{
    if (auxcell->move[d2])
    {
        auxcell->move[d2] = NULL;
        auxcell->succ[d2]->move[reverse[d2]] = NULL;   // Block the both directions
    }
}*/
auxcell->obstacle = 1;
auxcell->overexpanded = 0; //for update
}

void remove_update_cells()
{
for (d = 0; d < cont_closed; d++)
{
    if (CLOSED[d]->revised != mazeiteration1)
        remove_cell(CLOSED[d]);
    //else
        //CLOSED[d]->h = max(CLOSED[d]->h,f_value - CLOSED[d]->g);
}

}


void mark_revised(cell1 *auxcell)
{
while(auxcell != mazestart1)
{
    auxcell = auxcell->searchtree;
    auxcell->revised = mazeiteration1;
}
}


cell1* connected_components(cell1 *auxcell)
{
cell1 *queue[QUEUE2_SIZE], *cellreturn;
int pi2=0, pf2=0, n2=0;
double ming = LARGE,maxg = 0;

cont_connected = 1;
QUEUE2_PUSH(auxcell);auxcell->inqueue = mazeiteration1;
while (n2)
{
    QUEUE2_POP(auxcell);auxcell->inqueue = 0;
    auxcell->revised = mazeiteration1;
    if (auxcell->g < ming) {ming = auxcell->g;cellreturn = auxcell;} //calculate cell to connect
    //if (auxcell->g > maxg) {maxg = auxcell->g;cellreturn = auxcell;} //calculate cell to connect

    statexpanded1_prune++;
    for (d = 0; d < DIRECTIONS; ++d)
        if (auxcell->move[d] && auxcell->move[d]->frontier == mazeiteration1 && auxcell->move[d]->revised != mazeiteration1 && auxcell->move[d]->inqueue != mazeiteration1)
        {
            QUEUE2_PUSH(auxcell->move[d]);auxcell->move[d]->inqueue = mazeiteration1;
            cont_connected++;
        }
}
return cellreturn;
}


void prune()
{
cell1 *auxcell,*celltoconnect;
int end = sizeheap2(),i;
for (i = 1; i <= end;i++)
{
    auxcell = posheap2(i);
    if (auxcell->revised != mazeiteration1)
    {
        celltoconnect = connected_components(auxcell);
        if (i==1 && cont_connected == end) {cont_total_open_connected++;break;}
        //update_cell_connectivity(celltoconnect);
        mark_revised(celltoconnect);

    }
}
remove_update_cells();
}


//-------------------------------------------------------------------------------------------------------

int computeshortestpath_astar(int lookahead, double w)
{
long valuehpath,j,minimo;
int variant1,sal,i;
cell1 *cellminimo, *cellpas, *parent;
int pi = 0, pf = 0;
double ch,oh,difh,mindif,f;
time_deadline = 0;


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
insertheap2(mazestart1);
mazestart1->frontier = mazeiteration1;
flag_success1 = 0;

while (topheap2() != NULL)
{
    tmpcell1 = topheap2();
    if ((tmpcell1 == mazegoal1)|| (cont_closed == lookahead))
    {
         open_size = opensize2() + open_size;

        //Se calcul f para actualizar h
        f_value = tmpcell1->h + tmpcell1->g;
        cellpas = tmpcell1;
        update_cells(w);
        //update_connectivity();
        //print_grid(cellpas->x,cellpas->y,cellpas);getchar();

        //Se seleciona next state

        //ALSS-LRTA*
      /*
         tmpcell2 = tmpcell1;
          while (tmpcell1->h != HA(tmpcell1,mazegoal1))
        {
            if (topheap2() == NULL)
            {
                tmpcell1 = tmpcell2;
                break;
            }
            tmpcell1 = popheap2();
        }
      */

        //DALSS-LRTA*

        /*	ch = tmpcell1->h;oh = HA(tmpcell1,mazegoal1);difh = ch - oh;
            mindif = difh;
            while ((topheap2() != NULL) && (mindif !=0))
            {
                tmpcell2 = popheap2();
                ch = tmpcell2->h;oh = HA(tmpcell2,mazegoal1);difh = ch - oh;
                if (difh<mindif)
                {
                    tmpcell1 = tmpcell2;
                    mindif = difh;
                }
            }
            cellpas = tmpcell1;*/

        flag_success1 = 1;
        break;
    }
    tmpcell1 = popheap2();
    tmpcell1->frontier = 0;
    CLOSED[cont_closed] = tmpcell1;
    cont_closed++;
    tmpcell1->overexpanded = mazeiteration1;
    tmpcell1->interior = mazeiteration1;
    tmpcell1->h = LARGE;


    ++statexpanded1;
    for (d = 0; d < DIRECTIONS; ++d)
    {
        if (tmpcell1->move[d] && (!(tmpcell1->move[d]->obstacle)) && tmpcell1->move[d]->overexpanded != mazeiteration1)
        {
            initialize_state(tmpcell1->move[d]);
            if(tmpcell1->move[d]->g > tmpcell1->g + tmpcell1->cost[d])
            {
                tmpcell1->move[d]->g = tmpcell1->g + tmpcell1->cost[d];
                tmpcell1->move[d]->searchtree = tmpcell1;
                tmpcell1->move[d]->key = (tmpcell1->move[d]->g + tmpcell1->move[d]->h) * BASE - tmpcell1->move[d]->g;
                tmpcell1->move[d]->frontier = mazeiteration1;
                insertheap2(tmpcell1->move[d]);

            }
        }
    }/* end for */
} /* end while */

if (flag_success1 == 1)
{
    cellpas->trace = NULL;   // tracing back a path from the goal back to the start
    while(cellpas != mazestart1)
    {
        //printf("construyendo path\n");
        parent = cellpas->searchtree;
        parent->trace = cellpas;
        cellpas = parent;
    }
}  // end if (flag_success1 == 1)


return(flag_success1);
}


/* ------------------------------------------------------------------------------*/
void test_lss_lrta(int lookahead, double w)
{
cell1 *tempcell,*previous,*current;
int y,x;
long int m;
gettimeofday(&tv11c, NULL);
newrandommaze_astar();
for (y = 0; y < MAZEHEIGHT; ++y)
    for (x = 0; x < MAZEWIDTH; ++x)
          if(maze1[y][x].obstacle==0) {updatemaze1(&maze1[y][x]);}



gettimeofday(&tv22c, NULL);
time_astar_initialize1   += 1.0*(tv22c.tv_sec - tv11c.tv_sec) + 1.0*(tv22c.tv_usec - tv11c.tv_usec)/1000000.0;
initialize_astar();
current = mazestart1;
while(current != mazegoal1)
{

    if ((current->trace == NULL) || (current->trace->obstacle))
    {
        gettimeofday(&tv11e, NULL);

        if (w == 3.0) prune();
        update_cells(w);

        mazestart1 = current;
    //	if (RUN1 >= 0){printf("ANTES A* Start [%d,%d] Goal [%d,%d] h:%f step:%d nei:%d\n",mazestart1->y,mazestart1->x,mazegoal1->y,mazegoal1->x,mazestart1->h,robot_steps1,count_nei(mazestart1));print_grid(mazestart1->x,mazestart1->y,mazestart1);getchar();}


        if(!computeshortestpath_astar(lookahead,w))
        {
             printf("***********************************************************************\n");
             printf("*   A*  when mazeiteration1 = %d,    No path possible   !!!    *\n", mazeiteration1);
             printf("***********************************************************************\n");
             return;
         }
        gettimeofday(&tv22e, NULL);
        time_searches   += 1.0*(tv22e.tv_sec - tv11e.tv_sec) + 1.0*(tv22e.tv_usec - tv11e.tv_usec)/1000000.0;

     }
    previous = current;
    current = current->trace;
    solution_cost += euclidian(previous,current);
    robot_steps1++;
    //if (RUN1 >= 9){printf("DESPUES A* Start [%d,%d] Goal [%d,%d] h:%f step:%d nei:%d\n",current->y,current->x,mazegoal1->y,mazegoal1->x,current->h,robot_steps1,count_nei(current));print_grid(current->x,current->y,current);getchar();}
    previous->trace = NULL;
    //updatemaze1(previous,mazestart1);
}// end  while(mazestart1 != mazegoal1)
return;
}



//23677273--------------------------------------------------------------------------------------
void call_lss_lrta()
{
FILE *salida;
long long int j;
float time_astar = 0;
float average_expansion_persearch = 0;
float average_trace_persearch = 0;
float variance_expansion_persearch = 0;
float SDOM = 0;
int lookahead;
double w;

for (lookahead = 1; lookahead <515; lookahead = lookahead *2)
for (w = 1.0; w < 4; w = w * 3.0)
{
printf("lookahead == [%d] ___________________________________\n",lookahead);
for (RUN1 = 0; RUN1 < RUNS; ++RUN1)
{
    printf("case == [%d] ___________________________________\n",RUN1);
    srand(5*RUN1 + 100);
    generate_maze();
gettimeofday(&tv11, NULL);
                test_lss_lrta(lookahead,w);
gettimeofday(&tv22, NULL);
time_astar   += 1.0*(tv22.tv_sec - tv11.tv_sec) + 1.0*(tv22.tv_usec - tv11.tv_usec)/1000000.0;
 robotmoves_total1+= robot_steps1;
#ifdef STATISTICS
if(times_of_billion1 > 0)
    average_expansion_persearch = ((float)1000000000 / (float)searches_astar1 * (float)times_of_billion1) + ((float)statexpanded1 / (float)searches_astar1);
else
    average_expansion_persearch =  ((float)statexpanded1 / (float)searches_astar1);
#endif


if((salida=fopen("Output-rtaa","a"))==NULL){printf("No se puede abrir el archivo de salida");}
fprintf (salida, "%d %f %f %d %d %f %d %d %d %d %d %d",lookahead,w,solution_cost,robotmoves_total1,searches_astar1,(time_astar - time_astar_initialize1)*1000,cont_total_open_connected,statexpanded1_prune,RUN1,statexpanded1,statpercolated2);

fputs("\n",salida);
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
cont_total_open_connected = 0;

}// end for RUN1S
}//end lookahead
return;
}


#endif // end #
