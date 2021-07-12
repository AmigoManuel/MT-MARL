/////////////////////////////////////////////////////////////////////
// Xiaoxun Sun & Sven Koenig @ USC 2009
// All rights reserved
/////////////////////////////////////////////////////////////////////

#include "stdio.h"
#include "include.h"
#include "heap.h"
#include "maze.h"
#include "rtaastar.h"
#include "lss-lrta.h"
#include "math.h"

#include <ctype.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>

#ifdef EIGHTCONNECTED
#define HA(from,to) (  (14) * min(abs((from)->y - (to)->y), abs((from)->x - (to)->x))) + ((10) * ( max(abs((from)->y - (to)->y), abs((from)->x - (to)->x )) - min( abs((from)->y - (to)->y), abs((from)->x - (to)->x )))     )
#else
#define HA(from, to) ((10)* ( abs( (from)->y - (to)->y  ) + abs( (from)->x - (to)->x )   ))
#endif

char map_path[50] = "./GameMaps/test88.map2";
char locations_path[50] = "./GameMaps/test88.loc2";

/* Lectura de mapa desde fichero externo */
void read_gamemap(const char *filename) {
    FILE *f;
    int count;
    int y, x;
    // Abre el archivo
    f = fopen(filename, "r");
    // Si no encuentra el archivo
    if (f == NULL) {
        printf("No se puede abrir el fichero.\n");
        exit(1);
    }
    //rewind (f);
    // Itera por el alto y ancho definidos del laberinto

    for (y = 0; y < MAZEHEIGHT; y++) {
        for (x = 0; x < MAZEWIDTH; x++) {
            // Lee una entrada del fichero
            char what;
            count = fscanf(f, "%c", &what);
            // Identifica que hay en esa casilla
            // toupper convierte un lowercase a uppercase
            switch (toupper(what)) {
                case 'O':
                case 'S':
                case 'W':
                case 'T':
                case '@':
                    // O, S, W, T e Y los marca como obstacle = 1
                    maze1[y][x].obstacle = 1;
                    break;
                default:
                    // Otra entrada la marca como obstacle = 0
                    maze1[y][x].obstacle = 0;
                    break;
            }
            // TODO: Ver que significa overexpanded
            maze1[y][x].overexpanded = 0;
        }
        count = fscanf(f, "\n");
    }
    fclose(f);
}

// Lectura de fichero locations para los agentes
int **read_agents_locations(const char *filename) {
    char param[50];
    int value;
    int a;

    int **agent_locations;
    agent_locations = malloc(sizeof(int *) * NAGENTS);
    for (int i = 0; i < NAGENTS; i++) {
        agent_locations[i] = malloc(sizeof(int *) * 4);
    }

    FILE *fp = NULL;
    fp = fopen(filename, "r+");

    if (fp != NULL) {
        while (fscanf(fp, "%s", param) == 1) {
            if (fscanf(fp, "%d", &value) == 1) {
                if (strcmp(param, "a") == 0)
                    a = value;
                    if (a == NAGENTS) break;
                else if (strcmp(param, "goaly") == 0)
                    agent_locations[a][0] = value;
                else if (strcmp(param, "goalx") == 0)
                    agent_locations[a][1] = value;
                else if (strcmp(param, "starty") == 0)
                    agent_locations[a][2] = value;
                else if (strcmp(param, "startx") == 0)
                    agent_locations[a][3] = value;
            }
        }
    }
    fclose(fp);
    fp = NULL;
    return agent_locations;
}

/* Utils para cola */
#define QUEUE_SIZE 600000
#define QUEUE_PUSH(y, x) { queuey[pf1] = (y); queuex[pf1] = (x); pf1 = (pf1+1)%QUEUE_SIZE; n++; }
#define QUEUE_POP(y, x) { (y) = queuey[pi1];(x) = queuex[pi1]; pi1 = (pi1+1)%QUEUE_SIZE; n--; }

/* Determina si el movimiento es valido */
int IsValid(int y, int x, int y0, int x0, int d) {
    //int yprevd, xprevd, ypostd, xpostd;
    // Verifica si x e y se encuentran entre MAZEWIDTH y MAZEHEIGHT
    if ((y < 0) || (y >= MAZEHEIGHT) || (x < 0) || (x >= MAZEWIDTH)) return 0;
    if (maze1[y][x].obstacle) return 0;
    // TODO: Preguntar sobre los movimientos en diagonal y remover codigo de abajo
    /*if(d % 2 != 0) { //elimina movimientos diagonales no posibles
		yprevd = y0 + dy[d-1];xprevd = x0 + dx[d-1];
		if (d == 7) {ypostd = y0; xpostd = x0 + 1;} else{ypostd = y0 + dy[d+1]; xpostd = x0 + dx[d+1];}
				
		if(maze1[yprevd][xprevd].obstacle || maze1[ypostd][xpostd].obstacle)
			return 0;
	}*/
    return 1;
}

/* TODO: Verificar bien que hace esta función */
/* Al parecer verifica si el siguiente movimiento no se ha realizado antes */
int isnewproblem(int sy, int sx, int gy, int gx, int cont) {
    int i;
    // printf("cont:%d\n",cont);
    // Por cada elemento en el contador
    for (i = 0; i < cont; i++) {
        // Si la posición de destino "position[i]->y/x" es igual a la de origen "y/x"
        // No es un nuevo problema
        if (position[i]->y == sy && position[i]->x == sx) return 0;
        // Si la posición de destino goal es igual a la de origen 
        // No es un nuevo problema
        if (goal[i]->y == gy && goal[i]->x == gx) return 0;
    }
    // Caso contrario si es nuevo problema
    return 1;
}

/* Generate maze with solution */
/* Inicializa el mapa con los valores por defecto */
void generate_maze(int RUN1) {
    int x, y, out = 1, porc;
    // Instancia las colas
    int queuey[QUEUE_SIZE], queuex[QUEUE_SIZE], pi1 = 0, pf1 = 0, n = 0, i;
    int goaly, goalx, startx, starty;
    // printf("entro\n");
    
    
    // Define instancia de laberinto
    if (maze1 == NULL) {
        // Rerva espacio en memoria
        maze1 = (cell1 **) calloc(MAZEHEIGHT, sizeof(cell1 *));
        for (y = 0; y < MAZEHEIGHT; ++y) {
            maze1[y] = (cell1 *) calloc(MAZEWIDTH, sizeof(cell1));
            for (x = 0; x < MAZEWIDTH; ++x) {
                // Enumera las celdas en orden creciente
                maze1[y][x].y = y;
                maze1[y][x].x = x;
            }
        }
    }
    /* // TODO: Ver que significa exactamente un ifdef y ifndef
    #ifdef RANDOMMAZE
    #ifndef GAMEMAP
    // Por cada casilla
    for (y = 0; y < MAZEHEIGHT; ++y) {
        for (x = 0; x < MAZEWIDTH; ++x) {
            // Marca overexpanded y obstacle como 0
            maze1[y][x].overexpanded = 0;
            // if(maze1[y][x].obstacle == 0)
            maze1[y][x].obstacle = 0;
            // Luego asgina un valor 0 o 1 al obstaculo
            // este depende de MAZEDENSITY
            // por defecto es 0.2
            // Este es el pocentaje de celdas bloqueadas cuando se define randommaze
            maze1[y][x].obstacle = (random() % 10000 < 10000 * MAZEDENSITY);
        }
    }
    #else */

    //read_gamemap("../../../../Conferences/Maps/GameMapswc3/darkforest.map2");
    //read_gamemap("./mapa_prueba2.map2");
    //read_gamemap("./mapa_bloques.map2");
    //read_gamemap("./GameMaps/den520d.map2");
    //read_gamemap("./GameMaps/ost003d.map2");
    //read_gamemap("./GameMaps/brc202d.map2");
    read_gamemap(map_path);

    // FIXME: Nunca utiliza la variable porc
    porc = (MAZEWIDTH * 0.1);

    // Dentro de agent_locations se almacenan los goals y starts de cada agente
    int **agent_locations = read_agents_locations(locations_path);

    // Itera sobre cada agente
    for (int a = 0; a < NAGENTS; a++) {
        out = 1;
        while (out) {
            // FIXME: No utiliza pil ni pf1
            n = 0, pi1 = 0, pf1 = 0;
            // En todas las casillas marca overexpanded = 0 y g = 0
            for (y = 0; y < MAZEHEIGHT; ++y) {
                for (x = 0; x < MAZEWIDTH; ++x) {
                    maze1[y][x].overexpanded = 0;
                    maze1[y][x].g = 0;
                }
            }
            while (1) {
                /*goaly = (random() % ((MAZEHEIGHT + 1) / 2)) * 2;
                goalx = (random() % ((MAZEWIDTH + 1) / 2)) * 2;
                starty = (random() % ((MAZEHEIGHT + 1) / 2)) * 2;
                startx = (random() % ((MAZEWIDTH + 1) / 2)) * 2;*/
                
                // Determina un inicio y un goal aleatorios
                /* goaly = random() % MAZEHEIGHT;
                goalx = random() % MAZEWIDTH;
                starty = random() % MAZEHEIGHT;
                startx = random() % MAZEWIDTH; */
                goaly = agent_locations[a][0];
                goalx = agent_locations[a][1];
                starty = agent_locations[a][2];
                startx = agent_locations[a][3];
                // Si el goal no es un obstaculo,
                // el inicio no es un obstaculo,
                // el inicio_x/y no esta en goal_x/y y
                // se cumple que isnewproblem
                if ((maze1[goaly][goalx].obstacle == 0) &&
                    (startx != goalx || starty != goaly) &&
                    (maze1[starty][startx].obstacle == 0) &&
                    isnewproblem(starty, startx, goaly, goalx, a)
                    /*&& (HA(&maze1[starty][startx],&maze1[goaly][goalx]) > 4000)*/)
                    break;
            }

            //	 printf("starty:%d startx:%d goaly:%d goalx:%d\n",starty,startx,goaly,goalx);

            // Marca el inicio y objetivo sin obstaculo
            maze1[starty][startx].obstacle = 0;
            maze1[goaly][goalx].obstacle = 0;
            // targetall = &maze1[goaly][goalx];
            // robotall  = &maze1[starty][startx];
            // Find a solution with BFS
            //   printf("antes y:%d x:%d\n",y,x);
            //   queuey[pf1] = starty; queuex[pf1] = startx;pf1++; maze1[y][x].overexpanded = 1;
            QUEUE_PUSH(starty, startx);
            maze1[starty][startx].overexpanded = 1;
            maze1[starty][startx].g = 0;
            while (n > 0) {
                QUEUE_POP(y, x);
                //maze1[y][x].overexpanded = 1;
                //	      if (RUN1 >= 7 && a == 4)   printf("y:%d x:%d n:%d overexpanded:%d\n",y,x,n,maze1[y][x].overexpanded);
                for (i = 0; i < DIRECTIONS; ++i) {
                    if ((/*IsValid(y + dy[i], x + dx[i])*/IsValid(y + dy[i], x + dx[i], y, x, i)) &&
                        (maze1[y + dy[i]][x + dx[i]].overexpanded == 0)) {
                        if ((((y + dy[i]) == goaly) && ((x + dx[i]) == goalx))) {
                            out = 0;
                            break;
                        }
                        QUEUE_PUSH(y + dy[i], x + dx[i]);
                        maze1[y + dy[i]][x + dx[i]].overexpanded = 1;
                        maze1[y + dy[i]][x + dx[i]].g = maze1[y][x].g + 1;
                        //				     if (RUN1 >= 7 && a == 4) {printf("HIJO[%d,%d] Goal[%d,%d]Over:%d Valido:%d g:%f\n",y + dy[i],x + dx[i], goaly,goalx,maze1[y + dy[i]][x + dx[i]].overexpanded,IsValid(y + dy[i], x + dx[i],y,x,i),maze1[y + dy[i]][x + dx[i]].g);getchar();}
                    }
                }
            }
            //if (out == 1)  printf("no pillo\n");else printf("pillo %d\n",i);//getchar();
        }
        position[a] = &maze1[starty][startx];
        goal[a] = &maze1[goaly][goalx];
        printf("a:%d despues starty:%d startx:%d goaly:%d goalx:%d\n", a, position[a]->y, position[a]->x, goaly, goalx);
    }
}

/*----------------------------------------------------------------------------------*/

int main(int argc, char *argv[]) {

    // map y locations mediante parametros
    if (argc == 3) {
        strcpy(map_path, argv[1]);
        strcpy(locations_path, argv[2]);
    }
    remove("log-resultados");

    #ifdef UNKNOWN
    #ifdef DECREASE
        printf("Error!   Unknown maze is not allowed to decrease yet \n");
        return (0);
    #endif
    #endif

    #ifdef EIGHTCONNECTED
    #ifndef RANDOMMAZE
        printf("Error!  8-connected grids only, no 8-connected MAZE\n");
        return (0);
    #endif
    #endif

    #ifdef TESTRTAASTAR
        call_rtaastar();
    #endif
    #ifdef TESTLSSLRTA
        call_lss_lrta();
    #endif
}
