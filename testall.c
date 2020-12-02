#include "testall.h"

#include <ctype.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "agent.h"
#include "heap.h"
#include "include.h"
#include "lss-lrta.h"
#include "math.h"
#include "maze.h"
#include "rtaastar.h"
#include "stdio.h"
#include "rtaastar.h"

#ifdef EIGHTCONNECTED
#define HA(from,to) (  (14) * min(abs((from)->y - (to)->y), abs((from)->x - (to)->x))) + ((10) * ( max(abs((from)->y - (to)->y), abs((from)->x - (to)->x )) - min( abs((from)->y - (to)->y), abs((from)->x - (to)->x )))     )
#else
#define HA(from, to) ((10)* ( abs( (from)->y - (to)->y  ) + abs( (from)->x - (to)->x )   ))
#endif

/* Lectura de mapa desde fichero externo 
 * @param filename string con el nombre del fichero */
void read_gamemap(const char *filename) {
    FILE *fp;
    int y, x, w, z;
    // Abre el archivo
    fp = fopen(filename, "r");
    // Si no encuentra el archivo
    if (fp == NULL) {
        printf("No se puede abrir el fichero.\n");
        exit(1);
    }
    // Por cada casilla en el mapa
    for (y = 0; y < MAZEHEIGHT; y++) {
        for (x = 0; x < MAZEWIDTH; x++) {
            for (int i = 0; i < NAGENTS; i++) {
                //maze1[y][x].velx[i]=0;
                //maze1[y][x].vely[i]=0;
                for (w = 0; w < MAZEHEIGHT; w++) {
                    for (z = 0; z < MAZEWIDTH; z++) {
                        for (int t = 0; t < 100; t++) {
                            //	betweenTransition[x][y][z][w][i][t]=0;
                        }
                    }
                }
            }
            // Lee una entrada del fichero
            char what;
            fscanf(fp, "%c", &what);
            // Identifica que hay en esa casilla y toupper convierte un lowercase a uppercase
            switch (toupper(what)) {
                case 'O':
                case 'S':
                case 'W':
                case 'T':
                case '@': // O, S, W, T e Y los marca como obstacle = 1
                    maze1[y][x].obstacle = 1;
                    break;
                default: // Otra entrada la marca como obstacle = 0
                    maze1[y][x].obstacle = 0;
                    break;
            }
            maze1[y][x].overexpanded = 0; // Marca la casilla como sin expandir
        }
        fscanf(fp, "\n");
    }
    fclose(fp);
}

/* Utils para cola */
#define QUEUE_SIZE 600000
#define QUEUE_PUSH(y, x) { queuey[pf1] = (y); queuex[pf1] = (x); pf1 = (pf1+1)%QUEUE_SIZE; n++; }
#define QUEUE_POP(y, x) { (y) = queuey[pi1];(x) = queuex[pi1]; pi1 = (pi1+1)%QUEUE_SIZE; n--; }

/* Determina si el movimiento es valido 
 * @param y0 posición de origen en y
 * @param x0 posición de origen en x
 * @param y posición de destino en y
 * @param x posición de destino en x
 * @param d dirección en la que se realiza el movimiento
 * @return boolean - Si el movimiento es valido o no */
int IsValid(int y, int x, int y0, int x0, int d) {
    // FIXME: Eliminar esta linea
    // int yprevd, xprevd, ypostd, xpostd;
    // Verifica si x e y se encuentran entre MAZEWIDTH y MAZEHEIGHT
    if ((y < 0) || (y >= MAZEHEIGHT) || (x < 0) || (x >= MAZEWIDTH)) return 0;
    if (maze1[y][x].obstacle) return 0;
    // No se permiten movimientos diagonales
    /*	if(d % 2 != 0) { //elimina movimientos diagonales no posibles
		yprevd = y0 + dy[d-1];
        xprevd = x0 + dx[d-1];
		if (d == 7) {
            ypostd = y0;
            xpostd = x0 + 1;
        }else{
            ypostd = y0 + dy[d+1];
             xpostd = x0 + dx[d+1];
        }		
		if(maze1[yprevd][xprevd].obstacle || maze1[ypostd][xpostd].obstacle)
			return 0;
	}*/
    return 1;
}

/* Verifica si el siguiente movimiento no se ha realizado antes
 * @param sy el valor de origen en y
 * @param sx el valor de origen en x
 * @param gy el valor de destino en y
 * @param gx el valor de destino en x
 * @param cont contador 
 * @return boolean - Si el problema es nuevo o no */
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


    #ifdef RANDOMMAZE
    #ifndef GAMEMAP

    // Por cada casilla
    for (y = 0; y < MAZEHEIGHT; ++y) {
        for (x = 0; x < MAZEWIDTH; ++x) {
            // Marca overexpanded y obstacle como 0
            maze1[y][x].overexpanded = 0;
            // if(maze1[y][x].obstacle == 0)
            maze1[y][x].obstacle = 0;
            // TODO: Consultar significado de velx/vely
            maze1[y][x].velx=0;
            maze1[y][x].vely=0;
            // Luego asgina un valor 0 o 1 al obstaculo
            // este depende de MAZEDENSITY
            // por defecto es 0.2
            // Este es el pocentaje de celdas bloqueadas cuando se define randommaze
            maze1[y][x].obstacle = (random() % 10000 < 10000 * MAZEDENSITY);
        }
    }
    #else
    
    /* Mapas de prueba */
    //read_gamemap("../../../../Conferences/Maps/GameMapswc3/darkforest.map2");
    //read_gamemap("./mapa_prueba2.map2"); 
    //read_gamemap("./mapa_bloques.map2");
    //read_gamemap("./GameMaps/den520d.map2");
    read_gamemap("./GameMaps/test6.map2");
    //read_gamemap("./GameMaps/ost003d.map2");
    //read_gamemap("./GameMaps/brc202d.map2");
    //read_gamemap("./GameMaps/lak201d.map2");
    //read_gamemap("./GameMaps/bidi.map2");
    
    #endif
    #endif
    // FIXME: Nunca utiliza la variable porc
    porc = (MAZEWIDTH * 0.1);
    for (int a = 0; a < NAGENTS; a++) {
        out = 1;
        while (out) {
            // FIXME: No utiliza pil ni pf1
            n = 0, pi1 = 0, pf1 = 0;
            // En todas las casillas marca overexpanded = 0 y g = 0
            for (y = 0; y < MAZEHEIGHT; ++y)
                for (x = 0; x < MAZEWIDTH; ++x) {
                    maze1[y][x].overexpanded = 0;
                    maze1[y][x].g = 0;
                }

            /*  Definición de la ubicación de los agentes en el mapa
                4 agentes sobre test5.map2 */
            /* if (a == 0) {
                goaly = 4;
                goalx = 10;
                starty = 4;
                startx = 1;
            }

            if (a == 1) {
                goaly = 4;
                goalx = 1;
                starty = 4;
                startx = 14;
            }

            if (a == 2) {
                goaly = 2;
                goalx = 10;
                starty = 2;
                startx = 1;
            }

            if (a == 3) {
                goaly = 2;
                goalx = 1;
                starty = 2;
                startx = 14;
            } */

            /*  Definición de la ubicación de los agentes en el mapa
                3 agentes sobre test6.map2 */
            if (a == 0) {
                goaly = 2;
                goalx = 2;
                starty = 2;
                startx = 11;
            }

            if (a == 1) {
                goaly = 3;
                goalx = 2;
                starty = 1;
                startx = 10;
            }

            if (a == 2) {
                goaly = 4;
                goalx = 3;
                starty = 4; // starty = 4
                startx = 10;
            }

            // Marca el inicio y objetivo sin obstaculo
            maze1[starty][startx].obstacle = 0;
            maze1[goaly][goalx].obstacle = 0;
            // targetall = &maze1[goaly][goalx];
            // robotall  = &maze1[starty][startx];
            // Find a solution with BFS
            // printf("antes y:%d x:%d\n",y,x);
            // queuey[pf1] = starty; queuex[pf1] = startx;pf1++; maze1[y][x].overexpanded = 1;
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
        //	getchar();
    }
    printf("MAZE GENERATED!!! \n");

}

/*----------------------------------------------------------------------------------*/


int main(int argc, char *argv[]) {
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
