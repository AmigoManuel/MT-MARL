#ifndef HEAPH
#define HEAPH

#include "maze.h"

#define HEAPSIZE 10000000

/* ----------- */
/* Binary Heap */
/* ----------- */

/* Limpia el heap */
void emptyheap2();
/* pop sobre el heap, retorna una cell1 */
cell1 *popheap2();
/* Retorna el top del heap */
cell1 *topheap2();
/* Elimina cell1 del heap */
void deleteheap2(cell1 *thiscell);
/* Inserta cell1 dentro del heap */
void insertheap2(cell1 *thiscell);
/* Retorna una cell1 mediante su index en el heap */
cell1 *posheap2(int i);
/* Retorna el tamaño del heap */
int sizeheap2();

long int opensize2();

/* ------------------------- */
/* Binary Heap para Dijkstra */
/* ------------------------- */

/* Inserta cell1 dentro del heap */
void insertheap3(cell1 *thiscell);
/* Elimina cell1 del heap */
void deleteheap3(cell1 *thiscell);
/* Retorna el top del heap */
cell1 *topheap3();
/* Limpia el heap */
void emptyheap3();
/* pop sobre el heap, retorna una cell1 */
cell1 *popheap3();
/* Retorna el tamaño del heap */
int sizeheap3();

#endif
