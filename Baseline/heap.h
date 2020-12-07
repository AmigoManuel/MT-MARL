/////////////////////////////////////////////////////////////////////
// Xiaoxun Sun & Sven Koenig @ USC 2009
// All rights reserved
/////////////////////////////////////////////////////////////////////

#ifndef HEAPH
#define HEAPH

#include "maze.h"

#define HEAPSIZE 10000000

void emptyheap2();

//int testheap3();
cell1 *popheap2();

cell1 *topheap2();

void deleteheap2(cell1 *thiscell);

void insertheap2(cell1 *thiscell);

cell1 *posheap2(int i);

int sizeheap2();

long int opensize2();


void emptyheap3(); //for dijkstra
cell1 *popheap3();

cell1 *topheap3();

void deleteheap3(cell1 *thiscell);

void insertheap3(cell1 *thiscell);


#endif
