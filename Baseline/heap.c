/////////////////////////////////////////////////////////////////////
// Xiaoxun Sun & Sven Koenig @ USC 2009
// All rights reserved
/////////////////////////////////////////////////////////////////////


#include "include.h"
#include "heap.h"

// --------------------------    Binary Heap for A*  -----------------------------------------
#define HEAPSIZE2 1000000
cell1 *heap2[HEAPSIZE2];
long int heapsize2 = 0;
//long int statpercolated2 = 0;

// ---------------------------------------------------------------
void percolatedown2(int hole, cell1 *tmp){
  int child;

  if (heapsize2 != 0) {
        for (; 2*hole <= heapsize2; hole = child)
        {
          child = 2*hole;
          if (child != heapsize2 && heap2[child+1]->key < heap2[child]->key)
            ++child;
          if (heap2[child]->key < tmp->key)
            {
              heap2[hole] = heap2[child];
              heap2[hole]->heapindex = hole;
              ++statpercolated2;
            }
          else
            break;
        } // end for
      heap2[hole] = tmp;
      heap2[hole]->heapindex = hole;
    }
}
/* --------------------------------------------------------------- */
void percolateup2(int hole, cell1 *tmp)
{
  if (heapsize2 != 0)
    {
      for (; hole > 1 && tmp->key < heap2[hole/2]->key; hole /= 2)
    {
      heap2[hole] = heap2[hole/2];
      heap2[hole]->heapindex = hole;
      ++statpercolated2;
    }  
     heap2[hole] = tmp;
      heap2[hole]->heapindex = hole;
    }
}
/* --------------------------------------------------------------- */
void percolateupordown2(int hole, cell1 *tmp)
{
  if (heapsize2 != 0)
    {
      if (hole > 1 && heap2[hole/2]->key > tmp->key)
    percolateup2(hole, tmp);
      else
    percolatedown2(hole, tmp);
    }
}
/* --------------------------------------------------------------- */
void insertheap2(cell1 *thiscell)
{
  int hole;

  if (thiscell->heapindex == 0)
    { 
  //  assert(heapsize2 < HEAPSIZE2-1);
      percolateup2(++heapsize2, thiscell);
    }
  else
    percolateupordown2(thiscell->heapindex, heap2[thiscell->heapindex]);
}
/* --------------------------------------------------------------- */
void deleteheap2(cell1 *thiscell)
{
  if (thiscell->heapindex != 0)
    {
      percolateupordown2(thiscell->heapindex, heap2[heapsize2--]);
      thiscell->heapindex = 0;
    }
}
/* --------------------------------------------------------------- */
cell1 *topheap2()
{
  if (heapsize2 == 0)
    return NULL;
  return heap2[1];
}

/* Vacía el heap2 */
void emptyheap2(){
  int i;
  for (i = 1; i <= heapsize2; ++i) heap2[i]->heapindex = 0;
  heapsize2 = 0;
}

/* --------------------------------------------------------------- */
cell1* popheap2()
{
    cell1 *thiscell;

    if (heapsize2 == 0)
        return NULL;
    thiscell = heap2[1];
    thiscell->heapindex = 0;
    percolatedown2(1, heap2[heapsize2--]);
    return thiscell;
}


int sizeheap2()
{
  return heapsize2;
}

cell1* posheap2(int i)
{
//  if (heapsize2 == 0)
//    return NULL;
  return heap2[i];
}

// --------------------------    Binary Heap end --------------------------------------------
// --------------------------    Binary Heap for Dijkstra  -----------------------------------------
#define HEAPSIZE3 1000000
cell1 *heap3[HEAPSIZE3];
long int heapsize3 = 0;
long int statpercolated3 = 0;

// ---------------------------------------------------------------
void percolatedown3(int hole, cell1 *tmp)
{
  int child;

  if (heapsize3 != 0)
    {
        for (; 2*hole <= heapsize3; hole = child)
        {
          child = 2*hole;
          if (child != heapsize3 && heap3[child+1]->key3 < heap3[child]->key3)
            ++child;
          if (heap3[child]->key3 < tmp->key3)
            {
              heap3[hole] = heap3[child];
              heap3[hole]->heapindex3 = hole;
            ++statpercolated2;
            }
          else
            break;
        } // end for
      heap3[hole] = tmp;
      heap3[hole]->heapindex3 = hole;
    }
}
/* --------------------------------------------------------------- */
void percolateup3(int hole, cell1 *tmp)
{
  if (heapsize3 != 0)
    {
      for (; hole > 1 && tmp->key3 < heap3[hole/2]->key3; hole /= 2)
    {
      heap3[hole] = heap3[hole/2];
      heap3[hole]->heapindex3 = hole;
    ++statpercolated2;
    }  
      heap3[hole] = tmp;
      heap3[hole]->heapindex3 = hole;
    }
}
/* --------------------------------------------------------------- */
void percolateupordown3(int hole, cell1 *tmp)
{
  if (heapsize3 != 0)
    {
      if (hole > 1 && heap3[hole/2]->key3 > tmp->key3)
    percolateup3(hole, tmp);
      else
    percolatedown3(hole, tmp);
    }
}
/* --------------------------------------------------------------- */
void insertheap3(cell1 *thiscell)
{
  int hole;

  if (thiscell->heapindex3 == 0)
    { 
  //  assert(heapsize2 < HEAPSIZE2-1);
      percolateup3(++heapsize3, thiscell);
    }
  else
    percolateupordown3(thiscell->heapindex3, heap3[thiscell->heapindex3]);
}
/* --------------------------------------------------------------- */
void deleteheap3(cell1 *thiscell)
{
  if (thiscell->heapindex3 != 0)
    {
      percolateupordown3(thiscell->heapindex3, heap3[heapsize3--]);
      thiscell->heapindex3 = 0;
    }
}
/* --------------------------------------------------------------- */
cell1 *topheap3()
{
  if (heapsize3 == 0)
    return NULL;
  return heap3[1];
}
/* --------------------------------------------------------------- */
void emptyheap3()
{
  int i;

  for (i = 1; i <= heapsize3; ++i)
    heap3[i]->heapindex3 = 0;
  heapsize3 = 0;
}

/* --------------------------------------------------------------- */
cell1* popheap3()
{
    cell1 *thiscell;

    if (heapsize3 == 0)
        return NULL;
    thiscell = heap3[1];
    thiscell->heapindex3 = 0;
    percolatedown3(1, heap3[heapsize3--]);
    return thiscell;
}


long int opensize2()
{
        return heapsize2;
}

// --------------------------    Binary Heap end --------------------------------------------







/*
// --------------------------    Binary Heap 3, for GGAAstar(hybrid) h-value propagation ---------
// this heap is designed for h-values propagation.
// Therefore, it use h-value as the key. (The ->key has been replaced by ->h)

#define HEAPSIZE3 1000000
cell1 *heap3[HEAPSIZE3];
long int heapsize3 = 0;


void percolatedown3(int hole, cell1 *tmp)
{
  int child;

  if (heapsize3 != 0)
    {
        for (; 2*hole <= heapsize3; hole = child)
        {
          child = 2*hole;
          if (child != heapsize3 && heap3[child+1]->h < heap3[child]->h)
            ++child;
          if (heap3[child]->h < tmp->h)
            {
              heap3[hole] = heap3[child];
              heap3[hole]->heapindex3 = hole;
            }
          else
            break;
        } // end for
      heap3[hole] = tmp;
      heap3[hole]->heapindex3 = hole;
    }
}

void percolateup3(int hole, cell1 *tmp)
{
  if (heapsize3 != 0)
    {
      for (; hole > 1 && tmp->h < heap3[hole/2]->h; hole /= 2)
    {
      heap3[hole] = heap3[hole/2];
      heap3[hole]->heapindex3 = hole;
    }  
      heap3[hole] = tmp;
      heap3[hole]->heapindex3 = hole;
    }
}

void percolateupordown3(int hole, cell1 *tmp)
{
  if (heapsize3 != 0)
    {
      if (hole > 1 && heap3[hole/2]->h > tmp->h)
    percolateup3(hole, tmp);
      else
    percolatedown3(hole, tmp);
    }
}

void insertheap3(cell1 *thiscell)
{
  int hole;

  if (thiscell->heapindex3 == 0)
    { 
  //  assert(heapsize2 < HEAPSIZE2-1);
      percolateup3(++heapsize3, thiscell);
    }
  else
    percolateupordown3(thiscell->heapindex3, heap3[thiscell->heapindex3]);
}

void deleteheap3(cell1 *thiscell)
{
  if (thiscell->heapindex3 != 0)
    {
      percolateupordown3(thiscell->heapindex3, heap3[heapsize3--]);
      thiscell->heapindex3 = 0;
    }
}

cell1 *topheap3()
{
  if (heapsize3 == 0)
    return NULL;
  return heap3[1];
}

void emptyheap3()
{
  int i;

  for (i = 1; i <= heapsize3; ++i)
    heap3[i]->heapindex3 = 0;
  heapsize3 = 0;
}


cell1* popheap3()
{
    cell1 *thiscell;

    if (heapsize3 == 0)
        return NULL;
    thiscell = heap3[1];
    thiscell->heapindex3 = 0;
    percolatedown3(1, heap3[heapsize3--]);
    return thiscell;
}
// --------------------------    Binary Heap end ------------------------------------------
#endif
*/
