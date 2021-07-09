#include "include.h"
#include "heap.h"

#define HEAPSIZE2 1000000
// Crea un heap de celdas
cell1 *heap2[HEAPSIZE2];
long int heapsize2 = 0;

/* ----------- */
/* Binary Heap */
/* ----------- */

/* Heap cambiar hacia abajo, recibe heapindex donde insertar y cell1 a insertar */
void percolatedown2(int hole, cell1 *tmp) {
    int child;
    if (heapsize2 != 0) {
        for (; 2 * hole <= heapsize2; hole = child) {
            child = 2 * hole;
            if (child != heapsize2 && heap2[child + 1]->key < heap2[child]->key)
                ++child;
            if (heap2[child]->key < tmp->key) {
                heap2[hole] = heap2[child];
                heap2[hole]->heapindex = hole;
                ++statpercolated2;
            } else
                break;
        }
        heap2[hole] = tmp;
        heap2[hole]->heapindex = hole;
    }
}

/* Heap cambiar hacia arriba, recibe heapindex donde insertar y cell1 a insertar */
void percolateup2(int hole, cell1 *tmp){
    if (heapsize2 != 0) {
        int newkey = tmp->key;
        //SOMEHOW HERE THE F OF THE STATES ARE COMPARED
        for (; hole > 1 && tmp->key < heap2[hole / 2]->key; hole /= 2) {
            int oldkey = heap2[hole / 2]->key;
            heap2[hole] = heap2[hole / 2];
            heap2[hole]->heapindex = hole;
            ++statpercolated2;
        }
        heap2[hole] = tmp;
        heap2[hole]->heapindex = hole;
    }
}

/* Evalua si el cambio es hacia arriba o abajo, recibe heapindex y la cell1 */
void percolateupordown2(int hole, cell1 *tmp) {
    if (heapsize2 != 0) {
        if (hole > 1 && heap2[hole / 2]->key > tmp->key)
            percolateup2(hole, tmp);
        else
            percolatedown2(hole, tmp);
    }
}

/* Inserta cell1 dentro del heap */
void insertheap2(cell1 *thiscell) {
    int hole;
    if (thiscell->heapindex == 0) {
        percolateup2(++heapsize2, thiscell);
    } else
        percolateupordown2(thiscell->heapindex, heap2[thiscell->heapindex]);
}

/* Elimina cell1 del heap */
void deleteheap2(cell1 *thiscell) {
    if (thiscell->heapindex != 0) {
        percolateupordown2(thiscell->heapindex, heap2[heapsize2--]);
        thiscell->heapindex = 0;
    }
}

/* Retorna el top del heap */
cell1 *topheap2() {
    if (heapsize2 == 0) return NULL;
    return heap2[1];
}

/* Limpia el heap */
void emptyheap2() {
    int i;
    for (i = 1; i <= heapsize2; ++i)
        heap2[i]->heapindex = 0;
    heapsize2 = 0;
}

/* pop sobre el heap, retorna una cell1 */
cell1 *popheap2() {
    cell1 *thiscell;
    if (heapsize2 == 0) return NULL;
    thiscell = heap2[1];
    thiscell->heapindex = 0;
    percolatedown2(1, heap2[heapsize2--]);
    return thiscell;
}

/* Retorna el tamaño del heap */
int sizeheap2() {
    return heapsize2;
}

/* Retorna una cell1 mediante su index en el heap */
cell1 *posheap2(int i) {
    return heap2[i];
}

/* ------------------------- */
/* Binary Heap para Dijkstra */
/* ------------------------- */

#define HEAPSIZE3 1000000
cell1 *heap3[HEAPSIZE3];
long int heapsize3 = 0;
long int statpercolated3 = 0;

/* Heap cambiar hacia abajo, recibe heapindex donde insertar y cell1 a insertar */
void percolatedown3(int hole, cell1 *tmp) {
    int child;
    if (heapsize3 != 0) {
        for (; 2 * hole <= heapsize3; hole = child) {
            child = 2 * hole;
            if (child != heapsize3 && heap3[child + 1]->key3 < heap3[child]->key3)
                ++child;
            if (heap3[child]->key3 < tmp->key3) {
                heap3[hole] = heap3[child];
                heap3[hole]->heapindex3 = hole;
                ++statpercolated2;
            } else
                break;
        }
        heap3[hole] = tmp;
        heap3[hole]->heapindex3 = hole;
    }
}

/* Heap cambiar hacia arriba, recibe heapindex donde insertar y cell1 a insertar */
void percolateup3(int hole, cell1 *tmp) {
    if (heapsize3 != 0) {
        for (; hole > 1 && tmp->key3 < heap3[hole / 2]->key3; hole /= 2) {
            heap3[hole] = heap3[hole / 2];
            heap3[hole]->heapindex3 = hole;
            ++statpercolated2;
        }
        heap3[hole] = tmp;
        heap3[hole]->heapindex3 = hole;
    }
}

/* Evalua si el cambio es hacia arriba o abajo, recibe heapindex y la cell1 */
void percolateupordown3(int hole, cell1 *tmp) {
    if (heapsize3 != 0) {
        if (hole > 1 && heap3[hole / 2]->key3 > tmp->key3)
            percolateup3(hole, tmp);
        else
            percolatedown3(hole, tmp);
    }
}

/* Inserta cell1 dentro del heap */
void insertheap3(cell1 *thiscell) {
    int hole;
    if (thiscell->heapindex3 == 0) {
        percolateup3(++heapsize3, thiscell);
    } else
        percolateupordown3(thiscell->heapindex3, heap3[thiscell->heapindex3]);
}

/* Elimina cell1 del heap */
void deleteheap3(cell1 *thiscell) {
    if (thiscell->heapindex3 != 0) {
        percolateupordown3(thiscell->heapindex3, heap3[heapsize3--]);
        thiscell->heapindex3 = 0;
    }
}

/* Retorna el top del heap */
cell1 *topheap3() {
    if (heapsize3 == 0)
        return NULL;
    return heap3[1];
}

/* Limpia el heap */
void emptyheap3() {
    int i;
    for (i = 1; i <= heapsize3; ++i)
        heap3[i]->heapindex3 = 0;
    heapsize3 = 0;
}

/* pop sobre el heap, retorna una cell1 */
cell1 *popheap3() {
    cell1 *thiscell;
    if (heapsize3 == 0) return NULL;
    thiscell = heap3[1];
    thiscell->heapindex3 = 0;
    percolatedown3(1, heap3[heapsize3--]);
    // printf("%d\n", thiscell->x);
    // printf("%d\n", thiscell->y);
    return thiscell;
}

/* Retorna el tamaño del heap */
int sizeheap3() {
    return heapsize3;
}

long int opensize2() {
    return heapsize2;
}
