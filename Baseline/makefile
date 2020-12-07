CFLAGS      = -O3
LFLAGS      = -O3
CC      = gcc

OBJ     = heap.o maze.o rtaastar.o lss-lrta.o testall.o -lm
dstarlite:  $(OBJ)
	$(CC) $(LFLAGS) -o testall $(OBJ)

.cc.o:
	$(CC) $(CFLAGS) -c $<

clean:
	rm *.o
