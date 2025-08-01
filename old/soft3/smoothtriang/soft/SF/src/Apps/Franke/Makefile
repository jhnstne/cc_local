CC = gcc 

BASEDIR = ../../..

CFLAGS = -g

INCLUDE = -I${BASEDIR}/include

LIBS   = \
	 -L${BASEDIR}/lib \
	 -lfunc \
	 -lmesh \
	 -ldstruct \
	 -lgeomat \
	 -lSF \
	 -lgeometry \
	 -lgeomat \
	 -ly -ll -lm

Franke: franke.c userMalloc.c
	$(CC) franke.c userMalloc.c -o Franke  ${INCLUDE} ${CFLAGS} ${LIBS}

install: Franke
	cp Franke FrankeDiff ${BASEDIR}/bin
	
FrankeDiff: frankediff.c
	$(CC) frankediff.c -o FrankeDiff  ${INCLUDE} ${CFLAGS} ${LIBS}
	
clean:
	rm -f Franke FrankeDiff *.o *~
