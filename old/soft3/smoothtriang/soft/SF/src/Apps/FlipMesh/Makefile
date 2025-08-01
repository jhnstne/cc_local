CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I${BASEDIR}/include
LIBS = -L${BASEDIR}/lib\
	-lmesh \
	-lgeomat \
	-ldstruct \
	-lgeometry \
	-lm -ly -ll

FlipMesh: flip.o meshutils.o
	$(CC) flip.o meshutils.o -o FlipMesh ${LIBS}

flip.o: flip.c
	$(CC) -c flip.c ${INCLUDE} ${CFLAGS}

meshutils.o: meshutils.c
	$(CC) -c meshutils.c ${INCLUDE} ${CFLAGS}

install: FlipMesh
	cp FlipMesh ${BASEDIR}/bin

clean:
	rm -f *.o FlipMesh
