CC = gcc
BASEDIR = ../../..
CFLAGS = -g 
INCLUDE = -I${BASEDIR}/include
LIBS = -L${BASEDIR}/lib\
	-lmesh \
	-lgeomat \
	-ldstruct \
	-lgeometry \
	-lm -ly -ll

OrientMesh: orient.o meshutils.o meshsym.o edgeTable.o queue.o
	$(CC) orient.o meshutils.o meshsym.o  edgeTable.o queue.o \
		-o OrientMesh ${LIBS}

orient.o: orient.c
	$(CC) -c orient.c ${CFLAGS} ${INCLUDE}

meshutils.o: meshutils.c
	$(CC) -c meshutils.c ${INCLUDE} ${CFLAGS}

meshsym.o: meshsym.c
	$(CC) -c meshsym.c ${INCLUDE} ${CFLAGS}

edgeTable.o: edgeTable.c
	$(CC) -c edgeTable.c ${INCLUDE} ${CFLAGS}

queue.o: queue.c
	$(CC) -c queue.c ${INCLUDE} ${CFLAGS}

clean:
	rm -f *.o OrientMesh
