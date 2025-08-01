CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I${BASEDIR}/include
LIBS = -L${BASEDIR}/lib \
	-lmesh \
	-ldstruct \
	-lSF \
	-lgeometry \
	-lgeomat\
	-lm -ly -ll

stellate: stellate.o userData.h userMalloc.o
	$(CC) stellate.o userMalloc.o ${LIBS} -o stellate

stellate.o: stellate.c
	$(CC) ${CFLAGS} ${INCLUDE} -c stellate.c

userMalloc.o: userMalloc.c
	$(CC) ${CFLAGS} ${INCLUDE} -c userMalloc.c

Stellate: stellate2.o
	$(CC) stellate2.o ${LIBS} -o Stellate

stellate2.o: stellate2.c
	$(CC) ${CFLAGS} ${INCLUDE} -c stellate2.c

clean:
	rm -f *.o Stellate
