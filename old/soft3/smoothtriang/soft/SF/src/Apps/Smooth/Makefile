CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I${BASEDIR}/include
LIBS = -L${BASEDIR}/lib\
	-lgeometry \
	-lSF \
	-lmesh \
	-lgeomat \
	-ldstruct \
	-lm -ly -ll

Smooth: smooth.o userMalloc.o
	$(CC) smooth.o userMalloc.o ${LIBS} -o Smooth

smooth.o: smooth.c userData.h 
	$(CC) ${CFLAGS} ${INCLUDE} -c smooth.c

userMalloc.o: userMalloc.c userData.h
	$(CC) ${CFLAGS} ${INCLUDE} -c userMalloc.c
