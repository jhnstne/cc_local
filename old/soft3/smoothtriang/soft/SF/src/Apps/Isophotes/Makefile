CC = gcc

BASEDIR= ../../..

CFLAGS = -g -I${BASEDIR}/include 

LIBS   = -L${BASEDIR}/lib \
	 -lgeomat \
	 -ldstruct \
	 -lSF \
	 -lgeometry \
	 -ly -ll -lm

all : Isophote

Isophote : isophote.o
	$(CC) -o Isophote isophote.o ${LIBS} 

Isophote.0: Isophote.1
	nroff -man Isophote.1 > Isophote.0

install: all
	cp Isophote ${BASEDIR}/bin
	cp Isophote.0 ${BASEDIR}/man/cat1

clean:
	rm -f *.o Isophote

lint:
	lint -bhux isophote.c ${CFLAGS}
