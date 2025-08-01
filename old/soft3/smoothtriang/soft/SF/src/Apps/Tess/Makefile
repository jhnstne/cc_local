CC = gcc

BASEDIR= ../../..

CFLAGS = -g -I${BASEDIR}/include 

LIBS   = -L${BASEDIR}/lib \
	 -lgeomat \
	 -ldstruct \
	 -lgeometry \
	 -lSF \
	 -lgeomat \
	 -g -ly -ll -lm

Tess : Tess.o tptess.o patchtess.o ps.o
	$(CC) -o Tess Tess.o tptess.o patchtess.o ps.o ${LIBS}

fTess : Tess.o tptess.o patchtess.o ps.o
	$(CC) -o fTess Tess.o tptess.o patchtess.o ps.o \
		/projects/grail/src/geometry/fgeometry.a ${LIBS}

install:
	make Tess
	cp Tess ${BASEDIR}/bin

clean:
	rm -f *.o Tess 
