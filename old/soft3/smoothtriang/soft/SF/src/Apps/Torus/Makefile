CC = gcc

BASEDIR = ../../..

CFLAGS = -g

INCLUDE = -I${BASEDIR}/include

LIBS   = \
	 ${BASEDIR}/lib/libgeomat.a \
	 ${BASEDIR}/lib/libdstruct.a \
	 ${BASEDIR}/lib/libSF.a \
	 ${BASEDIR}/lib/libgeometry.a \
	 ${BASEDIR}/lib/libSF.a \
	 -ly -ll -lm


Torus: torus.o
	$(CC) -o Torus torus.o ${LIBS}

torus.o: torus.c
	$(CC) -c torus.c ${INCLUDE} ${CFLAGS}

install: Torus
	mv Torus ${BASEDIR}/bin
	rm -f *.o core

clean:
	rm -f Torus *.o
