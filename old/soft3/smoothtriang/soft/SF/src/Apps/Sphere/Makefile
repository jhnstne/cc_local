CC = gcc

BASEDIR = ../../..

CFLAGS = -g -I${BASEDIR}/include 

LIBS   = -L${BASEDIR}/lib\
	 -lmesh \
	 -ldstruct \
	 -lgeometry \
	 -lSF \
	 -lgeomat \
	 -ly -ll -lm


Sphere: sphere.o print.o
	$(CC) $(CFLAGS) -o Sphere print.o sphere.o  ${LIBS}

sphere.o: sphere.c
	$(CC) -c sphere.c ${CFLAGS}

print.o: print.c
	$(CC) -c print.c ${CFLAGS}

install: Sphere
	cp Sphere ${BASEDIR}/bin

clean:
	rm -f Sphere *.o *~
