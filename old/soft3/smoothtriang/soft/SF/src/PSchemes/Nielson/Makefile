#
# Authors: Tony DeRose and Steve Mann
# 
CC = gcc
BASEDIR= ../../..
CFLAGS = -g -I${BASEDIR}/include -L${BASEDIR}/lib
LIBS   = \
	-lkcurve \
	-lmesh \
	-lSF \
	-lgeomat\
	-ldstruct\
	-lgeometry \
	-lgeomat\
	-lm -ly -ll


Nielson : Nielson.o 
	$(CC) ${CFLAGS} -o Nielson Nielson.o $(LIBS)

Nielson.o: Nielson.c
	$(CC) ${CFLAGS} -c Nielson.c

clean:
	rm -f Nielson *.o *.wff *.wff.Z
