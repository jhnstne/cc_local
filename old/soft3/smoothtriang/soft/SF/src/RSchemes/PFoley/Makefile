#
# Author: Matthew Davidchuk
# 
CC = gcc
BASEDIR= ../../..
CFLAGS = -g -I${BASEDIR}/include -L${BASEDIR}/lib
LIBS   = \
	-lkcurve \
	-lmesh \
	-ldstruct\
	-lgeometry \
	-lSF \
	-lgeomat\
	-lm -ly -ll

PFoley : PFoley.o chukBoundary.o Gregory.o chukBisector.o chukBadpatch.o
	$(CC) ${CFLAGS} -o PFoley PFoley.o chukBoundary.o Gregory.o chukBisector.o chukBadpatch.o \
	$(LIBS)

clean:
	rm -f PFoley *.o *.wff *.wff.Z
