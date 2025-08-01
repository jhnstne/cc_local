#
# Authors: Tony DeRose and Steve Mann
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

Foley : Foley.o
	$(CC) ${CFLAGS} -o Foley Foley.o \
	$(LIBS)

clean:
	rm -f Foley *.o *.wff *.wff.Z *~
