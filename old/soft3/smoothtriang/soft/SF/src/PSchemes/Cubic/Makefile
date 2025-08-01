#
# Authors: Tony DeRose and Steve Mann
# 
CC = gcc
BASEDIR= ../../..
CFLAGS = -g -I${BASEDIR}/include -L${BASEDIR}/lib
LIBS   = \
	-lkcurve \
	-lmesh \
	-lgeomat\
	-ldstruct\
	-lgeometry \
	-lSF \
	-lgeomat\
	-lm -ly -ll

#	${BASEDIR}/../../lib/librecipes.a 

Cubic : Cubic.o 
	$(CC) ${CFLAGS} -o Cubic Cubic.o \
	$(LIBS)

clean:
	rm -f ShirmanSeqwuin Cubic.quad Cubic.save *.o \
		*.wff *.wff.Z
