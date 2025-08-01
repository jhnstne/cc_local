#
# Authors: Tony DeRose and Steve Mann
# Last Modified: 04/09/89 at 15:19:26
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

ShirmanSequin : ShirmanSequin.o stubRecipes.o outBoundary.o
	$(CC) ${CFLAGS} -o ShirmanSequin ShirmanSequin.o \
	outBoundary.o stubRecipes.o $(LIBS)

t: t.o stubRecipes.o outBoundary.o
	$(CC) ${CFLAGS} -o t t.o \
	outBoundary.o stubRecipes.o $(LIBS)

t2: t2.o stubRecipes.o outBoundary.o
	$(CC) ${CFLAGS} -o t2 t2.o \
	outBoundary.o stubRecipes.o $(LIBS)

S : ShirmanSequin.o 
	$(CC) ${CFLAGS} -o ShirmanSequin patch.o ShirmanSequin.o $(LIBS)

ShirmanSequin.quad : ShirmanSequin.quad.o 
	$(CC) ${CFLAGS} -o ShirmanSequin.quad ShirmanSequin.quad.o $(LIBS)

ShirmanSequin.cubic : ShirmanSequin.cubic.o SffScheme.o
	$(CC) ${CFLAGS} -o ShirmanSequin.cubic ShirmanSequin.cubic.o SffScheme.o\
	 $(LIBS)

clean:
	rm -f ShirmanSequin ShirmanSequin.quad ShirmanSequin.save *.o \
		*.wff *.wff.Z
