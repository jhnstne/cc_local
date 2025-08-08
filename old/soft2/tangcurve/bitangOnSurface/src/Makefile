# File: Common tangents on S2 software Makefile
# Author: J.K. Johnstone
# Last Modified: 11 April 2001

ARCH       = SGI
SHELL      = /bin/csh
CC         = CC
CLASSBASE  = ${HOME}/Cbin

LIBRARIES  = -lglut -lGLU -lGL -lXmu -lXext -lX11 -lm
LDFLAGS    = -cckr -float -I${CLASSBASE}

PROGRAMS = ${CLASSBASE}/Miscellany.o ${CLASSBASE}/Vector.o \
	${CLASSBASE}/Curve.o ${CLASSBASE}/BezierCurve.o ${CLASSBASE}/RatBezierCurve.o \
	${CLASSBASE}/Surf.o ${CLASSBASE}/BezierSurf.o 
all: dualS2

dualS2: dualS2.c++
	${CC} -g -o dualS2 dualS2.c++  \
	${CLASSBASE}/Miscellany.o \
	${CLASSBASE}/DisjointSet.o \
	${LIBRARIES} ${LDFLAGS}

# ########################################################################
	
clean :
	rm -f *.o *~ a.out core
	ls -l

touch :
	touch dualS2.c++
	
