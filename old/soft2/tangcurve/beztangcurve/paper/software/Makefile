# File: Dual curve software Makefile
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
all: dual

# original version
tang: tang.c++ 
	${CC} -g -o tang tang.c++  \
	${CLASSBASE}/Miscellany.o ${CLASSBASE}/BezierCurve.o \
	${CLASSBASE}/Contour.o \
	${CLASSBASE}/ContourFormat.o \
	${CLASSBASE}/DisjointSet.o \
	${CLASSBASE}/Path.o ${CLASSBASE}/ToroidalGraph.o \
	${CLASSBASE}/Vector.o \
	${LIBRARIES} ${LDFLAGS}
	
dual: dual.c++
	${CC} -g -o dual dual.c++  \
	${CLASSBASE}/Miscellany.o ${CLASSBASE}/BezierCurve.o \
	${CLASSBASE}/RatBezierCurve.o \
	${CLASSBASE}/Contour.o ${CLASSBASE}/ContourFormat.o \
	${CLASSBASE}/DisjointSet.o \
	${CLASSBASE}/Path.o ${CLASSBASE}/ToroidalGraph.o \
	${CLASSBASE}/Vector.o \
	${LIBRARIES} ${LDFLAGS}
	
# ########################################################################
	
clean :
	rm -f *.o *~ a.out core
	ls -l

touch :
	touch dual.c++
	
