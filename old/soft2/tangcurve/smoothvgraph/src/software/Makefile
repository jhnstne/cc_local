# File: Smooth visibility graph software Makefile
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
all: svgraph

svgraph: svgraph.c++
	${CC} -g -o svgraph svgraph.c++  \
	${CLASSBASE}/Miscellany.o ${CLASSBASE}/BezierCurve.o \
	${CLASSBASE}/BsplineCurve.o ${CLASSBASE}/Curve.o \
	${CLASSBASE}/RatBezierCurve.o ${CLASSBASE}/Polygon.o \
	${CLASSBASE}/Contour.o ${CLASSBASE}/ContourFormat.o \
	${CLASSBASE}/DisjointSet.o \
	${CLASSBASE}/Path.o ${CLASSBASE}/ToroidalGraph.o \
	${CLASSBASE}/Vector.o ${CLASSBASE}/TangCurve.o \
	${CLASSBASE}/Graph.o ${CLASSBASE}/Vgraph.o ${CLASSBASE}/Matrix.o \
	${LIBRARIES} ${LDFLAGS}
	
svgraph-25Sept00: svgraph-25Sept00.c++
	${CC} -g -o svgraph svgraph-25Sept00.c++  \
	${CLASSBASE}/Miscellany.o ${CLASSBASE}/BezierCurve.o \
	${CLASSBASE}/RatBezierCurve.o \
	${CLASSBASE}/Contour.o \
	${CLASSBASE}/ContourFormat.o \
	${CLASSBASE}/DisjointSet.o \
	${CLASSBASE}/Path.o ${CLASSBASE}/ToroidalGraph.o \
	${CLASSBASE}/Vector.o ${CLASSBASE}/TangCurve.o \
	${CLASSBASE}/Graph.o ${CLASSBASE}/Matrix.o \
	${LIBRARIES} ${LDFLAGS}
	
visibility: visibility.c++
	${CC} -g -o visibility visibility.c++  \
	${CLASSBASE}/Miscellany.o \
	${CLASSBASE}/Contour.o \
	${CLASSBASE}/ContourFormat.o \
	${CLASSBASE}/DisjointSet.o \
	${CLASSBASE}/Path.o ${CLASSBASE}/ToroidalGraph.o \
	${LIBRARIES} ${LDFLAGS}

# ########################################################################
	
clean :
	rm -f *.o *~ a.out core
	ls -l

touch :
	touch svgraph.c++
	
