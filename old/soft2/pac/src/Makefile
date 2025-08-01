# File: project Makefile
# Author: J.K. Johnstone
# Last Modified: 9/10/03
# History: 3/31/03: ported from SGI Irix to Linux
#          9/10/03: added Heckbert quad-edge library

ARCH	   = LINUX
SHELL      = /bin/csh
CC         = g++
HOME	   = /home/jj
CLASSBASE  = ${HOME}/software/Cbin
QEDIR      = $(HOME)/software/quadEdge

LIBRARIES  = -lglut -lGLU -lGL -lm
LDFLAGS    = -I${CLASSBASE}
LIBQE      = $(QEDIR)/libcell.a 

# PROGRAMS = ${CLASSBASE}/Miscellany.o ${CLASSBASE}/Vector.o ${CLASSBASE}/MiscVector.o  \
#	   ${CLASSBASE}/Curve.o ${CLASSBASE}/BezierCurve.o ${CLASSBASE}/RatBezierCurve.o \
#	   ${CLASSBASE}/Surf.o ${CLASSBASE}/BezierSurf.o 

all: dmesh
# tangentialCurve, bitang, convexhull, selfbitang, poletang, visgraph, umbra
# surfinterpolate, revosurf, surfintersect, tangentialSurf, tangdsurf, silhouette, drawPAC, bidev
# tmeshview, tmeshviewerXu, kernel, kernelseed, chai, bichai

.cpp: 
	${CC} -Wno-deprecated -g -o $@ $< ${LIBRARIES} ${LDFLAGS}

QEviewer: 
	$(CC) -Wno-deprecated -g -o $@ QEviewer.cpp $(LIBRARIES) $(LDFLAGS) -I$(QEDIR) $(LIBQE)

clean :
	rm -f *.o *~ a.out core
	ls -l

touch :
	touch dmesh.cpp

