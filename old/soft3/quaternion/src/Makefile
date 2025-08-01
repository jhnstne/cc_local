# File: quaternion spline project Makefile
# Author: J.K. Johnstone
# Last Modified: 3/7/04
# History: 3/7/04: ported from SGI Irix to Linux

ARCH	   = LINUX
SHELL      = /bin/csh
CC         = g++
HOME	   = /home/jj
BASIC      = ${HOME}/software/Cbin/basic
BASIC2     = ${HOME}/software/Cbin/basic2
CURVE      = ${HOME}/software/Cbin/curve
SURF       = ${HOME}/software/Cbin/surf
QUATERNION = ${HOME}/software/Cbin/quaternion
CLAPACK    = $(HOME)/software/CLAPACK
CLAPACKARC = $(CLAPACK)/lapack_LINUX.a \
             $(CLAPACK)/blas_LINUX.a \
	     $(CLAPACK)/F2CLIBS/libF77.a
LIBRARIES  = -lglut -lGLU -lGL -lm
LDFLAGS    = -I${BASIC} -I${BASIC2} -I${CURVE} -I${SURF} -I${QUATERNION} $(CLAPACKARC)

all: quaternionSpline
.cpp: 
	${CC} -Wno-deprecated -g -o $@ $< ${LIBRARIES} ${LDFLAGS}
clean :
	rm -f *.o *~ a.out core
	ls -l
touch :
	touch quaternionSpline.cpp

