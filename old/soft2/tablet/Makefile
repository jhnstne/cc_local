# File: project Makefile
# Author: J.K. Johnstone
# Last Modified: 12/22/03
# History: 3/31/03: ported from SGI Irix to Linux

ARCH	   = LINUX
SHELL      = /bin/csh
CC         = g++
HOME	   = /home/jj
BASIC      = ${HOME}/software/Cbin/basic
BASIC2     = ${HOME}/software/Cbin/basic2
CURVE      = ${HOME}/software/Cbin/curve
LIBRARIES  = -lglut -lGLU -lGL -lm
LDFLAGS    = -I${BASIC} -I${BASIC2} -I${CURVE}

all: interpolate
.cpp: 
	${CC} -Wno-deprecated -g -o $@ $< ${LIBRARIES} ${LDFLAGS}
clean :
	rm -f *.o *~ a.out core
	ls -l

touch :
	touch interpolate.cpp

