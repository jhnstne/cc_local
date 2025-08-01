# File: Dual Chaikin curve software Makefile
# Author: J.K. Johnstone
# Last Modified: 2 August 2002

ARCH       = SGI
SHELL      = /bin/csh
CC         = CC
CLASSBASE  = ${HOME}/Cbin

LIBRARIES  = -lglut -lGLU -lGL -lXmu -lXext -lX11 -lm
LDFLAGS    = -cckr -float -I${CLASSBASE}

all: chai

chai: chai.cpp
	${CC} -g -o chai chai.cpp  \
	${LIBRARIES} ${LDFLAGS}

bichai: bichai.cpp
	${CC} -g -o bichai bichai.cpp  \
	${LIBRARIES} ${LDFLAGS}
	
dualchai: dualchai.cpp
	${CC} -g -o dualchai dualchai.cpp  \
	${LIBRARIES} ${LDFLAGS}

# ########################################################################
	
clean :
	rm -f *.o *~ a.out core
	ls -l

touch :
	touch bichai.cpp
	
