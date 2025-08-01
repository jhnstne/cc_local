#
# File: Orientation/animation/Makefile
# Author: J.K. Johnstone
# Last Modified: 23 August 1996

ARCH = SGI

ORIBASE = /ca/jj/Orientation/bin
BINBASE = /ca/jj/cbin
OPENGL = /ca/jj/OpenGL/book/libaux
GLUT = /ca/jj/OpenGL/glut-3.0

DEST = ${ORIBASE}/bin
INCLUDES = -I${BINBASE} 
GLUT_LIB = ${GLUT}/lib/glut/libglut.a
SHELL   = /bin/csh
CC      = cc
# for OPENGL with GLUT library
LDOPTIONS_GLUT = -cckr -float -prototypes ${INCLUDES} -I${GLUT}
LOCAL_LIBRARIES_GLUT = -lGLU -lGL -lXmu -lXext -lX11 -lm ${GLUT_LIB}

PROGRAMS = GLUTS3curve
all: ${PROGRAMS}

GLUTS3curve: GLUTS3curve.c
	${CC} -o GLUTS3curve GLUTS3curve.c \
	${BINBASE}/Bez4d.o ${BINBASE}/Bez.o ${BINBASE}/RatBez.o \
	${BINBASE}/Fit4d.o ${BINBASE}/Fit.o \
	${BINBASE}/Matrix.o ${BINBASE}/Vec.o ${BINBASE}/Misc.o \
	${LOCAL_LIBRARIES_GLUT} ${LDOPTIONS_GLUT}

install : ${PROGRAMS}
	mv ${PROGRAMS} ${DEST}
	(cd ${DEST}; chmod a+rx ${PROGRAMS})
	ls -l ${DEST}

clean :
	rm -f *.o *~ a.out core
	ls -l


