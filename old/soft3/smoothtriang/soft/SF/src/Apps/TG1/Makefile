#
# Authors: Tony DeRose and Steve Mann
# 	This is a generic makefile that sets up the include paths
#  and libraries.  You will probably need to change BASEDIR to
#  refer to the top-level surface pipeline directory.

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

TG1 : tG1.o data.o avl.o
	$(CC) ${CFLAGS} -o TG1 tG1.o avl.o data.o $(LIBS)

install: TG1
	cp TG1 ${BASEDIR}/bin

clean:
	rm -f *.o TG1

