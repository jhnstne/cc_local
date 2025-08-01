#
# 	This is a generic makefile that sets up the include paths
#  and libraries.  You will probably need to change BASEDIR to
#  refer to the top-level surface pipeline directory.

CC = gcc
BASEDIR= ../../..
CFLAGS = -g -I${BASEDIR}/include -L${BASEDIR}/lib
LIBS   = \
	-lmesh \
	-lgeomat\
	-ldstruct\
	-lgeometry \
	-lSF \
	-lgeomat\
	-lm -ly -ll

ATess : ATess.o aputil.o apatch.o aptessellate.o
	$(CC) ${CFLAGS} -o ATess aptessellate.o apatch.o aputil.o ATess.o $(LIBS)

AFitter: AFitter.o aputil.o apatch.o apconstruct.o
	cc ${CFLAGS} -o AFitter apconstruct.o apatch.o aputil.o AFitter.o $(LIBS)

clean:
	rm -f *.o core ATess AFitter


