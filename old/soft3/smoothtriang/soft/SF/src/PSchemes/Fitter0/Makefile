#
# Author: Tony DeRose
# 
CC = gcc
BASEDIR= ../../..
CFLAGS = -g -I${BASEDIR}/include
LIBS   = ${BASEDIR}/lib/libmesh.a \
	 ${BASEDIR}/lib/libgeometry.a \
         ${BASEDIR}/lib/libSF.a \
	 ${BASEDIR}/lib/libgeomat.a \
	 ${BASEDIR}/lib/libdstruct.a \
	 -lm -ly -ll

Fitter0 : Fitter0.o
	$(CC) -o Fitter0 Fitter0.o $(LIBS)

Fitter0Tess : Fitter0Tess.o
	$(CC) -o Fitter0Tess Fitter0Tess.o $(LIBS)

install: Fitter0 Fitter0Tess
	cp Fitter0 Fitter0Tess ${BASEDIR}/bin

clean:
	rm -f Fitter0 *.o Fitter0Tess
