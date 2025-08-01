#
# Author: Tony DeRose
# 
CC = gcc
BASEDIR= ../../..

CFLAGS = -g -I${BASEDIR}/include 

LIBS   = \
	 ${BASEDIR}/lib/libgeomat.a \
	 ${BASEDIR}/lib/libdstruct.a \
	 ${BASEDIR}/lib/libSF.a \
	 ${BASEDIR}/lib/geometry.a \
	 -g -ly -ll -lm

Sgp:	sgpFormat.o sgp.o
	$(CC) -o Sgp sgp.o sgpFormat.o ${LIBS} 

install: Sgp
	cp Sgp ${BASEDIR}/bin

clean:
	rm -f Sgp sgp.o sgpFormat.o
