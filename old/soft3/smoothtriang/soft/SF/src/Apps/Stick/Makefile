CC = gcc
BASEDIR = ../../..
LIBS =  ${BASEDIR}/lib/libdstruct.a \
	${BASEDIR}/lib/libgeomat.a \
	${BASEDIR}/lib/libSF.a \
	${BASEDIR}/lib/libgeometry.a \
	-ll -lm

CFLAGS = -g -I${BASEDIR}/include

Stick: toSticks.o print.o data.o avl.o dstructTri.o triangulate.o \
	    sgpTri.o a3dTri.o s3dTri.o
	$(CC) -o Stick toSticks.o print.o data.o avl.o dstructTri.o \
	    triangulate.o sgpTri.o a3dTri.o s3dTri.o \
	    ${LIBS} ${CFLAGS}

install: Stick
	cp Stick ${BASEDIR}/bin
	cp Stick.l ${BASEDIR}/man/manl

clean:
	rm -f *.o Stick
