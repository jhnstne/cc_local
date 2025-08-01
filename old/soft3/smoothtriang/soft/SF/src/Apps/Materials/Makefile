CC = gcc

BASEDIR= ../../..

CFLAGS = -g -I${BASEDIR}/include 

LIBS   = \
	 ${BASEDIR}/lib/libgeomat.a \
	 ${BASEDIR}/lib/libdstruct.a \
	 ${BASEDIR}/lib/libSF.a \
	 ${BASEDIR}/lib/libgeometry.a \
	 -ly -ll -lm

all : Mat0 Mat1 Radial MarkMat

Mat0 : mat0.o
	$(CC) -o Mat0 mat0.o ${LIBS} 

MarkMat : markMat.o
	$(CC) -o MarkMat markMat.o ${LIBS} 

Mat1 : mat1.o
	$(CC) -o Mat1 mat1.o ${LIBS} 

Radial : radial.o
	$(CC) -o Radial radial.o ${LIBS}

install: all
	cp Mat0 Mat1 Radial MarkMat ${BASEDIR}/bin

clean:
	rm -f *.o Mat0 Mat1 Radial MarkMat

