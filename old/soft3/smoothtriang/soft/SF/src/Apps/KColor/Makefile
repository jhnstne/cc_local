CC = gcc

BASEDIR= ../../..

CFLAGS = -g -I${BASEDIR}/include 

LIBS   = \
	 ${BASEDIR}/lib/libgeomat.a \
	 ${BASEDIR}/lib/libdstruct.a \
	 ${BASEDIR}/lib/libSF.a \
	 ${BASEDIR}/lib/libgeometry.a \
	 -ly -ll -lm


KColor : KColor.o
	$(CC) -o KColor KColor.o ${LIBS}

install : KColor
	cp KColor ${BASEDIR}/bin

clean:
	rm -f KColor KColor.o
