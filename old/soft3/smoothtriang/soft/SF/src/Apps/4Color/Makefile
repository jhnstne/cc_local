CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I${BASEDIR}/include
LIBS = ${BASEDIR}/lib/libmesh.a \
	${BASEDIR}/lib/geometry.a \
	${BASEDIR}/lib/libSF.a \
	${BASEDIR}/lib/libgeomat.a\
	${BASEDIR}/lib/libdstruct.a \
	${BASEDIR}/lib/libSF.a \
	${BASEDIR}/lib/geometry.a \
	-lm -ly -ll

4Color: 4Color.o 
	$(CC) 4Color.o ${LIBS} -o 4Color

4Color.o: 4Color.c
	$(CC) ${CFLAGS} ${INCLUDE} -c 4Color.c

install: 4Color
	cp 4Color ${BASEDIR}/bin

clean:
	rm -f *.o 4Color
