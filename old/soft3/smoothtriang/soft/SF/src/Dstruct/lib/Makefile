CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I../dstruct -I${BASEDIR}/include

libgeomat.a: libgeo.o libmat.o
	ar r libgeomat.a libgeo.o libmat.o
	ranlib libgeomat.a

install: libgeomat.a
	cp libgeomat.a ${BASEDIR}/lib
	ranlib ${BASEDIR}/lib/libgeomat.a
	cp libgeo.h libmat.h ${BASEDIR}/include

libgeo.o: libgeo.c libgeo.h
	$(CC) ${CFLAGS} ${INCLUDE} -c libgeo.c

libmat.o: libmat.c libmat.h
	$(CC) ${CFLAGS} ${INCLUDE} -c libmat.c

clean:
	rm -f libgeomat.a libgeo.o libmat.o
