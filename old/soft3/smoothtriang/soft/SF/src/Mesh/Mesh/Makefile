CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I. -I${BASEDIR}/include
LIBS = ${BASEDIR}/lib/geometry.a -lm

libmesh.a: meshsym.o mesh.o y.tab.o meshutils.o \
		valid.o meshAlloc.o iterators.o debug.o Makefile
	ar r libmesh.a meshsym.o mesh.o y.tab.o meshutils.o \
		valid.o meshAlloc.o iterators.o debug.o
	ranlib libmesh.a
	chmod -f g+w $@

meshsym.o: meshsym.c mesh.h
	$(CC) ${CFLAGS} ${INCLUDE} -c meshsym.c

mesh.o: mesh.h mesh.c
	$(CC) ${CFLAGS} ${INCLUDE} -c mesh.c

iterators.o: mesh.h iterators.c
	$(CC) ${CFLAGS} ${INCLUDE} -c iterators.c

meshutils.o: mesh.h meshutils.c
	$(CC) ${CFLAGS} ${INCLUDE} -c meshutils.c

valid.o: valid.c
	$(CC) ${CFLAGS} ${INCLUDE} -c valid.c

meshAlloc.o: meshAlloc.c
	$(CC) ${CFLAGS} ${INCLUDE} -c meshAlloc.c

y.tab.o: y.tab.c mesh.h
	$(CC) ${CFLAGS} ${INCLUDE} -c y.tab.c

y.tab.c: newmesh.y
	yacc newmesh.y

install:
	touch libmesh.a
	(cd ..; make install)

clean:
	rm -f *.o libmesh.a y.tab.c bin/*
