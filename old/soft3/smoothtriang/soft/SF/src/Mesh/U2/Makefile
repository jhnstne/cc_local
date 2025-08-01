CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I. -I${BASEDIR}/include
LIBS = ${BASEDIR}/lib/geometry.a -lm
OBJS = meshGeometry.o userData.o userMalloc.o

libud.a: ${OBJS}
	ar r libud.a ${OBJS}
	ranlib libud.a
	chmod -f g+w $@

.c.o:
	$(CC) ${CFLAGS} ${INCLUDE} -c $<

clean:
	rm -f *.o libmesh.a y.tab.c bin/*
