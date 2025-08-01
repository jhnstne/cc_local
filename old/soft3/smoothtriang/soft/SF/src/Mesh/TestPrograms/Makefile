CC = gcc
CFLAGS = -g ${INCLUDE}
BASEDIR = ../../..
INCLUDE = -I${BASEDIR}/include -I../Mesh
LIBS = \
	../libmesh.a \
	../Operators/libmop.a \
	${BASEDIR}/lib/libgeomat.a \
	${BASEDIR}/lib/libdstruct.a \
	${BASEDIR}/lib/geometry.a \
	../Operators/libmop.a \
	-lm -ly -ll
590B = /u3/derose/590B

t: t.o
	$(CC) t.o -o t ${LIBS}

t.o: t.c
	$(CC) t.c -c ${INCLUDE} ${CFLAGS}

t2: t2.o
	$(CC) t2.o -o t2 ${LIBS}

t2.o: t2.c
	$(CC) t2.c -c ${INCLUDE} ${CFLAGS}

path: path.o
	$(CC) path.o -o path ${INCLUDE} ${LIBS}

icosahedron: icosahedron.o
	$(CC) icosahedron.o -o icosahedron ${INCLUDE} ${LIBS}

io: io.o 
	$(CC) io.o -g -o io ${INCLUDE} ${LIBS}

io.o: io.c
	$(CC) -g -c io.c ${INCLUDE} 

clean:
	rm -f *.o io icosahedron path t 
