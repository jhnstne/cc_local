BASEDIR = ../../..

CC = gcc

CFLAGS = -g

INCLUDE = -I${BASEDIR}/include

LIBS   = \
	userMalloc.o\
	 -L${BASEDIR}/lib \
	 -lkcurve \
	 -lmesh \
	 -ldstruct \
	 -lSFlib \
	 -lgeometry \
	 -lgeomat \
	 -ly -ll -lm


libfunc.a: func.o userMalloc.o
	@ rm -f libfunc.a 
	ar q libfunc.a func.o userMalloc.o
	ranlib libfunc.a

func.o: func.c
	$(CC) -c func.c ${INCLUDE} ${CFLAGS}

userMalloc.o: userMalloc.c
	$(CC) -c userMalloc.c ${INCLUDE} ${CFLAGS}

t: t.c libfunc.a
	$(CC) t.c -o t libfunc.a ${INCLUDE} ${CFLAGS} ${LIBS}

install: libfunc.a
	cp libfunc.a ${BASEDIR}/lib
	ranlib ${BASEDIR}/lib/libfunc.a

clean:
	rm -f libfunc.a *.o *~
