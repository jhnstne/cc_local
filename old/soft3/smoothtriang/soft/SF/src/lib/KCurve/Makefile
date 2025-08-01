CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I${BASEDIR}/include
LIBS = -L${BASEDIR}/lib \
	-lgeometry \
	-lsff \
	-l590B \
	-lgeomat \
	-lmesh \
	-ldstruct \
	-lm -ly -ll

kcurve.a: kcurve.o curve.o kcurve2.o
	ar r kcurve.a kcurve.o curve.o kcurve2.o
	ranlib kcurve.a

install:
	make kcurve.a
	cp kcurve.a ${BASEDIR}/lib/libkcurve.a
	ranlib ${BASEDIR}/lib/libkcurve.a 

clean:
	rm -f KCurve kcurve.a *.o

KCurve: kcurve.o curve.o kCurve.o ForSteve.o postscript.o
	$(CC) kcurve.o curve.o kCurve.o ForSteve.o \
		postscript.o ${LIBS} -o KCurve

kCurve.o: kCurve.c
	$(CC) ${CFLAGS} ${INCLUDE} -c kCurve.c

ForSteve.o: ForSteve.c
	$(CC) ${CFLAGS} ${INCLUDE} -c ForSteve.c

kcurve2.o: kcurve2.c
	$(CC) ${CFLAGS} ${INCLUDE} -c kcurve2.c

kcurve.o: kcurve.c
	$(CC) ${CFLAGS} ${INCLUDE} -c kcurve.c

curve.o: curve.c
	$(CC) ${CFLAGS} ${INCLUDE} -c curve.c

postscript.o: postscript.c
	$(CC) ${CFLAGS} ${INCLUDE} -c postscript.c

lint:
	lint -abhu ${INCLUDE} kcurve2.c
