#
# Authors: Tony DeRose and Steve Mann
# 
CC = gcc
BASEDIR= ../../..
CFLAGS = -g -I${BASEDIR}/include
LIBS   = ctutils.o\
	${BASEDIR}/lib/libgeometry.a \
	 ${BASEDIR}/lib/libSF.a \
	 ${BASEDIR}/lib/libgeomat.a \
	 ${BASEDIR}/lib/libmesh.a \
	 ${BASEDIR}/lib/libdstruct.a \
	 -lm -ly -ll

all: CloughTocher CloughTocherB Farin Foley MFarin

CloughTocher : CloughTocher.o ctutils.o
	$(CC) -o CloughTocher CloughTocher.o $(LIBS)

CloughTocherB : CloughTocherB.o ctutils.o
	$(CC) -o CloughTocherB CloughTocherB.o $(LIBS)

Farin : Farin.o ctutils.o
	$(CC) -o Farin Farin.o $(LIBS)

Foley : Foley.o ctutils.o
	$(CC) -o Foley Foley.o $(LIBS)

MFarin : MFarin.o ctutils.o
	$(CC) -o MFarin MFarin.o $(LIBS)

ctutils.o: ctutils.c
	$(CC) -c ctutils.c ${CFLAGS}

clean:
	rm -f CloughTocher Farin Foley MFarin *.o *.wff.Z *~
