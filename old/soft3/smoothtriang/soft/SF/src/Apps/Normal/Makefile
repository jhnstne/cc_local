CC = gcc
BASEDIR = ../../..
INCLUDE = -I${BASEDIR}/include
CFLAGS = -g ${INCLUDE}

LIBS   = -L${BASEDIR}/lib \
	 -lmesh \
	 -lgeometry \
	 -lSF \
	 -lgeomat \
	 -ldstruct \
	 -L${BASEDIR}/../../lib \
	 -lm -ly -ll

LoopNormals: loopNormals.o main.o AddNormals.o
	$(CC) loopNormals.o main.o AddNormals.o ${LIBS} -o LoopNormals

AngleNormals: angleNormals.o main.o AddNormals.o
	$(CC) angleNormals.o main.o AddNormals.o ${LIBS} -o AngleNormals

CrossNormals: crossNormals.o main.o AddNormals.o
	$(CC) crossNormals.o main.o AddNormals.o ${LIBS} -o CrossNormals

SloanNormals: sloanNormals.o main.o AddNormals.o
	$(CC) sloanNormals.o main.o AddNormals.o ${LIBS} -o SloanNormals

UnitCrossNormals: unitCrossNormals.o main.o AddNormals.o
	$(CC) unitCrossNormals.o main.o AddNormals.o ${LIBS} -o UnitCrossNormals

Principal: principal.o main.o AddNormals.o svdfit.o lfit.o
	$(CC) principal.o main.o AddNormals.o svdfit.o lfit.o \
	${BASEDIR}/../../src/recipes/covsrt.o ${LIBS} -o Principal

t: t.o
	$(CC) t.c ${INCLUDE} ${LIBS} -o t

all: CrossNormals SloanNormals UnitCrossNormals AngleNormals LoopNormals

clean:
	rm -f  CrossNormals SloanNormals UnitCrossNormals AngleNormals \
		LoopNormals Principal t *.o
