#
# Authors: Tony DeRose and Steve Mann
# 
CC = gcc
BASEDIR= ../../..
CFLAGS = -c -g -I${BASEDIR}/include -DNOPFOLEY
LFLAGS = -g -L${BASEDIR}/lib 
LIBS   = \
	-lkcurve \
	-lmesh \
	-ldstruct\
	-lgeometry \
	-lSF \
	-lgeomat\
	-lm -ly -ll

all : Gregory

OBJS = Gregory.o 

Gregory : $(OBJS)
	$(CC) $(LFLAGS) -o Gregory $(OBJS) $(LIBS)

clean : 
	rm -f Gregory *.o

