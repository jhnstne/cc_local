#
# Authors: Tony DeRose and Steve Mann
# 

CC = gcc 

OBJS= GregoryCharrot.o 

BASEDIR= ../../..
CFLAGS = -g -I${BASEDIR}/include -L${BASEDIR}/lib
LIBS   = \
	-lkcurve \
	-lmesh \
	-ldstruct\
	-lgeometry \
	-lSF \
	-lgeomat\
	-lm -ly -ll

GregoryCharrot : $(OBJS)
	$(CC) ${CFLAGS} -o GregoryCharrot $(OBJS) $(LIBS)

clean:
	-@ test -f core && rm -f core
	-@ test -f *.o && rm -f *.o
	-@ test -f GregoryCharrot && rm -f GregoryCharrot
