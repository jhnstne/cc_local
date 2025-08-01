CC = gcc
CFLAGS = -g
BASEDIR = ../../..
INCLUDE = -I.. -I${BASEDIR}/include
OBJS = \
    addremovemesh.o      error.o              \
    addvertexedge.o      removevertexedge.o   destroyvertex.o	\
    createface.o         joinfaces.o          splitedge.o       \
    destroyface.o        joinhalfedge.o       splitface.o	\
    joinvertices.o	 centersplitface.o    fixvertices.o	\
    createvertex.o
#newobjs.o

LIBS = ${BASEDIR}/lib/geometry.a \
	${BASEDIR}/lib/590B.a \
	../libmesh.a \
	${BASEDIR}/lib/libgeomat.a \
	${BASEDIR}/lib/libdstruct.a \
	${BASEDIR}/lib/geometry.a\
	-lm -ly -ll

libmop.a : ${OBJS}
	rm -f libmop.a
	ar q libmop.a ${OBJS}
	ranlib libmop.a

${OBJS} : all.h fve.h

.c.o :
	$(CC) ${CFLAGS} ${INCLUDE} -c $<

install:
	touch libmop.a
	(cd .. ; make install)

t: libmop.a t.o
	$(CC) t.o ${CFLAGS} libmop.a ${LIBS} -o t

rmcom: libmop.a rmcom.o
	$(CC) rmcom.o ${CFLAGS} libmop.a ${LIBS} -o rmcom

clean:
	rm -f *.o *.a *~ rmcom t
