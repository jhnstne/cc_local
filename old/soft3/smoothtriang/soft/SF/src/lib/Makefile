#
# Author: Tony DeRose
# Last Modified: Fri Feb 09, 1990 at 04:06:56 PM
# 
BASEDIR= ../..
CC = gcc
CFLAGS = -g -I${BASEDIR}/include 
INCL   = nhood.h material.h patch.h util.h vertex.h commandline.h\
	 usage.h tess.h tpbezier.h intersection.h geom.h bezier.h sff.h\
	mindex.h ratpatch.h boundary.h

OBJS = commandline.o geom.o intersection.o material.o nhood.o patch.o \
	patchsff.o print.o tptess.o tpbezier.o tess.o util.o usage.o \
	bezier.o PolyRoots.o sff.o mindex.o ratpatch.o boundary.o

libSF.a : ${OBJS}
	ar r libSF.a ${OBJS}
	ar ts libSF.a

install : libSF.a
	rm *.o
	cp libSF.a ${BASEDIR}/lib
	cp ${INCL} ${BASEDIR}/include
	ar ts ${BASEDIR}/lib/libSF.a
	make man

man: 
	cp material.3 ${BASEDIR}/man/man3
	nroff -man material.3 > ${BASEDIR}/man/cat3/material.0
	cp bezier.3 ${BASEDIR}/man/man3
	nroff -man bezier.3 > ${BASEDIR}/man/cat3/bezier.0
	cp commandline.3 ${BASEDIR}/man/man3
	nroff -man commandline.3 > ${BASEDIR}/man/cat3/commandline.0
	cp geom.3 ${BASEDIR}/man/man3
	nroff -man geom.3 > ${BASEDIR}/man/cat3/geom.0
	cp intersection.3 ${BASEDIR}/man/man3
	nroff -man intersection.3 > ${BASEDIR}/man/cat3/intersection.0
	cp patch.3 ${BASEDIR}/man/man3
	nroff -man patch.3 > ${BASEDIR}/man/cat3/patch.0
	cp sff.3 ${BASEDIR}/man/man3
	nroff -man sff.3 > ${BASEDIR}/man/cat3/sff.0
	cp tess.3 ${BASEDIR}/man/man3
	nroff -man tess.3 > ${BASEDIR}/man/cat3/tess.0
	cp tpbezier.3 ${BASEDIR}/man/man3
	nroff -man tpbezier.3 > ${BASEDIR}/man/cat3/tpbezier.0

clean:
	rm -f libSF.a *.o

CFLAGS = -g -I${BASEDIR}/include -L${BASEDIR}/lib
LIBS   = \
	-lkcurve \
	-lmesh \
	-lgeomat\
	-lSFlib.a \
	-ldstruct\
	-lgeometry \
	-lgeomat\
	-lm -ly -ll

t: t.o libSF.a
	$(CC) -o t t.o ${CFLAGS} ${LIBS}
