#
# Makefile for the affine/euclidean geometry library
#
# Copyright (c) 1987, 1988 Tony D. DeRose
#
OBJ        = geometry.o matrix.o
BASEDIR    = ../..
INSTALLDIR = ${BASEDIR}/lib
INCLUDEDIR = ${BASEDIR}/include
MANDIR     = ${BASEDIR}/man/man3
CFLAGS = -g
CC = gcc

geometry.a: $(OBJ)
	ar r geometry.a geometry.o matrix.o
	ranlib geometry.a

install: geometry.a
	cp geometry.a $(INSTALLDIR)
	ranlib $(INSTALLDIR)/geometry.a
	cp geometry.h matrix.h $(INCLUDEDIR)
	cp geometry.doc $(MANDIR)/geometry.3
	( cd ${MANDIR} ; nroff -man geometry.3 > ../cat3/geometry.0 )

clean:
	rm -f *.o geometry.a
	
geometry_p.o: geometry.c
	mv geometry.o save.o
	$(CC) -c geometry.c -pg $(CFLAGS)
	mv geometry.o geometry_p.o
	mv save.o geometry.o

matrix_p.o: matrix.c
	mv matrix.o save.o
	$(CC) -c matrix.c -pg $(CFLAGS)
	mv matrix.o matrix_p.o
	mv save.o matrix.o

geometry_p.a: geometry_p.o matrix_p.o
	ar q geometry_p.a geometry_p.o matrix_p.o
	ranlib geometry_p.a
