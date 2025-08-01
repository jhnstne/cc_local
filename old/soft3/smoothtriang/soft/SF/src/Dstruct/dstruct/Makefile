BASEDIR = ../../..
CC = gcc
CFLAGS = -g -I${BASEDIR}/include
LIBS = libdstruct.a -ly -ll

libdstruct.a: y.tab.o lex.yy.o dstruct.o getset.o dstructMem.o lineno.o
	ar r libdstruct.a y.tab.o lex.yy.o dstruct.o getset.o dstructMem.o\
		lineno.o
	ranlib libdstruct.a

install: libdstruct.a
	cp libdstruct.a ${BASEDIR}/lib
	ranlib ${BASEDIR}/lib/libdstruct.a
	cp dstruct.h getset.h ${BASEDIR}/include

parse : parse.o libdstruct.a dstruct.h
	gcc -o parse parse.o ${LIBS} -lm

y.tab.c : parser.y
	yacc -dv parser.y
	(sed -e 's/yy/DSyy/g' < y.tab.c > y.tab.c.$$; \
	 mv y.tab.c.$$ y.tab.c)
	(sed -e 's/yy/DSyy/g' < y.tab.h > y.tab.h.$$; \
	 mv y.tab.h.$$ y.tab.h)

lex.yy.c : scanner.l
	lex scanner.l
	(sed -e 's/yy/DSyy/g' < lex.yy.c > lex.yy.c.$$; \
	 mv lex.yy.c.$$ lex.yy.c)

clean:
	rm -f *.o libdstruct.a y.output lex.yy.c y.tab.c y.tab.h parse
