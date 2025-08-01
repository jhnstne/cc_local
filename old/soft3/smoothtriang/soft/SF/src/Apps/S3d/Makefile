#
# Author: Tony DeRose
# 
CC = gcc

BASEDIR= ../../..

CFLAGS = -g -I${BASEDIR}/include 

LIBS   = -L${BASEDIR}/lib \
	 -lgeomat -lSF -ldstruct -lgeometry -g -ly -ll -lm

S3d:	s3dFormat.o s3d.o
	$(CC) -o S3d s3d.o s3dFormat.o ${LIBS} 

install: S3d
	cp S3d ${BASEDIR}/bin

clean:
	rm -f S3d s3d.o s3dFormat.o
