# File: Makefile with OpenGL/CLAPACK/Cbin
# Author: J.K. Johnstone

ARCH	   = OSX
SHELL      = /bin/bash
CC         = g++
LIBPATH += -L"/System/Library/Frameworks/OpenGL.framework/Libraries"
FRAMEWORK = -framework GLUT
FRAMEWORK += -framework OpenGL
CLAPACK    = /Users/jj/Downloads/CLAPACK-3.2.1
CLAPACKLIB = $(CLAPACK)/lapack_LINUX.a \
             $(CLAPACK)/blas_LINUX.a \
	     $(CLAPACK)/F2CLIBS/libf2c.a
LIBRARIES  = -lGL -lGLU -lm -ltcl $(CLAPACKLIB)
CBIN	   = /Users/jj/Software/Cbin
INCLUDE    = -I$(CBIN) -I$(CLAPACK)
all: interpolate
.cpp:
	$(CC) $(FRAMEWORK) -o $@ $< $(LIBPATH) $(LIBRARIES) $(INCLUDE)
clean :
	rm -f *.o *~ a.out core
	ls -l
touch :
	touch interpolate.cpp

