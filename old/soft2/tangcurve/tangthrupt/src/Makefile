# Author: J.K. Johnstone

ARCH	   = OSX
SHELL      = /bin/bash
CC         = g++
LIBPATH   += -L"/System/Library/Frameworks/OpenGL.framework/Libraries"
FRAMEWORK  = -framework GLUT
FRAMEWORK += -framework OpenGL
CLAPACK    = /Users/jj/share/CLAPACK
CLAPACKLIB = $(CLAPACK)/lapack_LINUX.a \
             $(CLAPACK)/blas_LINUX.a \
	     $(CLAPACK)/F2CLIBS/libF77.a
LIBRARIES  = -lGL -lGLU -lm -ltcl $(CLAPACKLIB)
CBIN	   = /Users/jj/Cbin
INCLUDE    = -I$(CBIN) -I$(CLAPACK)

all: poletang
.cpp:
	$(CC) $(FRAMEWORK) -o $@ $< $(LIBPATH) $(LIBRARIES) $(INCLUDE)
clean :
	rm -f *.o *~ a.out core
	ls -l
touch :
	touch poletang.cpp

