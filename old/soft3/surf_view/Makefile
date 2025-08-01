# Author: J.K. Johnstone

ARCH	   = OSX
SHELL      = /bin/bash
CC         = g++
LIBPATH   += -L"/System/Library/Frameworks/OpenGL.framework/Libraries"
FRAMEWORK  = -framework GLUT
FRAMEWORK += -framework OpenGL
CBIN	   = /Users/jj/Software/Cbin
CLAPACK    = /Users/jj/Downloads/CLAPACK-3.2.1
CLAPACKLIB = $(CLAPACK)/lapack_LINUX.a \
             $(CLAPACK)/blas_LINUX.a \
	     $(CLAPACK)/F2CLIBS/libf2c.a
LIBRARIES  = -lGL -lGLU -lm -ltcl $(CLAPACKLIB)
INCLUDE    = -I$(CBIN) -I$(CLAPACK)/INCLUDE

all: surf_view

.cpp:
	$(CC) -frounding-math $(FRAMEWORK) -o $@ $< $(LIBPATH) \
	$(LIBRARIES) $(INCLUDE)

clean :
	rm -f *.o *~ a.out core
	ls -l

touch :
	touch surf_view.cpp

