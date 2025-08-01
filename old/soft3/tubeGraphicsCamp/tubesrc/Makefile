# Makefile for OpenGL code under OSX
# Author: J.K. Johnstone, inspired by www.macdevcenter.com/pub/a/mac/2005/04/01/opengl.html

ARCH	   = OSX
SHELL      = /bin/bash
CC         = g++
LIBPATH += -L"/System/Library/Frameworks/OpenGL.framework/Libraries"
FRAMEWORK = -framework GLUT
FRAMEWORK += -framework OpenGL
LIBRARIES  = -lGL -lGLU -lm

all: tube
.cpp:
	$(CC) $(FRAMEWORK) -o $@ $< $(LIBPATH) $(LIBRARIES)
clean :
	rm -f *.o *~ a.out core
	ls -l

touch :
	touch tube.cpp

