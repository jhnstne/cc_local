SHELL   = /bin/csh
CFLAGS  = -cckr -float -prototypes -O
LDFLAGS = -lsphere -lgl_s -lX11_s -lc_s -lm 
#PROGS   = `ls *.c | sed 's/\.c$$/'
CC      = cc
CCOPTS1 = -ansi -O2

sweep:	sweep.o
	$(CC) sweep.o $(CFLAGS) $(LDFLAGS) -o sweep

# If you want to run a debugger on these programs, remove the -s from LDFLAGS
# and change the -O in CFLAGS to -g.
