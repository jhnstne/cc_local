CFLAGS	= -g
all:	quat_min

quat_min:	deigens.o quat_min.c
	cc $(CFLAGS) -o quat_min quat_min.c deigens.o -lm
