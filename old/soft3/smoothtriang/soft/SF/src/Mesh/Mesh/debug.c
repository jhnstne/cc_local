/*
 *----------------------------------------------------------------------
 *  File:  debug.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>

int debugLevel(int n)
{
	int l;
	char* s;
	char* getenv();

	l = -1;
	s = getenv("SDEBUG");
	if ( s == NULL ) {
		return 0;
	}
	if ( *s == NULL && n == 0) {
		return 1;
	}
	sscanf(s,"%d",&l);
	if ( l >= n ) {
		return 1;
	} else {
		return 0;
	}
}
