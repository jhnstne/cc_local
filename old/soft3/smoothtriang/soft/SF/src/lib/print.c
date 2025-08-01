/*
 *----------------------------------------------------------------------
 *  File:  print.c
 *	I got tired of always writing code to do this, so here's
 *	some print routines.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "geometry.h"

int PrintPoint(FILE* fp, Point p)
{
	Scalar x,y,z;

	PCoords(p, StdFrame(SpaceOf(p)), &x, &y, &z);
	if ( Dim(SpaceOf(p)) == 2 ) {
		fprintf(fp,"(%g %g)",x,y);
	} else {
		fprintf(fp,"(%g %g %g)",x,y,z);
	}
	return 0;
}


int PrintVector(FILE* fp, Vector v)
{
	Scalar x,y,z;

	VCoords(v, StdFrame(SpaceOf(v)), &x, &y, &z);
	if ( Dim(SpaceOf(v)) == 2 ) {
		fprintf(fp,"(%g %g)",x,y);
	} else {
		fprintf(fp,"(%g %g %g)",x,y,z);
	}
	return 0;
}


int PrintNormal(FILE* fp, Normal n)
{
	Scalar x,y,z;

	NCoords(n, StdFrame(SpaceOf(n)), &x, &y, &z);
	if ( Dim(SpaceOf(n)) == 2 ) {
		fprintf(fp,"(%g %g)",x,y);
	} else {
		fprintf(fp,"(%g %g %g)",x,y,z);
	}
	return 0;
}

#define MAX_STRINGS 10

char* PString(Point p)
{
	Scalar x,y,z;
	static char s[MAX_STRINGS][100];
	static int cur=0;

	cur = cur % MAX_STRINGS;
	PCoords(p, StdFrame(SpaceOf(p)), &x, &y, &z);
	if ( Dim(SpaceOf(p)) == 2 ) {
		sprintf(s[cur],"(%g %g)",x,y);
	} else {
		sprintf(s[cur],"(%g %g %g)",x,y,z);
	}
	return s[cur++];
}


char* VString(Vector v)
{
	Scalar x,y,z;
	static char s[MAX_STRINGS][100];
	static int cur=0;

	cur = cur % MAX_STRINGS;
	VCoords(v, StdFrame(SpaceOf(v)), &x, &y, &z);
	if ( Dim(SpaceOf(v)) == 2 ) {
		sprintf(s[cur],"(%g %g)",x,y);
	} else {
		sprintf(s[cur],"(%g %g %g)",x,y,z);
	}
	return s[cur++];
}


char* NString(Normal n)
{
	Scalar x,y,z;
	static char s[MAX_STRINGS][100];
	static int cur=0;

	cur = cur % MAX_STRINGS;
	NCoords(n, StdFrame(SpaceOf(n)), &x, &y, &z);
	if ( Dim(SpaceOf(n)) == 2 ) {
		sprintf(s[cur],"(%g %g)",x,y);
	} else {
		sprintf(s[cur],"(%g %g %g)",x,y,z);
	}
	return s[cur++];
}
