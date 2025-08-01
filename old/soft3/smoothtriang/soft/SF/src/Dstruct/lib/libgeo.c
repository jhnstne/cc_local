/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  libgeo.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"

#define MAX 1000

BOOLEAN pPutVertexPosition(Lnode **d, char *s, Point p)
{
	Scalar x,y,z;
	char string[MAX];
	char *pt;
	int result=TRUE;

	if ( strlen(s) + 8 > MAX )
	  return FALSE;

	strcpy(string,s);
	strcat(string,".pos");
	pt = string + strlen(string);

	PCoords(p,StdFrame(SpaceOf(p)),&x,&y,&z);

	sprintf(pt,"[0]");
	result = pPutScalar( d, string, x) && result;

	sprintf(pt,"[1]");
	result = pPutScalar( d, string, y) && result;

	sprintf(pt,"[2]");
	result = pPutScalar( d, string, z) && result;

	return result;
}


BOOLEAN pPutVertexNormal(Lnode **d, char *s, Normal n)
{
	Scalar x,y,z;
	char string[MAX];
	char *p;
	int result=TRUE;

	if ( strlen(s) + 8 > MAX )
	  return FALSE;

	strcpy(string,s);
	strcat(string,".norm");
	p = string + strlen(string);

	NCoords(n,StdFrame(SpaceOf(n)),&x,&y,&z);

	sprintf(p,"[0]");
	result = pPutScalar( d, string, x) && result;

	sprintf(p,"[1]");
	result = pPutScalar( d, string, y) && result;

	sprintf(p,"[2]");
	result = pPutScalar( d, string, z) && result;

	return result;
}


BOOLEAN dGetVertexNormal(Lnode *d, char *s, Frame f, Normal *n)
{
	Scalar x,y,z;
	char string[MAX];
	char *p;

	if ( strlen(s) + 8 > MAX )
	  return FALSE;

	strcpy(string,s);
	strcat(string,".norm");
	p = string + strlen(string);

	sprintf(p,"[0]");
	if ( ! dGetScalar( d, string, &x) )
	  return FALSE;

	sprintf(p,"[1]");
	if ( ! dGetScalar( d, string, &y) )
	  return FALSE;

	sprintf(p,"[2]");
	if ( ! dGetScalar( d, string, &z) )
	  return FALSE;

	*n = NCreate( f, x, y, z );
	return TRUE;
}


BOOLEAN dGetVertexPosition(Lnode *d, char *s, Frame f, Point *p)
{
	Scalar x,y,z;
	char string[MAX];
	char *pt;

	if ( strlen(s) + 8 > MAX )
	  return FALSE;

	strcpy(string,s);
	strcat(string,".pos");
	pt = string + strlen(string);

	sprintf(pt,"[0]");
	if ( ! dGetScalar( d, string, &x) )
	  return FALSE;

	sprintf(pt,"[1]");
	if ( ! dGetScalar( d, string, &y) )
	  return FALSE;

	sprintf(pt,"[2]");
	if ( ! dGetScalar( d, string, &z) )
	  return FALSE;

	*p = PCreate( f, x, y, z );
	return TRUE;
}




BOOLEAN pPutVector(Lnode **d, char *s, Vector v)
{
	Scalar x,y,z;
	char string[MAX];
	char *p;
	int result=TRUE;

	if ( strlen(s) + 8 > MAX )
	  return FALSE;

	strcpy(string,s);
	p = string + strlen(string);

	VCoords(v,StdFrame(SpaceOf(v)),&x,&y,&z);

	sprintf(p,"[0]");
	result = pPutScalar( d, string, x) && result;

	sprintf(p,"[1]");
	result = pPutScalar( d, string, y) && result;

	sprintf(p,"[2]");
	result = pPutScalar( d, string, z) && result;

	return result;
}


BOOLEAN dGetVector(Lnode *d, char *s, Frame f, Vector *v)
{
	Scalar x,y,z;
	char string[MAX];
	char *p;

	if ( strlen(s) + 8 > MAX )
	  return FALSE;

	strcpy(string,s);
	p = string + strlen(string);

	sprintf(p,"[0]");
	if ( ! dGetScalar( d, string, &x) )
	  return FALSE;

	sprintf(p,"[1]");
	if ( ! dGetScalar( d, string, &y) )
	  return FALSE;

	sprintf(p,"[2]");
	if ( ! dGetScalar( d, string, &z) )
	  return FALSE;

	*v = VCreate( f, x, y, z );
	return TRUE;
}
