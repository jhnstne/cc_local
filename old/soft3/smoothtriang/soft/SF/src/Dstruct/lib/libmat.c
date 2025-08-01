/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  libmat.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "material.h"

#define MAX 1000

BOOLEAN dGetRGBMaterial(Lnode *d, char* p, Material* m)
{
	char buf[MAX];
	char *name, *pt;
	Scalar dr,dg,db,sr,sg,sb,spec;
  

	name = buf;

	strcpy(buf, p);
	pt = buf + strlen(buf);

	strcpy(pt, ".material.mat.diffuse[0]");
	if ( dGetScalar( d, buf, &dr ) ) {
		strcpy(pt, ".material.mat.diffuse[1]");
		if ( !dGetScalar( d, buf, &dg ) )
		  dg = 0.0;

		strcpy(pt, ".material.mat.diffuse[2]");
		if ( !dGetScalar( d, buf, &db ) )
		  db = 0.0;
    
		strcpy(pt, ".material.mat.specular[0]");
		if ( !dGetScalar( d, buf, &sr ) )
		  sr = 0.0;

		strcpy(pt, ".material.mat.specular[1]");
		if ( !dGetScalar( d, buf, &sg ) )
		  sg = 0.0;

		strcpy(pt, ".material.mat.specular[2]");
		if ( !dGetScalar( d, buf, &sb ) )
		  sb = 0.0;

		strcpy(pt, ".material.mat.specularity");
		if ( !dGetScalar( d, buf, &spec ) )
		  spec = 0.0;

		*m = MaterialCreate( dr, dg, db, sr, sg, sb, (int)spec );
	} else {
		strcpy(pt, ".material.name");
		if ( dGetString( d, buf, &name ) ) {
			if ( MaterialLookup( name, m ) ) {
				return TRUE;
			} else {
				return FALSE;
			}
		} else {
			return FALSE;
		}
	}
	return TRUE;
}


BOOLEAN pSetMaterialName(Lnode **d, char *p, char *name)
{
	char buf[MAX];
	strcpy(buf, p);
	strcat(buf,".material.name");

	return pPutString(d,buf,name);
}


BOOLEAN pSetRGBMaterial(Lnode **d, char* p, Material* m)
{
	char buf[MAX];
	char *pt;
	int result;

	strcpy(buf, p);
	pt = buf + strlen(buf);

	strcpy(pt, ".material.mat.diffuse[0]");
	pPutScalar(d, buf, m->diffuse.r);

	strcpy(pt, ".material.mat.diffuse[1]");
	pPutScalar(d, buf, m->diffuse.g);

	strcpy(pt, ".material.mat.diffuse[2]");
	pPutScalar(d, buf, m->diffuse.b);

	strcpy(pt, ".material.mat.specular[0]");
	pPutScalar(d, buf, m->specular.r);

	strcpy(pt, ".material.mat.specular[1]");
	pPutScalar(d, buf, m->specular.g);

	strcpy(pt, ".material.mat.specular[2]");
	pPutScalar(d, buf, m->specular.b);

	strcpy(pt, ".material.mat.specularity");
	pPutScalar(d, buf, (double)m->specularity);

	return TRUE;
}
