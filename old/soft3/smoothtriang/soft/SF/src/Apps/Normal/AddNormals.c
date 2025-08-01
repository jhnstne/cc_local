/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  AddNormals.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"

void AddNormals(Mesh* m, void (*normalFunc)())
{
	Vertex* v;
	
	ForeachMeshVertex(m,v){
/*  DANGER!!! "v->rep" is an illegal access (user code shouldn't
    access the internals of the data structure) */
		if ( v->rep != NULL ){
			normalFunc(v);
		} else {
			Point p;
			
			p = ReturnUDPoint(v);
			SetUDNormal( v, VDual( VZero( SpaceOf(p) ) ) );
		}
		SetUDGeoFlags( v, ReturnUDGeoFlags(v) | G_NORMAL );
	} EndForeach;
}
