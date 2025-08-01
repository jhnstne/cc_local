/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  angleNormals.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

void normalFunction(Vertex* v)
{
	Vertex* u;
	Point p;
	Vector x,y,w;
	int first=1;
	Vector vec;
	Point prev,current,firstP;
	Vertex* prevV;
	Vertex* firstV;
	
	ForeachVertexVertex(v,u){
		if ( first ) {
			p = ReturnUDPoint(u);
			vec = VCreate(StdFrame(SpaceOf(p)),0.0,0.0,0.0);
			firstP = prev = p;
			firstV = prevV = u;
			first = 0;
		} else {
			current = ReturnUDPoint(u);
			p = ReturnUDPoint(v);
			x = PPDiff(prev, p);
			if ( VMag(x) <= 1e-10 ) {
				AdjacentMessage(prevV, v);
				goto error1;
			}
			y = PPDiff(current, p);
			if ( VMag(y) <= 1e-10 ) {
				AdjacentMessage(u, v);
				goto error1;
			}
			w = VVCross( x, y );
			w = SVMult( 1.0/VMag(x) * 1.0/VMag(y), w );
			vec = VVAdd( vec, w );
error1: 
			prev = current;
			prevV = u;
		}
	} EndForeach;

	if ( !BoundaryVertex(v) ){
		current = firstP;
		p = ReturnUDPoint(v);
		x = PPDiff(prev, p);
		if ( VMag(x) <= 1e-10 ) {
			AdjacentMessage(prevV, v);
			goto error;
		}
		y = PPDiff(current, p);
		if ( VMag(y) <= 1e-10 ) {
			AdjacentMessage(prevV, v); /* hack! */
			goto error;
		}
		w = VVCross( x, y );
		w = SVMult( 1.0/VMag(x) * 1.0/VMag(y), w );
		vec = VVAdd( vec, w );
	}
error:

	if ( VMag(vec) <= 1e-10 ) {
		fprintf(stderr,"angleNormals: |vec| == %g\n",VMag(vec));
		fprintf(stderr,"	v = %s\n",v->name);
	}
	SetUDNormal(v, VDual(SVMult(1.0/VMag(vec),vec)));
}

AdjacentMessage(Vertex* v1, Vertex* v2)
{
	fprintf(stderr,"Vertex %s == Vertex %s\n",v1->name,v2->name);
}
