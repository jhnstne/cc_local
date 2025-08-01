/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  sloanNormals
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

#define EPS 0.0000000000001

void normalFunction(Vertex* v)
{
  Vertex* w;
  Vector t;
  int first=1;
  Vector vec;
  Point prev,current,firstP;
  double area2;
  Point p;

  GetUDPoint(v, &p);
  ForeachVertexVertex(v,w){
    if ( first ) {
      GetUDPoint(w, &firstP);
      vec = VCreate(StdFrame(SpaceOf(firstP)),0.0,0.0,0.0);
      prev = firstP;
      first = 0;
    } else {
      GetUDPoint(w, &current);
      t = VVCross(PPDiff(prev,p), PPDiff(current,p));
      area2 = VVDot(t,t);
      if ( area2 > 0.00000000001 )
	vec = VVAdd( vec, SVMult( 1.0/area2, t) );
      prev = current;
    }
  } EndForeach;
  if ( !BoundaryVertex(v) ){
    current = firstP;
    t = VVCross(PPDiff(prev,p), PPDiff(current,p));
    area2 = VVDot(t,t);
    if ( area2 > 0.00000000001 )
      vec = VVAdd( vec, SVMult( 1.0/area2, t) );
  }
  if ( VMag(vec) > EPS ){
    vec = SVMult( 1.0/VMag(vec), vec);
  } else {
#if 0
    if ( BoundaryVertex(v) ) {
      fprintf(stderr,"SloanNormals: zero length on boundary.\n");
    } else {
      fprintf(stderr,"SloanNormals: zero length NOT on boundary.\n");
    }
#endif
  }
  SetUDNormal(v, VDual(vec));
}
