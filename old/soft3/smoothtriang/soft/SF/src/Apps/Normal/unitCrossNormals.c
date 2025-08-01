/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  unitCrossNormals.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

void normalFunction(Vertex* v)
{
  Vertex* w;
  Point p;
  Vector t;
  int first=1;
  Vector vec;
  Point prev,current,firstP;

  ForeachVertexVertex(v,w){
    if ( first ) {
      p = ReturnUDPoint(w);
      vec = VCreate(StdFrame(SpaceOf(p)),0.0,0.0,0.0);
      firstP = prev = p;
      first = 0;
    } else {
      current = ReturnUDPoint(w);
      t = VVCross(PPDiff(prev,ReturnUDPoint(v)), 
		  PPDiff(current,ReturnUDPoint(v)));
      vec = VVAdd( vec, SVMult( 1.0/VMag(t),t) );
      prev = current;
    }
  } EndForeach;
  if ( !BoundaryVertex(v) ){
    current = firstP;
    t = VVCross(PPDiff(prev,ReturnUDPoint(v)), 
		PPDiff(current,ReturnUDPoint(v)));
    vec = VVAdd( vec, SVMult( 1.0/VMag(t), t) );
  }
  SetUDNormal(v, VDual(vec));
}
  
