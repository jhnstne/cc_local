/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  crossNormals
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"

void normalFunction(Vertex* v)
{
  Vertex* w;
  Point p;
  int first=1;
  Vector vec;
  Point prev,current,firstP;

  ForeachVertexVertex(v,w){
    if ( first ) {
      p = ReturnUDPoint(w);
      vec = VCreate(StdFrame(SpaceOf(p)),0.0,0.0,0.0);
      firstP = prev = ReturnUDPoint(w);
      first = 0;
    } else {
      current = ReturnUDPoint(w);
      vec = VVAdd( vec, VVCross(PPDiff(prev,ReturnUDPoint(v)),
				PPDiff(current,ReturnUDPoint(v))) );
      prev = current;
    }
  } EndForeach;
  if ( !BoundaryVertex(v) ){
    current = firstP;
    vec = VVAdd( vec, VVCross(PPDiff(prev,ReturnUDPoint(v)),
			      PPDiff(current,ReturnUDPoint(v))) );
  }
  SetUDNormal(v, VDual(SVMult(1.0/VMag(vec), vec)));
}
  
