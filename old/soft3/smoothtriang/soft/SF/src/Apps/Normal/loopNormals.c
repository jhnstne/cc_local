/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  loopNormals.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

#define PI  3.14159265358979323846
#define MAX_NEIGH 100

void normalFunction(Vertex* v)
{
  Vertex* w;
  Point p[MAX_NEIGH];
  Scalar sine[MAX_NEIGH];
  Scalar cosine[MAX_NEIGH];
  int i,n;
  Vector U,V;

  i = 0;
  ForeachVertexVertex(v,w){
    p[i] = ReturnUDPoint(w);
    i++;
  } EndForeach;
  n = i; 

  for(i=0; i<n; i++){
    sine[i]   = (Scalar) sin( 2.0 * PI * ((double) i) / ((double) n));
    cosine[i] = (Scalar) cos( 2.0 * PI * ((double) i) / ((double) n));
  }

  U = PPvcN( n, p, cosine );
  V = PPvcN( n, p, sine );

  SetUDNormal(v, VDual( VVCross( U, V ) ));
}
  
