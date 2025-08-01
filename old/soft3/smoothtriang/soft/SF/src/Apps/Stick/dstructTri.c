/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  dstructTri.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include "stick.h"

int ReadDstructTriangle(Triangle* t)
{
  Frame f;

  f = StdFrame(world);

  if ( ReadDstruct() == EOF ) {
    return EOF;
  }
  if ( !GetVertexPosition("Triangle.vertex1", f, &(t->p[0]))   ||
       !GetVertexPosition("Triangle.vertex2", f, &(t->p[1]))   ||
       !GetVertexPosition("Triangle.vertex3", f, &(t->p[2]))   ||
       !GetVertexNormal("Triangle.vertex1", f, &(t->n[0]))   ||
       !GetVertexNormal("Triangle.vertex2", f, &(t->n[1]))   ||
       !GetVertexNormal("Triangle.vertex3", f, &(t->n[2])) ) {
    fprintf(stderr, "ExtractDstructTriangle: dstruct not a triangle.  Exiting.\n");
    exit(1);
  }
  return 1;
}


void PrintDstructTriangle(Triangle t)
{
  PutVertexPosition("Triangle.vertex1", t.p[0]);
  PutVertexNormal("Triangle.vertex1", t.n[0]);
  if ( t.m.name != NULL ){
    SetMaterialName("Triangle.vertex1",t.m.name);
  } else {
    SetRGBMaterial("Triangle.vertex1", &t.m);
  }

  PutVertexPosition("Triangle.vertex2", t.p[1]);
  PutVertexNormal("Triangle.vertex2", t.n[1]);
  if ( t.m.name != NULL ){
    SetMaterialName("Triangle.vertex2",t.m.name);
  } else {
    SetRGBMaterial("Triangle.vertex2", &t.m);
  }

  PutVertexPosition("Triangle.vertex3", t.p[2]);
  PutVertexNormal("Triangle.vertex3", t.n[2]);
  if ( t.m.name != NULL ){
    SetMaterialName("Triangle.vertex3",t.m.name);
  } else {
    SetRGBMaterial("Triangle.vertex3", &t.m);
  }

  FlushDstruct();
}
