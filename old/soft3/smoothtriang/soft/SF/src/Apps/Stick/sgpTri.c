/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  sgpTri.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include "stick.h"

int ReadSGPTriangle(Triangle* t)
{
  float x,y,z;
  int i;

  for(i=0; i<3; i++){
    if ( scanf(" [ %f %f %f ]",&x,&y,&z) == EOF ){
      return EOF;
    }
    t->p[i] = PCreate(worldF, x,y,z);
  }

  for(i=0; i<3; i++){
    if ( scanf(" [ %f %f %f ]",&x,&y,&z) == EOF ) {
      return EOF;
    }
    t->n[i] = NCreate(worldF, x,y,z);
  }

  for(i=0;i<3;i++){
    int s;
    scanf(" [ %f %f %f ]",&x,&y,&z);
    scanf(" [ %f %f %f ]",&x,&y,&z);
    scanf(" %d ",&s);
  }
  if ( scanf(" sgpTriangle") == EOF ){
    return EOF;
  }
}

void PrintSGPTriangle(Triangle t)
{
  int i;
  Scalar x,y,z;

  for(i=0;i<3;i++){
    PCoords(t.p[i], worldF, &x, &y, &z);
    printf("[ %g %g %g ] ", x, y, z);
  }
  printf("\n");

  for(i=0;i<3;i++){
    NCoords(t.n[i], worldF, &x, &y, &z);
    printf("[ %g %g %g ] ", x, y, z);
  }
  printf("\n");

  for(i=0; i<3; i++){
    printf("[ %g %g %g ] ",t.m.diffuse.r, t.m.diffuse.g, t.m.diffuse.b);
    printf("[ %g %g %g ] ",t.m.specular.r, t.m.specular.g, t.m.specular.b);
    printf(" %d\n",t.m.specularity);
  }
  printf("sgpTriangle\n");
}
