/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  print.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include <math.h>

extern int curvature;
extern Scalar radius;

void PrintPt(name,point,norm)
char* name;
Point point;
Vector norm;
{
  Scalar x,y,z;
  SFF sff;

  sff.v0 = VVCross(norm,FV(StdFrame(SpaceOf(point)),0));
  if ( VMag(sff.v0) < .2 ) {
    sff.v0 = VVCross(norm,FV(StdFrame(SpaceOf(point)),1));
    if (VMag(sff.v0) < .2 ) {
      sff.v0 = VVCross(norm,FV(StdFrame(SpaceOf(point)),2));
      if (VMag(sff.v0) < .2 ) {
	fprintf(stderr,"x 2 < .2\n.  Exiting!\n");
	exit(0);
      }
    }
  }
  sff.v0 = VNormalize(sff.v0);
  sff.v1 = VNormalize(VVCross(norm,sff.v0));
  sff.m[0][0] = -1.0/radius;
  sff.m[0][1] = 0.0;
  sff.m[1][0] = 0.0;
  sff.m[1][1] = -1.0/radius;

  printf("%s = ",name);
  pPutVertexPosition(&dout,"Point",point);
  pPutVertexNormal(&dout,"Point",VDual(norm));
  if ( curvature ) {
    pPutSFF(&dout, "Point.sff", &sff);
  }
  WriteDstruct();
}
