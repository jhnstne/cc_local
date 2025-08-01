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
#include "stick.h"



void PrintTriangle(Triangle t)
{
  Point tempP;
  Normal tempN;
  extern int outtype;

  if ( VVDot( VVCross( PPDiff(t.p[1],t.p[0]), 
		       PPDiff(t.p[2],t.p[0]) ), NDual(t.n[0]) ) < 0 ){
    tempP = t.p[0]; tempN = t.n[0];
    t.p[0] = t.p[1];  t.n[0] = t.n[1];
    t.p[1] = tempP; t.n[1] = tempN;
  }

  switch(outtype){
  case I_DSTRUCT:
    PrintDstructTriangle(t);
    break;
  case I_SGP:
    PrintSGPTriangle(t);
    break;
  case I_A3D:
    PrintA3DTriangle(t);
    break;
  case I_S3D:
    PrintS3dTriangle(t);
    break;
  default:
    fprintf(stderr,"PrintTriangle: unknown output type %d.  Exiting.\n",
	    outtype);
    exit(1);
  }
}
