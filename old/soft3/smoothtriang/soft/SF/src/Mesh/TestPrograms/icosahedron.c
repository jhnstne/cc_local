/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  icosahedron.c
 *	Generate a mesh for an icosahedron.
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"

Space world;

void PrintPoint(p)
Point p;
{
  double x,y,z;
  PCoords(p,StdFrame(world),&x,&y,&z);
  printf("(point %g, %g, %g)",x,y,z);
}


void PrintVector(v)
Vector v;
{
  double x,y,z;
  VCoords(v,StdFrame(world),&x,&y,&z);
  printf("(normal %g, %g, %g)",x,y,z);
}


main()
{
  Vector top, topRow[6],bottomRow[6],bottom;
  Point origin;
  int i;

  world = SCreate("world",3);
  origin = FOrg(StdFrame(world));

  top = VCreate(StdFrame(world), 0.0, 0.0, 1.0);
  bottom = VCreate(StdFrame(world), 0.0,0.0,-1.0);

  topRow[0] = VCreate(StdFrame(world),   .866,    0.0, .5);
  topRow[1] = VCreate(StdFrame(world),  .2676,   .823, .5);
  topRow[2] = VCreate(StdFrame(world), -.7006,  .5090, .5);
  topRow[3] = VCreate(StdFrame(world), -.7006, -.5090, .5);
  topRow[4] = VCreate(StdFrame(world),  .2676,  -.823, .5);
  topRow[5] = VCreate(StdFrame(world),   .866,    0.0, .5);

  for(i=0;i<5;i++)
    bottomRow[i] = SVMult(-1.0, topRow[(i+3)%5]);
  bottomRow[5] = bottomRow[0];
  printf("top = <"); PrintPoint( PVAdd(origin,top) );printf("  ");
  PrintVector(top); printf(">;\n");

  for(i=0;i<5;i++){
    printf("t%d = <",i);PrintPoint( PVAdd(origin,topRow[i]));printf("  ");
    PrintVector(topRow[i]); printf(">;\n");
  }

  for(i=0;i<5;i++){
    printf("b%d = <",i);PrintPoint( PVAdd(origin,bottomRow[i]));printf("  ");
    PrintVector(bottomRow[i]); printf(">;\n");
  }

  printf("bottom = <"); PrintPoint( PVAdd(origin,bottom) );printf("  ");
  PrintVector(bottom); printf(">;\n");


  for(i=0;i<5;i++){
    printf("ft%d = [top, t%d, t%d];\n",i+1,i,(i+1)%5);
  }

  for(i=0;i<5;i++){
    printf("ftm%d = [t%d, b%d, t%d];\n",i+6,i,i,(i+1)%5);
  }

  for(i=0;i<5;i++){
    printf("fbm%d = [b%d, t%d, b%d];\n",i+11,(i+1)%5,(i+1)%5,i);
  }

  for(i=0;i<5;i++){
    printf("fb%d = [b%d, bottom, b%d];\n",i+16,i,(i+1)%5);
  }

  printf("icosahedron = {ft1,ft2,ft3,ft4,ft5,ftm6,ftm7,ftm8,ftm9,ftm10,\n");
  printf("               fbm11,fbm12,fbm13,fbm14,fbm15,fb16,fb17,fb18,fb19,fb20};\n");
}
