/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  smooth.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "libgeo.h"
#include "material.h"
#include "libmat.h"
#include "mesh.h"
#include "userData.h"
#include "operators.h"
#include "patch.h"
#include "util.h"
#include "commandline.h"
#include "usage.h"


Space world;

Scalar weight=0.5;


void SmoothVertex(v)
Vertex* v;
{
  Vertex* w;
  Vertex* firstW;
  int first=1;
  Point sum;
  int count;

  if ( BoundaryVertex(v) ){
    v->internalData->np = ReturnUDPoint(v);
  } else {
    count=0;
    ForeachVertexVertex(v,w){
      if ( first ) {
	count = 1;
	first = 0;
	sum = ReturnUDPoint(w);
      } else {
	count++;
	sum = PPac(sum, ReturnUDPoint(w), (float)(count-1)/(float)count);
      } 
    } EndForeach;
    v->internalData->np = PPac(ReturnUDPoint(v), sum, weight);
    v->internalData->geometryFlags |= G_POSITION;
  }
}


main(argc,argv)
int argc;
char* argv[];
{
  Mesh* m;
  Face* f;
  Vertex* v;

  ParseCommandLine(argc,argv);

  world = SCreate("world",3);
  m = MeshParse(stdin);
  AddGeometry(world,m);

  ForeachMeshVertex(m,v){
    SmoothVertex(v);
  } EndForeach;

  ForeachMeshVertex(m,v){
    SetUDPoint(v, v->internalData->np);
  } EndForeach;

  ConvertGeometryToExternal(m);
  WriteMesh(m,stdout);
}

void SetWeight(a)
char* a[];
{
  double atof();
  weight = atof(a[0]);
  fprintf(stderr,"Weight is %f\n",weight);
}



char* Banner = "smooth";
char* UsageString = "smooth [options] < mesh";

Option Options[]={
  "h",Usage,0,": \tprint help message.",
  "w",SetWeight,1,"weight :\tThe weight a vertex is given, relative to the centroid of its neighbors.",
  NULL,NULL,0,NULL
  };
