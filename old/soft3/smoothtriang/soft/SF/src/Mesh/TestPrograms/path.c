/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  path.c
 *	A simple test program that given two vertices and a
 *  a space separated integer path through the mesh, finds the
 *  destination vertex.  See mesh documentation for description
 *  of a path in the mesh.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"



main(argc,argv)
int argc;
char* argv[];
{
  Mesh* m;
  Face* f;
  Vertex* v;
  Vertex* w;
  Vertex* x;
  char* v1;
  char* v2;
  char* path;

  if ( argc != 4 ) {
    fprintf(stderr,"format: %s v1 v2 path\n",argv[0]);
    exit(1);
  } else {
    v1 = argv[1];
    v2 = argv[2];
    path = argv[3];
  }

  m = MeshParse(stdin);


  ForeachMeshVertex(m,v){
    if ( strcmp(v->name,v1) == 0 ) {
      ForeachVertexVertex(v,w){
	if ( strcmp(w->name,v2) == 0 ) {
	  x = VertexPath(v,w,path);
	  printf("vertex path %s-%s %s ",v->name,w->name,path);
	  printf(" leads to %s\n",x->name);
	  exit(0);
	}
      } EndForeach;
      printf("vertex %s not adjacent to vertex %s.\n",v1,v2);
      exit(0);
    } 
  } EndForeach;

}

