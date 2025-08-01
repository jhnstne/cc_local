/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  t.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"


main(argc,argv)
int argc;
char* argv[];
{
  Mesh* m;
  Face* f;
  Face* f2;
  Vertex* v;
  Vertex* w;
  Vertex* x;
  Edge* e;
  char* path;

  if ( argc != 1 ) {
    path = argv[1];
  } else {
    path = "1 2 3 -1 -2 -3";
  }

  m = MeshParse(stdin);
fprintf(stderr,"Mesh is %d\n",m);
fprintf(stderr,"Mesh name is %s\n",m->name);

  ForeachMeshFace(m,f){
    printf("face %s: ",f->name);
    ForeachFaceFace(f,f2){
      printf(" %s, ",f2->name);
    } EndForeach;
    printf("\n");
  } EndForeach;

  ForeachMeshFace(m,f){
      printf("face %s: ",f->name);
      ForeachFaceVertex(f,v){
	printf(" %s ",v->name);
      } EndForeach;
      printf("\n");
    } EndForeach;

  ForeachMeshVertex(m,v){
    printf("vertex %s: ",v->name);
    ForeachVertexVertex(v,w){
      printf(" %s ",w->name);
    } EndForeach;
    printf("\n");
  } EndForeach;

  ForeachMeshVertex(m,v){
    printf("vertex %s:\n",v->name);
    ForeachVertexEdge(v,e){
      GetEdgeVertices(e,&v,&w);
      printf("	%s -- %s",v->name,w->name);
      if ( BoundaryEdge(e) ){
	printf(" *\n");
      } else {
	printf("\n");
      }
    } EndForeach;
    printf("\n");
  } EndForeach;

  ForeachMeshVertex(m,v){
    printf("vertex %s:\n",v->name);
    ForeachVertexEdge(v,e){
      Edge* e2;
      GetEdgeVertices(e,&v,&w);
      printf("from %s to %s ",v->name,w->name);
      GetVertexEdge(v,w,&e2);
      GetEdgeVertices(e,&v,&w);
      printf("gave us from %s to %s\n",v->name,w->name);
    } EndForeach;
    printf("\n");
  } EndForeach;

exit(0);
  ForeachMeshVertex(m,v){
    printf("vertex %s: ",v->name);
    ForeachVertexFace(v,f){
      printf(" %s ",f->name);
    } EndForeach;
    printf("\n");
  } EndForeach;

printf("-----------\n");

  ForeachMeshVertex(m,v){
    ForeachVertexVertex(v,w){
      printf("vertex %s: path %s %s: ",v->name,w->name,path);
      x = VertexPath(v,w,path);
      printf(" leads to %s\n",x->name);
    } EndForeach;
  }EndForeach;

/* print the mesh again to make sure we haven't messed it up. */
printf("-----------\n");
  ForeachMeshFace(m,f){
      printf("face %s: ",f->name);
      ForeachFaceVertex(f,v){
	printf(" %s ",v->name);
      } EndForeach;
      printf("\n");
    } EndForeach;

  ForeachMeshFace(m,f){
      printf("face %s:\n",f->name);
      ForeachFaceEdge(f,e){
	GetEdgeVertices(e,&v,&w);
	printf("	%s -- %s\n",v->name,w->name);
      } EndForeach;
      printf("\n");
    } EndForeach;

  ForeachMeshVertex(m,v){
    printf("vertex %s: ",v->name);
    ForeachVertexVertex(v,w){
      printf(" %s ",w->name);
    } EndForeach;
    printf("\n");
  } EndForeach;

  ForeachMeshVertex(m,v){
    printf("vertex %s: ",v->name);
    ForeachVertexFace(v,f){
      printf(" %s ",f->name);
    } EndForeach;
    printf("\n");
  } EndForeach;

}

