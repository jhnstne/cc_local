/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  t2.c
 *	A simple test program that splits a mesh face and prints the
 *  new mesh.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"

#define assertx(A) if (!(A)) {fprintf(stderr,"assert failed %d\n",__LINE__);exit(1);}

Vertex *SplitFace3to1();

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
  Vertex* nv;
  Vertex* SplitFace();
  int i;
  static char *names[3] = {"nf1","nf2","nf3"};

  if ( argc != 1 ) {
    path = argv[1];
  } else {
    path = "1 2 3 -1 -2 -3";
  }

  m = MeshParse(stdin);

  if ( !ValidMesh(m) ) {
	fprintf(stderr,"Mesh read in not valid.\n");
	exit(1);
  }

  ForeachMeshFace(m, f) {
	break;
  } EndForeach;

  nv = SplitFace3to1(m, f);
  nv->name = "new";
  i = 0;
  ForeachVertexFace(nv, f) {
	  f->name = names[i++];
  } EndForeach;
  if ( !ValidMesh(m) ) {
	fprintf(stderr,"Mesh invalid after 3to1\n");
	exit(1);
  }
  WriteMesh(m, stdout);
  exit(0);


}



static facenumsides(f)
Face *f;
{
        int n = 0;
        Edge *e;
        ForeachFaceEdge(f,e) {
                n++;
        } EndForeach;
        return (n);
}

Vertex *SplitFace3to1(m,f)
Mesh *m;
Face *f;
{
        Edge *e,*ea[3],*enew1,*enew12;
        Vertex *v0,*v1,*v2,*vnew1,*vnew2;
        Face *fnew,*f1,*f2;
Face* fn2,*fn3;
        int i=0;
	Face* SplitFace();
	Vertex* AddVertexEdge();
	Vertex* CollapseEdge();



        assertx(facenumsides(f) == 3);
        ForeachFaceEdge(f,e) {
                ea[i++] = e;
        } EndForeach;
        GetEdgeVertices(ea[1],&v0,&v1);
        GetEdgeVertices(ea[2],&v1,&v2);
        assertx(v0 && v1 && v2);
        vnew1 = AddVertexEdge(m,ea[1]);   /* v0 to v1 */
        fnew = SplitFace(m,f,v2,vnew1);
        /* fnew is next to v1, f is next to v0 */
        GetVertexEdge(v2,vnew1,&enew1);
        assertx(enew1);
        vnew2 = AddVertexEdge(m,enew1);
        fn2 = SplitFace(m,f,v0,vnew2);
        fn3 = SplitFace(m,fnew,vnew2,v1);
        GetVertexEdge(vnew1,vnew2,&enew12);
        GetEdgeFaces(enew12,&f1,&f2);
        assertx(f1 == f || f1 == fnew);
        assertx(f2 == f || f2 == fnew);
        assertx(JoinFaces(m,f,fnew,0));
        assertx(JoinVertices(m,v0,vnew1,1));
        assertx(DestroyVertex(m,vnew1));
        return (vnew2);
}
