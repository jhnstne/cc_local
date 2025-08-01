/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  t.c
 *	A simple test program
 *----------------------------------------------------------------------
 */


#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "libgeo.h"
#include "libmat.h"
#include "userData.h"
#include "mesh.h"
#include "material.h"
#include "vertex.h"
#include "patch.h"
#include "tpbezier.h"
#include "util.h"
#include "commandline.h"
#include "usage.h"
#include "intersection.h"
#include "geom.h"
#include "sff.h"


Vertex *SplitTriFace3to1();
Vertex* CenterSplitFace();
static int curV=0;
static int curF=0;
static char vn[1000][1000];
static char fn[1000][1000];

FixIt(v1,v2)
Vertex* v1;
Vertex* v2;
{
	SetInternalData(v2, ReturnInternalData(v1));
}

char* uniq(m, v1, v2)
Mesh* m;
Vertex* v1;
Vertex* v2;
{
	SetName(v2, MeshUniqueName(m, "fv"));
}


main(argc,argv,envp)
int argc;
char **argv;
char **envp;
{
	Mesh* m;
	Vertex* va[1000];
	Vertex* v;
	Vertex* v2;
	int i;
	int n;
	Edge* e;
	Edge* ea[1000];
	Vertex* AddVertexEdge();
	Face* f;
	Face* fl;
	Space s;
	Frame wf;

	s = SCreate("duh",3);
	wf = StdFrame(s);
	
	m = MeshParse(stdin);
	if ( ValidMesh(m) ) {
		fprintf(stderr,"Initial mesh valid\n");
	} else {
		fprintf(stderr,"Initial mesh NOT valid\n");
	}
	AddGeometry(s,m);
	
#if 0
	ForeachMeshVertex(m,v) {
		va[i] = v;
	} EndForeach;
	ForeachMeshEdge(m,e) {
		ea[i] = e;
	} EndForeach;
	ForeachMeshFace(m,f) {
		fl = f;
	} EndForeach;
	f = fl;
	
	GetEdgeVertices(ea[0],&v,&v2);
	fprintf(stderr,"v = %s, v2 = %s\n",v->name,v2->name);
	v2 = ea[0]->next->p;
	v = AddVertexEdge(m, ea[0]);
	v->name = "new";

	fprintf(stderr,"Split edge %s to %s\n",v->name,v2->name);
	f = (Face*)SplitFace(m,f,v,v2);
	f->name = "newf";
#endif
	
#if 0
	ForeachMeshEdge(m,e) {
		Vertex* v1;
		Vertex* v2;

		GetEdgeVertices(e, &v1, &v2);
		v = AddVertexEdge(m, e);
		sprintf(vn[curV],"newV%d",curV);
		v->name = vn[curV++];
		dPutVertexPosition(v->externalData,"Point"
				   ,PPac(ReturnUDPoint(v1), 
					 ReturnUDPoint(v2), 0.5));
	} EndForeach;

	ForeachMeshFace(m, f) {
		i = 0;
		ForeachFaceVertex(f, v) {
			va[i++] = v;
		} EndForeach;
		if ( i > 3 ) {
			SubdivideFace(m, f, va, i);
		}
	} EndForeach;

	WriteMesh(m, stdout);
#endif

#if 0
	if ( argc > 1 ) {
		n = atoi(argv[1]);
	} else {
		n = 0;
	}
	i=0;
	ForeachMeshEdge(m, e) {
		Vertex* v1;
		Vertex* v2;

		if ( n == -1 ) {
			GetEdgeVertices(e, &v1, &v2);
			JoinVertices(m, &v1, &v2, 1);
			break;
		} else if ( i == n ) {
			GetEdgeVertices(e, &v1, &v2);
			JoinVertices(m, v1, v2, 1);
			break;
		}
		i++;
	} EndForeach;
#endif
	ForeachMeshFace(m, f){
		Vertex* v;
		Point p[100];
		Scalar s[100];
		switch(argc){
			Point pt;
		      case 1:
			fprintf(stderr,"Calling fix vertices\n");
			fprintf(stderr,"Fixed %d vertices\n",
				FixVertices(m, uniq));
			if(1)break;
			ForeachFaceEdge(f, e) {
				GetEdgeVertices(e,&v,&v2);
				JoinVertices(m,v,v2,1);
				DumpMesh(m);
				if(1)break;
			} EndForeach;
			break;
		      case 2:
			fprintf(stderr,"TriangulateFace\n");
			TriangulateFace(m,f);
			break;
		      case 3:
			fprintf(stderr,"CenterSplitFace\n");
			v = CenterSplitFace(m,f);
			i=0;
			ForeachVertexVertex(v, v2){
				p[i++] = ReturnUDPoint(v2);
			} EndForeach;
			n = i;
			for(i=0;i<n;i++) s[i] = 1./n;
			pt = PPacN(n, p, s);
			SetUDPoint(v, pt);
			v->name = "v";
			break;
		      case 4:
			fprintf(stderr,"Calling fix vertices\n");
			fprintf(stderr,"Fixed %d vertices\n",
				FixVertices(m, NULL));
		      case 5:
			fprintf(stderr,"Calling fix vertices\n");
			fprintf(stderr,"Fixed %d vertices\n",
				FixVertices(m, FixIt));
			break;
		}
		if(1)break;
	} EndForeach;

	if ( ValidMesh(m) ) {
		fprintf(stderr,"ValidMesh\n");
	} else {
		fprintf(stderr,"Mesh not valid\n");
	}
	ForeachMeshEdge(m, e) {
		;
	} EndForeach;
	ConvertGeometryToExternal(m);
	WriteMesh(m, stdout);
}

SubdivideFace(m, f, vl, vn)
Mesh* m;
Face* f;
Vertex* vl[];
int vn;
{
	int i,j;
	Face* ff;
	Face* SplitFace();
	
	for(i=0; i<vn; i++){
		if ( vl[i]->name[0] == 'n' ) {
			for(j=1; j<vn; j++ ) {
				if ( vl[(i+j)%vn]->name[0] == 'n' ) {
					ff = SplitFace(m, f, 
						       vl[(i+j)%vn],vl[i]);
					sprintf(fn[curF],"Fnew%d",curF);
					ff->name = fn[curF++];
					break;
				}
			}
			i = i+j-1;
		}
	}
}

#define assertx(A) A



TriangulateFace(m, f)
Mesh* m;
Face* f;
{
	Vertex* v;
	Vertex* vb;
	Vertex* vp;
	Vertex* vc;
	Vertex* vn;
	Vertex* vtmp;
	Face* f2;
	int i=0;

	ForeachFaceVertex(f, v) {
		switch (i) {
		      case 0:
			vb = v;
			break;
		      case 1:
			vp = v;
			break;
		      case 2:
			vc = v;
			break;
		}
		i++;
		if ( i > 2 ) {
			break;
		}
	} EndForeach;
	f2 = f;

	vn = VertexPath(vp, vc, "-1");
	while ( vn != vb ) {
		f2 = SplitFace(m, f2, vb, vc);
		vtmp = VertexPath(vc, vn, "-1");
		vc = vn;
		vn = vtmp;
	}
}

DumpMesh(m)
Mesh* m;
{
	Vertex* v;
	Edge* e;
	Face* f;

	ForeachMeshVertex(m, v) {
		fprintf(stderr,"V %s %d rep = %d\n",v->name,v,v->rep);
	} EndForeach;
	ForeachMeshFace(m, f) {
		fprintf(stderr,"F %d rep = %d\n",f,f->rep);
		ForeachFaceEdge(f, e) {
			fprintf(stderr,"\t%d ->p %s %d\n",e, e->p->name,e->p);
		} EndForeach;
	} EndForeach;
	ForeachMeshEdge(m, e) {
		fprintf(stderr,"%d \n",e);
		fprintf(stderr,"\t%s %d\n",e->p->name,e->p);
		fprintf(stderr,"\tsym %d\n",e->sym);
		fprintf(stderr,"\tnext %d\n",e->next);
		fprintf(stderr,"\tprev %d\n",e->prev);
		if ( e->sym ) {
			fprintf(stderr,"%d \n",e->sym);
			fprintf(stderr,"\t%s %d\n",e->sym->p->name,e->sym->p);
			fprintf(stderr,"\tsym %d\n",e->sym->sym);
			fprintf(stderr,"\tnext %d\n",e->sym->next);
			fprintf(stderr,"\tprev %d\n",e->sym->prev);
		}
	} EndForeach;
}
