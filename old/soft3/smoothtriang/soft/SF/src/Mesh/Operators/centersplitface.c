/*
 *----------------------------------------------------------------------
 *  File:  centersplitface.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


/*
 *----------------------------------------------------------------------
 *  Function:  CenterSplitFace
 *
 *	The first one is the hard one...
 *
 *                   e
 *             o<----------o
 *            / ^\ e1 e2 ^/ ^
 *           /   \\  f2 //   \
 *       en /   e4\\   //e3   \ ep
 *         /       \v /v       \
 *        v          o          \
 *       o          vn           o
 *       |                       |
 *       |           f           |
 *          
 *----------------------------------------------------------------------
 */
Vertex* CenterSplitFace(Mesh* m, Face* f )
{
	Vertex* v;
	int i=0;
	Face* f2;
	Vertex* vf;
	Vertex* vs;
	Vertex* vn;
	Edge* e;
	Edge* e1;
	Edge* e2;
	Edge* e3;
	Edge* e4;
	Edge* ep;	
	Edge* en;

	ForeachFaceEdge(f, e) {
		if(1)break;
	} EndForeach;
	if ( e==NULL ) {
		return NULL;
	}
	ep = e->prev;
	en = e->next;
	f2 = NewFace();
	AddMeshFace(m, f2, 0);
	f2->rep = e;
	e->d = f2;

	vn = NewVertex();
	AddMeshVertex(m, vn, 0);

	e1 = NewEdge();
	AddMeshEdge(m, e1, 1);
	e1->d = f2;
	e1->p = vn;
	e->next = e1;
	e1->prev = e;

	e2 = NewEdge();
	AddMeshEdge(m, e2, 1);
	e2->d = f2;
	e2->p = ep->p;
	e->prev = e2;
	e2->next = e;
	e2->prev = e1;
	e1->next = e2;

	e3 = NewEdge();
	/* AddMeshEdge(m, e3, 1); */
	e3->d = f;
	e3->p = vn;
	e3->prev = ep;
	ep->next = e3;
	e3->sym = e2;
	e2->sym = e3;

	e4 = NewEdge();
	/* AddMeshEdge(m, e4, 1); */
	e4->d = f;
	e4->p = e->p;
	e4->prev = e3;
	e3->next = e4;
	e4->next = en;
	en->prev = e4;
	e4->sym = e1;
	e1->sym = e4;

	if ( f->rep == e ) {
		f->rep = en;
	}
	vn->rep = e1;
	
	v = VertexPath(vn, e->p, "-1");
	while ( v != e4->p ) {
		f = SplitFace(m, f, vn, v);
		v = VertexPath(vn, v, "-1");
	}

	return vn;
}
