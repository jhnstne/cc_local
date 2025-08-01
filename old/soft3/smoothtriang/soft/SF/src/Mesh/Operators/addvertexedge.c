/*
 *  File:  addvertexedge.c
 *
    Returns vertex v which splits edge e.  Works for half edges also.
*/

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"

int sDebug=0;

Vertex *AddVertexEdge( Mesh *m, Edge *e )
{
	Vertex *v;		/* vertex created */
	Edge *he1, *he2;	/* new edges created */
	
	/*
	  create new vertex, edge(s), add to mesh
	  */
	v = NewVertex();
	AddMeshVertex( m, v, 1 );
	he1 = NewEdge();
	AddMeshEdge( m, he1, 1 );
	if ( e->sym != NULL ) {
		he2 = NewEdge();
		he2->sym = he1;
		he1->sym = he2;
	} else {
		he2 = NULL;
	}
	
	/*
	  splice in new edge(s), vertex
	  */
	he1->p = e->p;
	he1->d = e->d;
	he1->next = e->next;
	he1->prev = e;
	he1->next->prev = he1;
	e->next = he1;
	e->p = v;
	v->rep = e;
	
	if ( he1->p->rep == e ) {
		he1->p->rep = he1;
	}
	
	if ( he2 != NULL ) {
		he2->p = v;
		he2->d = e->sym->d;
		he2->next = e->sym;
		he2->prev = e->sym->prev;
		he2->prev->next = he2;
		e->sym->prev = he2;
	}
	
	if ( getenv("SDEBUG") &&  !ValidMesh(m) ) {
		fprintf(stderr,"invalid leaving AddVertexEdge\n");
	} 
	return( v );
}
