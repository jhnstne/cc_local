/*
    Joins two half edges; they must point to two different faces.
    Uses the first edge's vertices; discards the second edge's vertices if
    they are not the same as the first
*/

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


BOOLEAN JoinHalfEdge(Mesh *m, Edge *he1, Edge *he2 )
{
	Vertex *v1, *v2;	/* vertices of first half edge */
	Vertex *dv1, *dv2;	/* vertices of second half edge */
	Edge *tra;		/* traverses edges pointing to dv1, */
	/*  dv2, to point them to v1, v2    */
	
	if(getenv("SDEBUG"))fprintf(stderr,"JoinHalfEdge\n");
	/* 	
	  can't join edges with same face, or full edges
	  */
	if ( he1->d == he2->d || he1->sym != NULL || he2->sym != NULL ) {
		return( FALSE );
	}
	
	v1 = he1->p;
	v2 = he1->prev->p;
	dv1 = he2->p;
	dv2 = he2->prev->p;
	he1->sym = he2;
	he2->sym = he1;
	
	if ( dv2 != v1 ) {
		/* make all edges of dv2 point to v1 */
		ForeachVertexEdge( dv2, tra ) {
			tra->p = v1;
		} EndForeach;
		
		/*
		  Set v1's 'rep' edge pointer to dv2's 'rep'.  Since
		  v1 is 'in front' of he1, it points to a full edge.
		  If dv2 points to a half edge, it is required for
		  v1 to point to it.  If dv2 points to a full edge,
		  not big deal if change or not.
		  */
		v1->rep = dv2->rep;
		
		DeleteMeshVertex( m, dv2 );
	}
	if ( dv1 != v2 ) {
		/*
		  make all edges of dv1 point to v2
		  */
		ForeachVertexEdge( dv1, tra ) {
			tra->p = v2;
		} EndForeach;
		  
		RemoveMeshVertex( m, dv1 );
	}
	/*
	  remove he2 from the mesh edge list, since it is now part
	  of a full edge
	  */
	RemoveMeshEdge( m, he2 );
	if (getenv("SDEBUG") &&  !ValidMesh(m) ) {
		fprintf(stderr, "invalid in JoinHalfEdge\n");
	}
	
	return( TRUE );
}
