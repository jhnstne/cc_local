/*
 *  File:  splitedge.c
 *
    Splits edge into two distinct half edges; if a vertex of the edge is 
    already on a boundary, then create a new vertex for one edge
    Returns a pointer to the previous sym edge of the input edge, NULL
    if the edge is not a full edge
*/

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


Edge *SplitEdge(Mesh *m, Edge *e1 )
{
    BOOLEAN ok;
    Vertex *v1, *v2;	/* vertices of edge e1 */
    Vertex *nv1, *nv2;	/* vertices of edge e2 */
    Edge *e2;		/* half edge returned */
    Edge *e;

    e2 = e1->sym;
    if ( e2 == NULL )
	return( NULL );
    v1 = e1->p;
    v2 = e2->p;
     /*
		check if need new vertex for nv1 or if v2 is good
    */
    if ( BoundaryEdge( v2->rep ) == TRUE )
    {
	nv1 = NewVertex();
	AddMeshVertex( m, nv1, 0 );
	nv1->rep = e2;
	/*
			set edge vertex pointers to nv1
	*/
	ok = FALSE;
	ForeachVertexEdge( v2, e )
	    if ( e == e2 )
		ok = TRUE;
	    if ( ok == TRUE )
		e->p = nv1;
	EndForeach
    }
    /*
		check if need new vertex for nv2 or if v1 is good
    */
    if ( BoundaryEdge( v1->rep ) == TRUE )
    {
	nv2 = NewVertex();
	AddMeshVertex( m, nv2, 0 );
	nv2->rep = v1->rep;
	/*
			set edge vertex pointers to nv2
	*/
	ForeachVertexEdge( v1, e )
	    if ( e == e1 )
		break;
	    e->p = nv2;
	EndForeach
	v1->rep = e1;
    }
    /*
		split edge
    */
    e1->sym = e2->sym = NULL;

    AddMeshEdge( m, e1, 0 );
    AddMeshEdge( m, e2, 0 );

    if ( getenv("SDEBUG")  &&  !ValidMesh(m) ) {
	fprintf(stderr,"invalid leaving SplitEdge\n");
    }
    return( e2 );
}
