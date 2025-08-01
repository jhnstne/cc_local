/*
 *  File:  removevertexedge.c
 *
    Removes vertex v from the mesh.  v must have degree 2; faces around
    v must have > 3 vertices; joins the edges incident on v into one edge
    Returns the resulting edge, NULL if could not remove v.
*/

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


Edge *RemoveVertexEdge(Mesh *m, Vertex *v )
{
    Edge *fe1, *fe2;	/* edge pointing to vertex, fe1->p = v */
    Edge *be1, *be2;	/* edge pointing away from vertex, fe1->next = be1 */
    Edge *e;
    Vertex *vv;
    int count = 0;
    /* 
		get edge fe1, test that v has degree 2, faces > 3 vertices
    */
    ForeachVertexEdge( v, e )
	count++;
    EndForeach
    if ( count > 2 ) 
	return( NULL );
    fe1 = v->rep;
    fe2 = fe1->sym;
    be1 = fe1->next;
    be2 = be1->sym;
    count = 0;
    ForeachFaceVertex( fe1->d, vv )
        count++;
    EndForeach
    if ( count <= 3 ) 
        return( FALSE );
    if ( fe2 )
    {
        count = 0;
	ForeachFaceVertex( fe2->d, vv )
	    count++;
	EndForeach
	if ( count <= 3 )
	    return( FALSE );
    }
    /*
			remove be1, be2, v 
    */
    fe1->next = be1->next;
    fe1->next->prev = fe1;
    fe1->p = be1->p;
    if ( be1->p->rep == be1 )
	be1->p->rep = fe1;
    if ( be2 != NULL )
    {
	fe2->prev = be2->prev;
	fe2->prev->next = fe2;
    }
    DeleteMeshVertex( m, v );
    DeleteMeshEdge( m, be1 );
    DeleteMeshEdge( m, be2 );

    return( fe1 );
}
