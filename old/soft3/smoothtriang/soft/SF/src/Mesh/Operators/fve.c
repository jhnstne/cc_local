/*
 *----------------------------------------------------------------------
 *  File:  fve.c
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


/*
    NOTE: the VertexEdge iterator returns all edges pointing to the vertex;
          if the vertex is on a boundary, then it will not return the edge
 	  on the boundary pointing away from the vertex.
*/
void InitVertexEdgeIterator( Vertex *v, MeshIterator *iter )
{
    iter->currentEdge = iter->edge = v->rep;
    iter->vertex = v;
}


/*
    NOTE: the VertexEdge iterator returns all edges pointing to the vertex;
          if the vertex is on a boundary, then it will not return the edge
 	  on the boundary pointing away from the vertex.
*/
Edge *NextVertexEdgeIterator( MeshIterator *iter )
{
    Edge *e, *e1;

    e1 = iter->currentEdge;
    if ( e1 == NULL )
	return( NULL );

    e = e1->next->sym;
    if ( e == iter->edge )
	iter->currentEdge = NULL;		/* terminate next call */
    else
	iter->currentEdge = e;

    return( e1 );
}
