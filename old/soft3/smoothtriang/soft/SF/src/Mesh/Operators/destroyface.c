/*
 *----------------------------------------------------------------------
 *  File:  destroyface.c
 *----------------------------------------------------------------------
 */

/*
    Removes a face from existence.  The face will be disconnected from
    the rest of the mesh before being destroyed.

    There are restrictions on what faces can be removed.  In particular,
    if a vertex of the face is a boundary face, then one of the edges
    on the vertex must be on the boundary (ie, have no sym).

    Returns TRUE if face remove, FALSE if face not removed due to face being
    connected to other faces, etc.
*/

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"



BOOLEAN DestroyFace( Mesh *m, Face *f )
{
    Edge *e;
    Edge *e1, *e2, *e3;      /* traverse edges of face */

    /*
			make sure it is legal to remove face
    */
    ForeachFaceEdge( f, e ) {
	if ( BoundaryVertex(e->p) ) {
	    if ( !BoundaryEdge(e)  &&  !BoundaryEdge(e->next) ) {
		return FALSE;
	    }
	}
    } EndForeach;


    /*
			make sure that the vertices point to a rep
			  that will be around after we remove face.
		**	code depends on fact that a vertex's rep
			  points to the vertex and that faces are
			  complete rings of edges.
    */
    ForeachFaceEdge( f, e ) {
	if ( e->p->rep == e ) {
	    if ( e->sym != NULL ) {
		e->p->rep = e->sym->prev;
	    } else if ( e->next->sym != NULL ) {
		e->p->rep = e->next->sym;
	    } else {	/* point is now a free agent */
		e->p->rep = NULL;
	    }
	}
    } EndForeach;


    /*
		split along half edges
		THIS CODE MAKES OUR MESH INVALID
		However, at end of loop, mesh will once
		again be valid, with the exception of
		the face being removed.
    */
    ForeachFaceEdge( f, e ) {
	if ( e->sym != NULL ) {
	    e->sym->sym = NULL;
	}
	e->sym = NULL;
    } EndForeach;


    ForeachFaceEdge( f, e1 ) {
	    if (1) break;
    } EndForeach;
    e2 = e1;
    do {
	    e3 = e2->next;
	    if ( debugLevel(2) ) fprintf(stderr,"DME(%d, %d)\n",e2,e2->externalData);
	    DeleteMeshEdge(m, e2);
	    e2 = e3;
    } while ( e2 != e1  &&  e2 != NULL );

    /*
			remove from mesh
    */
    DeleteMeshFace( m, f );

    return( TRUE );
}    
