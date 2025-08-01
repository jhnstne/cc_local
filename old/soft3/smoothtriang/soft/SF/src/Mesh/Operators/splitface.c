/*
    Splits f1 by creating edge between vertices v1 & v2, creates new face.
    v1 & v2 must lie on the boundary of f1
    Returns new face, NULL  if not created
*/

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"

Face *SplitFace(Mesh *m, Face *f1, Vertex *v1, Vertex *v2)
{
    BOOLEAN found1 = FALSE;	/* True when find v1 on face f1 */
    BOOLEAN found2 = FALSE;	/* TRUE when find v2 on face f1 */
    Edge *he1, *he2;		/* new edges created between v1, v2 */
    Face *f2;			/* new face created and returned */
    Edge *e;
    Vertex *v;

    /*
		check v1, v2 on f1 boundary
    */
    ForeachFaceVertex( f1, v )
 	found1 |= v == v1;
	found2 |= v == v2;
    EndForeach

    if ( !( found1 && found2 && v1 != v2 )) 
	return( NULL );

    GetVertexEdge(v1, v2, &e);
    if ( e != NULL ) {
	    return NULL;
    }
    he1 = NewEdge();
    he2 = NewEdge();
    AddMeshEdge( m, he1, 0 );
    he1->sym = he2;
    he2->sym = he1;
    he1->p = v1;
    he2->p = v2;
    /*
		add he2, he1 into edge lists
    */
    ForeachVertexEdge( v2, e )
	if ( e->d == f1 )
	    break;
    EndForeach
    e->next->prev = he2;
    he2->next = e->next;
    e->next = he1;
    he1->prev = e;

    ForeachVertexEdge( v1, e )
	if ( e->d == f1 )
	    break;
    EndForeach
    e->next->prev = he1;
    he1->next = e->next;
    e->next = he2;
    he2->prev = e;
    /*
		set face pointers
    */
    he1->d = f1;
    f2 = NewFace();
    AddMeshFace( m, f2, 0 );
    he2->d = f2;
    for ( e = he2->next; e != he2; e = e->next )
	e->d = f2;
    f2->rep = he2;
    f1->rep = he1;

    if ( getenv("SDEBUG") &&  !ValidMesh(m) ) {
	    fprintf(stderr,"invalid leaving SplitFace\n");
    }
    return( f2 );
}
