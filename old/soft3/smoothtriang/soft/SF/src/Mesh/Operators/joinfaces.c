/*
    Removes face f2 by removing edge e.  Edge e must bound f1 and f2.  Will
    not create an f1 such that there is a duplicate vertex in a path of f1's
    edge.
*/

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


BOOLEAN JoinFaces(Mesh *m, Face *f1,Face *f2, int flg )
{
    Edge* e;
    Vertex *v1, *v2; 	/* vertex ahead (v1) & behind (v2) of e */
    Edge *e1;		/* edge right of e pointing to v1 */
    Edge *e2;		/* edge right of e->sym pointing to v2 */ 
    Vertex *vt;		/* test for ok to remove edge e */
    Edge *et;		/* test for ok to remove edge e */
    int count;		/* number vertices adjacent to f1 & f2 */
    int nf1;		/* number of edges of vertice adjacent to f1 */

    GetFaceEdge(f1, f2, &e);
    if ( e == NULL  ||  flg ) {
	    return 0;
    }

    /*
		check that edge borders f1 and f2
    */
    if ( ! ((( e->d == f1 ) && ( e->sym && e->sym->d == f2 )) ||
	    (( e->d == f2 ) && ( e->sym && e->sym->d == f1 ))))
	return( FALSE );
    /*
		check that f1, f2 only adjacent on two vertices
    */
    count = 0;
    ForeachFaceVertex( f2, vt )
	nf1 = 0;
	ForeachVertexEdge( vt, et )
	    if ( et->d == f1 )
		nf1++;
	EndForeach
	count += nf1 > 0 ? 1 : 0;
    EndForeach

    if ( count != 2 )
	return( FALSE );
    /*
		ok to remove edge e ! 
    */
    v1 = e->p;
    v2 = e->sym->p;

    e1 = e->sym->prev;
    v1->rep = e1;
    e2 = e->prev;
    v2->rep = e2;
    /*
		set face pointers of f2's edges to f1
    */
    ForeachFaceEdge( f2, et )
	et->d = f1;
    EndForeach;
    /*
		remove f2 from mesh
    */
    DeleteMeshFace(m, f2);

    /*
      make sure f1 rep isn't e 
    */
    f1->rep = e1;

    /*
	 	remove e from edge list, mesh
    */
    e1->next = e->next;
    e1->next->prev = e1;

    e2->next = e->sym->next;
    e2->next->prev = e2;

    /* redundant */
    if ( e->p->rep == e )
        e->p->rep = e1;
    if ( e2->p->rep == e->sym )
        e2->p->rep = e2;

    DeleteMeshEdge( m, e->sym );
    DeleteMeshEdge( m, e );

    if ( getenv("SDEBUG") &&  !ValidMesh(m) ) {
	    fprintf(stderr,"invalid leaving JoinFaces\n");
    }
    return( TRUE );
}
