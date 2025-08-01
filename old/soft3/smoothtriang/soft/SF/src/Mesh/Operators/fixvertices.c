/*
 *----------------------------------------------------------------------
 *  File:  fixvertices.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


int FixVertices(Mesh* m, void (*f)(Mesh*, Vertex*, Vertex*) )
{
	Vertex* v;
	Vertex* vn;
	Edge* e;
	Edge* e2;
	Edge* fst;
	int count=0;

	ForeachMeshEdge(m, e) {
		if ( BoundaryVertex(e->p) ) {
			fst = e;
			while ( fst->sym != NULL ) {
				fst = fst->sym->prev;
			}
			e2 = fst;
			while ( e2 != NULL ) {
				if ( e2 == e->p->rep ) {
					break;
				}
				e2 = e2->next->sym;
			}
			if ( e2 == NULL ) {
				count++;
				/* fix it */
				v = e->p;
				if ( getenv("SDEBUG") ) {
					fprintf(stderr,"Fix vertex %s\n",
						ReturnName(v));
				}
				vn = NewVertex();
				AddMeshVertex(m, vn, 1);
				vn->rep = fst;

				e2 = fst;
				while ( e2 != NULL ) {
					e2->p = vn;
					e2 = e2->next->sym;
				}

				if ( f != NULL ) {
					f(m, v, vn);
				}
			}
		}
	} EndForeach;
	if ( debugLevel(0) &&  !ValidMesh(m) ) {
		fprintf(stderr, "invalid after fixvertices.\n");
	}
	return count;
}
