/*
 *----------------------------------------------------------------------
 *  File:  collapseedge.c
 *
 *	Given two vertices along an edge, collapse the edge between them,
 *	making the two vertices into one vertex.  Return the vertex that
 *	is disconnected (the second vertex).
 *	Operation is invalid if both vertices are on the boundary, unless
 *	the edge itself is a boundary edge.  If invalid, return NULL.
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"

static remove2face(Mesh* m, Face* f );

#define IS_TRI(e) ((e) == (e)->next->next->next)

/* don't want to joinvertices along an edge that bounds a
   triangle where the syms of the other two edges bound
   the same face */
static int badTri(Edge* e)
{
    if ( IS_TRI(e)  &&
	 ((e->next->sym == NULL  &&  e->next->next->sym == NULL) ||
	  (e->next->sym != NULL  &&  e->next->next->sym != NULL  &&
	   e->next->sym->d == e->next->next->sym->d)) ) {
	    return 1;
    } else {
	    return 0;
    }
}

static int badEdge(Edge* e)
{
	Vertex* h;
	Vertex* t;
	Vertex* l;
	Vertex* r;
	Vertex* v1;
	Vertex* v2;

	h = e->p;
	t = e->prev->p;
	if ( IS_TRI(e) ) {
		r = e->next->p;
	} else {
		r == NULL;
	}
	if ( e->sym != NULL  &&  IS_TRI(e->sym) ) {
		l = e->sym->next->p;
	} else {
		l = NULL;
	}
	ForeachVertexVertex(h, v1) {
		if ( v1 != t  &&  v1 != r   &&  
		     v1 != l ) {
			ForeachVertexVertex(v1, v2) {
				if ( v2 == t ) {
					return 1;
				}
			} EndForeach;
		}
	} EndForeach;

	return 0;
}


BOOLEAN JoinVertices(Mesh* m, Vertex* v1,Vertex* v2, int flg )
{
    Edge* e;
    Edge* de;	/* the edge to be collapsed (along with its sym) */
    Edge* e1;
    Edge* e2;

    GetVertexEdge(v1, v2, &de);
    if ( de == NULL ) {
	return 0;
    }
    if ( !BoundaryEdge(de)  &&  BoundaryVertex(v1)  &&  
				BoundaryVertex(v2) ) {
	return 0;
    }

    if ( badTri(de)  ||  (de->sym != NULL  &&  badTri(de->sym) ) ) {
	    return 0;
    }
    if ( badEdge(de)  ||  (de->sym != NULL  &&  badEdge(de->sym)) ) {
	    return 0;
    }

    if(debugLevel(1)){
	    Face* f;
	    Edge* e;
	    fprintf(stderr,"%s %s\n",de->p->name,de->sym->p->name);
	    fprintf(stderr,"%s\n",de->d->name);
	    ForeachFaceEdge(de->d, e) {
		    fprintf(stderr,"\t%d %s\n",e,e->p->name);
	    } EndForeach;
	    fprintf(stderr,"%s\n",de->sym->d->name);
	    ForeachFaceEdge(de->sym->d, e) {
		    fprintf(stderr,"\t%d %s\n",e,e->p->name);
	    } EndForeach;
    }	    
    /*
		Make edges around v2 point to v1
    */
    ForeachVertexEdge( v2, e ) {
	    e->p = v1;
    } EndForeach;
    v2->rep = NULL;

    /*
		Make sure faces don't point to de or its sym
    */
    if ( de->d->rep == de ) {
	    de->d->rep = de->next;
    }
    if ( de->sym != NULL  &&  de->sym->d->rep == de->sym ) {
	    de->sym->d->rep = de->sym->next;
    }

    /*
		Now make prev edges point past de and de->sym;
		remember the prev edges so we can delete 2 sided
		  faces.
    */
    e1 = de->prev;
    e1->next = de->next;
    de->next->prev = de->prev;
    if ( de->sym != NULL ) {
	e2 = de->sym->prev;
	if ( v1->rep == de->sym ) {
		v1->rep = e2;
	}
	e2->next = de->sym->next;
	de->sym->prev->next = de->sym->next;
	de->sym->next->prev = e2;
    } else {
	    e2 = NULL;
    }

    if ( flg  &&  e1->next->next == e1 ) {
	    remove2face(m, e1->d);
    }
    if ( flg  &&  e2 != NULL  &&  e2->next->next == e2 ) {
	    remove2face(m, e2->d);
    }

    if ( de->sym != NULL ) {
	de->sym->sym = NULL;
	DeleteMeshEdge(m, de->sym);
    }
    de->sym = NULL;
    DeleteMeshEdge(m, de);

    DeleteMeshVertex(m, v2);
    if ( getenv("SDEBUG") &&  !ValidMesh(m) ) {
	    fprintf(stderr,"invalid leaving JoinVertices\n");
    }
    return 1;
}


static remove2face(Mesh* m, Face* f )
{
    Edge* e1;
    Edge* e2;
    Edge* e1s;
    Edge* e2s;

    if ( f->rep->next->next != f->rep ) {
	Error("remove2face: face not two sided!\n");
    }
    e1 = f->rep;
    e2 = e1->next;
    e1s = e1->sym;
    e2s = e2->sym;
#if 0
    if ( e1s != NULL  &&  e2s != NULL  &&  e1s->d == e2s->d ) {
	    e1s->sym = NULL;
	    e2s->sym = NULL;
	    e1s = e2s = NULL;
    }
#endif

    /*
		Make sure vertices of face have valid reps
    */
    if ( e1->p->rep == e1 ) {
	if ( e2s != NULL ) {
	    e1->p->rep = e2s;
	} else if ( e1s != NULL ) {
	    e1->p->rep = e1s->prev;
	} else {
	    /* an alternative version would set the rep to NULL */
	    /* Error("remove2face: bad 2 face\n");*/
		e1->p->rep = NULL;
	}
    }
    if ( e2->p->rep == e2 ) {
	if ( e1s != NULL ) {
	    e2->p->rep = e1s;
	} else if ( e2s != NULL ) {
	    e2->p->rep = e2s->prev;
	} else {
	    /* an alternative version would set the rep to NULL */
	    /* Error("remove2face: bad 2 face\n");*/
		e2->p->rep = NULL;
	}
    }

    /*
		Set the syms to have correct syms
    */
    if ( e1s != NULL ) {
	e1s->sym = e2s;
    }
    if ( e2s != NULL ) {
	e2s->sym = e1s;
    }

    DeleteMeshFace(m, f);
    DeleteMeshEdge(m, e1);
    DeleteMeshEdge(m, e2);

    if ( e1s != NULL ) {
      RemoveMeshEdge(m, e1s);
    }
    if ( e2s != NULL ) {
      RemoveMeshEdge(m, e2s);
    }
    if ( e1s != NULL ) {
      AddMeshEdge(m, e1s, 0);
    } else if ( e2s != NULL ) {
      AddMeshEdge(m, e2s, 0);
    }
}
