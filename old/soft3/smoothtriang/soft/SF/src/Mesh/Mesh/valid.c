/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  valid.c
 *    Make sure a mesh is valid.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <strings.h>
#include <ctype.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"

struct elem {
	struct elem* next;
	char* p;
};

#define HASH_SIZE 1001
struct elem* table[HASH_SIZE];

struct elem* newElem()
{
	char* malloc();
	return (struct elem*) malloc (sizeof(struct elem));
}


static int hash(char* p)
{
	return ((((int)p) % HASH_SIZE) + HASH_SIZE) % HASH_SIZE;
}

static int addToList(void* p)
{
	int h;
	struct elem* l;
	struct elem* n;

	h = hash(p);
	l = table[h];
	while( l != NULL ) {
		if ( l->p == p ) {
			return 1;
		}
		l = l->next;
	}

	n = newElem();
	n->next = table[h];
	table[h] = n;
	n->p = p;
	
	return 0;	  
}

static clearTable(void)
{
	struct elem* l;
	struct elem* n;
	int i;

	for (i=0; i<HASH_SIZE; i++) {
		l = table[i];
		while ( l != NULL ) {
			n = l->next;
			free(l);
			l = n;
		}
		table[i] = NULL;
	}
}

static int validEdge(Mesh* m)
{
	Edge* e;
	int found;
	Edge* e2;
	Vertex* v;
	int retVal=1;

	clearTable();
	ForeachMeshEdge(m, e) {
		if ( e->prev->p == e->p ) {
			fprintf(stderr, "Self loop.\n");
			if ( e->p->name ) {
				fprintf(stderr,"\tv = %s\n",e->p->name);
				fprintf(stderr,"\tf = %d %s\n",e->d,
					e->d->name);
			}
			clearTable();
			return 0;
		}
		if ( BoundaryEdge(e)  &&  addToList(e->p) ) {
			fprintf(stderr, "Two boundaries at vertex\n");
			clearTable();
			return 0;
		}
		found = 0;
		ForeachVertexEdge(e->p, e2){
			if ( e == e2 ) {
				found = 1;
				break;
			}
		}EndForeach;
		if ( !found ) {
			fprintf(stderr,"Edge not in vertex edge ring.\n");
			if ( ReturnName(e->p) ) {
				fprintf(stderr,"\tv=%s\n",ReturnName(e->p));
			}
			return 0;
		}
		if ( e->sym != NULL  &&  e->sym->sym != e ) {
			fprintf(stderr,"Invalid edge: e->sym->sym != sym\n");
			clearTable();
			return 0;
		}
		if ( e->next->prev != e  ||  e->prev->next != e ) {
			fprintf(stderr, "Bad next/prev pointers\n");
			if(getenv("SDEBUG"))fprintf(stderr,"bf=%d\n",e->d);
			clearTable();
			return 0;
		}
	} EndForeach;
	ForeachMeshVertex( m, v ) {
		clearTable();
		ForeachVertexEdge(v, e) {
			if ( addToList(e->prev->p) ) {
				retVal=0;
				fprintf(stderr, "Half edge appears twice\n");
				if ( ReturnName(v) != NULL  &&
				     ReturnName(e->prev->p) != NULL ) {
					fprintf(stderr,"\t%s -> %s\n",
						ReturnName(e->prev->p),
						ReturnName(v));
				}
			}
		} EndForeach;
	} EndForeach;
	clearTable();
	return retVal;
}

static int validFaces(Mesh* m)
{
	Face* f;
	Edge* e;
	Edge* ef;

	clearTable();
	ForeachMeshFace(m, f) {
		ef = f->rep;
		ForeachFaceEdge(f, e) {
			if ( addToList(e) ) {
				clearTable();
				fprintf(stderr,"Face ring infinite loop\n");
				return 0;
			}

			if ( e->d != f ) {
				fprintf(stderr,"Edge has invalid dual\n");
				return 0;
			}

		} EndForeach;

		/* I don't think the following can happen */
		if ( !addToList(ef) ) {
			clearTable();
			fprintf(stderr, "Face rep not in ring\n");
			return 0;
		}
		clearTable();
		
	} EndForeach;

	return 1;
}

static int validVertices(Mesh* m)
{
	Vertex* v;
	Edge* e;
	int bc;
	int found;

	ForeachMeshVertex(m, v) {
		if ( v->rep  &&  v->rep->p != v ) {
			return 0;
		}
		if ( v->rep == NULL ) {
			continue;
		}

		bc = 0;
		clearTable();
		ForeachVertexEdge(v, e) {
			if ( addToList(e) ) {
				fprintf(stderr,"Bad edge ring\n");
				return 0;
			}
			if ( e->p != v ) {
				fprintf(stderr,"Edge in ring around v doesn't point to v.\n");
				fprintf(stderr," %d is %d should be %d\n",e,e->p,v);
				return 0;
			}
			if ( BoundaryEdge(e) ) {
				bc += 1;
			}
		} EndForeach;
		if ( bc > 1 ) {
			return 0;
		}

		found = 0;
		ForeachFaceEdge( v->rep->d, e ) {
			if ( e == v->rep ) {
				found = 1;
				break;
			}
		} EndForeach;
		if ( !found ) {
			return 0;
		}
	} EndForeach;
	clearTable();

	return 1;
}

static int validIterators(Mesh* m)
{
	Face* f;
	Edge* e;
	Vertex* v;

	ForeachMeshFace( m, f ) {
		if ( f->rep == NULL  ||  f->tag != FACE ) {
			fprintf(stderr, "validIterators: bad face\n");
			return 0;
		}
		ForeachFaceVertex( f, v ) {
			if ( v->tag != POINT ) {
				fprintf(stderr,
					"validIterators: bad face vertex\n");
				return 0;
			}
		} EndForeach;
		ForeachFaceEdge( f, e ) {
			if ( e->next == NULL  ||  e->prev == NULL  ||
			     e->p == NULL  ||  e->d == NULL ) {
				fprintf(stderr,
					"validIterators: bad face edge\n");
				return 0;
			}
		} EndForeach;
	} EndForeach;

	ForeachMeshEdge( m, e ) {
		if ( e->next == NULL  ||  e->prev == NULL  ||
		    e->p == NULL  ||  e->d == NULL ) {
			fprintf(stderr, 
				"validIterators: bad edge %d %d %d %d\n",
				e->next == NULL, e->prev==NULL, e->p == NULL,
				e->d == NULL);
			return 0;
		}
	} EndForeach;

	ForeachMeshVertex( m, v ) {
		if ( v->tag != POINT ) {
			fprintf(stderr, "validIterators: bad vertex %d\n",
				v->tag);
			return 0;
		}
		ForeachVertexFace( v, f ) {
			if ( f->rep == NULL  ||  f->tag != FACE ) {
				fprintf(stderr,
					"validIterators: bad vertex face\n");
				return 0;
			}
		} EndForeach;
		ForeachVertexEdge( v, e ) {
			if ( e->next == NULL  ||  e->prev == NULL  ||
			    e->p == NULL  ||  e->d == NULL ) {
				fprintf(stderr,
					"validIterators: bad vertex edge\n");
				return 0;
			}
		} EndForeach;
	} EndForeach;

	return 1;
}


static int validPieces(Mesh* m)
{
	Face* f;
	Edge* e;
	Vertex* v;

	clearTable();
	ForeachMeshFace(m, f) {
		addToList(f);
	} EndForeach;
	ForeachMeshEdge(m, e) {
		Face* f1;
		Face* f2;
		GetEdgeFaces(e, &f1, &f2);
		if ( !addToList(f1) ) {
			fprintf(stderr,"validPieces: bad face on edge\n");
			return 0;
		}
		if ( f2 != 0  &&  !addToList(f2) ) {
			fprintf(stderr,"validPieces: bad face on edge\n");
			return 0;
		}
	} EndForeach;
	ForeachMeshVertex(m, v) {
		ForeachVertexFace(v, f) {
			if ( !addToList(f) ) {
				fprintf(stderr,
					"validPieces: bad face on vertex\n");
				return 0;
			}
		} EndForeach;
	} EndForeach;
	clearTable();

	ForeachMeshVertex(m, v) {
		addToList(v);
	} EndForeach;
	ForeachMeshEdge(m, e) {
		Vertex* v1;
		Vertex* v2;
		GetEdgeVertices(e, &v1, &v2);
		if ( !addToList(v1) ) {
			fprintf(stderr, "validPieces: bad vertex on edge\n");
			return 0;
		}
		if ( !addToList(v2) ) {
			fprintf(stderr, "validPieces: bad vertex on edge\n");
			return 0;
		}
	} EndForeach;
	ForeachMeshFace(m, f) {
		ForeachFaceVertex(f, v) {
			if ( !addToList(v) ) {
				fprintf(stderr,
					"validPieces: bad vertex on face\n");
				return 0;
			}
		} EndForeach;
	} EndForeach;
	clearTable();

	ForeachMeshEdge(m, e) {
		addToList(e);
		if ( e->sym != NULL ) {
			addToList(e->sym);
		}
	} EndForeach;
	ForeachMeshFace(m, f) {
		ForeachFaceEdge(f, e) {
			if ( !addToList(e) ) {
				fprintf(stderr,
					"validPieces: bad edge on face\n");
				fprintf(stderr,"\te %d, %d d %d, f %d\n",
					e,e->sym,e->d,f);
				fprintf(stderr, "\t %d %d\n", QIFlags(e, 7),
					QIFlags(e->sym, 7));
				return 0;
			}
		} EndForeach;
	} EndForeach;
	ForeachMeshVertex(m, v) {
		ForeachVertexEdge(v, e) {
			if ( !addToList(e) ) {
				fprintf(stderr,
					"validPieces: bad edge on vertex\n");
				fprintf(stderr,
					" e = %d, e->p = %d, v = %d\n",
					e, e->p, v);
				return 0;
			}
		} EndForeach;
	} EndForeach;
	clearTable();
	return 1;
}


/*
 *----------------------------------------------------------------------
 *  Function:  ValidMesh
 *	Return non-zero if a mesh is valid.  A mesh is considered
 *	valid if the following hold:
 *
 *	Edges:
 *		a) If e->sym is non-zero, then e->sym->sym == e
 *		b) Each edge appears in the ring of faces surrounding
 *		   its representative face.
 *	Faces:
 *		a) Each face is composed of a complete ring of vertices.  
 *		   Furthermore, the representive edge of the face appears
 *		   in this ring, and every edge in this ring has the face
 *		   as its dual.
 *	Vertices:
 *		a) Each vertex has no more than one boundary edge
 *		   directed into it and no more than one boudary
 *		   directed out of it.
 *		b) The representive edge of each vertex is directed towards
 *		   the vertex and appears in a valid face.
 *----------------------------------------------------------------------
 */
int ValidMesh(Mesh* m)
{
	Edge* e;
	Face* f;
	Edge* ef;
	int found;

	if ( !validEdge(m) ) {
fprintf(stderr,"validEdge failed\n");
		return 0;
	}

	if ( !validFaces(m) ) {
fprintf(stderr,"validFaces failed\n");
		return 0;
	}

	if ( !validVertices(m) ) {
fprintf(stderr,"validVertices failed\n");
		return 0;
	}

	if ( !validIterators(m) ) {
fprintf(stderr, "validIterators failed\n");
		return 0;
	}

	if ( !validPieces(m) ) {
fprintf(stderr, "validPieces failed\n");
		return 0;
	}
	return 1;
}

int CheckEdge(Mesh* m, Edge* e)
{
	Link* l;
	Edge* e2;

	for( l=m->edgeList; l!=NULL; l=l->next ) {
		e2 = (Edge*)(l->data);
		if ( e == e2 || (e2->sym != NULL && e == e2->sym) ) {
			return 1;
		} 
	}
	return 0;
}
