/*
 *----------------------------------------------------------------------
 *  File:  Createface
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


static void InitializeEdgeHashTable();
static void InitializeEdgeHashTable();
static Edge *LookupEdge(Node *p,Node *q);
static void InstallEdge(Node *p,Edge  *newedge);

/*
    Creates a new face, 3 half edges, 3 vertices.  Sets internalData,
    externalData, name pointers to NULL.
    Returns the face created.
*/
Face *CreateFace( Mesh *m )
{
	Face *f;
	Vertex *v1, *v2, *v3;
	Edge *e1, *e2, *e3;
	
	f = NewFace();
	e1 = NewEdge();
	e2 = NewEdge();
	e3 = NewEdge();
	v1 = NewVertex();
	v2 = NewVertex();
	v3 = NewVertex();
	
	/* it is safe to do all the adds before setting other pointers
	   as everything is new; thus, all are 'fast' adds */
	AddMeshFace( m, f, 1 );
	AddMeshVertex( m, v1, 1 );
	AddMeshVertex( m, v2, 1 );
	AddMeshVertex( m, v3, 1 );
	AddMeshEdge( m, e1, 1 );
	AddMeshEdge( m, e2, 1 );
	AddMeshEdge( m, e3, 1 );
	
	e1->next = e2;
	e2->next = e3;
	e3->next = e1;
	e1->prev = e3;
	e2->prev = e1;
	e3->prev = e2;
	e3->d = e2->d = e1->d = f;
	e1->p = v1;
	e2->p = v2;
	e3->p = v3;
	v1->rep = e1;
	v2->rep = e2;
	v3->rep = e3;
	f->rep = e1;
	
	return( f );
}


Face *CreateNFace( Mesh *m, Vertex* v[], int n )
{
	Face *f;
	Edge* ep;
	Edge* e1;
	Edge* e2;
	Edge* e;
	int i;
	static Edge** ea=NULL;
	static eaN=0;

	if ( eaN < n ) {
		if ( ea != NULL ) {
			free(ea);
		}
		ea = (Edge**) malloc ( n * sizeof(Edge*) );
	}

	/* make sure it is legal to add this face.
	   The restrictions on each vertex are
	   	a) The vertex not attached to any edges OR
		b) The vertex is on the boundary AND
		   1. The half edge v[i],v[i+1] must not exist (but
		      the half edge v[i+1],v[i] may exit).
		   2. One or more of the edges from the vertex 
		      to its neighbors (in the face to be created)
		      must exist.
	*/		   
 	for (i=0; i<n; i++) {
		ea[i] = NULL;
		/* a */
		if ( v[i]->rep == NULL ) {
			continue;
		}
		/* b */
		if ( !BoundaryVertex(v[i]) ) {
			return NULL;
		}
		GetVertexEdge(v[i], v[(i+n-1)%n], &e1);
		GetVertexEdge(v[(i+1)%n], v[i], &e2);
		ea[i] = e1; /* we need to save these edges */

		/* b 1 */
		if ( ( e1 != NULL  &&  e1->p == v[i])   ||
		     ( e2 != NULL  &&  e2->p != v[i])  ) {
			return NULL;
		}

		/* b 2 */
		if ( ( e1 == NULL  ||  e1->sym != NULL )  &&
		     ( e2 == NULL  ||  e2->sym != NULL ) ) {
			return NULL;
		}
	}

	/* it is safe to do all the adds before setting other pointers
	   as everything is new; thus, all are 'fast' adds */
	f = NewFace();
	AddMeshFace( m, f, 1 );

	e1 = NewEdge();
	e1->p = v[0];
	if ( v[0]->rep == NULL ) {
		v[0]->rep = e1;
	}
	e1->d = f;
	f->rep = e1;
/*	GetVertexEdge(v[0], v[n-1], &e2);*/
	e2 = ea[0];
	if ( e2 != NULL ) {
		if ( e2->sym != NULL  ||  e2->p != v[n-1] ) {
			fprintf(stderr,"CreateNFace: internal error.\n");
			exit(1);
		}
		e1->sym = e2;
		e2->sym = e1;
	} else {
		AddMeshEdge( m, e1, 1 );
	}

	e = e1;
	for (i=1; i<n; i++) {
		e->next = NewEdge();
		e->next->prev = e;
		e = e->next;
		e->p = v[i];
		if ( v[i]->rep == NULL ) {
			v[i]->rep = e;
		}
		e->d = f;
		/* What we really want to do is the GetVertexEdge.
		   However, because we're modifying the mesh, we
		   can't do this.  Thus, we saved the edges from
		   above when we looped through them. */
/*		GetVertexEdge(v[i], v[i-1], &e2);*/
		e2 = ea[i];
		if ( e2 != NULL ) {
			if ( e2->sym != NULL  ||  e2->p != v[i-1] ) {
				fprintf(stderr,
					"CreateNFace: internal error.\n");
				exit(1);
			}
			e->sym  = e2;
			e2->sym = e;
		} else {
			AddMeshEdge( m, e, 1 );
		}
	}
	e1->prev = e;
	e->next = e1;

	return( f );
}

/*
 *----------------------------------------------------------------------
 *  Function:  CreateOpposingTriFace
 *	Create a face that opposes two consecutive half-edge.  That is,
 *	given:
 *		v1a
 *		 |
 *		 |
 *		 e1
 *		 |
 *		 v
 *	  v1b = v2a -- e2 --> v2b
 *
 *	create the face F:
 *
 *		  v1a -\
 *		 |   ^   ---
 *		 |   |       \
 *		 e1 e[1]  F  e[2]
 *		 |   |          \
 *		 v   /<- e[0]--\ v
 *	     v1b = v2a -- e2 --> v2b
 *
 *
 *	If e2 is NULL, then a face is created that just borders e1 (this
 *	also requires creating a new vertex).
 *----------------------------------------------------------------------
 */
Face* CreateOpposingTriFace(Mesh* m, Edge* e1, Edge* e2)
{
	Face* nf;
	Edge* e[3];
	Vertex* v1a;
	Vertex* v1b;
	Vertex* v2a;
	Vertex* v2b;
	int i;

	if ( e1->sym != NULL  ||  (e2 != NULL  &&  e2->sym != NULL) ) {
		return 0;
	}

	GetEdgeVertices(e1, &v1a, &v1b);
	if ( e2 != NULL ) {
		Edge* et;

		GetEdgeVertices(e2, &v2a, &v2b);
		if ( v1b != v2a ) {
			return NULL;
		}
		if ( GetVertexEdge(v1a,v2b,&et) ) {
			if ( !BoundaryEdge(et) ) {
				return NULL;
			}
		}
	} else {
		v2a = v1b;
		v2b = NewVertex();
		AddMeshVertex( m, v2b, 1 );
	}

	nf = NewFace();
	AddMeshFace( m, nf, 1 );

	e[0] = NewEdge();
	e[1] = NewEdge();
	e[2] = NewEdge();
	AddMeshEdge( m, e[2], 1 );	/* it is safe to do add before setting 
					   sym as this is a new edge */

	e[0]->p = v1b;
	e[1]->p = v1a;
	e[2]->p = v2b;

	for ( i=0; i<3; i++ ) {
		e[i]->d = nf;
		e[i]->next = e[(i+1)%3];
		e[i]->prev = e[(i+2)%3];
	}
	e[1]->sym = e1; e1->sym = e[1];
	if ( e2 == NULL ) {
		e[0]->sym = NULL;
		v2b->rep = e[2];
		AddMeshEdge( m, e[0], 1 );
	} else {
		e[0]->sym = e2; e2->sym = e[0];
	}
	nf->rep = e[0];

	if ( getenv("SDEBUG")  &&  !ValidMesh(m) ) {
		fprintf(stderr, "invalid in CreateOpposingTriFace\n");
	}
	return nf;
}


/*
 *----------------------------------------------------------------------
 *  Function:  AddNFace
 *	Create a new face linking the vertices with half edges, but
 *  make no attempt to connect the half edges.  This is to be done
 *  later by the routine ConnectHalfEdges().
 *----------------------------------------------------------------------
 */
Face *AddNFace( Mesh *m, Vertex* v[], int n )
{
	Face *f;
	Edge* ep;
	Edge* e1;
	Edge* e2;
	Edge* e;
	int i;

	/* it is safe to do all the adds before setting other pointers
	   as everything is new; thus, all are 'fast' adds */
	f = NewFace();
	AddMeshFace( m, f, 1 );

	e1 = NewEdge();
	e1->p = v[0];
	if ( v[0]->rep == NULL ) {
		v[0]->rep = e1;
	}
	e1->d = f;
	f->rep = e1;

	e = e1;
	for (i=1; i<n; i++) {
		e->next = NewEdge();
		e->next->prev = e;
		e = e->next;
		e->p = v[i];
		if ( v[i]->rep == NULL ) {
			v[i]->rep = e;
		}
		e->d = f;
	}
	e1->prev = e;
	e->next = e1;

	return( f );
}

/*
 *----------------------------------------------------------------------
 *  Function:  ConnectHalfEdges
 *  Return:
 *	0: no error
 *	1: error
 *----------------------------------------------------------------------
 */
int ConnectHalfEdges( Mesh* m )
{
	Face* f;

	InitializeEdgeHashTable();

	ForeachMeshFace(m, f) {
		Edge* e;

		e = f->rep;
		do {
			Edge* esym;

			if ( LookupEdge(e->prev->p, e->p) != NULL ) {
				return 1;
			}
			if ( (esym = LookupEdge(e->p, e->prev->p)) ) {
				if ( esym->sym != NULL ) {
					return 1;
				}
				esym->sym = e;
				e->sym = esym;
			} else {
				InstallEdge(e->prev->p, e);
				AddMeshEdge(m, e, 1);
			}
		} while ( e != f->rep );
	} EndForeach;

	return 0;
}


/* Edge Table data structures */

typedef struct EdgeEntry {
  Node *p,*q;
  Edge *e;
  struct EdgeEntry *next;
} EdgeEntry;


#define EDGE_HASH_SIZE 1021

static EdgeEntry *EdgeTable[EDGE_HASH_SIZE];

static void InitializeEdgeHashTable()
{
	int i;

	for (i=0; i<EDGE_HASH_SIZE; i++) {
		EdgeEntry* ep;

		ep = EdgeTable[i];
		while (ep != NULL) {
			EdgeEntry* ed;
			ed = ep;
			ep = ep->next;
			free(ed);
		}
		EdgeTable[i] = NULL;
	}
}

static EdgeHash(Node *p,Node *q)
{
        return (((unsigned int)p + (unsigned int)q)>>2 % EDGE_HASH_SIZE);
}

static Edge *LookupEdge(Node *p,Node *q)
{
        EdgeEntry *ep;
	
	for (ep = EdgeTable[EdgeHash(p,q)];
	     ep != (EdgeEntry *) 0; ep = ep->next) {
		if (ep->p == p && ep->q == q) { /* entry found */
			return ep->e;
		}
	}
	
	return (Edge *) 0;
}

static void InstallEdge(Node *p,Edge  *newedge)
{
	EdgeEntry  *ep;
	Edge       *oldedge;
	Node      *q = newedge->p;
	int        hashval;
	
	if (oldedge = LookupEdge(q, p)) {
		if (oldedge->sym)
		  fprintf(stderr, "edge usage conflict %s %s\n",
			  ReturnName(p), ReturnName(q));
		
		newedge->sym = oldedge;
		oldedge->sym = newedge;
	} else {
		if (LookupEdge(p, q)){
			fprintf(stderr, "edge orientation conflict at edge between %s and %s\n",p->name,q->name);
		} 
		
		ep = (EdgeEntry *) calloc(1,sizeof(EdgeEntry));
		ep->p = p;
		ep->q = q;
		ep->e = newedge;
		hashval = EdgeHash(p,q);
		ep->next = EdgeTable[hashval];
		EdgeTable[hashval] = ep;
	}
}
