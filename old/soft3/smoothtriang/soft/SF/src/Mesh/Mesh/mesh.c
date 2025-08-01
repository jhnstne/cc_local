/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  mesh.c
 *  Last Modified: Tue Jun 27, 1989 at 07:43:54 AM
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
#include "meshsym.h"


/* ========= FLAGS =========== */

/*
 *----------------------------------------------------------------------
 *  Function:  SetIFlags
 *----------------------------------------------------------------------
 */
void SetIFlags(Node* n, int f)
{
	n->iflags |= f;
}

void ClearIFlags(Node* n, int f)
{
	n->iflags &= ~f;
}

int QIFlags(Node* n, int f)
{
	return (n->iflags & f);
}


/* ========= PREDICATES ========= */

/*
 *----------------------------------------------------------------------
 *  Function:  BoundaryEdge
 *	Return TRUE if an edge is a boundary edge.
 *----------------------------------------------------------------------
 */
BOOLEAN BoundaryEdge(Edge* e)
{
  if ( e == NULL  ||  e->sym == NULL )
    return TRUE;
  else return FALSE;
}



/*
 *----------------------------------------------------------------------
 *  Function:  BoundaryVertex
 *	Return TRUE if a vertex is a boundary vertex.
 *----------------------------------------------------------------------
 */
BOOLEAN BoundaryVertex(Vertex* v)
{
  Edge* e;

  if ( v->tag != POINT ){
    fprintf(stderr,"BoundaryVertex(): %s not a point.  Exiting.\n",v->name);
    exit(1);
  }
  if ( v->rep == NULL )
    return TRUE;
  e = v->rep;
  do {
    if ( e->next == NULL  ||  e->next->sym == NULL )
      return TRUE;
    e = e->next->sym;
  } while ( e != v->rep );
  return FALSE;
}




/*
 *----------------------------------------------------------------------
 *  Function:  BoundaryFace
 *	Return TRUE if a face is a boundary face.
 *  A face is a boundary face if any of it's edges or vertices
 *  lie on the boundary.
 *----------------------------------------------------------------------
 */
BOOLEAN BoundaryFace(Face* f)
{
  Edge* e;
  Vertex* v;

  if ( f->tag != FACE ){
    fprintf(stderr,"BoundaryFace(): %s is not a face.  Exiting.\n",f->name);
    exit(1);
  }

  e = f->rep;
  do {
    if ( e->sym == NULL  ||  e->next == NULL )
      return TRUE;
    e = e->next;
  } while ( e != f->rep );

  ForeachFaceVertex(f,v){
    if ( BoundaryVertex(v) )
      return TRUE;
  } EndForeach;

  return FALSE;
}






/*
 *----------------------------------------------------------------------
 *  Function:  GetEdgeVertices
 *----------------------------------------------------------------------
 */
void GetEdgeVertices(Edge* e, Vertex** v1, Vertex** v2)
{
  *v1 = e->prev->p;
  *v2 = e->p;
}


/*
 *----------------------------------------------------------------------
 *  Function:  GetVertexEdge
 *	Get the directed edge from one vertex to another.
 *  Parameters:
 *	Vertex *v1,*v2		- first and second vertices
 *	Edge** e		- the edge from v1 to v2
 *  Return Value:
 *	void
 *----------------------------------------------------------------------
 */
void GetVertexEdge(Vertex *v1,Vertex *v2, Edge **e )
{
	Edge* ei;
	Edge* ve=NULL;
	Vertex* u;
	Vertex* v;
	
	ForeachVertexEdge(v2,ei){
		GetEdgeVertices(ei,&u,&v);
		if ( u == v1 ){
			ve = ei;
		}
	} EndForeach;
	
	*e = ve;
}

/*
 *----------------------------------------------------------------------
 *  Function:  GetFaceEdge
 *	Get the directed edge between two faces, bounding face f1.
 *----------------------------------------------------------------------
 */
void GetFaceEdge(Face* f1, Face* f2, Edge** e)
{
	Edge* ei;
	Edge* fe=NULL;
	Face* u;
	Face* v;
	void GetEdgeFaces();

	ForeachFaceEdge(f1, ei) {
		if ( ei->sym != NULL ) {
			GetEdgeFaces(ei, &u, &v);
			if ( v == f2 ) {
				fe = ei;
			}
		}
	} EndForeach;

	*e = fe;
}


/*
 *----------------------------------------------------------------------
 *  Function:  GetVertexFaceEdge
 *	Get the edge shared by both v and f that runs in 
 *  counter-clockwise direction from v around f.
 *----------------------------------------------------------------------
 */
void GetVertexFaceEdge(Vertex* v, Face* f, Edge** e)
{
	Edge* e1;

	*e = NULL;
	ForeachFaceEdge(f, e1) {
		if ( e1->p == v ) {
			*e = e1->next;
			return;
		}
	} EndForeach;
}


/*
 *----------------------------------------------------------------------
 *  Function:  GetEdgeFaces
 *	Get the two faces neighboring an edge.  If the edge is a
 *  boundary edge, the second of the two faces will be NULL.
 *----------------------------------------------------------------------
 */
void GetEdgeFaces(Edge* e, Face** f1, Face** f2)
{
	*f1 = e->d;
	if ( e->sym == NULL ) {
		*f2 = NULL;
	} else {
		*f2 = e->sym->d;
	}
}






printEdge(Edge* e)
{
  if ( e == NULL ){
    printf("null_edge");
    return;
  }
  printf("'%s' - ",e->p->name);
  if ( e->sym != NULL )
    printf("'%s'",e->sym->p->name);
  else
    printf("X");
}




static void CompleteVertexEdgeRing(Vertex* v, Edge* e1, Edge* e2)
{
  Edge* e;

  e = v->rep;
  while ( e->next->sym != NULL ) {
    e = e->next->sym;
  }
  e->next->sym = e1;
  e1->sym = e->next;
  e1->p = v;
  e1->next = e2;
  e1->prev = NULL;
  e1->d = NULL;

  while( e->sym != NULL ) {
    e = e->sym->prev;
  }
  e->sym = e2;
  e2->sym = e;
  e2->p = e->prev->p;
  e2->next = NULL;
  e2->prev = e1;
  e1->d = NULL;
}



/*
 *----------------------------------------------------------------------
 *  Function:  sscani
 *	Read an integer from a string consisting only of spaces
 *  and integers, returning the next space after the integer.
 *----------------------------------------------------------------------
 */
static char* sscani(char* s, int *i)
{
  int n;
  int sign;
  while( *s == ' ' ){
    s++;
  }
  if ( !isdigit(*s) && *s != '-' )
    return NULL;
#if 0
  n = sscanf(s,"%d",i);
  if ( n == 0 )
    return NULL;
  else {
    s++; /* we know there is at least one char; this eats the '-' */
    while( isdigit(*s) )
      s++;
    return s;
  }
#endif
  if ( *s == '-' ){
    sign = -1;
    s++;
    if ( !isdigit(*s) )
      return NULL;
  }
  else {
    sign = 1;
  }
  *i = *s++ - '0';
  while(isdigit(*s)){
    *i = 10* *i + *s++ - '0';
  }
  *i = *i * sign;
  return s;
}


/*
 *----------------------------------------------------------------------
 *  Function:  vPath
 *	Traverse the vertex path p starting at v from e.
 *  e is the half edge that points to v, and is numbered 0.
 *
 *  HACK ALERT:
 *	There is a subtle hack that makes things work.
 *  There is a problem when the vertex is a boundary vertex.
 *  For simplicity, we "complete" the vertex ring, making traversal
 *  easier.  However, the edge we pass in the recursive call could
 *  be one of the half edges we introduced to complete the ring.
 *  *BECAUSE WE ONLY LOOK AT THE SYM POINTER OF THIS EDGE AND NEVER
 *   USE IT IN ANY OTHER MANNER (IN THE RECURSIVE CALL* this code
 *  works.  Be careful of your use of the parameter 'e' when modifing 
 *  this code since it doesn't really exist.
 *----------------------------------------------------------------------
 */
static Vertex* vPath(Vertex* v, Edge* e, char* p)
{
  int n;
  Edge e1,e2;
  int i;

  p = sscani(p,&n);
  if ( p == NULL ) 
    return v;

  if ( BoundaryVertex(v) ) {
    CompleteVertexEdgeRing(v,&e1,&e2);
  } else { /* do this to make later code work */
    e1.sym = &e1;
    e2.sym = &e2;
  }
  if ( n >= 0 ) {
    for( e=e->sym, i=0; i<n; i++ ){
      e = e->prev->sym;
    }
    e1.sym->sym = NULL;
    e2.sym->sym = NULL;
    return vPath(e->p,e,p);
  } else /* n < 0 */ {
    for( e=e->sym, i=0; i>n; i-- ){
      e = e->sym->next;
    }
    e1.sym->sym = NULL;
    e2.sym->sym = NULL;
    return vPath(e->p,e,p);
  }
}



Vertex* VertexPath(Vertex* v1, Vertex* v2, char* path)
{
  Edge* e;
  Edge e1,e2;
  int i;

  /* check and see if path is well formed */
  for(i=0;i<strlen(path);i++){
    switch(path[i]){
    case ' ': case '0': case '1': case '2': case '3': case '4': 
    case '5': case '6': case '7': case '8': case '9': case '-':
      continue;
    default:
      fprintf(stderr,"VertexPath: path '%s' malformed.  Must contain ",path);
      fprintf(stderr,"'-', ' ', and digits only.  Exiting.\n");
      exit(1);
    }
  }

  /* find edge between v1 and v2 */
  if ( BoundaryVertex(v1) ) {
    CompleteVertexEdgeRing(v1,&e1,&e2);
  } else { /* do this to make later code work */
    e1.sym = &e1;
    e2.sym = &e2;
  }
  e = v1->rep;
  while( e->sym->p != v2 ) {
    e = e->sym->prev;
    /* if we looped around and didn't find v2, then v2 doesn't exist */
    if ( e == v1->rep ){
      return NULL;
    }
  }
  e = e->sym;

  e1.sym->sym = NULL;
  e2.sym->sym = NULL;

  return vPath( v2, e, path );
}


/*
 *----------------------------------------------------------------------
 *  Function:  SetInternalData
 *----------------------------------------------------------------------
 */
void SetInternalData(Vertex* v, void* d)
{
	v->internalData = (Data*)d;
}

/*
 *----------------------------------------------------------------------
 *  Function:  ReturnInternalData
 *----------------------------------------------------------------------
 */
void* ReturnInternalData(Vertex* v)
{
	return v->internalData;
}



/* ============= NAMES ============ */

static int curName=0;

static int numSuffix(char* s)
{
	int len;
	int i;

	len = strlen(s);
	for (i=len-1; i>=0; i--) {
		if ( !isdigit(s[i]) ) {
			break;
		}
	}
	if ( i+1 == len ) {
		return 0;
	} else {
		return atoi(&(s[i+1]));
	}
}


static int maxNum(Mesh* m)
{
	Face* f;
	Edge* e;
	Vertex* v;
	int n;
	int maxn=0;

	ForeachMeshFace (m, f) {
		if ( ReturnName(f) ) {
			n = numSuffix(ReturnName(f));
			if ( n > maxn ) {
				maxn = n;
			}
		}
	} EndForeach;
	ForeachMeshEdge (m, e) {
		if ( ReturnName(e) ) {
			n = numSuffix(ReturnName(e));
			if ( n > maxn ) {
				maxn = n;
			}
		}
	} EndForeach;
	ForeachMeshVertex (m, v) {
		if ( ReturnName(v) ) {
			n = numSuffix(ReturnName(v));
			if ( n > maxn ) {
				maxn = n;
			}
		}
	} EndForeach;

	return maxn;
}

/*
 *----------------------------------------------------------------------
 *  Function:  SetName
 *----------------------------------------------------------------------
 */
void SetName(void* p, char* n)
{
	Vertex* v=(Vertex*)p;
	if ( v->name != NULL ) {
		free(v->name);
	}
	if ( n != NULL ) {
		if ( numSuffix(n) > curName ) {
			curName = numSuffix(n) + 1;
		}
		v->name = (char*)malloc(strlen(n)+1);
		strcpy(v->name, n);
	} else {
		v->name = NULL;
	}
}

/*
 *----------------------------------------------------------------------
 *  Function:  ReturnName
 *----------------------------------------------------------------------
 */
char* ReturnName(void* v)
{
	return ((Vertex *)v)->name;
}


char* MeshUniqueName(Mesh* m, char* s)
{
	static char buf[100];
	static int first=1;

	/* we'll assume one mesh... */
	if ( first  ) {
		curName = maxNum(m)+1;
	}
	first = 0;

	if ( s == NULL  ||  *s == NULL ) {
		sprintf(buf, "X%d", curName++);
	} else {
		sprintf(buf, "%sX%d", s, curName++);
	}
	return buf;
}
