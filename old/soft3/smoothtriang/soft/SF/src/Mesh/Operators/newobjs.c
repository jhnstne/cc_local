/*
    Routines which return new faces, edges, and vertices.
    All fields are NULL.
*/

#include <stdio.h>
#include "all.h"

Face *newFace()
{
    Face *f;

    f = (Face *)malloc( sizeof( Face ));
    f->externalData  = NULL;
    f->internalData = NULL;
    f->name = NULL;
    f->tag = FACE;
    f->rep = NULL;

    return( f );
}


Vertex  *newVertex()
{
    Vertex *v;

    v = (Vertex *)malloc( sizeof( Vertex ));
    v->externalData  = NULL;
    v->internalData = NULL;
    v->name = NULL;
    v->rep = NULL;
    v->tag = POINT;

    return( v );
}


Edge *newEdge()
{
     Edge *e;

    e = (Edge *)malloc( sizeof( Edge ));
    e->externalData = NULL;
    e->internalData = NULL;
    e->name = NULL;
    e->p = NULL;
    e->d = NULL;
    e->sym = NULL;
    e->next = NULL;
    e->prev = NULL;

    return( e );
}


FreeEdge( e )
Edge *e;
{
    if ( e->internalData != NULL ) {
	free( e->internalData );
    }
    if ( e->externalData != NULL ) {
	pDeleteLnode( &(e->externalData) );
    }
    free(e);
}

FreeFace( f )
Face *f;
{
    if ( f->internalData != NULL ) {
	free( f->internalData );
    }
    if ( f->externalData != NULL ) {
	pDeleteLnode( &(f->externalData) );
    }
    if ( f->name != NULL ) {
	free( f->name );
    }
    f->externalData = NULL;
    f->internalData = NULL;
    f->name = NULL;
    f->tag = 0;
    f->rep = NULL;
    free(f);
}


FreeVertex( v )
Vertex *v;
{
    if ( v->internalData != NULL ) {
	free( v->internalData );
    }
    if ( v->externalData != NULL ) {
	pDeleteLnode( &(v->externalData) );
    }
    if ( v->name != NULL ) {
	free( v->name );
    }
    v->externalData = NULL;
    v->internalData = NULL;
    v->name = NULL;
    v->tag = 0;
    v->rep = NULL;
    free(v);
}
