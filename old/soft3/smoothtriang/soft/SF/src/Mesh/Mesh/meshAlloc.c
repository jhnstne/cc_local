/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  meshAlloc.c
 *	New/Free for mesh entities.
 *  Last Modified: Wed Aug 14, 1991 at 06:27:51 PM
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

char* checkMalloc(int n, char* f, int l)
{
  char* p;
  char* malloc();

  p = malloc(n);
  if ( p == NULL ) {
    fprintf(stderr,"Malloc() failed.  Line %d in file %s.  ",
	    l,f);
    fprintf(stderr,"Exiting.\n");
    exit(1);
  } 
    return p;
}

#define CheckMalloc(n) checkMalloc((n),__FILE__,__LINE__)


/* ========= CREATION ROUTINES ========= */


/*
 *----------------------------------------------------------------------
 *  Function:  NewNode
 *	Create a new node.
 *----------------------------------------------------------------------
 */
Node* NewNode(void)
{
  Node* n;

  n = (Node*)CheckMalloc(sizeof(Node));
  n->externalData=NULL;
  n->internalData = NULL;
  n->name=NULL;
  n->tag = 0;
  n->rep = NULL;
  n->iflags = NULL;
  return n;
}

Vertex* NewVertex(void)
{
  Node* n;

  n = NewNode();
  n->tag = POINT;
  return n;
}

Face* NewFace(void)
{
  Node* n;

  n = NewNode();
  n->tag = FACE;
  return n;
}

/*
 *----------------------------------------------------------------------
 *  Function:  NewEdge
 *	Create a new edge.
 *----------------------------------------------------------------------
 */
Edge* NewEdge(void)
{
  Edge* e;

  e = (Edge*)CheckMalloc(sizeof(Edge));
  e->externalData=NULL;
  e->internalData = NULL;
  e->name=NULL;
  e->p = NULL;
  e->d = NULL;
  e->sym = NULL;
  e->next = NULL;
  e->prev = NULL;
  e->iflags = NULL;
  return e;
}


/*
 *----------------------------------------------------------------------
 *  Function:  NewMesh
 *	Create a new mesh.
 *----------------------------------------------------------------------
 */
Mesh* NewMesh(void)
{
  Mesh* m;

  m = (Mesh*)CheckMalloc(sizeof(Mesh));
  m->externalData=NULL;
  m->internalData = NULL;
  m->name=NULL;
  m->vertexList=NULL;
  m->faceList=NULL;
  m->edgeList=NULL;

  return m;
}

/*
 *----------------------------------------------------------------------
 *  Function:  FreeEdge
 *----------------------------------------------------------------------
 */
void FreeEdge(Edge *e)
{
    if ( e->internalData != NULL ) {
	free( e->internalData );
    }
    if ( e->externalData != NULL ) {
	pDeleteLnode( &(e->externalData) );
    }
    e->externalData = NULL;
    e->internalData = NULL;
    e->name = NULL;
    e->p = NULL;
    e->d = NULL;
    e->sym = NULL;
    e->next = NULL;
    e->prev = NULL;
    free(e);
}

/*
 *----------------------------------------------------------------------
 *  Function:  FreeFace
 *----------------------------------------------------------------------
 */
void FreeFace(Face *f)
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


/*
 *----------------------------------------------------------------------
 *  Function:  FreeVertex
 *----------------------------------------------------------------------
 */
void FreeVertex(Vertex *v)
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
