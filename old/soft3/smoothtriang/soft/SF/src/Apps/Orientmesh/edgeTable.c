/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  edgeTable.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"

int SameEdge(e1, e2)
Edge* e1;
Edge* e2;
{
  Edge* p1;
  Edge* p2;

  p1 = e1->prev;
  p2 = e2->prev;
  if ( (e1->p == e2->p  &&  p1->p == p2->p) ||
       (e1->p == p2->p  &&  p1->p == e2->p) ) {
    return 1;
  } else {
    return 0;
  }
}




struct edgeEntry {
  Edge* e1;
  Edge* e2;
  Face* f1;
  Face* f2;
  struct edgeEntry* next;
};

#define MAXE 10001
static struct edgeEntry* etable[MAXE];

static int hash(e)
Edge* e;
{
  float x,y,z;
  double dx,dy,dz;
  Point p;
  int sum;

  p = ReturnUDPoint(e->p);
  PCoords(p, StdFrame(SpaceOf(p)), &dx, &dy, &dz);
  x = dx; y = dy; z = dz;
  sum = *(int*)&x + *(int*)&y + *(int*)&z;

  p = ReturnUDPoint(e->prev->p);
  PCoords(p, StdFrame(SpaceOf(p)), &dx, &dy, &dz);
  x = dx; y = dy; z = dz;
  sum += (*(int*)&x + *(int*)&y + *(int*)&z);

  return ((sum%MAXE)+MAXE)%MAXE;
}


void AddToEdgeTable(e, f)
Edge* e;
Face* f;
{
  int Eindex;
  struct edgeEntry* p;

  Eindex = hash(e);
  p = etable[Eindex];
  while ( p != NULL ) {
    if ( p->e1 == e  ||  p->e2 == e ) {
      fprintf(stderr,"AddToEdgeTable: edge added twice.  Exiting.\n");
      exit(1);
    }
    if ( SameEdge(e, p->e1) ) {
      if ( p->e2 != NULL ) {
	fprintf(stderr,"AddToEdgeTable: mesh not manifold.  Exiting.\n");
	exit(1);
      }
      p->e2 = e;
      p->f2 = f;
      return;
    }
    p = p->next;
  }

  p = (struct edgeEntry*)malloc(sizeof(struct edgeEntry));
  p->e1 = e;
  p->f1 = f;
  p->e2 = NULL;
  p->f2 = NULL;
  p->next = etable[Eindex];
  etable[Eindex] = p;
}


Face* GetFaceFromEdgeTable(e, f)
Edge* e;
Face* f;
{
  int Eindex;
  struct edgeEntry* p;

  Eindex = hash(e);
  p = etable[Eindex];
  while ( p != NULL ) {
    if ( p->f1 == f ) {
      return p->f2;
    } else if ( p->f2 == f ) {
      return p->f1;
    } else {
      p = p->next;
    }
  }
  return NULL;
}

