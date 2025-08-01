/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  orient.c
 *	A program to change the consitantly orientation of all the faces 
 *  of a mesh.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include "edgeTable.h"
#include "queue.h"

#define UNPROCESSED 0
#define OK 1
#define FLIP 2









static void SetOrientation(f, n)
Face* f;
Face* n;
{
  Edge* e1;
  Edge* e2;

  if ( !ReturnUDFlags(f, OK|FLIP) ) {
    fprintf(stderr,"SetOrientation: face unoriented.  Exiting.\n");
    exit(1);
  }
  ForeachFaceEdge(f, e1) {
    ForeachFaceEdge(n, e2) {
      if ( SameEdge(e1, e2) ) {
	if ( e1->p != e2->p ) {
	  SetUDFlags(n, ReturnUDFlags(f, OK|FLIP));
	} else {
	}
	return;
      }
    } EndForeach;
  } EndForeach;
  fprintf(stderr,"SetOrientation: faces not adjacent.  Exiting.\n");
  exit(1);
}


static void OrientFirst(f)
Face* f;
{
	SetUDFlags(f, OK);
}

static void OrientNeighbors(f)
Face* f;
{
  Edge* e;
  Face* n;

  ForeachFaceEdge(f, e) {
    if ( e->sym != NULL ) {
      n = e->sym->d;
      if ( !ReturnUDFlags(n, OK|FLIP) ) {
	SetUDFlags(n, ReturnUDFlags(f, OK|FLIP));
	AddToQueue(n);

      } else if ( ReturnUDFlags(n, OK|FLIP) != ReturnUDFlags(f, OK|FLIP) ) {
	fprintf(stderr,"OrientNeighbors: mesh unorientable!  Exiting.\n");
	fprintf(stderr,"%s (%d) and %s (%d)\n",
		f->name,ReturnUDFlags(f, OK|FLIP),
		n->name,ReturnUDFlags(n, OK|FLIP));
	exit(1);

      }
    } else {
      n = GetFaceFromEdgeTable(e, f);
      if ( n != NULL ) {
	if ( !ReturnUDFlags(n, OK|FLIP) ) {
	  if ( ReturnUDFlags(f, OK) ) {
		  SetUDFlags(n, FLIP);
	  } else if ( ReturnUDFlags(f, FLIP) ) {
		  SetUDFlags(n, OK);
	  }
	  AddToQueue(n);
	} else if ( ReturnUDFlags(n, OK|FLIP) == ReturnUDFlags(f, OK|FLIP) ) {
	  fprintf(stderr,"Orient neighbor: mesh not orientable!  Exiting.\n");
	  fprintf(stderr,"%s (%d) and %s (%d)\n",
		  f->name,ReturnUDFlags(f, OK|FLIP),
		  n->name,ReturnUDFlags(n, OK|FLIP));
	  exit(1);
	}
      }
    }


  } EndForeach;
}


static void OrientMesh(m)
Mesh* m;
{
  Face* f;
  Face* f1;

  ForeachMeshFace(m, f) {
    if ( !ReturnUDFlags(f, OK|FLIP) ) {
      static int component=1;

      fprintf(stderr,"Processing component %d\n",component++);
      AddToQueue(f);
    }
    while ( f1 = FirstInQueue() ) {
      if ( !ReturnUDFlags(f1, OK|FLIP) ) {
	OrientFirst(f1);
      }
      OrientNeighbors(f1);
    }
  } EndForeach;
}

void InitData(m)
Mesh* m;
{
  Face* f;
  Edge* e;

  ForeachMeshFace(m, f) {
    if ( f->internalData == NULL ) {
      f->internalData = UDMalloc();
    }
    ClearUDFlags(f, OK|FLIP);
    ForeachFaceEdge(f, e) {
      if ( e->sym == NULL ) {
	AddToEdgeTable(e, f);
      }
    } EndForeach;
  } EndForeach;
}

main()
{
  Mesh* m;
  Space w;

  w = SCreate("World", 3);

  m = MeshParse(stdin);
  AddGeometry(w, m);
  InitData(m);
  OrientMesh(m);
  WriteMesh(m,stdout);
}
