/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  userData.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"


/*
 *----------------------------------------------------------------------
 *  Function:  GetUDPoint
 *----------------------------------------------------------------------
 */
BOOLEAN GetUDPoint(n,p)
Node* n;
Point* p;
{
  if ( n->internalData == NULL  ||
       !(n->internalData->geometryFlags & G_POSITION) ) {
    return FALSE;
  } else {
    *p = PCreate(StdFrame(n->internalData->s), 
		 n->internalData->x,
		 n->internalData->y, 
		 n->internalData->z);
    return TRUE;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  ReturnUDPoint
 *----------------------------------------------------------------------
 */
Point ReturnUDPoint(n)
Node* n;
{
  if ( n->internalData == NULL  ||
       !(n->internalData->geometryFlags & G_POSITION) ) {
    fprintf(stderr,"ReturnUDPoint: no such Point.  Dumping core.\n");
    *(char*)0 = 0;
    exit(1);
  } else {
    return PCreate(StdFrame(n->internalData->s), n->internalData->x,
		   n->internalData->y, n->internalData->z);
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  SetUDPoint
 *----------------------------------------------------------------------
 */
BOOLEAN SetUDPoint(n,p)
Node* n;
Point p;
{
  Scalar x,y,z;

  if ( n->internalData == NULL ) {
    n->internalData = UDMalloc();
    if ( n->internalData == NULL )
      return FALSE;
  }
  n->internalData->geometryFlags |= G_POSITION;
  n->internalData->s = SpaceOf(p);
  PCoords(p, StdFrame(SpaceOf(p)), &x, &y, &z);
  n->internalData->x = x;
  n->internalData->y = y;
  n->internalData->z = z;
  return TRUE;
}


/*
 *----------------------------------------------------------------------
 *  Function:  GetUDNormal
 *----------------------------------------------------------------------
 */
BOOLEAN GetUDNormal(node,n)
Node* node;
Normal* n;
{
  if ( node->internalData == NULL ||
       !(node->internalData->geometryFlags & G_NORMAL) ) {
    return FALSE;
  } else {
    *n = NCreate(StdFrame(node->internalData->s),
		 node->internalData->nx,
		 node->internalData->ny,
		 node->internalData->nz);
    return TRUE;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  ReturnUDNormal
 *----------------------------------------------------------------------
 */
Normal ReturnUDNormal(node)
Node* node;
{
  if ( node->internalData == NULL  ||
       !(node->internalData->geometryFlags & G_NORMAL) ) {
    fprintf(stderr,"ReturnUDPoint: no such Point.  Exiting.\n");
    exit(1);
  } else {
    return NCreate(StdFrame(node->internalData->s),
		   node->internalData->nx,
		   node->internalData->ny,
		   node->internalData->nz);
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  SetUDNormal
 *----------------------------------------------------------------------
 */
BOOLEAN SetUDNormal(node,n)
Node* node;
Normal n;
{
  Scalar x,y,z;

  if ( node->internalData == NULL ) {
    node->internalData = UDMalloc();
    if ( node->internalData == NULL )
      return FALSE;
  }
  node->internalData->geometryFlags |= G_NORMAL;
  node->internalData->s = SpaceOf(n);
  NCoords(n, StdFrame(SpaceOf(n)), &x, &y, &z);
  node->internalData->nx = x;
  node->internalData->ny = y;
  node->internalData->nz = z;
  return TRUE;
}


/*
 *----------------------------------------------------------------------
 *  Function:  SetUDGeoFlags
 *----------------------------------------------------------------------
 */
int SetUDGeoFlags(n,f)
Node* n;
int f;
{
  if ( n->internalData == NULL ) {
    n->internalData = UDMalloc();
    if ( n->internalData == NULL )
      return FALSE;
  }
  n->internalData->geometryFlags = f;
  return TRUE;
}

/*
 *----------------------------------------------------------------------
 *  Function:  SetUDFlags
 *----------------------------------------------------------------------
 */
int SetUDFlags(n, f)
Node* n;
int f;
{
  if ( n->internalData == NULL ) {
    n->internalData = UDMalloc();
    if ( n->internalData == NULL )
      return FALSE;
  }
  f = (f<<4) & USER_FLAGS;
  n->internalData->geometryFlags |= f;
}

/*
 *----------------------------------------------------------------------
 *  Function:  ClearUDFlags
 *----------------------------------------------------------------------
 */
int ClearUDFlags(n, f)
Node* n;
int f;
{
  if ( n->internalData == NULL ) {
    n->internalData = UDMalloc();
    if ( n->internalData == NULL )
      return FALSE;
  }
  f = (f<<4) & USER_FLAGS;
  n->internalData->geometryFlags &= ~f;
}

/*
 *----------------------------------------------------------------------
 *  Function:  ReturnUDFlags
 *----------------------------------------------------------------------
 */
int ReturnUDFlags(n, f)
Node* n;
int f;
{
	if ( n->internalData == NULL ) {
		return 0;
	} else {
		return (n->internalData->geometryFlags & 
			USER_FLAGS & 
			(f<<4))		  >>4;
	}
}
