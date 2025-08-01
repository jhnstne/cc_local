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
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"



/*
 *----------------------------------------------------------------------
 *  Function:  GetUDPoint
 *----------------------------------------------------------------------
 */
BOOLEAN GetUDPoint(void* ptr, Point* p)
{
  Node* n;
  n = ptr;
  if ( n->internalData == NULL  ||
       !(n->internalData->geometryFlags & G_POSITION) ) {
    return FALSE;
  } else {
    *p = n->internalData->p;
    return TRUE;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  ReturnUDPoint
 *----------------------------------------------------------------------
 */
Point ReturnUDPoint(void* ptr)
{
  Node* n;
  n = ptr;
  if ( n->internalData == NULL  ||
       !(n->internalData->geometryFlags & G_POSITION) ) {
    fprintf(stderr,"ReturnUDPoint: no such Point.  Dumping core.\n");
    *(char*)0 = 0;
    exit(1);
  } else {
    return n->internalData->p;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  SetUDPoint
 *----------------------------------------------------------------------
 */
BOOLEAN SetUDPoint(void* ptr, Point p)
{
  Node* n=ptr;
  if ( n->internalData == NULL ) {
    n->internalData = UDMalloc();
    if ( n->internalData == NULL )
      return FALSE;
  }
  n->internalData->geometryFlags |= G_POSITION;
  n->internalData->p = p;
  return TRUE;
}


/*
 *----------------------------------------------------------------------
 *  Function:  GetUDNormal
 *----------------------------------------------------------------------
 */
BOOLEAN GetUDNormal(void* ptr, Normal* n)
{
  Node* node=ptr;
  if ( node->internalData == NULL ||
       !(node->internalData->geometryFlags & G_NORMAL) ) {
    return FALSE;
  } else {
    *n = node->internalData->n;
    return TRUE;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  ReturnUDNormal
 *----------------------------------------------------------------------
 */
Normal ReturnUDNormal(void* ptr)
{
  Node* node=ptr;
  if ( node->internalData == NULL  ||
       !(node->internalData->geometryFlags & G_NORMAL) ) {
    fprintf(stderr,"ReturnUDNormal: no such Point.  Exiting.\n");
    exit(1);
  } else {
    return node->internalData->n;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  SetUDNormal
 *----------------------------------------------------------------------
 */
BOOLEAN SetUDNormal(void* ptr, Normal n)
{
  Node* node=ptr;
  if ( node->internalData == NULL ) {
    node->internalData = UDMalloc();
    if ( node->internalData == NULL )
      return FALSE;
  }
  node->internalData->geometryFlags |= G_NORMAL;
  node->internalData->n = n;
  return TRUE;
}


/*
 *----------------------------------------------------------------------
 *  Function:  SetUDGeoFlags
 *----------------------------------------------------------------------
 */
int SetUDGeoFlags(void* ptr, int f)
{
  Node* n;
  n = ptr;
  if ( n->internalData == NULL ) {
    n->internalData = UDMalloc();
    if ( n->internalData == NULL )
      return FALSE;
  }
  n->internalData->geometryFlags |= f;
  return TRUE;
}

/*
 *----------------------------------------------------------------------
 *  Function:  SetUDFlags
 *----------------------------------------------------------------------
 */
int SetUDFlags(Node* n, int f)
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
int ClearUDFlags(Node* n, int f)
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
int ReturnUDFlags(Node* n, int f)
{
	if ( n->internalData == NULL ) {
		return 0;
	} else {
		return (n->internalData->geometryFlags & 
			USER_FLAGS & 
			(f<<4))		  >>4;
	}
}
