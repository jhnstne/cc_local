/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** dstructMem.c
**
** Author: Michael Lounsbery
** Created: Sat Aug 12, 1989 at 02:56:10 PM
** Last Modified: Sat Aug 12, 1989 at 02:56:10 PM
**
** Purpose: Memory management for Dstructs
*/


#include <stdio.h>
#include <malloc.h>
#include "dstruct.h"
#include "dstructMem.h"


typedef	struct _LnObj
   {
   struct _LnObj	*Next;
   } LnObj;

typedef	struct _ScalObj
   {
   struct _ScalObj	*Next;
   } ScalObj;



static	LnObj	*LnList = NULL;
static	ScalObj	*ScalList = NULL;

/* Make two lists, one for Lnodes, one for Scalars.
   When creating, use items in list.  If list empty,
   malloc a new one. 
   When we delete, add to free list rather than calling 'free' 
*/

/* Make and return a newly initialized Lnode structure */
Lnode *NewLnode()
{
	Lnode *new;

	if (LnList == NULL) {
		new = (Lnode *) malloc( sizeof(Lnode));
	} else {
		new = (Lnode *) LnList;
		LnList = LnList->Next;
	}
    
	new->tag = UNDEF;
	new->car = new->cdr = NULL;
	new->name = new->sval = NULL;
	new->rval= 0.0;	/* perhaps should be Nan */
	return new;
}


/* Puts Lnode on the free list, sets the pointer to NULL */

void	pFreeLnode(Lnode **p)
{
	LnObj* lpt;
	lpt = (LnObj*)(*p);
	if (lpt != NULL) {
		lpt->Next = LnList;
		LnList = lpt;
		(*p) = NULL;
	}
}




char	*NewStringMem(int len)
{
	return (char *) malloc(len);
}


void	pFreeString(char **p)
{
	free(*p);
	*p = NULL;
}
