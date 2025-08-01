/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
**
** Purpose: Routines to maintain dynamic structures.
**
** Author: Tony DeRose
** Created: Tue Jul 25, 1989 at 08:26:26 PM
** Last Modified: Tue Jul 25, 1989 at 08:32:47 PM
**
*/
#include <stdio.h>
#include "dstruct.h"
#include "dstructMem.h"
#include "math.h"

Lnode *din=NULL, *dout=NULL;

extern void *malloc();

/* Make and return a newly initialized Lnode structure */
/* that represents a dynamic structure.  Fields is a   */
/* list of field specifiers.                           */
Lnode *NewNVPair( char *name, Lnode *value )
{
	Lnode *new = NewLnode();

	new->tag = NV;
	new->name = name;
	new->cdr = value;
	return new;
}



/* Make and return a new list cell (analogous to a */
/* cons cell in lisp).                             */
/* We're going to hack; use sval field to keep track of last in list */
Lnode *NewNVList(Lnode *list, Lnode *elem)
{
	Lnode *new = NewLnode();
	
	new->tag = NVLIST;
	new->car = elem;
	
/*  if ( list == NULL ) {
	    new->cdr = list;
	    new->sval = (char*)new;
	    return new;
	    } else if (  list->sval == NULL ) {
*/
	new->cdr = list;
	return new;
/*
	   } else {
	   ((Lnode*)(list->sval))->cdr = new;
	   new->cdr = NULL;
	   list->sval = (char*)new;
	   return list;
	   }
*/
}


/* Make and return a new list cell (analogous to a */
/* cons cell in lisp).                             */
/* We're want to hack; use sval field to keep track of last in list.
 *  However, if sval is NULL, we'll need to skip to the end. */
Lnode *NewVList(Lnode *head, Lnode *newValue)
{
	Lnode *t;
	Lnode *new = NewLnode();
	int count=0;
	
	new->tag = VLIST;
	new->car = newValue;
	new->cdr = NULL;
	
	if ( head->sval == NULL ) {
		for(t=head; t->cdr != NULL; t=t->cdr){
			t->position = count;
			count++;
		}
		t->position = count++;
		t->cdr = new;
		new->position = count;
	} else {
		((Lnode*)(head->sval))->cdr = new;
		new->position = ((Lnode*)(head->sval))->position +1;
		head->sval = (char*)new;
	}
	
	return head;
}

/* Make a new terminal VNode */
/* We're going to hack; use sval field to keep track of last in list */
Lnode *NewVNode(Lnode *car)
{
	Lnode *new = NewLnode();

	new->tag = VLIST;
	new->position = 0;
	new->car = car;
	new->cdr = NULL;
	new->sval = (char*) new;

	return new;
}


/* Make and return a new lnode that stores a string value. */
/* buf points to the buffer containing the string and must */
/* be carved from the heap by the caller.                  */
Lnode *NewString(char *buf)
{
	Lnode *new = NewLnode();

	new->tag = STRING;
	new->sval = buf;
	return new;
}


/* Make and return a new lnode that stores a real value.   */
/* rval points to the buffer containing the double and must */
/* be carved from the heap by the caller.                  */
Lnode *NewReal(Scalar rval)
{
	Lnode *new = NewLnode();

	new->tag = REAL;
	new->rval = rval;
	return new;
}


/* Print out 'n' spaces */
static void PrintSpaces(int n)
{
	int i;
	for(i=0;i<n;i++)printf(" ");
}


/*
 *----------------------------------------------------------------------
 *  Function:  dsPrintFloat
 *	Print out a float to 10 digits accuracy, using five or less
 *  digits if you can get away with it.
 *----------------------------------------------------------------------
 */
static void dsPrintFloat(FILE *fp, double f)
{
	if ( 1000.0 * f == floor( 1000.0 * f ) ){
		fprintf(fp,"%g",f);
	} else {
		fprintf(fp,"%10.10f",f);
	}
}


static int indent;	/* amount of indentation */

/* Pretty print an arbitrary lnode structure */
/* set nl to 1 if you want new lines, 0 if not */
void fLnodePrint(FILE *fp, Lnode *lnode, int nl)
{
    int i;
    Lnode* node;

    if (lnode == NULL) return;

    switch (lnode->tag) {
      case NV:
	{
	    Lnode *node;

	    if ( nl ) {
	      fprintf(fp, "\n");PrintSpaces(indent);indent++;
	    }
	    fprintf(fp, "(%s . ", lnode->name);
	    fLnodePrint(fp, lnode->cdr,nl);
	    fprintf(fp, ")");indent--;
	}
	break;
      case NVLIST:
	fLnodePrint( fp, lnode->car, nl );
	fLnodePrint( fp, lnode->cdr, nl );
	break;
      case VLIST:
	{
	  int first=1;
	  if ( nl ) {
	    fprintf(fp, "\n");PrintSpaces(indent); indent++;
	  }
	  fprintf(fp, "[");
	  for(node = lnode; node != NULL; node = node->cdr){
	    if ( first ) first =0;
	    else fprintf(fp, ", ");
	    fLnodePrint( fp, node->car, nl );
	  }
	  fprintf(fp, "]");indent--;
	  break;
	}
      case REAL:
	dsPrintFloat(fp,lnode->rval);
	break;
      case STRING:
	fprintf(fp, "\"%s\"", lnode->sval);
	break;
      default:
	fprintf(stderr, "\n\nUnknown tag: %d\n", (int) lnode->tag);
	exit(1);
    }
}


/*
  In here because sed don't know no better.
 */
DSyywrap()
{
  return yywrap();
}
