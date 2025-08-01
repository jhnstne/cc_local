/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** getset.c
**
** Author: Michael Lounsbery
** Created: Mon Aug 07, 1989 at 10:28:44 AM
** Last Modified: Mon Aug 07, 1989 at 10:28:44 AM
**
** Purpose: Package for manipulating (setting, looking up, copying)
** Lnode structures.
*/

#include	<stdio.h>
#include 	"all.h"
#include	"dstructMem.h"


extern	Lnode	*NewLnode();


BOOLEAN	WasNV;		/* just says whether last guy was value from NV */
Lnode	**FoundPtr;	/* Global, storing the actual pointer to the
			   cell found in GetAdr() by a call that didn't
			   create a new cell. (Used in deletion) */

Lnode	**FoundPtr2;	/* yet another filthy hack: we need to go a further
			   level back when we're deleting cells that
			   need a list cell.  So, we need to keep a ptr
			   to the guy that is a ptr to their list cell,
			   so we know to remove that cell as well. */

/*
** Looks up String in *Struct.  Returns s.t. *Adr points to the cell that
** is looked for.
**
** Complications: if ForceMake is TRUE, then if the thing we're looking for
** doesn't exist, it is created, and *Adr will point to it.
*/

BOOLEAN	GetAdr(Struct, String, Adr, ForceMake)
Lnode	**Struct, **Adr;
char	*String;
BOOLEAN	ForceMake;	/* says whether we make a new one when needed */
   {
   char		WorkMem[MAX_STRINGLEN], *Work,
		TokenMem[MAX_STRINGLEN], *Token;
   Lnode	*NextAdr;
   BOOLEAN	MakeNew, Error;

   if ((*Struct != NULL) && ((*Struct)->tag != NVLIST))
      {
      fprintf(stderr, "LookUp: Struct points to non-NVLIST\n");
      exit(1);
      }

   Work = WorkMem;
   Token = TokenMem;

   MakeNew = Error = FALSE;
   WasNV = TRUE;

   StripBlanks(String, Work);	/* removes useless blanks */

   if (*Work == '.')		/* treat initial '.' as optional */
      strcpy(Work, Work + 1);

   BreakOffToken(Work, Token, Work);	   /* gets 1st token, resets Work */

   FoundPtr2 = Struct;
   *Adr = FindNameInList(Token, Struct, &MakeNew, &Error, ForceMake);

   /* The above looks into the structure pointed to by *Struct, looks for
      the name referred to by Token, and sets *Adr to point to it.
      MakeNew is set to TRUE iff the Token doesn't exist yet, and a new
      list element is then allocated if ForceMake is true.
   */

   if (Error)
      return FALSE;
   
   while (*Work != '\0')
      {
      if (*Work == '[')
	 {
	 ProcessArray(Work, Adr, &NextAdr, &MakeNew, &Error, ForceMake);

	 /* The above should read through ], then index into *Adr, and set
	    NextAdr to be the new value.  It should set MakeNew true iff
	    it had to make a new array element, and should set Error to
	    be true if there is a serious problem.  At the return, Work must
	    be advanced to the new position to scan.
	 */

	 if (Error)
	    return FALSE;
	 }

      else if (*Work == '.')
	 {
	 ProcessValue(Work, Adr, &NextAdr, &MakeNew, &Error, ForceMake);

	 /* The above should ensure there's only 1 '.' at the beginning,
	    then it should break off the token following it, and use
	    it to address into the structure pointed to by *Adr.  It then
	    must make NextAdr point to the new place. Set MakeNew, Work, and
	    Error as above.
	 */

	 if (Error)
	    return FALSE;
	 }

      else
	 {
	 fprintf(stderr, "Error in LookUp: unable to parse lookup string\n");
	 return FALSE;
	 }

      *Adr = NextAdr;		/* moves ahead */
      }

   return	!MakeNew;	/* return TRUE iff we never made a new one */
   }


/*
** Advances s past its leading blanks until the first non-blank is enountered
*/

void	EatLeadingBlanks(s)
char	**s;
   {
   for ( ; **s == ' '; (*s)++)
      /* null */ ;

   /* now **s is the first non-blank in the original *s */
   }

/*
** Strips blanks from sFrom, copies the resulting string into sTo
*/

void	StripBlanks(sFrom, sTo)
char	*sFrom, *sTo;
   {
   char	*sFromIndex;
   int	NextBitSize;

   sFromIndex = sFrom;

   strcpy(sTo, "");

   while (*sFromIndex != '\0')
      {
      EatLeadingBlanks(&sFromIndex);

      NextBitSize = strcspn(sFromIndex, " ");
      strncat(sTo, sFromIndex, NextBitSize);
      *(sTo + NextBitSize) = '\0';
      sFromIndex += NextBitSize;
      }
   }


/*
** Scans From to find the first token.  This is copied into Token, and
** To gets what is left.
*/

void	BreakOffToken(From, Token, To)
char	*From, *Token, *To;
   {
   char	*Temp, TempMem[MAX_STRINGLEN];
   int	TokenSize;

   Temp = TempMem;
   strcpy(Temp, From);

   TokenSize = strcspn(Temp, ".[");	/* all up to . or [ */
   strncpy(Token, Temp, TokenSize);
   *(Token + TokenSize) = '\0';
   Temp += TokenSize;
   strcpy(To, Temp);			/* leads with . or [ or \0 */
   }


/*
** Given that AccessStr points at a description string beginning with a '[',
** this reads the integer that follows up through the ']'.  The element
** indexed by the integer into Adr is returned, and made if ForceMake is
** true.
*/

void	ProcessArray(AccessStr, Adr, NextAdr, MakeNew, Error, ForceMake)
char	*AccessStr;
Lnode	**Adr, **NextAdr;
BOOLEAN	*MakeNew, *Error, ForceMake;
   {
   int	IndexNum;
   char	*NameStr, NameStrMem[MAX_STRINGLEN];
   BOOLEAN	IntoArray, NewVlist = FALSE;

   IntoArray = !WasNV;

   WasNV = FALSE;

   NameStr = NameStrMem;

   strcpy(NameStr, AccessStr);

   NameStr++;	/* move past '[' */

   IndexNum = -1;	/* flag errors */

   while((*NameStr != '\0') && (*NameStr != ']') &&
	 (*NameStr >= '0') && (*NameStr <= '9'))
      {
      IndexNum = (IndexNum < 0) ? (*NameStr  - '0')
				: (IndexNum * 10 + (*NameStr - '0'));
      NameStr++;
      }

   if ((*NameStr != ']') || (IndexNum < 0))
      {
      fprintf(stderr, "LookUp Error: Bad index into array\n");
      *Error = TRUE;
      return;
      }
   NameStr++;	/* move past ']' */

   if ((*Adr)->tag == UNDEF)
      {
      if (IntoArray)
	 {
	 (*Adr)->tag = VLIST;		/* what it will be */
	 (*Adr)->position = IndexNum;
	 *NextAdr = NewLnode();
	 (*Adr)->car = *NextAdr;
	 strcpy(AccessStr, NameStr);
	 return;
	 }
      else
         (*Adr)->tag = NV;
      }

   else if (((*Adr)->tag != NV) && ((*Adr)->tag != VLIST))
      {
      fprintf(stderr, "LookUp Error: Index into non-NV pair\n");
      *Error = TRUE;
      return;
      }

   if ((*Adr)->tag == VLIST)
      {
      if ((*Adr)->car->tag == NV)
					/* coming from array of pairs */
	 {
         FoundPtr2 = &((*Adr)->car->cdr);
         *NextAdr = FindArrayElement(IndexNum, &((*Adr)->car->cdr), MakeNew, Error, ForceMake);
	 }
      else if (((*Adr)->car->tag == VLIST)  || ((*Adr)->car->tag == NVLIST))
		/* coming from array of arrays */
	 {
	 FoundPtr2 = &((*Adr)->car);
         *NextAdr = FindArrayElement(IndexNum, &((*Adr)->car), MakeNew, Error, ForceMake);
	 }
      }

   else		/* might be NVLIST */
      {
      FoundPtr2 = &((*Adr)->cdr);
      *NextAdr = FindArrayElement(IndexNum, &((*Adr)->cdr), MakeNew, Error, ForceMake);
      }

   strcpy(AccessStr, NameStr);
   }


/*
** Finds where in the array pointed to by Struct the element i should go.
** If it doesn't find it, it makes a new list element, asks for a new
** value element, sets MakeNew, and returns a pointer to the new value
** element
*/

Lnode	*FindArrayElement(i, Struct, MakeNew, Error, ForceMake)
int	i;
Lnode	**Struct;
BOOLEAN	*MakeNew, *Error, ForceMake;
   {
   Lnode	*NewListEl, *NewValue;

   if ((*Struct == NULL) || ((*Struct)->position > i))
      {
      if (!ForceMake)
	 {
	 *Error = TRUE;
	 return NULL;
	 }

      else
	 {
         /* maintains an ordered list coming from (*Struct)->cdr */

         /* here we insert at the head: */

         *MakeNew = TRUE;

         NewValue = NewLnode();

         NewListEl = NewLnode();
         NewListEl->tag = VLIST;
         NewListEl->car = NewValue;
         NewListEl->position = i;
         NewListEl->cdr = (*Struct);

         *Struct = NewListEl;

         return	NewValue;
	 }
      }

   else if ((*Struct)->position == i)	/* found it */
      {
      FoundPtr = Struct;
      return (*Struct);
      }

   else
      {
      FoundPtr2 = &((*Struct)->cdr);
      return FindArrayElement(i, &((*Struct)->cdr), MakeNew,
					Error, ForceMake);
      }
   }


/*
** Given that AccessStr begins with a '.', the name following is looked up
** in the NVLIST beginning at *Adr->cdr.  If there isn't one, one is created if
** ForceMake is true.
*/

void	ProcessValue(AccessStr, Adr, NextAdr, MakeNew, Error, ForceMake)
char	*AccessStr;
Lnode	**Adr, **NextAdr;
BOOLEAN	*MakeNew, *Error, ForceMake;
   {
   char	*NameStr, Token[MAX_STRINGLEN], NameStrMem[MAX_STRINGLEN];
   BOOLEAN	IntoArray;

   IntoArray = !WasNV;

   WasNV = TRUE;

   NameStr = NameStrMem;

   strcpy(NameStr, AccessStr);

   NameStr++;	/* advance past '.' */

   if ((*Adr)->tag == UNDEF)
      {
      if (IntoArray)
	 {
         (*Adr)->tag = NVLIST;		/* what it will be */
	 *NextAdr = NewLnode();
	 (*Adr)->car = *NextAdr;
         BreakOffToken(NameStr, Token, AccessStr);
         (*NextAdr)->name = NewStringMem(strlen(Token) + 1);
	 strcpy((*NextAdr)->name, Token);
	 return;
	 }
      else
	 (*Adr)->tag = NV;
      }

   else if (((*Adr)->tag != NV) && ((*Adr)->tag != VLIST))
      {
      fprintf(stderr, "LookUp Error: Index into non-NV pair\n");
      *Error = TRUE;
      return;
      }

   BreakOffToken(NameStr, Token, AccessStr);

   if ((*Adr)->tag == VLIST)
      {
      FoundPtr2 = &((*Adr)->car);
      *NextAdr = FindNameInList(Token, &((*Adr)->car), MakeNew, Error, ForceMake);
      }
   else if ((*Adr)->tag == NV)
      {
      FoundPtr2 = &((*Adr)->cdr);
      *NextAdr = FindNameInList(Token, &((*Adr)->cdr), MakeNew, Error, ForceMake);
      }
   else /* so ((*Adr)->tag == NVLIST) */
      {
      FoundPtr2 = Adr;
      *NextAdr = FindNameInList(Token, Adr, MakeNew, Error, ForceMake);
      }
   }


/*
** Looks up Name in the NVlist pointed to by name.  Returns the
** actual cell we want, and, if ForceMake is set, makes a new one
** if asked for.
*/

Lnode	*FindNameInList(GSName, List, MakeNew, Error, ForceMake)
char	*GSName;
Lnode	**List;
BOOLEAN	*MakeNew, *Error, ForceMake;
   {
   Lnode	*NewValue, *NewListEl;

   if (*List == NULL)
      {
      if (!ForceMake)
	 {
	 *Error = TRUE;
	 return NULL;	 
	 }

      else
	 {
         /* maintains a list coming from *List */

         /* here we insert at the end: */

         *MakeNew = TRUE;

         NewValue = NewLnode();
         NewValue->name = NewStringMem(strlen(GSName) + 1);
         strcpy(NewValue->name, GSName);

         NewListEl = NewLnode();
         NewListEl->tag = NVLIST;
         NewListEl->car = NewValue;

         *List = NewListEl;

         return	NewValue;
	 }
      }

   else if (!strcmp((*List)->car->name, GSName))	/* found it */
      {
      FoundPtr = &((*List)->car);
      return	(*List)->car;
      }

   else
      {
      FoundPtr2 = &((*List)->cdr);
      return	FindNameInList(GSName, &((*List)->cdr), MakeNew,
				Error, ForceMake);
      }
   }


BOOLEAN	pGetScalar(Struct, FieldSpec, ScalarPtr)
Lnode	**Struct;
char	*FieldSpec;
Scalar	*ScalarPtr;
   {
   Lnode	*LookUp;
   BOOLEAN	FoundIt, Error = FALSE;

   FoundIt = GetAdr(Struct, FieldSpec, &LookUp, FALSE);
   if (FoundIt)
      {
      if (WasNV)
	 {
         if ((LookUp->cdr != NULL) && (LookUp->cdr->tag == REAL))
	    *ScalarPtr = *(LookUp->cdr->rval);
         else
	    {
	    fprintf(stderr, "Error in GetScalar: found type not REAL\n");
	    Error = TRUE;
	    }
	 }
      else
	 {
	 if ((LookUp->car != NULL) && (LookUp->car->tag == REAL))
	    *ScalarPtr = *(LookUp->car->rval);
         else
	    {
	    fprintf(stderr, "Error in GetString: found type not REAL\n");
	    Error = TRUE;
	    }
	 }
      }

   else
      Error = TRUE;

   if (Error)
      *ScalarPtr = 0.0;

   return !Error;
   }


BOOLEAN	pGetString(Struct, FieldSpec, StringPtr)
Lnode	**Struct;
char	*FieldSpec;
char	**StringPtr;
   {
   Lnode	*LookUp;
   BOOLEAN	FoundIt, Error = FALSE;

   FoundIt = GetAdr(Struct, FieldSpec, &LookUp, FALSE);
   if (FoundIt)
      {
      if (WasNV)
	 {
         if ((LookUp->cdr != NULL) && (LookUp->cdr->tag == STRING))
	    *StringPtr = LookUp->cdr->sval;
         else
	    {
	    fprintf(stderr, "Error in GetString: found type not STRING\n");
	    Error = TRUE;
	    }
         }
      else
	 {
	 if ((LookUp->car != NULL) && (LookUp->car->tag == STRING))
	    *StringPtr = LookUp->car->sval;
         else
	    {
	    fprintf(stderr, "Error in GetString: found type not STRING\n");
	    Error = TRUE;
	    }
	 }
      }

   else
      Error = TRUE;

   if (Error)
      {
      *StringPtr = NewStringMem(1);
      **StringPtr = '\0';
      }

   return !Error;
   }


BOOLEAN	pPutScalar(Struct, FieldSpec, scalar)
Lnode	**Struct;
char	*FieldSpec;
Scalar	scalar;
   {
   Lnode	*LookUp;
   BOOLEAN	FoundIt;
   Scalar	*newRval;

   FoundIt = GetAdr(Struct, FieldSpec, &LookUp, TRUE);

   newRval = NewScalarMem();
   *newRval = scalar;

   if (WasNV)
      {
      LookUp->tag = NV;

      if (LookUp->cdr != NULL)
	 dDeleteLnode(LookUp->cdr);

      LookUp->cdr = NewReal(newRval);
      }

   else
      {
      if (FoundIt)
	 LookUp = LookUp->car;

      LookUp->tag = REAL;

      if (LookUp->rval != NULL)
         FreeScalar(LookUp->rval);

      LookUp->rval = newRval;
      }

   return FoundIt;
   }


BOOLEAN	pPutString(Struct, FieldSpec, String)
Lnode	**Struct;
char	*FieldSpec;
char	*String;
   {
   Lnode	*LookUp;
   BOOLEAN	FoundIt;
   char		*StringSpace;

   FoundIt = GetAdr(Struct, FieldSpec, &LookUp, TRUE);

   StringSpace = NewStringMem(strlen(String) + 1);
   strcpy(StringSpace, String);

   if (WasNV)
      {
      LookUp->tag = NV;

      if (LookUp->cdr != NULL)
         dDeleteLnode(LookUp->cdr);

      LookUp->cdr = NewString(StringSpace);
      }
   else
      {
      LookUp->tag = STRING;

      if (LookUp->sval != NULL)
         FreeString(LookUp->sval);

      LookUp->sval = StringSpace;
      }

   return FoundIt;
   }


/*
** LnodeCopy returns a copy of the Lnode structure passed into it.  It
** mallocs enough space to create the new one on its way.
*/

Lnode	*LnodeCopy(Struct)
Lnode	*Struct;
   {
   Lnode *New;

   if (Struct == NULL)
      return NULL;

   New = NewLnode();

   New->tag = Struct->tag;

   switch (Struct->tag)
      {
      case NV:
	 New->name = NewStringMem(strlen(Struct->name) + 1);
	 strcpy(New->name, Struct->name);

	 New->cdr = LnodeCopy(Struct->cdr);
      break;

      case VLIST:
	 New->position = Struct->position;
	 /* flow into NVLIST */

      case NVLIST:
	 New->car = LnodeCopy(Struct->car);
	 New->cdr = LnodeCopy(Struct->cdr);
      break;

      case REAL:
	 New->rval = NewScalarMem();
	 *(New->rval) = *(Struct->rval);
      break;

      case STRING:
	 New->sval = NewStringMem(strlen(Struct->sval) + 1);
	 strcpy(New->sval, Struct->sval);
      break;

      default:
	 fprintf(stderr, "\n\nError in Copy: Unknown tag: %d\n", (int) Struct->tag);
	 exit(1);
      }

   return	New;
   }



/*
** Looks up s1 in d1, and looks up s2 in d2.  Then it copies the list
** starting at the named place in d1 to the named place in d2, creating
** new structures if the lookup in d2 doesn't return anything.
**
** If the lookup in d1 doesn't return anything, there is no effect, and FALSE
** is returned.  TRUE is returned all other times, even if the lookup of s2
** wasn't successful.
*/

BOOLEAN	pCopyDstructFields(d1, s1, d2, s2)
Lnode	**d1, **d2;
char	*s1, *s2;
   {
   BOOLEAN	Found;
   Lnode	*From, *To, *Temp, *Copy;
   char		*GSName;

   Found = GetAdr(d1, s1, &From, FALSE);

   if (!Found)
      return	FALSE;

   Found = GetAdr(d2, s2, &To, TRUE);

   Copy = LnodeCopy(From);		/* recursively copies From */

   /* This next part is a little bit complex: To now points to a new
      cell that was returned from GetAdr.  There may be other cells
      pointing to it, so we can't free it, but we want it to be at the
      head of the list pointed to by Copy.  That is, it must take
      Copy's place.

      We have to first clear out its old fields, then copy the info
      from Copy's head into it, then we can free Copy's head.
   */

   if (To->car != NULL)			/* begin cleaning up */
      pDeleteLnode(&(To->car));

   if (To->cdr != NULL)
      pDeleteLnode(&(To->cdr));

   if (To->sval != NULL)
      FreeString(To->sval);

   if (To->rval != NULL)
      FreeScalar(To->rval);

   GSName = To->name;		/* saves name for later */

   bcopy(Copy, To, sizeof(Lnode));	/* Puts To at head of Copy's list */

   FreeString(To->name);
   To->name = GSName;		/* restores name */

   FreeLnode(Copy);    /* Free the old head of the list, since it's unneeded */
   }


/*
** pDeleteLnode recursively deletes the Lnode pointed to by *Struct,
** then sets *Struct to NULL
*/

void	pDeleteLnode(Struct)
Lnode	**Struct;
   {
   if (*Struct == NULL)
      return;

   switch ((*Struct)->tag)
      {
      case NV:
	 FreeString((*Struct)->name);
	 pDeleteLnode(&((*Struct)->cdr));
      break;

      case VLIST:
      case NVLIST:
	 pDeleteLnode(&((*Struct)->car));
	 pDeleteLnode(&((*Struct)->cdr));
      break;

      case REAL:
	 FreeScalar((*Struct)->rval);
      break;

      case STRING:
	 FreeString((*Struct)->sval);
      break;

      default:
	 fprintf(stderr, "\n\nError in Delete: Unknown tag: %d\n", (int) (*Struct)->tag);
	 exit(1);
      }

   FreeLnode(*Struct);
   *Struct = NULL;

   return;   
   }


/*
** pDeleteField looks up in *Struct the field referred to by s1.  If
** it finds it, it returns true, and recursively deletes the Lnode
** referred to by s1.  If it doesn't find it, it returns FALSE
**
** Uses the global var FoundPtr which was set in the call to GetAdr(),
** and which contains the actual address of the pointer in the list
** which points to the cell we're deleting.
*/

BOOLEAN	pDeleteField(Struct, s1)
Lnode	**Struct;
char	*s1;
   {
   Lnode	*From, *DeadListEl;

   if (*s1 == '\0')
      {
      pDeleteLnode(Struct);
      return	TRUE;
      }

   else if (!GetAdr(Struct, s1, &From, FALSE))
      return	FALSE;

   else
      {
      pDeleteLnode(FoundPtr);

      if (((*FoundPtr2)->tag == NVLIST) || ((*FoundPtr2)->tag == VLIST))
	 {
	 /* This is ugly: we have deleted the actual node, but we still
	 ** need to delete the list element that points to it.  FoundPtr2
	 ** is a pointer in the structure that points directly to this
	 ** list element.  So we follow it to remove the list element
	 */

	 /* first, rearrange the list to pass over the list element: */

	 DeadListEl = *FoundPtr2;
	 *FoundPtr2 = (*FoundPtr2)->cdr;

	 /* now, actually free the list element */

         FreeLnode(DeadListEl);
	 }

      return	TRUE;
      }
   }


int pReadDstruct(d)
Lnode **d;
{
  extern Lnode *DSyyparse();

  if (*d != NULL)
      {
      pDeleteLnode(d);
      }

  *d = DSyyparse();
  if ( *d == 0 )
    return EOF;
  else
    return 1;
}


/* Top level pretty printing of dynamic structures. */
void dDstructPrint( lnode)
Lnode *lnode;
{
  if ( lnode != NULL ){
    fLnodePrint(stdout,lnode,1);
    printf(";\n");
  }
}


void fdDstructPrintOneLine( f, lnode )
FILE *f;
Lnode *lnode;
{
  fLnodePrint(f,lnode,0);
  fprintf(f,";");
}


BOOLEAN pQueryDstructPath(l1, str)
Lnode	**l1;
char	*str;
   {
   Lnode	*l2;

   return pLookUpLnode(l1, str, l2);
   }
