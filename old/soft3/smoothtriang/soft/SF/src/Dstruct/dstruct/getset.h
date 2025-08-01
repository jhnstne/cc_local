/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#ifndef _GETSET_H_
#define _GETSET_H_

/*
** getset.h
**
** Author: Michael Lounsbery
** Created: Mon Aug 07, 1989 at 10:29:32 AM
** Last Modified: Mon Aug 07, 1989 at 10:29:33 AM
**
** Purpose: definitions and macros for using getset.c
*/

#ifndef FALSE
#define FALSE 0
#define TRUE 1
#endif

#define MAX_STRINGLEN	81


extern	Lnode	*din, *dout;	/* Global dstruct */

#define GetScalar(FSpec, scalarptr)	pGetScalar(&din, FSpec, scalarptr)
#define	GetString(FSpec, StringPtr)	pGetString(&din, FSpec, StringPtr)
#define	PutScalar(FSpec, scalar)	pPutScalar(&dout, FSpec, scalar)
#define	PutString(FSpec, String)	pPutString(&dout, FSpec, String)

#define dGetScalar(d, FSpec, scalarptr)	pGetScalar(&(d), FSpec, scalarptr)
#define	dGetString(d, FSpec, StringPtr)	pGetString(&(d), FSpec, StringPtr)
#define	dPutScalar(d, FSpec, scalar)	pPutScalar(&(d), FSpec, scalar)
#define	dPutString(d, FSpec, String)	pPutString(&(d), FSpec, String)

#define	dDeleteField(d, s)		pDeleteField(&(d), s)
#define	DeleteField(s)			pDeleteField(&dout, s)

#define	dDeleteDstruct(d)		pDeleteField(&(d), "")
#define	DeleteDstruct()			pDeleteField(&dout, "")

#define	dDeleteLnode(d)			pDeleteLnode(&(d))

void dDstructPrint(Lnode *lnode);
#define dWriteDstruct(d)		dDstructPrint(d)
#define	WriteDstruct()			dDstructPrint(dout)

void fdDstructPrintOneLine(FILE *f, Lnode *lnode);
#define fdWriteDstructOneLine(f,d)		fdDstructPrintOneLine(f,d)
#define	fWriteDstructOneLine(f)		fdDstructPrintOneLine(f,dout)

#define	dCopyDstructFields(d1, s1, d2, s2)	\
	pCopyDstructFields(&(d1), s1, &(d2), s2)

#define	CopyDstructFields(s1, s2)	\
	pCopyDstructFields(&(din), s1, &(dout), s2)

#define	FlushDstruct()	\
	{	dWriteDstruct(dout);	\
		dDeleteDstruct(dout);	\
	}

#define	dReadDstruct(d)			pReadDstruct(&(d))
#define ReadDstruct()			pReadDstruct(&din)

/* in the next ones, *into (type of into is **Lnode) will point to
					the looked-up Lnode */

#define	pLookUpLnode(d, s, into)	GetAdr(d, s, &(into), FALSE)
#define	dLookUpLnode(d, s, into)	GetAdr(&(d), s, &(into), FALSE)
#define	LookUpLnode(s, into)		GetAdr(&(din), s, &(into), FALSE)

BOOLEAN pQueryDstructPath(Lnode **l1, char *str);
#define	dQueryDstructPath(d, s)		pQueryDstructPath(&(d), s)
#define	QueryDstructPath(s)		pQueryDstructPath(&(din), s)

extern BOOLEAN	GetAdr(Lnode **Struct, char *String, Lnode **Adr,
		       BOOLEAN ForceMake);
extern BOOLEAN	pGetScalar(Lnode **Struct,char *FieldSpec,Scalar *ScalarPtr);
extern BOOLEAN	pGetString(Lnode **Struct, char *FieldSpec,char **StringPtr);
extern BOOLEAN	pPutScalar(Lnode **Struct, char *FieldSpec,Scalar scalar);

extern BOOLEAN	pPutString(Lnode **Struct, char *FieldSpec,char *String);
extern BOOLEAN	pCopyDstructFields(Lnode **d1, char *s1, Lnode **d2, char *s2);
extern BOOLEAN	pDeleteField(Lnode **Struct, char *s1);

extern	void	ProcessValue(char *AccessStr,Lnode **Adr, Lnode **NextAdr,BOOLEAN *MakeNew, BOOLEAN *Error, BOOLEAN ForceMake);
extern	void pDeleteLnode(Lnode **Struct);

extern	Lnode	*FindNameInList(char *GSName,Lnode **List,BOOLEAN *MakeNew, 
				BOOLEAN* Error, BOOLEAN ForceMake);
extern	Lnode	 *LnodeCopy(Lnode *Struct);

Lnode	*FindArrayElement( int i, Lnode **Struct,
			  BOOLEAN *MakeNew, BOOLEAN *Error,BOOLEAN  ForceMake);

int pReadDstruct(Lnode **d);
int fpReadDstruct(FILE* fp, Lnode **d);

#endif /* _GETSET_H_ */
