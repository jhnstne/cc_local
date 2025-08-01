/*
** geometry.h: Header file for the affine geometry package.
**
** Copyright (c) 1989, Graphics and AI Laboratory, University of Washington
** Copying, use and development for non-commercial purposes permitted.
**                  All rights for commercial use reserved.
**		    This software is unsupported.
**
** $Header: geometry.h,v 1.5 89/04/08 19:13:17 derose Locked $
**
*/
#ifndef _GEOMETRY
#define _GEOMETRY

#include "matrix.h"

/* Constants */
#define TWOSPACE	2
#define THREESPACE	3

/* Access macros */
#define SpaceOf(obj)	(obj.s)	        /* Pointer to containing space */
#define Name(S)		(S->name)	/* The name of the space.  */
#define Dim(S)		(S->dim)	/* The dimension of an affine space */
#define StdFrame(S)	(S->stdf)	/* Standard frame for a space */

/* Macros for Derived operations */
#define VMag(v)			sqrt(VVDot(v,v))
#define PPDist(p1,p2)		VMag(PPDiff(p2,p1))

typedef struct frame {			/* An affine frame.                */
	struct euclideanSpace *s;	/* The containing space.           */
	char *name;			/* Printable name for debugging.   */
	Matrix tostdf;			/* Rep of frame rel to s->stdf.    */
	Matrix fromstdf;		/* Rep of s->stdf rel to frame.    */
} Frame;

typedef struct euclideanSpace {		/* A Euclidean space.            */
	int dim;			/* The dimension of the space    */
	char *name;			/* Printable name for debugging  */
	Frame stdf;			/* A predefined Cartesian frame. */
} *Space;

typedef struct {			/* A point.                      */
	Space s;			/* The containing space.         */
	Matrix p;			/* Coords relative to s->stdf    */
} Point;

typedef struct {			/* A vector.                     */
        Space s;                        /* The containing space.         */
	Matrix v;			/* Coords relative to s->stdf    */
} Vector;

typedef struct {			/* A normal vector.              */
	Space s;         		/* The containing space.         */
	Matrix n;			/* Coords relative to s->stdf    */
} Normal;

typedef struct {			/* An affine transformation.     */
        char invertible;                /* TRUE if an invertible xform.  */
	Space range, domain;
	Matrix t, invt;			/* For efficiency, both the  */
					/* transform and its inverse */
					/* are stored.               */
} AffineMap;

/* Imported routines */
extern double sqrt();

/* Creation routines */
extern Space       SCreate();
extern Frame       FCreate();
extern Point       PCreate();
extern Vector      VCreate(), VZero();
extern Normal      NCreate();
extern AffineMap   ACreate(), ACreateF(), AIdentity();

/* Routines that return coordinates */
extern void        PCoords();
extern void        VCoords();
extern void        NCoords();

/* Functions for accessing frames */
extern Point       FOrg();
extern Vector	   FV();

/* Functions for combining points and vectors */
extern Point       PVAdd();
extern Vector      PPDiff();
extern Point	   PPrr(), PPac(), PPac3(), PPacN();
extern Vector      PPvcN();
extern Normal      PPPNormal();

/* Primitive vector space operations */
extern Vector      SVMult();
extern Vector      VVAdd();
extern Vector      VVDiff();
extern Scalar      VVDot();
extern Vector	   VVCross();
extern Vector      VVProj();
extern Normal      VDual();
extern Vector      NDual();
extern Scalar      NVApply();
extern Vector      VVlcN();

/* Derived operations */
extern Vector	   VNormalize();

/* Transformations */
extern Point       PAxform();
extern Vector      VAxform();
extern Normal      NAxform();
extern AffineMap   AACompose();

#endif
