/*
 * Copyright (c) 1996, CGL, University of Waterloo
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#ifndef _RATPATCH_H_
#define _RATPATCH_H_

#include "material.h"

typedef struct {
    Material material; 		/* The patch is made of this material.  */
    int degree; 		/* The degree of the patch.             */
    int normaldim; 		/* Zero if no normals, three otherwise. */
    Space range; 		/* Containing space of control vertices.*/
    Point *net; 		/* The control net.                     */
    Scalar* w;			/* The weights */
    Lnode *ln; 			/* External Data */
} RationalPatch;

#define	GAUSSIAN	0
#define	MEAN		1

/*
 RationalPatch RationalPatchCreate( degree, range )
 int degree;
 Space range;

 Return a new, rational triangular Bezier patch of the given degree.
*/
extern RationalPatch RationalPatchCreate(int degree, Space range);

/*
 void RationalPatchFree(patch)
 RationalPatch;

 Free the storage associated with a patch.
*/
extern void RationalPatchFree( RationalPatch patch);

/*
 void RationalPatchEvalWithNormal( patch, point, normal, r, s, t)
 RationalPatch patch;
 Point *point;
 Normal *normal;
 Scalar r, s, t;

 Evaluate the patch at the point whose barycentric coordinates are (r, s, t)
 Also computed is the normal vector to the surface at the point.
*/
extern void RationalPatchEvalWithNormal(RationalPatch patch, Point *point, Normal *normal,
			 Scalar r, Scalar s, Scalar t);

/*
  Point RationalPatchEval( patch, r, s, t)
  RationalPatch patch;
  Scalar r, s, t;

  Evaluate the patch at the point whose barycentric coordinates are (r, s, t).
*/
extern Point RationalPatchEval(RationalPatch  patch, Scalar r, Scalar s, Scalar t);
    
/*
  void RationalPatchSetPoint( patch, pt, i1, i2, i3)
  RationalPatch *patch;
  Point pt;
  int i1, i2, i3;

  Set the vertex whose multi-index is (i1,i2,i3) to pt.
*/
extern void RationalPatchSetPoint(RationalPatch *patch, Point pt, int i1, int i2, int i3);
    
/*
  Point RationalPatchGetPoint( p, i1, i2, i3)
  RationalPatch p;
  int i1, i2, i3;

  Return the position of the control point whose multi-index is (i1,i2,i3);
*/
extern Point RationalPatchGetPoint(RationalPatch p, int i1, int i2, int i3);
extern Point RationalPatchSafeGetPoint(RationalPatch p, int i1, int i2, int i3);


/*
  int RationalPatchRead( fp, patch, range)
  FILE *fp;
  RationalPatch *patch;
  Space range;

  Read a patch from the file fp.  One is returned on success, zero
  on failure.  The coordinates read in are assumed to be coordinates
  relative to the standard frame of the range space.
*/
int RationalPatchRead(FILE* fp,RationalPatch* patch, Space range);

int pGetRationalPatch(Lnode** ds, RationalPatch* patch, Space range);
#define dGetRationalPatch(ds,patch,string) pGetRationalPatch(&(ds),(patch),(string))
#define GetRationalPatch(patch,string) pGetRationalPatch(&(din),(patch),(string))

/*
  void RationalPatchWrite( fp, patch)
  FILE *fp;
  RationalPatch patch;

  Write the patch out on the named stream.
*/
extern void RationalPatchWrite(FILE *fp, RationalPatch patch);


/* WARNING: ROUTINES BELOW THIS POINT HAVE NOT BEEN IMPLEMENTED! */

/*
  Patch RationalPatchSplit( p, pr, ps, pt, r, s, t)
  RationalPatch p;
  RationalPatch *pr, *ps, *pt;
  Scalar r, s, t;

  Split (subdivide) the patch at the point whose barycentric coordinates
  are (r,s,t).  The subpatches are pr, ps, and pt.
*/
extern RationalPatch RationalPatchSplit(RationalPatch p, RationalPatch *pr, 
					RationalPatch *ps, RationalPatch *pt,
					Scalar r, Scalar s, Scalar t);


/*
  RationalPatch RationalPatchDegreeRaise(p)
  RationalPatch	p;

  Returns a patch resulting from degree-raising p.  The new patch
  will have the same dimension of normals as p, but they are not set.
*/
extern RationalPatch RationalPatchDegreeRaise(RationalPatch p);

/*
  Vector RationalPatchDerivEval(p, r,s,t, Vr,Vs,Vt)
  RationalPatch p;
  Scalar r,s,t;
  Scalar Vr, Vs, Vt;

  Return the first derivative of patch p at r,s,t in direction Vr,Vs,Vt.
*/
extern Vector RationalPatchDerivEval(RationalPatch	p, Scalar r, Scalar s, Scalar t,
		       Scalar Vr, Scalar Vs, Scalar Vt);

/*
  Vector RationalPatchDeriv2Eval(p, r,s,t, Vr,Vs,Vt, Wr,Ws,Wt)
  RationalPatch p;
  Scalar r,s,t;
  Scalar Vr, Vs, Vt;
  Scalar Wr, Ws, Wt;

  Return the second derivative of patch p at r,s,t in directions Vr,Vs,Vt
  and Ws,Wr,Wt.
*/
extern Vector RationalPatchDeriv2Eval(RationalPatch	p, Scalar r, Scalar s, Scalar t, 
			Scalar Vr, Scalar Vs, Scalar Vt, 
			Scalar Wr, Scalar Ws, Scalar Wt);

/*
  Scalar RationalPatchCurvature(p, r,s,t, Vr,Vs,Vt)
  RationalPatch p;
  Scalar r,s,t;
  Scalar Vr,Vs,Vt;

  Return the curvature of the patch in direction Vr,Vs,Vt.
*/
Scalar RationalPatchCurvature(RationalPatch p, Scalar r,Scalar s,Scalar t,
		      Scalar Vr, Scalar Vs, Scalar Vt);

/*
  Scalar RationalPatchNormalCurvature(p, r,s,t, Vr,Vs,Vt)
  RationalPatch p;
  Scalar r,s,t;
  Scalar Vr,Vs,Vt;

  Reurn the normal section curvature of the patch in direction Vr,Vs,Vt.
*/
Scalar RationalPatchNormalCurvature(RationalPatch p, 
				    Scalar r,Scalar s,Scalar t,
				    Scalar Vr, Scalar Vs, Scalar Vt);


#endif /* _RATPATCH_H_ */
