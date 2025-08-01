/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: Wed Jul 05, 1989 at 03:58:59 PM
** Purpose: Interface module for triangular Bezier surface patches.
*/

#ifndef _PATCH_H_
#define _PATCH_H_

#include "material.h"
#include "vertex.h"

typedef struct {
    Material material;        /* The patch is made of this material.  */
    int degree;               /* The degree of the patch.             */
    int normaldim;            /* Zero if no normals, three otherwise. */
    Space range;              /* Containing space of control vertices.*/
    VERTEX *net;              /* The control net.                     */
    Lnode *ln;		      /* External Data */
} Patch;

#define	GAUSSIAN	0
#define	MEAN		1

/*
 int VertexIndex(i1, i2, i3)
 int i1, i2, i3;

 Return the index of the control vertex whose multi-index is (i1,i2,i3).
*/
int VertexIndex(int i1, int i2, int i3);

/*
 int NetSize( degree)
 int degree;

 Return the number of control vertices necessary for a BezierTriangle
 of the given degree.
*/
int NetSize(int degree);

/*
 Patch PatchCreate( degree, range, normaldim)
 int degree;
 Space range;
 int normaldim;

 Return a new triangular Bezier patch of the given degree.
 If normals are associated with the patch, normaldim
 should be 3, otherwise it should be 0.
*/
extern Patch PatchCreate(int degree, Space range, int normaldim);

/*
 void PatchFree(patch)
 Patch;

 Free the storage associated with a patch.
*/
extern void PatchFree( Patch patch);

/*
 void PatchEvalWithNormal( patch, point, normal, r, s, t)
 Patch patch;
 Point *point;
 Normal *normal;
 Scalar r, s, t;

 Evaluate the patch at the point whose barycentric coordinates are (r, s, t)
 Also computed is the normal vector to the surface at the point.
*/
extern void PatchEvalWithNormal(Patch patch, Point *point, Normal *normal,
			 Scalar r, Scalar s, Scalar t);

/*
  Point PatchEval( patch, r, s, t)
  Patch patch;
  Scalar r, s, t;

  Evaluate the patch at the point whose barycentric coordinates are (r, s, t).
*/
extern Point PatchEval(Patch  patch, Scalar r, Scalar s, Scalar t);
    
/*
  void PatchSetPoint( patch, pt, i1, i2, i3)
  Patch *patch;
  Point pt;
  int i1, i2, i3;

  Set the vertex whose multi-index is (i1,i2,i3) to pt.
*/
extern void PatchSetPoint(Patch *patch, Point pt, int i1, int i2, int i3);
    
/*
  void PatchSetNormal( patch, normal, i1, i2, i3)
  Patch *patch;
  Normal normal;
  int i1, i2, i3;

  Set the normal of the vertex whose multi-index is (i1,i2,i3) to normal.
*/
extern void PatchSetNormal(Patch *patch, Normal normal, int i1, int i2, int i3);

/*
  Point PatchGetPoint( p, i1, i2, i3)
  Patch p;
  int i1, i2, i3;

  Return the position of the control point whose multi-index is (i1,i2,i3);
*/
extern Point PatchGetPoint(Patch p, int i1, int i2, int i3);
extern Point PatchSafeGetPoint(Patch p, int i1, int i2, int i3);


/*
  Normal PatchGetNormal( p, i1, i2, i3)
  Patch p;
  int i1, i2, i3;

  Return the normal of the control point whose multi-index is (i1,i2,i3);
*/
extern Normal PatchGetNormal( Patch p, int i1, int i2, int i3);


/*
  VERTEX PatchGetVertex( p, i1, i2, i3)
  Patch p;
  int i1, i2, i3;

  Return the vertex (a point and a normal) whose multi-index
  is (i1,i2,i3).
*/
extern VERTEX PatchGetVertex(Patch p, int i1, int i2, int i3);


/*
  int PatchRead( fp, patch, range)
  FILE *fp;
  Patch *patch;
  Space range;

  Read a patch from the file fp.  One is returned on success, zero
  on failure.  The coordinates read in are assumed to be coordinates
  relative to the standard frame of the range space.
*/
int PatchRead(FILE* fp,Patch* patch, Space range);

int pGetPatch(Lnode** ds, Patch* patch, Space range);
#define dGetPatch(ds,patch,string) pGetPatch(&(ds),(patch),(string))
#define GetPatch(patch,string) pGetPatch(&(din),(patch),(string))

/*
  void PatchWrite( fp, patch)
  FILE *fp;
  Patch patch;

  Write the patch out on the named stream.
*/
extern void PatchWrite(FILE *fp, Patch patch);

/*
  Patch PatchSplit( p, pr, ps, pt, r, s, t)
  Patch p;
  Patch *pr, *ps, *pt;
  Scalar r, s, t;

  Split (subdivide) the patch at the point whose barycentric coordinates
  are (r,s,t).  The subpatches are pr, ps, and pt.
*/
extern Patch PatchSplit(Patch p, Patch *pr, Patch *ps, Patch *pt,
		 Scalar r, Scalar s, Scalar t);


/*
  Patch PatchDegreeRaise(p)
  Patch	p;

  Returns a patch resulting from degree-raising p.  The new patch
  will have the same dimension of normals as p, but they are not set.
*/
extern Patch PatchDegreeRaise(Patch p);

/*
  Scalar KGauss(p, r, s, t)
  Patch p;
  Scalar r,s,t;
  
  Return the Gaussian curvature of patch p at r,s,t.
*/
extern Scalar KGauss(Patch	p, Scalar r, Scalar s, Scalar t);
  
/*
  Scalar KMean(p, r, s, t)
  Patch p;
  Scalar r,s,t;
  
  Return the mean curvature of patch p at r,s,t.
*/
extern Scalar KMean(Patch	p, Scalar r, Scalar s, Scalar t);

/*
  Vector PatchDerivEval(p, r,s,t, Vr,Vs,Vt)
  Patch p;
  Scalar r,s,t;
  Scalar Vr, Vs, Vt;

  Return the first derivative of patch p at r,s,t in direction Vr,Vs,Vt.
*/
extern Vector PatchDerivEval(Patch	p, Scalar r, Scalar s, Scalar t,
		       Scalar Vr, Scalar Vs, Scalar Vt);

/*
  Vector PatchDeriv2Eval(p, r,s,t, Vr,Vs,Vt, Wr,Ws,Wt)
  Patch p;
  Scalar r,s,t;
  Scalar Vr, Vs, Vt;
  Scalar Wr, Ws, Wt;

  Return the second derivative of patch p at r,s,t in directions Vr,Vs,Vt
  and Ws,Wr,Wt.
*/
extern Vector PatchDeriv2Eval(Patch	p, Scalar r, Scalar s, Scalar t, 
			Scalar Vr, Scalar Vs, Scalar Vt, 
			Scalar Wr, Scalar Ws, Scalar Wt);

/*
  Scalar PatchCurvature(p, r,s,t, Vr,Vs,Vt)
  Patch p;
  Scalar r,s,t;
  Scalar Vr,Vs,Vt;

  Return the curvature of the patch in direction Vr,Vs,Vt.
*/
Scalar PatchCurvature(Patch p, Scalar r,Scalar s,Scalar t,
		      Scalar Vr, Scalar Vs, Scalar Vt);

/*
  Scalar PatchNormalCurvature(p, r,s,t, Vr,Vs,Vt)
  Patch p;
  Scalar r,s,t;
  Scalar Vr,Vs,Vt;

  Reurn the normal section curvature of the patch in direction Vr,Vs,Vt.
*/
Scalar PatchNormalCurvature(Patch p, Scalar r,Scalar s,Scalar t,
			    Scalar Vr, Scalar Vs, Scalar Vt);


#endif /* _PATCH_H_ */
