/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: Sun Aug 13, 1989 at 03:01:58 PM
** Purpose: Implementation module for surface patches.
**          The only supported type of patch is currently BezierTriangle.
**
** This implementation is exclusively for Bezier Triangles.  To extend it
** to Bezier simplexes a bit more work would need to be done, especially
** if an efficient solution was wanted.
**
** The present form should be sufficient to support 590B, Spring '89.
*/

#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "util.h"
#include "vertex.h"
#include "dstruct.h"
#include "getset.h"
#include "material.h"
#include "ratpatch.h"

extern void *malloc();


/*
** Return the index of the control vertex whose multi-index
** is (i1, i2, i3).  Vertices are stored in lexographic order
** within a linear array.
*/
static int VertexIndex(int i1, int i2, int i3)
{
    return i1*(i1+ i2 + i3 + 1) - i1*(i1 - 1)/2 + i2;
}


/*
** Return the number of control vertices needed for a Bezier
** Triangle of the given degree.
*/
static int NetSize(int degree)
{
    return (degree+1)*(degree+2)/2;
}


/*
** Do an affine blending of a triple of points in the given control
** point net.
*/
static Point AffineBlend(Point *net, Scalar *w, int i1, int i2, int i3, 
			 Scalar r, Scalar s, Scalar t)
{
	Scalar wn;

	wn = w[VertexIndex(i1+1,i2,i3)]*r + 
	  w[VertexIndex(i1,i2+1,i3)]*s +
	    w[VertexIndex(i1,i2,i3+1)]*t;
	return PPac3(net[VertexIndex(i1+1,i2,i3)],
		     net[VertexIndex(i1,i2+1,i3)],
		     net[VertexIndex(i1,i2,i3+1)], 
		     w[VertexIndex(i1+1,i2,i3)]*r/wn, 
		     w[VertexIndex(i1,i2+1,i3)]*s/wn, 
		     w[VertexIndex(i1,i2,i3+1)]*t/wn);
}



/*
** Create and return a new patch.
*/
RationalPatch RationalPatchCreate(int degree, Space range)
{
    RationalPatch new;

    new.material = NullMaterial;
    new.degree = degree;
    new.range  = range;
    new.net    = (Point *) malloc(sizeof(VERTEX) * NetSize( degree));
    new.w	= (Scalar*) malloc(sizeof(Scalar) * NetSize(degree));
    new.ln = NULL;
    return new;
}


/*
** Free storage associated with the patch.
*/
void RationalPatchFree( RationalPatch patch )
{
    free( patch.net);
    free( patch.w);
    pDeleteLnode(&(patch.ln));
}



/*
**  Set the vertex whose multi-index is (i1,i2,i3) to pt.
*/
void RationalPatchSetPoint(RationalPatch *patch, 
			   Point pt, int i1, int i2, int i3)
{
    patch->net[VertexIndex(i1, i2, i3)] = pt;
}


/*
** Return the vertex (a point and a normal) whose multi-index
** is (i1,i2,i3).
*/
Point RationalPatchGetPoint(RationalPatch p, int i1, int i2, int i3)
{
    return p.net[VertexIndex(i1,i2,i3)];
}


/*
** If the index is valid for a patch of this degree, just generate a call to
** PatchGetPoint.  Otherwise, return the Zero point.  The point of this is
** to avoid bus errors from accessing non-existent places in the net.
*/

Point	RationalPatchSafeGetPoint(RationalPatch p, int i1, int i2, int i3)
   {
   Point	Zero;
   int	Index = VertexIndex(i1, i2, i3);
   int	MaxIndex = NetSize(p.degree) - 1;

   if ((Index >= 0) && (Index <= MaxIndex))
      return RationalPatchGetPoint(p, i1, i2, i3);

   else
      {
      Zero = PCreate(StdFrame(p.range), 0., 0., 0.);
      return Zero;
      }
   }


/*
**  Read a patch from stdin.  fp is ignored. One is returned on success, zero
**  on end-of-file.  The coordinates read in are assumed to be coordinates
**  relative to the standard frame of the range space.
*/
int	RationalPatchRead(FILE *fp, RationalPatch* patch, Space range)
{
	char buf[BUFSIZE];
	Scalar numsides, degree, dim;
	int i, j, numvertices;
	Scalar x[3], nx[3];
	Lnode *Lookup, *LRead;
	Material mat;

	LRead = NULL;

	if (dReadDstruct( LRead) == EOF) {
		return 0;
	}

	if ( !dQueryDstructPath( LRead, "RationalBezierTriangle" ) ) {
		fprintf(stderr, "RationalPatchRead: RationalBezierTriangle not read in\n");
		exit(1);
	}

	if ( (!dGetScalar( LRead, "RationalBezierTriangle.degree", &degree)) ) {
		fprintf(stderr, "RationalPatchRead: Incomplete patch read\n");
		exit(1);
	}
 
	*patch = RationalPatchCreate((int) degree, range);
	patch->ln = LRead;

	if ( dQueryDstructPath(LRead,"RationalBezierTriangle.material") ) {
		if ( !dGetRGBMaterial(LRead, "RationalBezierTriangle", 
				      &(patch->material)) ) {
			    fprintf(stderr, "RationalPatchRead: Incomplete material field\n");
			    exit(1);
		    }
	}

	numvertices = (degree + 1)*(degree + 2)/2;
	for (i = 0; i < numvertices; i++) {
		for (j = 0; j < 3; j++) {
			sprintf(buf, 
				"RationalBezierTriangle.net[%d].pos[%d]", i, j);
			if (!dGetScalar(LRead, buf, &(x[j]))) {
				fprintf(stderr, 
					"RationalPatchRead: Error reading control net; 3 coords expected\n");
				exit(1);
			}
		}

		patch->net[i] = PCreate(StdFrame(range), x[0], x[1], x[2]);

		sprintf(buf, "RationalBezierTriangle.w[%d]", i);
		if ( !dGetScalar(LRead, buf, &(patch->w[i])) ) {
			fprintf(stderr, 
				"RationalPatchRead: Error reading weight;\n");
			exit(1);
		}
	}

	return 1;
}

/*
 *----------------------------------------------------------------------
 *  Function: pGetRationalPatch
 *	Read a patch from a dstruct.
 *----------------------------------------------------------------------
 */
int	pGetRationalPatch(Lnode** ds, RationalPatch* patch, Space range)
{
	char buf[BUFSIZE];
	Scalar numsides, degree, dim;
	int i, j, numvertices;
	Scalar x[3], nx[3];
	Lnode *Lookup, *LRead=NULL;
	Material mat;
	
	LRead = *ds;
	
	if ( !dQueryDstructPath( LRead, "RationalBezierTriangle" ) ) {
		fprintf(stderr, "pGetRationalPatch: RationalBezierTriangle not read in\n");
		exit(1);
	}
	
	if ( (!dGetScalar( LRead, "RationalBezierTriangle.degree", &degree)) ) {
		fprintf(stderr, "pGetRationalPatch: Incomplete patch read\n");
		exit(1);
	}
	
	*patch = RationalPatchCreate((int) degree, range);
	
	dCopyDstructFields(LRead,"RationalBezierTriangle",patch->ln,"RationalBezierTriangle");
	
	if ( dQueryDstructPath(LRead,"RationalBezierTriangle.material") ) {
		if ( !dGetRGBMaterial(LRead, "RationalBezierTriangle", &(patch->material)) ) {
			fprintf(stderr, "pGetRationalPatch: Incomplete material field\n");
			exit(1);
		}
	}
	
	numvertices = (degree + 1)*(degree + 2)/2;
	for (i = 0; i < numvertices; i++)       {
		for (j = 0; j < 3; j++)	 {
			sprintf(buf, "RationalBezierTriangle.net[%d].pos[%d]", i, j);
			if (!dGetScalar(LRead, buf, &(x[j])))	    {
				fprintf(stderr, 
					"pGetRationalPatch: Error reading control net; 3 coords expected\n");
				exit(1);
			}
		}
    
		patch->net[i] = PCreate(StdFrame(range), x[0], x[1], x[2]);
    
		sprintf(buf, "RationalBezierTriangle.w[%d]", i);
		if (!dGetScalar(LRead, buf, &(patch->w[i])))	       {
			fprintf(stderr, 
				"pGetRationalPatch: Error reading weights\n");
			exit(1);
		}
	}
	
	return 1;
}


/*
** Writes out the patch on stdout.  fp is ignored.
*/

void	RationalPatchWrite(FILE *fp, RationalPatch patch)
{
	int	i, j, numvertices;
	char buf[100];
	Scalar x[3], nx[3];
	int destroyDstruct=0;
	Lnode *ln=NULL;

	dCopyDstructFields(patch.ln,"RationalBezierTriangle",ln,"RationalBezierTriangle");
	dPutScalar(ln, "RationalBezierTriangle.degree", (Scalar) patch.degree);

	if ( !dQueryDstructPath(ln,"RationalBezierTriangle.material")  &&
	    !IsNullMaterial(patch.material) ){
		pSetRGBMaterial(&(ln), "RationalBezierTriangle", &(patch.material));
	}

	numvertices = (patch.degree + 1)*(patch.degree + 2)/2;

	for (i = 0; i < numvertices; i++) {
		PCoords(patch.net[i], StdFrame(patch.range),
			&(x[0]), &(x[1]), &(x[2]));
		for (j = 0; j < 3; j++) {
			sprintf(buf, "RationalBezierTriangle.net[%d].pos[%d]", i, j);
			dPutScalar(ln, buf, x[j]);
		}

		sprintf(buf, "RationalBezierTriangle.w[%d]", i);
		dPutScalar(ln, buf, patch.w[i]);
	}
	dDstructPrint(ln);	/* writes all on stdout */
	pDeleteLnode(&(ln));
}

/*
** Evaluate the patch at the point whose barycentric coordinates
** are (r, s, t)
*/
Point RationalPatchEval(RationalPatch  patch, Scalar r, Scalar s, Scalar t)
{
    Point surfacePoint;
    Normal normal;

    RationalPatchEvalWithNormal( patch, &surfacePoint, &normal, r, s, t);
    return surfacePoint;
}




/*
** Evaluate the patch at the point whose barycentric coordinates are
** (r, s, t). Also computed is the normal vector to the surface at
** the point.
*/
void RationalPatchEvalWithNormal(RationalPatch patch, 
				 Point *point, Normal *normal,
				 Scalar r, Scalar s, Scalar t)
{
	int i;                        /* Misc index */
	Point *tmp;                   /* Temporary net for deCasteljau alg */
	Scalar *tmpW;
	int i1, i2, i3;               /* multi-indices */
	int k;                        /* The step of deCasteljau's alg */
	
	/* Create the temporary net */
	tmp = (Point *) malloc( sizeof(Point) * NetSize( patch.degree));
	tmpW = (Scalar*) malloc( sizeof(Scalar) * NetSize( patch.degree ));
	
	/* Copy the patches net into tmp */
	for (i = 0; i < NetSize( patch.degree); i++) {
		tmp[i] = patch.net[i];
		tmpW[i] = patch.w[i];
	}
	

	/* Do de Casteljau's algorithm in place on the temporary net, but 
	   don't do the last step of the algorithm.                       */
	for (k = patch.degree - 1; k > 0; k--) {
		for (i1 = 0; i1 <= k; i1++) {
			for (i2 = 0; i2 <= k - i1; i2++) {
				i3 = k - i1 - i2;
				tmp[VertexIndex(i1,i2,i3)] = 
				  AffineBlend(tmp,tmpW,i1,i2,i3,r,s,t);
				tmpW[VertexIndex(i1,i2,i3)] = 	
				  tmpW[VertexIndex(i1+1,i2,i3)]*r + 
				    tmpW[VertexIndex(i1,i2+1,i3)]*s +
				      tmpW[VertexIndex(i1,i2,i3+1)]*t;
			}
		}
	}

	/* tmp[0], tmp[1], and tmp[2] span the tangent plane. */
	/* Compute the normal from them...                    */
	*normal = PPPNormal( tmp[0], tmp[2], tmp[1]);

	/* Do the last step of de Casteljau to compute the point on the 
	   surface */
	*point  = AffineBlend( tmp, tmpW, 0, 0, 0, r, s, t);

	/* Free the temporary storage and return*/
	free(tmp);
	free(tmpW);
}

/*  CHANGED TO HERE */

#if 0
/*
** Split (subdivide) the patch at the point whose barycentric coordinates
** are (r,s,t).  The subpatches are pr, ps, and pt.
*/
Patch PatchSplit(Patch p, Patch *pr, Patch *ps, Patch *pt,
		 Scalar r, Scalar s, Scalar t)
{
    Patch *Tetra;              /* the degenerate tetrahedron */
    int k, i1, i2, i3;

    /* Create the subpatches */
    *pr = PatchCreate( p.degree, p.range, 0);
    *ps = PatchCreate( p.degree, p.range, 0);
    *pt = PatchCreate( p.degree, p.range, 0);

    /* Initialize the tetrahedron.  The tetrahedron is represented as */
    /* an array of patches, one for each level.                       */
    Tetra = (Patch *) malloc( (p.degree + 1) * sizeof( Patch));
    for (k = 0; k < p.degree; k++) {
	Tetra[k] = PatchCreate( k, p.range, 0);
    }

    /* Initialize the bottom face of the tetrahedron to be the control */
    /* net of the patch.                                               */
    Tetra[p.degree] = p;

    /* Build up the tetrahedron a level at a time starting at the bottom */
    for (k = p.degree - 1; k >= 0; k--) {
	for (i1 = 0; i1 <= k; i1++) {
	    for (i2 = 0; i2 <= k - i1; i2++) {
		i3 = k - i1 - i2;

		Tetra[k].net[VertexIndex(i1,i2,i3)].position =
		        PPac3( PatchGetPoint(Tetra[k+1], i1+1, i2, i3),
			       PatchGetPoint(Tetra[k+1], i1, i2+1, i3),
			       PatchGetPoint(Tetra[k+1], i1, i2, i3+1),
			       r, s, t);
	    }
	}
    }

    /* Extract the control points */
    for (i1 = 0; i1 <= p.degree; i1++) {
	for (i2 = 0; i2 <= p.degree - i1; i2++) {
	    i3 = p.degree - i1 - i2;

	    /* pr corresponds to the i1=0 face of the tetrahedron */
	    PatchSetPoint( pr, PatchGetPoint( Tetra[p.degree-i1], 0, i2, i3),
			                                    i1, i2, i3);

	    /* ps corresponds to the i2=0 face of the tetrahedron */
	    PatchSetPoint( ps, PatchGetPoint( Tetra[p.degree-i2], i1, 0, i3),
			                                    i1, i2, i3);

	    /* pt corresponds to the i3=0 face of the tetrahedron */
	    PatchSetPoint( pt, PatchGetPoint( Tetra[p.degree-i3], i1, i2, 0),
			                                    i1, i2, i3);
	}
    }

    /* Free the tetrahedron storage */
    for (k = 0; k < p.degree; k++) {
	PatchFree( Tetra[k]);
    }
    free(Tetra);
}



/*
** Returns a patch resulting from degree-raising p.  The new patch
** will have the same dimension of normals as p, but they are not set.
*/

Patch PatchDegreeRaise(Patch p)
   {
   Point	NewPt;
   int		i1, i2, i3;
   int		n = p.degree;
   Patch pUp;

   pUp = PatchCreate(n + 1, p.range, p.normaldim);

   for (i1 = 0; i1 <= n + 1; i1++)
      {
      for (i2 = 0; i2 <= n + 1 - i1; i2++)
	 {
	 i3 = n + 1 - i1 - i2;

	 NewPt = PPac3(
			PatchSafeGetPoint(p, i1 - 1, i2, i3),
			PatchSafeGetPoint(p, i1, i2 - 1, i3),
			PatchSafeGetPoint(p, i1, i2, i3 - 1),
			((Scalar) i1 / (n + 1)),
			((Scalar) i2 / (n + 1)),
			((Scalar) i3 / (n + 1))
		      );

	 PatchSetPoint(&pUp, NewPt, i1, i2, i3);
	 }
      }
   return pUp;
   }


/* The following code used to be in a file called kurvature as part
 * of Tess. */

/*
** gauss.c
**
** Author: Michael Lounsbery
** Created: Mon Aug 07, 1989 at 09:44:08 AM
** Last Modified: Mon Aug 07, 1989 at 09:44:08 AM
**
** Purpose: package for Gaussian and Mean curvature
*/





extern	Point	AffineBlend();

/*
** Given a patch p, returns the Gaussian curvature at (r,s,t)
*/

static Scalar	Kurvature(Patch	p, Scalar r, Scalar s, Scalar t, int KType)
   {
   Vector	s10, s01, s11, s20, s02;	/* derivative vectors */
   Vector	Nh;				/* normal - used as vector */
   Scalar	E, F, G, L, M, N;	/* components of 1st, 2nd fund. form */
   Scalar	Det1, Det2;		/* determinants of matrix for E-M */
   Scalar	H, K;			/* different curvature types */
   Scalar	Vr, Vs, Vt, Wr, Ws, Wt;	/* different'n direction components */

   /* determines V, W vectors to be used in differentiation */

   /* V will correspond to a change towards r, W to a change towards s */

   /* First we compute the new place in the domain that V, W take us. */
   /* Vr is set to be a jump towards r, Vs, Vt set to be valid barycentric */
   /* coords.  W is then set the same way, except the jump towards s is key. */

#if 1
   Vr = -1.0; Vs = 1.0; Vt =  0.0;
   Wr =  1.0; Ws = 0.0; Wt = -1.0;
#else

   Vr = 0.0; Vs = 1.0; Vt = -1.0;
   Wr = 1.0; Ws = 0.0; Wt = -1.0;
#endif
   s10 = PatchDerivEval(p, r, s, t, Vr, Vs, Vt);
   s01 = PatchDerivEval(p, r, s, t, Wr, Ws, Wt);

   s11 = PatchDeriv2Eval(p, r, s, t, Vr, Vs, Vt,  Wr, Ws, Wt);
   s20 = PatchDeriv2Eval(p, r, s, t, Vr, Vs, Vt,  Vr, Vs, Vt);
   s02 = PatchDeriv2Eval(p, r, s, t, Wr, Ws, Wt,  Wr, Ws, Wt);
   s20 = PatchDeriv2Eval(p, r, s, t, Vr, Vs, Vt,  Vr, Vs, Vt);

   Nh = VVCross(s10, s01);
   Nh = SVMult((1.0 / VMag(Nh)), Nh);

   E = VMag(s10);
   E = E * E;

   F = VVDot(s10, s01);

   G = VMag(s01);
   G = G * G;

   Det1 = E * G - F * F;

   L = VVDot(s20, Nh);
   M = VVDot(s11, Nh);
   N = VVDot(s02, Nh);

   Det2 = L * N - M * M;

   if (KType == GAUSSIAN)
      {
      K = Det2 / Det1;
      return K;
      }
   else if (KType == MEAN)
      {
      H = (E * N) + (G * L) - (2.0 * F * M);
      H = H / (2.0 * Det1);
      return H;
      }
   }


Scalar	KGauss(Patch	p, Scalar r, Scalar s, Scalar t)
   {
   return	Kurvature(p, r, s, t, GAUSSIAN);
   }


Scalar	KMean(Patch	p, Scalar r, Scalar s, Scalar t)
   {
   return	Kurvature(p, r, s, t, MEAN);
   }


/*
** PatchDerivEval:
**
** Return the derivative at a point (r,s,t) on a patch in the direction given
** by v = (Vr, Vs, Vt).
**
*/

Vector	PatchDerivEval(Patch	p, Scalar r, Scalar s, Scalar t,
		       Scalar Vr, Scalar Vs, Scalar Vt)
{
	int i;                        /* Misc index */
	Point *tmp;                   /* Temporary net for deCasteljau alg */
	int i1, i2, i3;               /* multi-indices */
	int k;                        /* The step of deCasteljau's alg */
	Point	VSpot, USpot;
	Vector	Result;
	
	/* Create the temporary net */
	tmp = (Point *) malloc( sizeof(Point) * NetSize( p.degree));
	
	/* Copy the patches net into tmp */
	for (i = 0; i < NetSize( p.degree); i++) {
		tmp[i] = p.net[i].position;
	}
	
	/* Now evaluate at u = (r,s,t) for a while: */
	
	/* Do de Casteljau's algorithm in place on the temporary net, but */
	/* don't do the last step of the algorithm.                       */
	
	for (k = p.degree - 1; k > 0; k--) {
		for (i1 = 0; i1 <= k; i1++) {
			for (i2 = 0; i2 <= k - i1; i2++) {
				i3 = k - i1 - i2;
				tmp[VertexIndex(i1,i2,i3)] =
					AffineBlend(tmp,i1,i2,i3,r,s,t);
			}
		}
	}
	
	/* Now the only ones left are tmp[0], tmp[1], and tmp[2]  */
	/* We now evaluate them at the same u as before, and also */
	/* at u+v.						     */
	
	USpot = AffineBlend(tmp, 0, 0, 0, r, s, t);
	VSpot = AffineBlend(tmp, 0, 0, 0, r + Vr, s + Vs, t + Vt);
	
	Result = PPDiff(VSpot, USpot);
	Result = SVMult((Scalar) p.degree, Result);
	
	/* Free the temporary storage and return*/
	free(tmp);
	
	return	Result;
}


/*
** PatchDeriv2Eval:
**
** Return the 2nd derivative at a point (r,s,t) on a patch in the direction
** given by v = (Vr, Vs, Vt) and w = (Wr, Ws, Wt).
**
** Based heavily on a routine from Tony's patch library.
*/

Vector	PatchDeriv2Eval(Patch	p, Scalar r, Scalar s, Scalar t, 
			Scalar Vr, Scalar Vs, Scalar Vt, 
			Scalar Wr, Scalar Ws, Scalar Wt)
   {
   int i;                        /* Misc index */
   Point *tmp, *tmp2;            /* Temporary net for deCasteljau alg */
   int i1, i2, i3;               /* multi-indices */
   int k;                        /* The step of deCasteljau's alg */
   Point	VSpot, USpot, WSpot, VWSpot;
   Vector	Result, WVector, VVector;

   if (p.degree < 2)
      return	VCreate(StdFrame(p.range), 0, 0, 0);

   /* Create the temporary net */
   tmp = (Point *) malloc( sizeof(Point) * NetSize( p.degree));
   tmp2 = (Point *) malloc( sizeof(Point) * NetSize( p.degree));

   /* Copy the patches net into tmp */
   for (i = 0; i < NetSize( p.degree); i++) {
      tmp[i] = p.net[i].position;
   }

   /* Now evaluate at u = (r,s,t) for a while: */

   /* Do de Casteljau's algorithm in place on the temporary net, but */
   /* don't do the last 2 steps of the algorithm.                    */

   for (k = p.degree - 1; k > 1; k--)
      {
      for (i1 = 0; i1 <= k; i1++)
	 {
	 for (i2 = 0; i2 <= k - i1; i2++)
	    {
	    i3 = k - i1 - i2;
	    tmp[VertexIndex(i1,i2,i3)] = AffineBlend(tmp,i1,i2,i3,r,s,t);
	    }
	 }
      }

   /* Now the only ones left are tmp[0] through tmp[5]	     */
   /* We now evaluate them at the same u as before, and also */
   /* at u+v and u+w.					     */

   /* Copy the tmp net into tmp2 */
   for (i = 0; i < 6; i++) {
      tmp2[i] = tmp[i];
   }

   /* Now we've evaluated at u+w */

   /* Evaluate tmp at u+w: */

   k = 1;
   for (i1 = 0; i1 <= k; i1++)
      {
      for (i2 = 0; i2 <= k - i1; i2++)
	 {
	 i3 = k - i1 - i2;
	 tmp[VertexIndex(i1,i2,i3)] = AffineBlend(tmp,i1,i2,i3,
						  r+Wr, s+Ws, t+Wt);
	 }
       }

   /* Evaluate tmp2 at u: */

   k = 1;
   for (i1 = 0; i1 <= k; i1++)
      {
      for (i2 = 0; i2 <= k - i1; i2++)
	 {
	 i3 = k - i1 - i2;
	 tmp2[VertexIndex(i1,i2,i3)] = AffineBlend(tmp2,i1,i2,i3,r,s,t);
	 }
       }

   /* tmp has now been eval'd at u+w, tmp2 has been eval'd at u */

   WSpot = AffineBlend(tmp, 0, 0, 0, r, s, t);
   VWSpot = AffineBlend(tmp, 0, 0, 0, r + Vr, s + Vs, t + Vt);

   WVector = PPDiff(VWSpot, WSpot);

   USpot = AffineBlend(tmp2, 0, 0, 0, r, s, t);
   VSpot = AffineBlend(tmp2, 0, 0, 0, r + Vr, s + Vs, t + Vt);

   VVector = PPDiff(VSpot, USpot);

   Result = VVDiff(WVector, VVector);
   Result = SVMult((Scalar) (p.degree * (p.degree - 1)), Result);

   /* Free the temporary storage and return*/
   free(tmp);
   free(tmp2);

   return Result;
   }


/* end of kurvature.c */

Scalar PatchCurvature(Patch p, Scalar r,Scalar s,Scalar t,
		      Scalar Vr, Scalar Vs, Scalar Vt)
{
	Vector first;
	Vector second;
	Scalar magfirst;
	Point pt;
	Normal n;
	Scalar sign;

	first = PatchDerivEval(p, r,s,t, Vr,Vs,Vt);
	second = PatchDeriv2Eval(p, r,s,t, Vr,Vs,Vt, Vr,Vs,Vt);
	magfirst = VMag(first);
	PatchEvalWithNormal( p, &pt, &n, r,s,t);
	sign = VVDot(VNormalize(second),VNormalize(NDual(n)));
	if ( fabs(sign) < 1e-14 ) {
		Scalar x,y,z;
		fprintf(stderr,"PatchCurvature: normal perp to sec deriv.\n");
		fprintf(stderr,"%g %g\n",VMag(second),VMag(NDual(n)));
		VCoords(second,StdFrame(SpaceOf(second)),&x,&y,&z);
		fprintf(stderr,"%g %g %g\n",x,y,z);
		NCoords(n,StdFrame(SpaceOf(second)),&x,&y,&z);
		fprintf(stderr,"%g %g %g\n",x,y,z);
		exit(1);
	}
	sign /= fabs(sign);
	return sign*VMag(VVCross(first,second))/(magfirst*magfirst*magfirst);
}

Scalar PatchNormalCurvature(Patch p, Scalar r,Scalar s,Scalar t,
			    Scalar Vr, Scalar Vs, Scalar Vt)
{
	Vector first;
	Vector second;
	Scalar magfirst;
	Point pt;
	Normal n;
	Scalar sign;
	Vector binormal;
	Vector nv;

	first = PatchDerivEval(p, r,s,t, Vr,Vs,Vt);
	second = PatchDeriv2Eval(p, r,s,t, Vr,Vs,Vt, Vr,Vs,Vt);
	binormal = VVCross(first,second);
	if ( VMag(binormal) < 1e-7 ) {
		return 0.;
	}
	nv = VNormalize(VVCross(binormal,first));
	magfirst = VMag(first);
	PatchEvalWithNormal( p, &pt, &n, r,s,t);
	sign = VVDot(nv, VNormalize(NDual(n)));
	return sign*VMag(VVCross(first,second))/(magfirst*magfirst*magfirst);
}
#endif
