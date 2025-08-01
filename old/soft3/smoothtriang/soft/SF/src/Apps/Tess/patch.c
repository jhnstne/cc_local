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
#include "all.h"
extern char *malloc();


/*
** Return the index of the control vertex whose multi-index
** is (i1, i2, i3).  Vertices are stored in lexographic order
** within a linear array.
*/
int VertexIndex(i1, i2, i3)
int i1, i2, i3;
{
    return i1*(i1+ i2 + i3 + 1) - i1*(i1 - 1)/2 + i2;
}


/*
** Return the number of control vertices needed for a Bezier
** Triangle of the given degree.
*/
int NetSize( degree)
int degree;
{
    return (degree+1)*(degree+2)/2;
}


/*
** Do an affine blending of a triple of points in the given control
** point net.
*/
Point AffineBlend( net, i1, i2, i3, r, s, t)
Point *net;
int i1, i2, i3;
Scalar r, s, t;
{
 return PPac3(net[VertexIndex(i1+1,i2,i3)],
	      net[VertexIndex(i1,i2+1,i3)],
	      net[VertexIndex(i1,i2,i3+1)], r, s, t);
}



/*
** Create and return a new patch.
*/
Patch PatchCreate( degree, range, normaldim)
int degree;
Space range;
int normaldim;
{
    Patch new;

    new.material = NullMaterial;
    new.degree = degree;
    new.range  = range;
    if ((normaldim != 0) && (normaldim != 3)) {
	fprintf( stderr, "PatchCreate: normal dimension not 0 or 3.\n");
	exit(1);
    }
    new.normaldim = normaldim;
    new.net    = (VERTEX *) malloc(sizeof(VERTEX) * NetSize( degree));
    new.ln = NULL;
    return new;
}


/*
** Free storage associated with the patch.
*/
void PatchFree( patch)
Patch patch;
{
    free( patch.net);
    pDeleteLnode(&(patch.ln));
}



/*
**  Set the vertex whose multi-index is (i1,i2,i3) to pt.
*/
void PatchSetPoint( patch, pt, i1, i2, i3)
Patch *patch;
Point pt;
int i1, i2, i3;
{
    patch->net[VertexIndex(i1, i2, i3)].position = pt;
}


/*
**  Set the normal of the vertex whose multi-index is (i1,i2,i3) to normal
*/
void PatchSetNormal( patch, normal, i1, i2, i3)
Patch *patch;
Normal normal;
int i1, i2, i3;
{
    patch->net[VertexIndex(i1, i2, i3)].normal = normal;
}

/*
** Return the vertex (a point and a normal) whose multi-index
** is (i1,i2,i3).
*/
VERTEX PatchGetVertex( p, i1, i2, i3)
Patch p;
int i1, i2, i3;
{
    return p.net[VertexIndex(i1,i2,i3)];
}


/*
** Return the position of the control point whose multi-index is (i1,i2,i3);
*/
Point PatchGetPoint( p, i1, i2, i3)
Patch p;
int i1, i2, i3;
{
    VERTEX v;

    v = PatchGetVertex( p, i1, i2, i3);
    return v.position;
}

/*
** If the index is valid for a patch of this degree, just generate a call to
** PatchGetPoint.  Otherwise, return the Zero point.  The point of this is
** to avoid bus errors from accessing non-existent places in the net.
*/

Point	PatchSafeGetPoint(p, i1, i2, i3)
Patch	p;
int	i1, i2, i3;
   {
   Point	Zero;
   int	Index = VertexIndex(i1, i2, i3);
   int	MaxIndex = NetSize(p.degree) - 1;

   if ((Index >= 0) && (Index <= MaxIndex))
      return PatchGetPoint(p, i1, i2, i3);

   else
      {
      Zero = PCreate(StdFrame(p.range), 0, 0, 0);
      return Zero;
      }
   }


/*
** Return the normal of the control point whose multi-index is (i1,i2,i3);
*/
Normal PatchGetNormal( p, i1, i2, i3)
Patch p;
int i1, i2, i3;
{
    VERTEX v;

    v = PatchGetVertex( p, i1, i2, i3);
    return v.normal;
}


/*
**  Read a patch from stdin.  fp is ignored. One is returned on success, zero
**  on end-of-file.  The coordinates read in are assumed to be coordinates
**  relative to the standard frame of the range space.
*/
int	PatchRead(fp, patch, range)
FILE	*fp;
Patch	*patch;
Space	range;
   {
   char buf[BUFSIZE];
   Scalar numsides, degree, dim, ratdim, normaldim;
   int i, j, numvertices;
   Scalar x[3], nx[3];
   Lnode *Lookup, *LRead;
   Material mat;

   LRead = NULL;

   if (dReadDstruct( LRead) == EOF)
      return 0;

   if ( !dQueryDstructPath( LRead, "BezierTriangle" ) ) {
     fprintf(stderr, "PatchRead: BezierTriangle not read in\n");
     exit(1);
   }

   if ( (!dGetScalar( LRead, "BezierTriangle.degree", &degree))
					||
        (!dGetScalar(LRead,"BezierTriangle.normaldim",&normaldim)) ) {
     fprintf(stderr, "PatchRead: Incomplete patch read\n");
     exit(1);
   }

   if ((normaldim != 0.0) && (normaldim != 3.0)) {
     fprintf(stderr, "PatchRead: Normaldim=%lg; must be either 0 or 3.\n", 
	     normaldim);
     exit(1);
   }


   *patch = PatchCreate((int) degree, range, (int) normaldim);
   patch->ln = LRead;

   if ( dQueryDstructPath(LRead,"BezierTriangle.material") ) {
     if ( !dGetRGBMaterial(LRead, "BezierTriangle", &(patch->material)) ) {
       fprintf(stderr, "PatchRead: Incomplete material field\n");
       exit(1);
     }
   }

   numvertices = (degree + 1)*(degree + 2)/2;
   for (i = 0; i < numvertices; i++)
      {
      for (j = 0; j < 3; j++)
	 {
	 sprintf(buf, "BezierTriangle.net[%d].pos[%d]", i, j);
	 if (!dGetScalar(LRead, buf, &(x[j])))
	    {
	    fprintf(stderr, 
		"PatchRead: Error reading control net; 3 coords expected\n");
	    exit(1);
	    }
	 }

      patch->net[i].position = PCreate(StdFrame(range), x[0], x[1], x[2]);

      if (normaldim == 3.0)
	 {
         for (j = 0; j < 3; j++)
	    {
	    sprintf(buf, "BezierTriangle.net[%d].norm[%d]", i, j);
	    if (!dGetScalar(LRead, buf, &(nx[j])))
	       {
	       fprintf(stderr, 
		  "PatchRead: Error reading control net; 3 coords expected\n");
	       exit(1);
	       }
	    }
	 patch->net[i].normal = VDual(VCreate(StdFrame(range),
						nx[0], nx[1], nx[2]));
	 }
      }

   return 1;
   }

/*
 *----------------------------------------------------------------------
 *  Function: pGetPatch
 *	Read a patch from a dstruct.
 *----------------------------------------------------------------------
 */
int	pGetPatch(ds, patch, range)
Lnode** ds;
Patch	*patch;
Space	range;
{
  char buf[BUFSIZE];
  Scalar numsides, degree, dim, ratdim, normaldim;
  int i, j, numvertices;
  Scalar x[3], nx[3];
  Lnode *Lookup, *LRead=NULL;
  Material mat;
  
  LRead = *ds;

  if ( !dQueryDstructPath( LRead, "BezierTriangle" ) ) {
    fprintf(stderr, "PatchRead: BezierTriangle not read in\n");
    exit(1);
  }
  
  if ( (!dGetScalar( LRead, "BezierTriangle.degree", &degree))
      ||
      (!dGetScalar(LRead,"BezierTriangle.normaldim",&normaldim)) ) {
    fprintf(stderr, "PatchRead: Incomplete patch read\n");
    exit(1);
  }
  
  if ((normaldim != 0.0) && (normaldim != 3.0)) {
    fprintf(stderr, "PatchRead: Normaldim=%lg; must be either 0 or 3.\n", 
	    normaldim);
    exit(1);
  }
  
  
  *patch = PatchCreate((int) degree, range, (int) normaldim);

  dCopyDstructFields(LRead,"BezierTriangle",patch->ln,"BezierTriangle");
  
  if ( dQueryDstructPath(LRead,"BezierTriangle.material") ) {
    if ( !dGetRGBMaterial(LRead, "BezierTriangle", &(patch->material)) ) {
      fprintf(stderr, "PatchRead: Incomplete material field\n");
      exit(1);
    }
  }
  
  numvertices = (degree + 1)*(degree + 2)/2;
  for (i = 0; i < numvertices; i++)       {
    for (j = 0; j < 3; j++)	 {
      sprintf(buf, "BezierTriangle.net[%d].pos[%d]", i, j);
      if (!dGetScalar(LRead, buf, &(x[j])))	    {
	fprintf(stderr, 
		"PatchRead: Error reading control net; 3 coords expected\n");
	exit(1);
      }
    }
    
    patch->net[i].position = PCreate(StdFrame(range), x[0], x[1], x[2]);
    
    if (normaldim == 3.0)	 {
      for (j = 0; j < 3; j++)	    {
	sprintf(buf, "BezierTriangle.net[%d].norm[%d]", i, j);
	if (!dGetScalar(LRead, buf, &(nx[j])))	       {
	  fprintf(stderr, 
		  "PatchRead: Error reading control net; 3 coords expected\n");
	  exit(1);
	}
      }
      patch->net[i].normal = VDual(VCreate(StdFrame(range),
					   nx[0], nx[1], nx[2]));
    }
  }
  
  return 1;
}


/*
** Writes out the patch on stdout.  fp is ignored.
*/

void	PatchWrite(fp, patch)
FILE *fp;
Patch patch;
   {
   int	i, j, numvertices;
   char buf[100];
   Scalar x[3], nx[3];
   int destroyDstruct=0;
   LNODE *ln=NULL;

   dCopyDstructFields(patch.ln,"BezierTriangle",ln,"BezierTriangle");
   dPutScalar(ln, "BezierTriangle.degree", (Scalar) patch.degree);
   dPutScalar(ln, "BezierTriangle.normaldim", (Scalar) patch.normaldim);

   if ( !dQueryDstructPath(ln,"BezierTriangle.material")  &&
        !IsNullMaterial(patch.material) ){
     pSetRGBMaterial(&(ln), "BezierTriangle", &(patch.material));
   }

   numvertices = (patch.degree + 1)*(patch.degree + 2)/2;

   for (i = 0; i < numvertices; i++)
      {
      PCoords(patch.net[i].position, StdFrame(patch.range),
					&(x[0]), &(x[1]), &(x[2]));
      for (j = 0; j < 3; j++)
	 {
	 sprintf(buf, "BezierTriangle.net[%d].pos[%d]", i, j);
	 dPutScalar(ln, buf, x[j]);
	 }

      if (patch.normaldim == 3)
	 {
         NCoords(patch.net[i].normal, StdFrame(patch.range),
					&(nx[0]), &(nx[1]), &(nx[2]));
         for (j = 0; j < 3; j++)
	    {
	    sprintf(buf, "BezierTriangle.net[%d].norm[%d]", i, j);
	    dPutScalar(ln, buf, nx[j]);
	    }
	 }
      }
   dDstructPrint(ln);	/* writes all on stdout */
   pDeleteLnode(&(ln));
   }


/*
** Evaluate the patch at the point whose barycentric coordinates
** are (r, s, t)
*/
Point PatchEval( patch, r, s, t)
Patch patch;
Scalar r, s, t;
{
    Point surfacePoint;
    Normal normal;

    PatchEvalWithNormal( patch, &surfacePoint, &normal, r, s, t);
    return surfacePoint;
}



/*
** Evaluate the patch at the point whose barycentric coordinates are
** (r, s, t). Also computed is the normal vector to the surface at
** the point.
*/
void PatchEvalWithNormal( patch, point, normal, r, s, t)
Patch patch;
Point *point;
Normal *normal;
Scalar r, s, t;
{
    int i;                        /* Misc index */
    Point *tmp;                   /* Temporary net for deCasteljau alg */
    int i1, i2, i3;               /* multi-indices */
    int k;                        /* The step of deCasteljau's alg */

    /* Create the temporary net */
    tmp = (Point *) malloc( sizeof(Point) * NetSize( patch.degree));

    /* Copy the patches net into tmp */
    for (i = 0; i < NetSize( patch.degree); i++) {
      tmp[i] = patch.net[i].position;
    }


    /* Do de Casteljau's algorithm in place on the temporary net, but */
    /* don't do the last step of the algorithm.                       */
    for (k = patch.degree - 1; k > 0; k--) {
	for (i1 = 0; i1 <= k; i1++) {
	    for (i2 = 0; i2 <= k - i1; i2++) {
		i3 = k - i1 - i2;
		tmp[VertexIndex(i1,i2,i3)] = AffineBlend(tmp,i1,i2,i3,r,s,t);
	    }
	}
    }

    /* tmp[0], tmp[1], and tmp[2] span the tangent plane. */
    /* Compute the normal from them...                    */
    *normal = PPPNormal( tmp[0], tmp[2], tmp[1]);

    /* Do the last step of de Casteljau to compute the point on the surface */
    *point  = AffineBlend( tmp, 0, 0, 0, r, s, t);

    /* Free the temporary storage and return*/
    free(tmp);
}


/*
** Split (subdivide) the patch at the point whose barycentric coordinates
** are (r,s,t).  The subpatches are pr, ps, and pt.
*/
Patch PatchSplit( p, pr, ps, pt, r, s, t)
Patch p;
Patch *pr, *ps, *pt;
Scalar r, s, t;
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

Patch PatchDegreeRaise(p)
Patch	p;
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
