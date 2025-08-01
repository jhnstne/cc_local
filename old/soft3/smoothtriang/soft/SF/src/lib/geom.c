/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: 03/12/89 at 13:26:37
** Purpose: Random Utilities.
**
*/
#include <stdio.h>
#include <math.h>
#include "all.h"

Scalar GaussianCurvature(Vector d2u, Vector d2uv, Vector d2v, 
			 Vector du, Vector dv, Normal n)
{
	Scalar L,M,N,E,F,G;

	L = NVApply(n, d2u);
	M = NVApply(n, d2uv);
	N = NVApply(n, d2v);
	E = VVDot(du, du);
	F = VVDot(du, dv);
	G = VVDot(dv, dv);

	return (L*N - M*M)/(E*G - F*F);
}


/*
** Return the point of intersection between the line determined by
** p1 and v1 with the plane determined by p2 and n2.  
** No error checking is done (ie, if n2 is perpendicular to v1,
** then an error should (and will :-) occur).
*/
Point LineIntersectPlane(Point p1, Vector v1, Point p2, Normal n2)
{
  double t;

  t = NVApply(n2,PPDiff(p2,p1)) / NVApply(n2,v1);
  return PVAdd(p1, SVMult(t,v1));
}


/*
 *----------------------------------------------------------------------
 *  Function:  LineIntersectLine
 *	Find the intersection of two three space lines.  
 *  Parameters:
 *	Point p1, Vector v1  - the first line (passing through p1,
 *					       in direction v1)
 *	Point p2, Vector v2  - the second line
 *	Point* pr	     - the intersection of the two lines
 *  Return Value:
 *	BOOLEAN		     - TRUE if intersection exists, FALSE otherwise
 *  Details:
 *	Our lines are  p1 + t1*v1  and  p2 + t2*v2.
 *  They will intersect when  p1 + t1*v1 = p2 + t2*v2.
 *  Rearranging, we see that  t1*v1 - t2*v2 = p2-p1.
 *  If we write  p2-p1  in terms of a frame with direction v1 and v2,
 *  then the v1 coefficient can be used to find the intersection point.
 *	Two checks are made to make sure the intersection:
 *  First, we make sure v1 and v2 aren't parallel.
 *  Second, when extracting coords in our v1, v2 frame, we make sure
 *  that there isn't a third component of the vector  p2-p1.
 *----------------------------------------------------------------------
 */
BOOLEAN LineIntersectLine(Point p1, Vector v1,Point p2, Vector v2,
			  Point* pr)
{
  Vector cross;
  Frame f;
  Scalar t1,t2,t3;

  cross = VVCross(v1,v2);

  /* if vectors are parallel, then no intersection or infinte intersections */
  if ( VMag( cross ) < 0.000000001 ){
/*fprintf(stderr,"vectors are parallel\n");*/
    return FALSE;
  }

  /* create a frame based on v1 and v2.  We don't actually care where
     the origin is.  */
  f = FCreate( "temp", p1, v1, v2, VVCross(v1,v2) );

  /* check to see if lines were in same plane */
  VCoords( PPDiff(p2,p1), f, &t1, &t2, &t3 );
  if ( fabs(t3) > 0.000000001 ){
/*fprintf(stderr,"lines not coplanar.  t3 = %g\n",t3);*/
    return FALSE;
  }

  *pr = PVAdd(p1, SVMult( t1, v1 ) );
  return TRUE;
}


/*
 *----------------------------------------------------------------------
 *  Function:  ComputePlanarTangents
 *	Compute tangent vectors that point lie in a common plane with
 *  both specified points and each other, and lie in the given tangent
 *  planes.
 *	Vectors should both be "oriented" from p0 to p1.
 *----------------------------------------------------------------------
 */
void ComputePlanarTangents(Point p0, Normal n0,Point p1, Normal n1,
		      Vector* v0, Vector* v1,Vector* plane)
{
  Vector u1;	/* the direction from p0 to p1 */
  Vector u3;	/* the plane containing the two points and the
		   average of the two normals */
  u1 = PPDiff(p1, p0);
  u1 = VNormalize(u1);
  u3 = VVCross(u1, VVAdd(NDual(n0), NDual(n1)));
  u3 = VNormalize(u3);
  *plane = u3;

  *v0 = VVCross(NDual(n0), u3);
  *v0 = VNormalize(*v0);
  *v1 = VVCross(NDual(n1), u3);
  *v1 = VNormalize(*v1);
  
								    
  /* we may need to flip the vectors */
  if (VVDot(VVCross(NDual(n1), u3), u1) < 0.0) {
    *v0 = SVMult(-1.0, *v0);
    *v1 = SVMult(-1.0, *v1);
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  ConstructQuadratic
 *      Construct the quadratic precision point of a cubic patch.
 *  Parameters:
 *	Patch* p	- a cubic Bezier patch with all control points
 *			  assigned, except possibly the center point.
 *  Return Value:
 *	Point		- A point which if used as the center point of
 *			  the cubic would yield a quadratic patch if
 *			  the boundaries are degree raised quadratics.
 *----------------------------------------------------------------------
 */
Point ConstructQuadratic(Patch* p)
{
  Point P[9];
  Scalar w[9];

  P[0] = PatchGetPoint(*p,0,1,2);
  P[1] = PatchGetPoint(*p,0,2,1);
  P[2] = PatchGetPoint(*p,1,0,2);
  P[3] = PatchGetPoint(*p,1,2,0);
  P[4] = PatchGetPoint(*p,2,0,1);
  P[5] = PatchGetPoint(*p,2,1,0);
  P[6] = PatchGetPoint(*p,3,0,0);
  P[7] = PatchGetPoint(*p,0,3,0);
  P[8] = PatchGetPoint(*p,0,0,3);

  w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
  w[6] = w[7] = w[8] = -(1.0/6.0);
  return PPacN( 9, P, w);
}


#define DEFAULT_ITERS 1000
#define DEFAULT_EPS 1e-6

#define projPointOnLine(p,lp,lv)\
	PVAdd((lp), SVMult(VVDot(PPDiff((p),(lp)),(lv)),(lv)))


int LineIterLine(Point p1, Vector v1, Point p2, Vector v2, int maxIters, 
		 double eps, Point* rp1, Point* rp2, Point* midp)
{
  int i;
  Point p1p,p2p;
  Point m,pm,pm2,pm3;

  if (maxIters<0) {
    maxIters = DEFAULT_ITERS;
  }
  if ( eps <= 0. ) {
    eps = DEFAULT_EPS;
  }

  v1 = VNormalize(v1);
  v2 = VNormalize(v2);

  m = PPac(p1,p2,0.5);

  for(i=0;i<maxIters;i++){
    pm3 = pm2;
    pm2 = pm;
    pm = m;
    p2p = projPointOnLine(p1,p2,v2);
    p1p = projPointOnLine(p2,p1,v1);
    m = PPac(p1p,p2p,0.5);
    if ( PPDist(m,pm) < eps ) {
      *midp = m;
      *rp1 = p1p;
      *rp2 = p2p;

      return i+1;
    }

    if ( i >= 2 ) {
      if ( (PPDist(m, pm2) < eps  &&  PPDist(pm,pm3) < eps)  ||
	    (0&&PPDist(PPac(m,pm,0.5),PPac(pm2,pm3,0.5)) < eps) ) {
	*midp = PPac(m,pm,0.5);

	return -(i+1);
      }
    }
    p1 = p1p;
    p2 = p2p;
  }

#if 0
  {
      Scalar x,y,z;
      Frame f;

      f = StdFrame(SpaceOf(m));
      PCoords(m, f, &x, &y, &z);
      fprintf(stderr,"FAILED: (%17.17f %17.17f),",x,y);
      PCoords(pm, f, &x, &y, &z);
      fprintf(stderr,"(%17.17f %17.17f)\n",x,y);
      PCoords(pm2, f, &x, &y, &z);
      fprintf(stderr,"        (%17.17f %17.17f),",x,y);
      PCoords(pm3, f, &x, &y, &z);
      fprintf(stderr,"(%17.17f %17.17f)\n",x,y);
    }
#endif
  *rp1 = p1;
  *rp2 = p2;
  return i;
}


/*
 *----------------------------------------------------------------------
 *  Function:  LineIntLine
 *	find the intersection of two three space lines.
 *  Parameters:
 *	Point p1, Vector v1  - the first line (passing through p1,
 *					       in direction v1)
 *	Point p2, Vector v2  - the second line
 *	Point* pr	     - the intersection of the two lines
 *  Return Value:
 *	BOOLEAN		     - TRUE if intersection exists, FALSE otherwise
 *  Details:
 *	Our lines are  p1 + t1*v1  and  p2 + t2*v2.
 *	The intersection will be performed by calling LineIterLine().
 *	If the two points are close, then we'll their average the
 *	intersection.
 *----------------------------------------------------------------------
 */
BOOLEAN LineIntLine(Point p1, Vector v1, Point p2, Vector v2, Point* pr)
{
  Point rp1;
  Point rp2;
  Point midp;
  double d;

  if ( LineIterLine(p1,v1,p2,v2,-1,-1.,&rp1,&rp2,pr) < DEFAULT_ITERS ) {
    return TRUE;
  } else {
    return FALSE;
  }
}


Point projPointOnPlane(Point p1, Point p2, Normal n2)
{
  Vector v;
  Vector nv;

  nv = NDual(n2);
  if ( fabs(1.-VVDot(nv,nv)) > 0.00001 ) {
    nv = VNormalize(nv);
  }

  v = PPDiff(p1,p2);
  return PVAdd(p2,VVDiff(v,SVMult(VVDot(nv,v), nv)));
}

int LineIterPlane(Point p1, Vector v1, Point p2, Normal n2, 
		  int maxIters, double eps, Point* rp1, Point* rp2, 
		  Point* midp)
{
  int i;
  Point p1p,p2p;
  Point m,pm,pm2,pm3;

  if (maxIters<0) {
    maxIters = DEFAULT_ITERS;
  }
  if ( eps <= 0. ) {
    eps = DEFAULT_EPS;
  }

  v1 = VNormalize(v1);
  n2 = VDual(VNormalize(NDual(n2)));

  m = PPac(p1,p2,0.5);

  for(i=0;i<maxIters;i++){
    pm3 = pm2;
    pm2 = pm;
    pm = m;
    p2p = projPointOnPlane(p1,p2,n2);
    p1p = projPointOnLine(p2,p1,v1);
    m = PPac(p1p,p2p,0.5);
    if ( PPDist(m,pm) < eps ) {
      *midp = m;
      *rp1 = p1p;
      *rp2 = p2p;

      return i+1;
    }

    if ( i >= 2 ) {
      if ( (PPDist(m, pm2) < eps  &&  PPDist(pm,pm3) < eps)  ||
	    (0&&PPDist(PPac(m,pm,0.5),PPac(pm2,pm3,0.5)) < eps) ) {
	*midp = PPac(m,pm,0.5);

	return -(i+1);
      }
    }
    p1 = p1p;
    p2 = p2p;
  }

  *midp = m;
  *rp1 = p1;
  *rp2 = p2;
  return i;
}

BOOLEAN LineIntPlane(Point p1, Vector v1, Point p2, Normal n2, Point* pr)
{
  Point rp1;
  Point rp2;
  Point midp;
  double d;

  if ( LineIterPlane(p1,v1,p2,n2,-1,-1.,&rp1,&rp2,pr) < DEFAULT_ITERS ) {
    return TRUE;
  } else {
    return FALSE;
  }
}

#if 1
int Iter3Planes(Point p1, Normal n1, Point p2, Normal n2, 
		Point p3, Normal n3, int maxIters, double eps, Point* midp)
{
  int i;
  Point p1p,p2p,p3p;
  Point m,pm;
  Point mid1,mid2,mid3;

  if (maxIters<0) {
    maxIters = DEFAULT_ITERS;
  }
  if ( eps <= 0. ) {
    eps = DEFAULT_EPS;
  }

  n1 = VDual(VNormalize(NDual(n1)));
  n2 = VDual(VNormalize(NDual(n2)));
  n3 = VDual(VNormalize(NDual(n3)));

  m = PPac3(p1,p2,p3,1./3.,1./3.,1./3.);

  for(i=0;i<maxIters;i++){
    pm = m;

    mid1 = PPac(p2,p3,0.5);
    mid2 = PPac(p3,p1,0.5);
    mid3 = PPac(p1,p2,0.5);

    p1p = projPointOnPlane(mid1,p1,n1);
    p2p = projPointOnPlane(mid2,p2,n2);
    p3p = projPointOnPlane(mid3,p3,n3);

    m = PPac3(p1p,p2p,p3p,1./3.,1./3.,1./3.);

    if ( PPDist(m,pm) < eps ) {
      *midp = m;
      return i+1;
    }

    p1 = p1p;
    p2 = p2p;
    p3 = p3p;
  }

  *midp = m;
  return i;
}

/*
 *----------------------------------------------------------------------
 *  Function:  Int3Planes
 *	find the intersection of three planes.
 *  Parameters:
 *	Point p1, Normal n1  - The first plane
 *	Point p2, Normal n2  - The second plane
 *	Point p3, Normal n3  - The third plane
 *	Point* pr	     - the intersection of the three planes.
 *  Return Value:
 *	BOOLEAN		     - TRUE if intersection exists, FALSE otherwise
 *  Details:
 *	The intersection will be performed by calling Iter3Planes().
 *	If the iterations didn't converge, we'll return false and
 *	pr will contain the "best guess."
 *----------------------------------------------------------------------
 */
BOOLEAN Int3Planes(Point p1, Normal n1, Point p2, Normal n2, 
		   Point p3, Normal n3, Point* pr)
{
  Point rp1;
  Point rp2;
  Point midp;
  double d;

  if ( Iter3Planes(p1,n1,p2,n2,p3,n3,-1,-1.,pr) < DEFAULT_ITERS ) {
    return TRUE;
  } else {
    return FALSE;
  }
}

#endif


/*
 *----------------------------------------------------------------------
 *  Function:  CircArcLen
 *	Find the length of the circular arc interpolating 2 points and
 *	one vector
 *----------------------------------------------------------------------
 */
Scalar CircArcLen(Point p1, Vector v1, Point p2)
{
	Scalar length;
	Vector v2;
	Scalar theta, sintheta;

	length = PPDist(p1,p2);
	if ( length < 1e-12 ) return length;
	v1 = VNormalize(v1);
	v2 = SVMult(1/length, PPDiff(p2,p1));
	sintheta = VMag(VVCross(v1,v2));
	if ( sintheta < 2e-08 ) {
		return 1;
	} else {
		theta = asin(sintheta);
		return length*theta/sintheta;
	}
}
