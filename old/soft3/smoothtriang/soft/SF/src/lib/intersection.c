/*
 *----------------------------------------------------------------------
 *  File:  intersection.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "intersection.h"

      
/*
 *----------------------------------------------------------------------
 *  Function:  PrintIntersection
 *	Print out an intersection.  Used for debugging.
 *----------------------------------------------------------------------
 */
void PrintIntersection(FILE* fp, Intersection i, Frame WF)
{
  Scalar x,y,z;
  Scalar dx,dy,dz;
  Scalar nx,ny,nz;

  switch(i.type){
  case I_EMPTY:
    fprintf(fp,"intersection is empty.\n");
    break;
  case I_POINT:
    PCoords(i.p, WF, &x,&y,&z);
    fprintf(fp,"intersection is the point (%g,%g,%g)\n",x,y,z);
    break;
  case I_LINE:
    PCoords(i.p, WF, &x,&y,&z);
    VCoords(i.v, WF, &dx,&dy,&dz);
    fprintf(fp,"intersection is the line (%g,%g,%g) + t (%g,%g,%g)\n",
	   x,y,z,dx,dy,dz);
    break;
  case I_PLANE:
    PCoords(i.p, WF, &x,&y,&z);
    NCoords(i.n, WF, &nx,&ny,&nz);
    fprintf(fp,"intersection is the plane through (%g,%g,%g) w/Normal  (%g,%g,%g)\n",
	   x,y,z,nx,ny,nz);
    break;
  default:
    fprintf(fp,"unknown intersection %d.\n",i.type);
    break;
  }
}
#define EPS 0.00001


/*
 *----------------------------------------------------------------------
 *  Function:  IntersectTwoPlanes
 *	Find the intersection between two planes.
 *  There are three cases: the planes don't intersect, they intersect
 *  in a line, or they are the same plane.
 *	The first and third case occur when the cross product of the
 *  normals to the planes is zero.  We can then determine which case
 *  it is by seeing if the vector p1-p2 is perpendicular to the normals.
 *	The case where the intersection is a line itself has two cases.
 *  One case is where the normals are perpendicular, the other when
 *  they are not.
 *----------------------------------------------------------------------
 */
Intersection IntersectTwoPlanes(Point p1, Normal n1, Point p2, Normal n2)
{
  double t;
  Vector v,w;
  Vector n1Xn2;
  Scalar n2Dw;
  Intersection intersection;

  n1 = VDual( VNormalize(NDual(n1)) );
  n2 = VDual( VNormalize(NDual(n2)) );

  n1Xn2 = VVCross(NDual(n1),NDual(n2));
  if ( VMag(n1Xn2) < EPS ){
    if ( fabs(VVDot(PPDiff(p1,p2), NDual(n1))) < EPS ){
      intersection.type = I_PLANE;
      intersection.p = p1;
      intersection.n = n1;
      return intersection;
    } else {
      intersection.type = I_EMPTY;
      return intersection;
    }
  }

  intersection.type = I_LINE;
  intersection.v = n1Xn2;

  w = VVCross(NDual(n1),n1Xn2);
  n2Dw = VVDot(NDual(n2),w);

  if ( fabs(n2Dw) < EPS ){
    intersection.p = PVAdd(p2,SVMult(VVDot(PPDiff(p2,p1),NDual(n2)), 
				    NDual(n2)));
    return intersection;
  } else {
    Scalar t;
    t = VVDot(PPDiff(p2,p1),NDual(n2))/n2Dw;
    intersection.p = PVAdd(p1, SVMult(t,w));
    return intersection;
  }
}


Intersection LineIntersectP(Point p1, Vector v1, Point p2, Normal n2)
{
  double t;
  Intersection i;
  Scalar n2v1;

  n2v1 = NVApply(n2,v1);

  if ( fabs(NVApply(n2,v1)) < EPS ){
    if ( fabs(NVApply(n2,PPDiff(p1,p2))) < EPS ){
      i.type = I_LINE;
      i.p = p1;
      i.v = v1;
      return i;
    } else {
      i.type = I_EMPTY;
      return i;
    }
  }

  i.type = I_POINT;
  t = NVApply(n2,PPDiff(p2,p1)) / NVApply(n2,v1);
  i.p = PVAdd(p1, SVMult(t,v1));
  return i;
}


/* only one of GEO_WAY, ITER_WAY, and NR_WAY should be defined */

#define ITER_WAY


/*
 *----------------------------------------------------------------------
 *  Function:  IntersectThreePlanes
 *	Find the intersection of three planes.
 *----------------------------------------------------------------------
 */
#ifdef GEO_WAY
Intersection IntersectThreePlanes(Point p1, Normal n1, Point p2, Normal n2,
				  Point p3, Normal n3)
{
  Point p;
  Vector v;
  Intersection intersection;

/*
  fprintf(stderr,"volume = %g\n",VVDot(VNormalize(NDual(n1)),
				       VVCross(VNormalize(NDual(n2)),
					       VNormalize(NDual(n3)))));
*/
  intersection = IntersectTwoPlanes(p1,n1,p2,n2,&p,&v);

  switch(intersection.type){
  case I_EMPTY:
    return intersection;
  case I_LINE:
    return LineIntersectP(intersection.p,intersection.v, p3,n3);
  case I_PLANE:
    return IntersectTwoPlanes(intersection.p,intersection.n, p3,n3);
  default:
    fprintf(stderr,"IntersectionThreePlanes: unknown intersection type %d.  Exiting.\n",intersection.type);
    exit(1);
  }
}
#endif
#ifdef ITER_WAY
Intersection IntersectThreePlanes(Point p1, Normal n1, Point p2, Normal n2,
				  Point p3, Normal n3)
{
  Point p;
  Vector v;
  Intersection intersection;
  int i;

  n1 = VDual( VNormalize(NDual(n1)) );
  n2 = VDual( VNormalize(NDual(n2)) );
  n3 = VDual( VNormalize(NDual(n3)) );

  for (i=0;i<1000;i++){
    Point np1,np2,np3;
    Vector v;
    v = PPDiff(p1,PPac(p2,p3,.5));
    np1 = PVAdd(PPac(p2,p3,.5), SVMult(NVApply(n1,v),NDual(n1)));

    v = PPDiff(p2,PPac(p1,p3,.5));
    np2 = PVAdd(PPac(p1,p3,.5), SVMult(NVApply(n2,v),NDual(n2)));

    v = PPDiff(p3,PPac(p2,p1,.5));
    np3 = PVAdd(PPac(p2,p1,.5), SVMult(NVApply(n3,v),NDual(n3)));

    p1 = np1;  p2 = np2;  p3 = np3;
  }
  intersection.type = I_POINT;
  intersection.p = PPac3( p1,p2,p3, 1./3., 1./3., 1.-1./3.-1./3. );
  return intersection;
}
#endif
#ifdef NR_WAY
/*
	started 04/24/91
	test of curve fitting using adaptive manifold
*/

#include "recipes.h"
#include "recipesutil.h"


/* SVD decompostion of u, backsbustitution on each coordinate */
/* o=u*w*vt */
static svdsol(o, b, x)
float **o;
float *b;
float *x;
{
fprintf(stderr,"svdsol not implemented!  Core dumping...\n");
*(char*)0 = 0;
#if 0
	int i,j,c;
	static float **u,*w,**v;
	static int first=1;
	float wmax, wmin;

	/* create temporary version of u */
	if ( first ) {
		u = matrix(1,3, 1,3);
		w = vector(1,3);
		v = matrix(1,3, 1,3);
		first = 0;
	}

	for (i=1; i<=3; i++) {
		for(j=1; j<=3; j++) {
			u[i][j] = o[i][j];
		}
	}

	/* u[1..3][1..3] -> u[1..3][1..3], w[1..3], v[1..3][1..3] */
	svdcmp(u,3,3,w,v);

	/* Examine scalars in w for singular values */
	wmax = 0.;
	for (i=1; i<=3; i++) {
		if ( !(wmax > w[i]) ) {
			wmax = w[i];
		}
		if ( w[i] < 0 ) {
			fprintf(stderr,"svdsol: warning!  w[%d] < 0\n",i);
		}
	}
	wmin = wmax * 1e-6;
	if(0) {
		static float minmax=10.;
		if(!(minmax<wmax)){
			minmax=wmax;
			fprintf(stderr,"wmax = %g\n",wmax);
		}
        }
	for (i=1; i<=3; i++) {
		if(0) {
			static float min=10.;
			if(!(min<w[i])){
				min=w[i];
				fprintf(stderr,"min = %g\n",min);
			}
		}
		if ( w[i] < wmin ) {
			/*fprintf(stderr,"zeroing %g\n",w[i]);*/
			w[i] = 0.;
		}
	}


	for (c=1; c<=2; c++) {
		/* u,w,v, b[1..3] -> x[1..3] */
		svbksb(u,w,v,3,3,b,x);
	}
#endif
}

Intersection IntersectThreePlanesWF(Point p1, Normal n1, Point p2, Normal n2,
				    Point p3, Normal n3, Point C)
{
fprintf(stderr,"IntersectThreePlanesWF not implemented!  Core dumping...\n");
*(char*)0 = 0;
#if 0
	static float** A;
	static float* B;
	static float* X;
	static int first=1;
	Frame f;
	Vector x,y,z;
	Scalar nx,ny,nz;
	Scalar px,py,pz;
	Intersection intersection;

	/* create a frame centered at the fall back point */
	x = VNormalize( PPDiff(p2, p1) );
	z = VVCross(x, PPDiff(p3,p1));
	if ( VMag(z) > 1e-4 ) {
		z = VNormalize(z);
	} else {
		if ( VVDot( x, FV(StdFrame(SpaceOf(p1)), 0) ) < 0.8 ) {
			z = VNormalize(VVCross(x, 
					       FV(StdFrame(SpaceOf(p1)), 0)));
		} else if ( VVDot( x, FV(StdFrame(SpaceOf(p1)), 1) ) < 0.8 ) {
			z = VNormalize(VVCross(x, 
					       FV(StdFrame(SpaceOf(p1)), 1)));
		} else if ( VVDot( x, FV(StdFrame(SpaceOf(p1)), 2) ) < 0.8 ) {
			z = VNormalize(VVCross(x, 
					       FV(StdFrame(SpaceOf(p1)), 2)));
		} else {
			fprintf(stderr,"IntersectThreePlanesWF: bad frame\n");
			exit(1);
		}
	}
	y = VVCross(z, x);
	f = FCreate("IntersectThreePlanes frame", C, x, y, z);

	if ( first ) {
		A = matrix(1,3, 1,3);
		B = vector(1,3);
		X = vector(1,3);
		first = 0;
	}

	NCoords(n1, f, &nx, &ny, &nz);
	A[1][1] = nx;	A[1][2] = ny;	A[1][3] = nz;
	PCoords(p1, f, &px, &py, &pz);
	B[1] = px*nx + py*ny + pz*nz;
	
	NCoords(n2, f, &nx, &ny, &nz);
	A[2][1] = nx;	A[2][2] = ny;	A[2][3] = nz;
	PCoords(p2, f, &px, &py, &pz);
	B[2] = px*nx + py*ny + pz*nz;
	
	NCoords(n3, f, &nx, &ny, &nz);
	A[3][1] = nx;	A[3][2] = ny;	A[3][3] = nz;
	PCoords(p3, f, &px, &py, &pz);
	B[3] = px*nx + py*ny + pz*nz;

	svdsol(A, B, X);

	intersection.type = I_POINT;
	intersection.p = PCreate(f, X[1], X[2], X[3]);
	return intersection;
#endif
}

Intersection IntersectThreePlanes(Point p1, Normal n1, Point p2, Normal n2,
				  Point p3, Normal n3)
{
	Point C;

	C = PPac3(p1, p2, p3, 1./3., 1./3., 1./3.);
	return IntersectThreePlanesWF(p1,n1,p2,n2,p3,n3,C);
}

#endif
