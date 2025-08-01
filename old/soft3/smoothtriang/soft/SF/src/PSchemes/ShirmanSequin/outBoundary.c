/*
 *----------------------------------------------------------------------
 *  File:  outBoundary.c
 *	Output a boundary curve.
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include <math.h>
#include "all.h"

static Space World;

static int ComputePlane(Point p0, Point p1, Point p2, Point p3, Normal* n)
{
	Vector v1;
	Vector v2;

	v1 = PPDiff(p0, p1);
	if ( VMag(v1) < 1e-6 ) {
		v1 = PPDiff(p2, p1);
	}
	v2 = PPDiff(p2, p1);
	if ( VMag(v2) < 1e-6  ||  VMag(VVCross(v1,VNormalize(v2))) < 1e-6 ) {
		v2 = VNormalize(PPDiff(p3, p1));
		/* if all points lie on a line, return 0 */
		if ( VMag(VVCross(v1,v2)) < 1e-6 ) {
			return 0;
		}
	}
	*n = VDual(VNormalize(VVCross(v1,v2)));
	/* if point aren't coplanar, return 0 */
	if ( fabs(NVApply(*n, VNormalize(PPDiff(p3, p1)))) > 1e-6 ) {
		return 0;
	}
	return 1;
}

/*
 *----------------------------------------------------------------------
 *  Function: ComputeNormalSectionCurvature
 *	Compute the normal section curvature in a direction.
 *----------------------------------------------------------------------
 */
static Scalar ComputeNormalSectionCurvature(SFF* sff, Vector dir, 
					    Normal n, Normal plane)
{
  Scalar k;

  k = -1.0 * EvalSFF(sff, VNormalize(dir));
  k = k/VMag(VVCross(VNormalize(NDual(n)),VNormalize(NDual(plane))));

  return k;
}


static void ComputePointCurvature(Vertex* v, Point p, Normal n, Scalar* k)
{
	SFF sff;

	if ( !dGetSFF(v->externalData,"Point.sff", StdFrame(World),&sff)) {
		*k = 0;
	} else {
		*k = ComputeNormalSectionCurvature(&sff, 
				      VNormalize(PPDiff(p, ReturnUDPoint(v))),
				      ReturnUDNormal(v), n);
	}
}

static void ComputeCurvature(Vertex* v0, Point p1, Point p2, Vertex* v3,
			     Scalar* k0, Scalar* k1)
{
	Normal n;

	if ( ComputePlane(ReturnUDPoint(v0), p1, 
			  p2, ReturnUDPoint(v3),&n) == 0) {
		*k0 = *k1 = 0;
		return;
	}
	ComputePointCurvature(v0, p1, n, k0);
	ComputePointCurvature(v3, p2, n, k1);
}


int OutputBoundary(Vertex* v0, Point p1, Point p2, Vertex* v3)
{
	Scalar x,y,z;
	Point p0; Point p3;
	static FILE* fp;
	Scalar k0, k1;

	p0 = ReturnUDPoint(v0);
	p3 = ReturnUDPoint(v3);

	World = SpaceOf(p0);
	
	if ( fp==NULL ) {
		fp = fopen("ss.boundaries.s3d","w");
		if ( fp == NULL ) {
			fprintf(stderr,
				"Can't open file ss.boundaries.s3d\n");
			return 0;
		}
	}
	ComputeCurvature(v0, p1, p2, v3, &k0, &k1);
	fprintf(fp, "# k0 = %g, k1 = %g\n",k0,k1);
	fprintf(fp, "L 4 0 0\n");
	PCoords(p0, StdFrame(World), &x, &y, &z);
	fprintf(fp, "v %g %g %g\n",x,y,z);
	PCoords(p1, StdFrame(World), &x, &y, &z);
	fprintf(fp, "v %g %g %g\n",x,y,z);
	PCoords(p2, StdFrame(World), &x, &y, &z);
	fprintf(fp, "v %g %g %g\n",x,y,z);
	PCoords(p3, StdFrame(World), &x, &y, &z);
	fprintf(fp, "v %g %g %g\n",x,y,z);
	fprintf(fp, "E 0 0 0\n");
	fflush(fp);
	return 1;
}
