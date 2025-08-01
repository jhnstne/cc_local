/*
 *----------------------------------------------------------------------
 *  File:  spatch.c
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include <math.h>
#include "all.h"
#include "mindex.h"
#include "spatch.h"

#define Z(A) (fabs(A)<1e-9?0.:(A))

static int pie;

static double dp[100][2];
static ddim=0;

extern int boundary;

static void buildDomain(int s)
{
	int i;

	if ( ddim == s ) {
		return;
	}
	for (i=0; i<s; i++) {
		dp[i][0] = cos(2.*M_PI*i/s);
		dp[i][1] = sin(2.*M_PI*i/s);
	}
	ddim = s;
}

static Point eval(Spatch* s, Scalar b[3])
{
	double p[3];
	buildDomain(s->s);

	/* the domain is dp[i], dp[i+1], and (0,0) */
	p[0] = b[0]*dp[pie][0] + b[1]*dp[(pie+1)%s->s][0];
	p[1] = b[0]*dp[pie][1] + b[1]*dp[(pie+1)%s->s][1];

	return EvalSpatch(*s, p);
}

static void G3dMoveTo(Point p)
{
	Scalar x,y,z;

	PCoords(p, StdFrame(SpaceOf(p)), &x,&y,&z);
	fprintf(stdout, "%g %g %g 0\n",x,y,z);
}

static void G3dDrawTo(Point p)
{
	Scalar x,y,z;

	PCoords(p, StdFrame(SpaceOf(p)), &x,&y,&z);
	fprintf(stdout, "%g %g %g 1\n",x,y,z);
}

extern Scalar offsetDist;

static outputG3dBoundary(Spatch sp, int samples, Frame f)
{
	int i,j;
	Point p;
	double d[2];
	extern int split;
	Normal n;
	SFF sff;

	buildDomain(sp.s);

	for (i=0; i<sp.s; i++) {
		for (j=0; j<=samples; j++) {
			d[0] = (samples-j)*dp[i][0] + j*dp[(i+1)%sp.s][0];
			d[1] = (samples-j)*dp[i][1] + j*dp[(i+1)%sp.s][1];
			d[0] /= samples;	d[1] /= samples;
			if ( offsetDist > 0. ) {
				EvalSpatchWNormalSFF(sp, d, &p, &n, &sff, 0);
				p = PVAdd(p, SVMult(offsetDist, 
						    VNormalize(NDual(n))));
			} else {
				p = EvalSpatch(sp, d);
			}
			if ( i==0 && j==0 ) {
				G3dMoveTo(p);
			} else {
				G3dDrawTo(p);
			}
		}
	}

	if ( split ) {
		for (i=0; i<sp.s; i++) {
			for (j=0; j<=samples; j++) {
				d[0] = (samples-j)*dp[i][0];
				d[1] = (samples-j)*dp[i][1];
				d[0] /= samples;	d[1] /= samples;
				if ( offsetDist > 0. ) {
					EvalSpatchWNormalSFF(sp, d, &p, &n, 
							     &sff, 0);
					p = PVAdd(p, 
						  SVMult(offsetDist, 
							 VNormalize(NDual(n))));
				} else {
					p = EvalSpatch(sp, d);
				}
				if ( j==0 ) {
					G3dMoveTo(p);
				} else {
					G3dDrawTo(p);
				}
			}
		}
	}
}

fOT(FILE* fp, Point p, Point q, Point r, Normal np, Normal nq, Normal nr,
    SFF* sp, SFF* sq, SFF* sr)
{
	extern int DoGaussian;
        PutVertexPosition("Triangle.vertex1", p);
        PutVertexPosition("Triangle.vertex2", q);
        PutVertexPosition("Triangle.vertex3", r);
        PutVertexNormal("Triangle.vertex1", np);
        PutVertexNormal("Triangle.vertex2", nq);
        PutVertexNormal("Triangle.vertex3", nr);
	if ( DoGaussian ) {
		PutScalar("Triangle.vertex1.curvature.gaussian",
			  SFFGaussianCurvature(sp));
		PutScalar("Triangle.vertex2.curvature.gaussian",
			  SFFGaussianCurvature(sq));
		PutScalar("Triangle.vertex3.curvature.gaussian",
			  SFFGaussianCurvature(sr));

		PutScalar("Triangle.vertex1.curvature.mean",
			  SFFMeanCurvature(sp));
		PutScalar("Triangle.vertex2.curvature.mean",
			  SFFMeanCurvature(sq));
		PutScalar("Triangle.vertex3.curvature.mean",
			  SFFMeanCurvature(sr));
	}
	if ( QueryDstructPath("Spatch.material") ) {
		CopyDstructFields("Spatch.material", "Triangle.material");
	}
        FlushDstruct();
}


SpatchTessellate(Spatch sp, int samples, Frame f)
{
	extern int DoGaussian;
	extern int g3d;

	if ( boundary ) {
		outputG3dBoundary(sp, samples, f);
		return;
	}

	if ( samples <= 0 ) {
		if ( g3d ) {
			fTessSpatchNet(stdout, sp);
		}
		return;
	}

	fprintf(stderr,"s=%d, d=%d\n",sp.s,sp.d);
	if ( !g3d ) {
		SetSpatchOutTri(fOT);
	}
	fTessSpatch(stdout, sp, samples, DoGaussian);
}
