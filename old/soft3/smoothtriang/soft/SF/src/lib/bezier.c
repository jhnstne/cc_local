/*
 *----------------------------------------------------------------------
 *  File:  bezier.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "bezier.h"

Point EvalBezier(int n, Point pt[], Scalar t)
{
	static Point* p=NULL;
	static int maxN=0;
	int i,j;

	if ( n > maxN ) {
		if ( p != NULL ) {
			free(p);
		}
		p = (Point*) malloc ( (n+1)*sizeof(Point) );
		maxN = n;
	}

	for (i=0; i<=n; i++) {
		p[i] = pt[i];
	}

	for (i=0; i<n; i++) {
		for (j=0; j<n-i; j++) {
			p[j] = PPac(p[j], p[j+1], (1-t));
		}
	}

	return p[0];
}

Vector EvalBezierDeriv(int n, Point pt[],Scalar t)
{
	static Point* p=NULL;
	static int maxN=0;
	int i,j;

	if ( n > maxN ) {
		if ( p != NULL ) {
			free(p);
		}
		p = (Point*) malloc ( (n+1)*sizeof(Point) );
		maxN = n;
	}

	for (i=0; i<=n; i++) {
		p[i] = pt[i];
	}

	for (i=0; i<n-1; i++) {
		for (j=0; j<n-i; j++) {
			p[j] = PPac(p[j], p[j+1], (1-t));
		}
	}

	return SVMult((Scalar)n, PPDiff(p[1],p[0]));
}

Vector EvalBezierNthDeriv(int n, Point pt[], Scalar t, int deriv)
{
	static Point* p=NULL;
	static int maxN=0;
	static Vector* v=NULL;
	static int maxD=0;
	int i,j;

	if ( deriv < 1 ) {
		fprintf(stderr, "EvalBezierNthDeriv: deriv = %d < 1\n", deriv);
		exit(1);
	}

	if ( deriv > n  ||  n==0 ) {
		return VZero(SpaceOf(pt[0]));
	}

	if ( n > maxN ) {
		if ( p != NULL ) {
			free(p);
		}
		p = (Point*) malloc ( (n+1)*sizeof(Point) );
		maxN = n;
	}
	if ( deriv > maxD ) {
		if ( v != NULL ) {
			free(v);
		}
		v = (Vector*) malloc ( (deriv)*sizeof(Vector) );
		maxD = deriv;
	}

	for (i=0; i<n-deriv; i++) {
		for (j=0; j<n-i; j++) {
			p[j] = PPac(p[j], p[j+1], (1-t));
		}
	}

	for (i=0; i<deriv; i++) {
		v[i] = SVMult((Scalar)n, PPDiff(p[i+1], p[i]));
	}

	for (i=2; i<=deriv; i++) {
		for (j=0; j<deriv-i+1; j++) {
			v[j] = SVMult((Scalar)(n-i+1), VVDiff(v[j+1], v[j]));
		}
	}
	return v[0];
}

Scalar BezierCurvature(int n, Point p[],Scalar t)
{
	Vector fd;
	Vector sd;
	Scalar fx,fy,sx,sy,flen;

	fd = EvalBezierDeriv(n, p, t);
	sd = EvalBezierNthDeriv(n, p, t, 2);
	if ( Dim(SpaceOf(fd)) == 2 ) {		
		VCoords(fd,StdFrame(SpaceOf(fd)),&fx,&fy);
		VCoords(sd,StdFrame(SpaceOf(sd)),&sx,&sy);
		flen = VMag(fd);
		return fabs(( fx*sy - sx*fy ) / (flen*flen*flen));
	} else {
		return VMag(VVCross(fd,sd))/(VMag(fd)*VMag(fd)*VMag(fd));
	}
}


Scalar ScalarBezier(int n, Scalar w[], Scalar t)
{
	static Scalar* sc;
	static int maxN=0;
	int i,j;

	if ( n > maxN ) {
		if ( sc != NULL ) {
			free(sc);
		}
		sc = (Scalar*) malloc ( (n+1)*sizeof(Scalar) );
		maxN = n;
	}

	for (i=0; i<=n; i++) {
		sc[i] = w[i];
	}

	for (i=0; i<n; i++) {
		for (j=0; j<n-i; j++) {
			sc[j] = (1-t)*sc[j] + t*sc[j+1];
		}
	}

	return sc[0];
}

Scalar ScalarBezierDeriv(int n, Scalar w[], Scalar t)
{
	static Scalar* sc;
	static int maxN=0;
	int i,j;

	if ( n > maxN ) {
		if ( sc != NULL ) {
			free(sc);
		}
		sc = (Scalar*) malloc ( (n+1)*sizeof(Scalar) );
		maxN = n;
	}

	for (i=0; i<=n; i++) {
		sc[i] = w[i];
	}

	for (i=0; i<n-1; i++) {
		for (j=0; j<n-i; j++) {
			sc[j] = (1-t)*sc[j] + t*sc[j+1];
		}
	}

	return n*(sc[1]-sc[0]);
}

Point RatBezier(int n, Point p[], Scalar w[], Scalar t)
{
	static Scalar* sc=NULL;
	static Point* pts=NULL;
	static int maxN=0;
	int i,j;

	if ( n > maxN ) {
		if ( sc != NULL ) {
			free(sc);
			free(pts);
		}
		sc = (Scalar*) malloc ( (n+1)*sizeof(Scalar) );
		pts = (Point*) malloc ( (n+1)*sizeof(Point) );
		maxN = n;
	}

	for (i=0; i<=n; i++) {
		sc[i] = w[i];
		pts[i] = p[i];
	}

	for (i=0; i<n; i++) {
		for (j=0; j<n-i; j++) {
			pts[j] = PPac(pts[j], pts[j+1], 
				      (1-t)*sc[j]/((1-t)*sc[j]+t*sc[j+1]));
			sc[j] = (1-t)*sc[j] + t*sc[j+1];
		}
	}

	return pts[0];
}

/*
 *----------------------------------------------------------------------
 *  Function:  RatBezierDeriv
 *	Compute the first derivative of a rational Bezier curve.
 *	Note that some real sleazy-slimey hacking was done to
 *	work around the geometry package.
 *----------------------------------------------------------------------
 */
Vector RatBezierDeriv(int n, Point p[], Scalar w[], Scalar t)
{
	static Point* pts=NULL;
	static int maxN=0;
	Scalar x,y,z;
	int i;
	Frame f;
	Scalar wt, wdt;
	Point pd,xp,wx;
	Vector pdv;

	if ( n > maxN ) {
		if ( pts != NULL ) {
			free(pts);
		}
		pts = (Point*) malloc ( (n+1)*sizeof(Point) );
	}

	f = StdFrame(SpaceOf(p[0]));
	for (i=0; i<=n; i++) {
		PCoords(p[i], f, &x, &y, &z);
		pts[i] = PCreate(f, w[i]*x, w[i]*y, w[i]*z);
	}

	wt = ScalarBezier(n, w, t);
	wdt = ScalarBezierDeriv(n, w, t);
	pdv = EvalBezierDeriv(n, pts, t);
	pd = PVAdd(FOrg(f), pdv);
	xp = RatBezier(n, p, w, t);
				   
	PCoords(xp, f, &x, &y, &z);
	wx = PCreate(f, wdt*x, wdt*y, wdt*z);

	return SVMult(1./wt, PPDiff(pd, wx));
}
