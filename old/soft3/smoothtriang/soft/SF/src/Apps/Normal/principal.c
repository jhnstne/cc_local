/*
 *----------------------------------------------------------------------
 *  File:  principle.c
 *	Modified from Hugues Hoppe's code.
 *----------------------------------------------------------------------
 */
/* Copyright (c) 1991 Hugues H. Hoppe */
#include <stdio.h>
#include "all.h"

extern float* vector();
extern float** matrix();
static int secondOrder=1;
static int interP=1;

/* given n points pts,
   compute eigentransform eit, and redundant magnitudes eimag
   */
PrincipalComponents(pts, n, eit)
Point *pts;
int n;
Frame *eit;
{
	float covar[3][3];
	float avg[3];
	float f;
	Point avgp;
	int i,j, k, nrot;
	float** mcovar;
	float* d;
	float** v;
	Scalar coords[3];
	Vector vec[3];

	for (i=0; i<3; i++) {
		avg[i] = 0.;
	}
	for (k=0; k<n; k++) {
		PCoords(pts[k], StdFrame(SpaceOf(pts[k])),
			&coords[0], &coords[1], &coords[2]);
		for (i=0; i<3; i++) {
			avg[i] += coords[i];
		}
	}
	for (i=0; i<3; i++) {
		avg[i] /= n;
	}

	avgp = PCreate(StdFrame(SpaceOf(pts[0])), avg[0], avg[1], avg[2]);
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			covar[i][j] = 0.;
		}
	}
	for (k=0; k<n; k++) {
		PCoords(pts[k], StdFrame(SpaceOf(pts[k])),
			&coords[0], &coords[1], &coords[2]);
		for (i=0; i<3; i++) {
			for (j=0; j<3; j++) {
				covar[i][j] += (coords[i]-avg[i]) *
				  (coords[j]-avg[j])/n;
			}
		}
	}
	mcovar = matrix(1,3, 1,3);
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) { 
			mcovar[i+1][j+1] = covar[i][j];
		}
	}
	d = vector(1,3);
	v = matrix(1,3, 1,3);
	jacobi(mcovar, 3, d, v, &nrot);
	eigsrt(d, v, 3);

	for (i=1; i<=3; i++) {
		f = sqrt(d[i]);
		if (f==0.) {
			f = 1e-15; /* very small but non-zero vector */
		}
		vec[i-1] = VCreate(StdFrame(SpaceOf(pts[0])),
				   v[1][i]*f, v[2][i]*f, v[3][i]*f);
	}
	free_matrix(mcovar, 1,3, 1,3);
	free_vector(d, 1,3);
	free_matrix(v, 1,3, 1,3);
	vec[0] = VNormalize(vec[0]);
	vec[1] = VNormalize(vec[1]);
	vec[2] = VNormalize(VVCross(vec[0], vec[1]));
	*eit = FCreate(avgp, vec[0], vec[1], vec[2]);
}

#define MAX_VERT 20

void qfun(x, r, n)
float x[2];
float r[6];
int n;
{
	int i;

	if ( n != 6  &&  n != 3 ) {
		fprintf(stderr,"qfun: n == %d != 6 | 3\n",n);
		exit(1);
	}
	if ( n == 6 ) {
		r[1] = 1;
		r[2] = x[1];
		r[3] = x[2];
		r[4] = x[1]*x[2];
		r[5] = x[1]*x[1];
		r[6] = x[2]*x[2];
	} else {
		r[1] = x[1]*x[2];
		r[2] = x[1]*x[1];
		r[3] = x[2]*x[2];
	}
#if 0
fprintf(stderr,"%g %g -- ",x[0],x[1]);
for(i=1;i<=6;i++)fprintf(stderr,"%g ",r[i]);
fprintf(stderr,"\n");
#endif
}


SFF LstSq(p, n, f, up)
Point p[];
int n;
Frame f;
Vector up;
{
	Scalar xc,yc,zc;
	float** x;
	float* y;
	float* sig;
	float* a;
	float** u;
	float** v;
	float* w;
	float chisq;
	int lista[10];
	int mfit;
	float** covar;
	Vector vn;
	int i;
	SFF sff;

	x = matrix(1,n, 1,2);
	y = vector(1,n);
	sig = vector(1,n);
	a = vector(1,6);
	u = matrix(1,n,1,6);
	v = matrix(1,6, 1,6);
	w = vector(1,6);

	covar = matrix(1,6, 1,6);

	for (i=0; i<n; i++) {
		PCoords(p[i], f, &xc, &yc, &zc);
		x[i+1][1] = xc;
		x[i+1][2] = yc;
		y[i+1]    = zc;
		sig[i+1] = 1;
	}
	if ( interP ) {
		a[1] = 0.;
		for (i=1; i<=5; i++) {
			lista[i] = i+1;
		}
	} else {
		for (i=0; i<=6; i++) {
			lista[i] = i;
		}
	}
#if 1
#if 0
	svdfit(x, y, sig, n, a, 6, u, v, w, &chisq, qfun);
#else
	a[1] = a[2] = a[3] = 0.;
	svdfit(x, y, sig, n, a+3, 3, u, v, w, &chisq, qfun);
#endif
#else
	if ( interP ) {
		lfit(x, y, sig, n, a, 6, lista, 5, covar, &chisq, qfun);
	} else {
		lfit(x, y, sig, n, a, 6, lista, 6, covar, &chisq, qfun);
	}
#endif

	sff.v0 = VCreate(f, 1., 0., a[1]);
	sff.v1 = VCreate(f, 0., 1., a[2]);
	vn = VNormalize(VVCross(sff.v0, sff.v1));
	if ( VVDot(vn, up) < 0. ) {
		vn = SVMult(-1., vn);
	}
	sff.m[0][0] = VVDot(vn, VCreate(f, 1., 0., 2*a[5]));
	sff.m[0][1] = sff.m[1][0] = VVDot(vn, VCreate(f, 0., 0., a[4]));
	sff.m[1][1] = VVDot(vn, VCreate(f, 0., 1., 2*a[6]));

	free_matrix(x, 1,n, 1,2);
	free_vector(y, 1,n);
	free_vector(sig, 1,n);
	free_vector(a, 1,6);
	free_matrix(u, 1,n, 1,6);
	free_matrix(v, 1,6, 1,6);
	free_vector(w, 1,6);
	free_matrix(covar, 1,6, 1,6);

	return sff;
}


normalFunction(v)
Vertex* v;
{
	Point p[MAX_VERT];
	int i;
	Frame f;
	Vector vn;
	int numV;
	Vertex* v2;
	Vector up;
	Vertex* va[2];

	p[0] = ReturnUDPoint(v);
	i = 1;
	ForeachVertexVertex(v, v2) {
		if ( i >= MAX_VERT ) {
			fprintf(stderr,"normalFunc: vertex %s degree > %d\n",
				ReturnName(v), MAX_VERT);
			exit(1);
		}
		p[i] = ReturnUDPoint(v2);
		i++;
	} EndForeach;
	numV = i;

	if ( numV > 1 ) {
		PrincipalComponents(p+1, numV-1, &f);
		f = FCreate(p[0], FV(f, 0), FV(f, 1), FV(f, 2));
		vn = VNormalize(VVCross(FV(f, 0), FV(f, 1)));
		
		i = 0;
		ForeachVertexVertex(v, v2) {
			va[i] = v2;
			i++;
			if ( i == 2 ) {
				break;
			}
		} EndForeach;
		up = VVCross(PPDiff(ReturnUDPoint(va[0]), ReturnUDPoint(v)),
			     PPDiff(ReturnUDPoint(va[1]), ReturnUDPoint(v)));
		if ( secondOrder ) {
			SFF sff;

			sff = LstSq(p, numV, f, up);
			pPutSFF(&(v->externalData), "Point.sff", &sff);
			vn = VNormalize(VVCross(sff.v0, sff.v1));
		}
		if ( VVDot(up, vn) < 0. ) {
			vn = SVMult(-1., vn);
		}

	} else {
		fprintf(stderr,"Warning: vertex %s has no neighbors.\n");
		vn = VZero(SpaceOf(ReturnUDPoint(v)));
	}
	SetUDNormal(v, VDual(vn));
}
