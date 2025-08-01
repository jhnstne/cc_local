/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 ** File: tptess.c
 ** Author: Steve Mann
 **
 */
#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "sff.h"
#include "tpbezier.h"


#define MAX_DATA 5000

static double EPS = 1e-7;

void SetTPTessEps(double e)
{
	EPS = e;
}

#define SQ(A) ((A)*(A))

#if 0

void TessTPPatch(void* patch, Point (*eval)(), int samples, Frame frame)
{
	Scalar b[3];
	int i;
	int j;
	Patch SamplePatch;
	Point p;
	Normal n;
	
	SamplePatch = PatchCreate( samples, SpaceOf(frame), 3 );
	
	for(i=0;i<=samples;i++){
		for(j=0;j<=samples-i;j++){
			b[0] = (double)i/(double)samples;
			b[1] = (double)j/(double)samples;
			b[2] = 1. - b[0] - b[1];
			p = eval(patch,b);
			n = CalcNormal(patch,eval,i,j,samples-i-j);
			PatchSetPoint(&SamplePatch, p, i, j, samples-i-j );
			PatchSetNormal(&SamplePatch, n, i, j, samples-i-j );
		}
	}
	OutputPatchNet2( stdout, SamplePatch, frame, NULL );
	PatchFree( SamplePatch );
}




static Scalar E(void* patch, Point (*eval)(), Scalar b[3])
{
	Point p1,p2;
	Vector v;
	
	Scalar beps[3];
	
	beps[0] = b[0] + EPS;
	beps[1] = b[1] - EPS;
	beps[2] = b[2];
	p2 = eval(patch,beps);
	beps[0] = b[0] - EPS;
	beps[1] = b[1] + EPS;
	p1 = eval(patch,beps);
	v = PPDiff(p1,p2);
	return VVDot(v,v)/SQ(2.*EPS);
	
}

static Scalar F(void* patch, Point (*eval)(), Scalar b[3])
{
	Point p1,p2;
	Vector v,w;
	
	Scalar beps[3];
	
	beps[0] = b[0] + EPS;
	beps[1] = b[1] - EPS;
	beps[2] = b[2];
	p2 = eval(patch,beps);
	beps[0] = b[0] - EPS;
	beps[1] = b[1] + EPS;
	p1 = eval(patch,beps);
	v = PPDiff(p1,p2);
	
	beps[0] = b[0] + EPS;
	beps[1] = b[1];
	beps[2] = b[2] - EPS;
	p2 = eval(patch,beps);
	beps[0] = b[0] - EPS;
	beps[2] = b[2] + EPS;
	p1 = eval(patch,beps);
	w = PPDiff(p1,p2);
	
	return VVDot(v,w)/SQ(2.*EPS);
}

static Scalar G(void* patch, Point (*eval)(), Scalar b[3])
{
	Point p1,p2;
	Vector v;
	Scalar beps[3];
	
	beps[0] = b[0] + EPS;
	beps[1] = b[1];
	beps[2] = b[2] - EPS;
	p2 = eval(patch,beps);
	beps[0] = b[0] - EPS;
	beps[2] = b[2] + EPS;
	p1 = eval(patch,beps);
	v = PPDiff(p1,p2);
	return VVDot(v,v)/SQ(2.*EPS);
	
}

#endif

static Scalar wt[3]={1.,-2.,1.};

#if 0

static Scalar L(patch,eval,b,n)
void* patch;
Point (*eval)();
Scalar b[3];
Normal n;
{
	Point p[3];
	Vector v;
	
	p[1] = eval(patch,b);
	b[0] += EPS;
	b[1] -= EPS;
	p[0] = eval(patch,b);
	b[0] -= 2.*EPS;
	b[1] += 2.*EPS;
	p[2] = eval(patch,b);
	b[0] += EPS;
	b[1] -= EPS;
	
	v = PPvcN(3, p, wt);
	return NVApply(n,v)/SQ(EPS);
}


static Scalar wt2[4]={1.,-1.,-1.,1.};

static Scalar M(patch,eval,b,n)
void* patch;
Point (*eval)();
Scalar b[3];
Normal n;
{
	Point p[4];
	Vector v;
	
	b[0] += 2.*EPS;
	b[1] -= EPS;
	b[2] -= EPS;
	p[0] = eval(patch,b);
	b[0] -= 2.*EPS;
	b[1] += 2.*EPS;
	p[1] = eval(patch,b);
	b[0] -= 2.*EPS;
	b[2] += 2.*EPS;
	p[3] = eval(patch,b);
	b[0] += 2.*EPS;
	b[1] -= 2.*EPS;
	p[2] = eval(patch,b);
	b[0] += 0.;
	b[1] += EPS;
	b[2] -= EPS;
	
	v = PPvcN(4, p, wt2);
	return NVApply(n,v)/(4.*SQ(EPS));
}

static Scalar N(patch,eval,b,n)
void* patch;
Point (*eval)();
Scalar b[3];
Normal n;
{
	Point p[3];
	Vector v;
	
	p[1] = eval(patch,b);
	b[0] += EPS;
	b[2] -= EPS;
	p[0] = eval(patch,b);
	b[0] -= 2.*EPS;
	b[2] += 2.*EPS;
	p[2] = eval(patch,b);
	b[0] += EPS;
	b[2] -= EPS;
	
	v = PPvcN(3, p, wt);
	return NVApply(n,v)/SQ(EPS);
}





Scalar EstTPGaussianCurvature(patch,eval,i,j,k,normal)
void* patch;
Point (*eval)();
int i,j,k;
Normal normal;
{
	Scalar b[3];
	Scalar e,f,g,l,m,n;
	Scalar gk;
	Scalar sum;
	
	sum=i+j+k;
	
	/* first set b's to true values */
	b[0] = (double)i/sum;
	b[1] = (double)j/sum;
	b[2] = 1. - b[0] - b[1];
	
	/* now peturb point to be slighty inside the patch if needed */
	if ( i == 0  ||  j == 0  ||  k == 0 ){
		if ( i == 0 ){
			b[0] += 2.*EPS;
			b[1] -= EPS;
			b[2] -= EPS;
		}
		if ( j == 0 ){
			b[1] += 2.*EPS;
			b[0] -= EPS;
			b[2] -= EPS;
		}
		if ( k == 0 ){
			b[2] += 2.*EPS;
			b[0] -= EPS;
			b[1] -= EPS;
		}
		/* need more adjustments if at vertex */
		if ( i==0 && j==0 ){
			b[2] -= 2.*EPS;
			b[0] += EPS;
			b[1] += EPS;
		}
		if ( i==0 && k==0 ){
			b[1] -= 2.*EPS;
			b[0] += EPS;
			b[2] += EPS;
		}
		if ( j==0 && k==0 ){
			b[0] -= 2.*EPS;
			b[1] += EPS;
			b[2] += EPS;
		}
	}
	/* now compute all points needed for approx */
	e = E(patch,eval,b);
	f = F(patch,eval,b);
	g = G(patch,eval,b);
	l = L(patch,eval,b,normal);
	m = M(patch,eval,b,normal);
	n = N(patch,eval,b,normal);
	gk = (l*n - m*m)/(e*g - f*f);
	
	return gk;
}



void gkTessTPPatch(patch,eval,samples,frame)
void* patch;
Point (*eval)();
int samples;
Frame frame;
{
	Scalar b[3];
	int i,j;
	Patch SamplePatch;
	Point p;
	Normal n;
	Scalar* gk;
	
	SamplePatch = PatchCreate( samples, SpaceOf(frame), 3 );
	
	gk = (Scalar*)malloc( NetSize(samples)*sizeof(Scalar) );
	
	for(i=0;i<=samples;i++){
		for(j=0;j<=samples-i;j++){
			b[0] = (double)i/(double)samples;
			b[1] = (double)j/(double)samples;
			b[2] = 1. - b[0] - b[1];
			p = eval(patch,b);
			PatchSetPoint(&SamplePatch, p, i, j, samples-i-j );
			n = CalcNormal(patch,eval,i,j,samples-i-j);
			PatchSetNormal(&SamplePatch, n, i, j, samples-i-j );
			
			*(gk+VertexIndex(i,j,samples-i-j)) = 
			  EstGaussianCurvature(patch,eval,i,j,samples-i-j,n);
			
		}
	}
	OutputPatchNet2( stdout, SamplePatch, frame, gk );
	PatchFree( SamplePatch );
}
#endif

Vector FirstTPDer(void* patch, Point (*eval)(), Scalar b[3], int dir)
{
	Point p1,p2;
	Vector v;
	Scalar beps[3];
	
	beps[dir] = b[dir]+EPS;
	beps[(dir+1)%2] = b[(dir+1)%2];
	p1 = eval(patch,beps);
	beps[dir] = b[dir]-EPS;
	beps[(dir+1)%2] = b[(dir+1)%2];
	p2 = eval(patch,beps);
	v = PPDiff(p1,p2);
	return SVMult(.5/EPS,v);
}


Normal CalcTPNormal(void* patch, Point (*eval)(), Scalar u, Scalar v)
{
	Scalar b[2];
	Vector v1,v2;
	
	b[0] = u;
	b[1] = v;
	
	v1 = FirstTPDer(patch,eval,b,0);
	v2 = FirstTPDer(patch,eval,b,1);
	return VDual(VNormalize(VVCross(v1,v2)));
}

Vector SecondTPDer(void* patch, Point (*eval)(), Scalar b[3], 
		   int dir1, int dir2)
{
	Point p[5];
	Vector v;
	Scalar beps[3];
	
	if ( dir1 == dir2 ) {
		beps[dir1] = b[dir1] + EPS;
		beps[(dir1+1)%2] = b[(dir1+1)%2];
		p[0] = eval(patch,beps);
		
		p[1] = eval(patch,b);
		
		beps[dir1] = b[dir1] - EPS;
		beps[(dir1+1)%2] = b[(dir1+1)%2];
		p[2] = eval(patch,beps);
		
		v = PPvcN(3, p, wt);
		return SVMult(1./SQ(EPS),v);
	} else {
		Scalar wts[4];
		wts[0] = 1.;
		wts[1] = -1.;
		wts[2] = -1.;
		wts[3] = 1.;
		
		beps[0] = b[0] + EPS;
		beps[1] = b[1] + EPS;
		p[0] = eval(patch,beps);
		
		beps[0] = b[0] + EPS;
		beps[1] = b[1] - EPS;
		p[1] = eval(patch,beps);
		
		beps[0] = b[0] - EPS;
		beps[1] = b[1] + EPS;
		p[2] = eval(patch,beps);
		
		beps[0] = b[0] - EPS;
		beps[1] = b[1] - EPS;
		p[3] = eval(patch,beps);
		
		v = PPvcN(4, p, wts);
		return SVMult(.25/SQ(EPS),v);
	}
}

SFF TPSff(void* patch, Point (*eval)(), Scalar u, Scalar v)
{
	SFF sff;
	Vector nv;
	Scalar b[2];

	b[0] = u;
	b[1] = v;
	sff.v0 = FirstTPDer(patch,eval,b,0);
	sff.v1 = FirstTPDer(patch,eval,b,1);
	nv = VNormalize(VVCross(sff.v0, sff.v1));
	sff.m[0][0] = VVDot(SecondTPDer(patch,eval,b,0,0),nv);
	sff.m[0][1] = VVDot(SecondTPDer(patch,eval,b,0,1),nv);
	sff.m[1][0] = sff.m[0][1];
	sff.m[1][1] = VVDot(SecondTPDer(patch,eval,b,1,1),nv);
	return sff;
}

static Scalar CUBE(a)
Scalar a;
{
	return a*a*a;
}

#if 0
static void CompCurvature(patch,eval,b0,b1,b2)
void* patch;
Point (*eval)();
Scalar b0;
Scalar b1;
Scalar b2;
{
	int base;
	Scalar k;
	Vector vfd,vsd;
	Scalar b[3];
	
	b[0] = b0;
	b[1] = b1;
	b[2] = b2;
	base = MAX3(b[0],b[1],b[2]);
	
#if 1
	if ( 1-b[base] < EPS ) {
		b[base] -= EPS;
		b[(base+2)%3] += EPS;
	}
#endif
	
	vfd = FirstDer(patch,eval,b,(base+1)%3);
	vsd = SecondDer(patch,eval,b,(base+1)%3,(base+1)%3);
	k = VMag( VVCross(vfd, vsd) ) / CUBE(VMag(vfd));
	{static int first=1; static Scalar prev;
	 if ( first || EPS != prev )
	   fprintf(stderr,"eps = %g\n",EPS);
	 prev = EPS; first=0;
 }
	fprintf(stderr,"CompCurvature: %g %g => %g / %g = %17.17f\n",
		VMag(vfd), VMag(vsd), VMag(VVCross(vfd,vsd)), CUBE(VMag(vfd)),k);
}
#endif
