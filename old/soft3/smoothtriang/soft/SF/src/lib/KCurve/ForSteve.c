/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include <stdio.h>
#include <math.h>
#include "all.h"
#include <sys/time.h>
#include <sys/resource.h>
#include "curve.h"

extern	Frame	WF;

double lambda0=1.5;
double lambda1=1.5;

double mu0=0.;
double mu1=0.;

Point	HPPos();

Vector	HPDeriv(), HPCurvature(), gEval(), H5Eval();

/* TOP-SECRET routines for confusing points and vectors */

#define	V2P(v,f,p)  {Scalar	x, y, z;		\
		       VCoords((v), (f), &x, &y, &z);	\
			 (p) = PCreate((f), x, y, z); }

#define	P2V(p,f,v)  {Scalar	x, y, z;		\
		       PCoords((p), (f), &x, &y, &z);	\
			 (v) = VCreate((f), x, y, z); }



/* Quintic Hermite functions */

#define	H5_0(t, t2, t3, t4, t5)	(-6.0*(t5)  + 15.0*(t4)  - 10.0*(t3)  + 1.0)
#define	H5_1(t, t2, t3, t4, t5)	(-3.0*(t5)  + 8.0*(t4)  - 6.0*(t3)  + (t))
#define	H5_2(t, t2, t3, t4, t5)	((-(t5)  + 3.0*(t4)  - 3.0*(t3)  + (t2)) / 2.0)
#define	H5_3(t, t2, t3, t4, t5)	(((t5) - 2.0*(t4) +  (t3)) / 2.0)
#define	H5_4(t, t2, t3, t4, t5)	(-3.0*(t5)  + 7.0*(t4)  - 4.0*(t3))
#define	H5_5(t, t2, t3, t4, t5)	(6.0*(t5)  - 15.0*(t4)  + 10.0*(t3))

/* Derivatives of quintic Hermite functions */

#define dH5_0(t, t2, t3, t4)	(-30.0*(t4)  + 60.0*(t3)  - 30.0*(t2))
#define dH5_1(t, t2, t3, t4)	(-15.0*(t4)  + 32.0*(t3)  - 18.0*(t2)  + 1.0)
#define dH5_2(t, t2, t3, t4)	((-5.0*(t4)  + 12.0*(t3)  - 9.0*(t2)  + 2.0*(t)) / 2.0)
#define dH5_3(t, t2, t3, t4)	((5.0*(t4)  - 8.0*(t3)  + 3.0*(t2)) / 2.0)
#define dH5_4(t, t2, t3, t4)	(-15.0*(t4)  + 28.0*(t3)  - 12.0*(t2))
#define dH5_5(t, t2, t3, t4)	(30.0*(t4)  - 60.0*(t3)  + 30.0*(t2))

/* 2nd derivatives of quintic Hermite functions */

#define d2H5_0(t, t2, t3)	(-120.0*(t3)  + 180.0*(t2)  - 60.0*(t))
#define d2H5_1(t, t2, t3)	(-60.0*(t3)  + 96.0*(t2)  - 36.0*(t))
#define d2H5_2(t, t2, t3)	(-10.0*(t3)  + 18.0*(t2)  - 9.0*(t) + 1.0)
#define d2H5_3(t, t2, t3)	(10.0*(t3)  - 12.0*(t2)  + 3.0*(t))
#define d2H5_4(t, t2, t3)	(-60.0*(t3)  + 84.0*(t2)  - 24.0*(t))
#define d2H5_5(t, t2, t3)	(120.0*(t3)  - 180.0*(t2)  + 60.0*(t))




/**************************************************************************
 ** H5Eval:
 **
 ** Returns the dth derivative of the quintic Hermite at t.
 ** If d==0, the original quintic Hermite is evaluated.
 **
 ** WARNING: returns a Point (NOT a Vector) if d==0
 **************************************************************************/


Vector	H5Eval(d, p0, p1, d1_0, d1_1, d2_0, d2_1, t)
int	d;		/* order of derivative to evaluate at */
Point	p0, p1;		/* pos'n at ends */
Vector	d1_0, d1_1;	/* 1st deriv. at ends */
Vector	d2_0, d2_1;	/* 2nd deriv. at ends */
Scalar	t;		/* arg to evaluate curve at */
{
	Vector  V, vp0, vp1, v0, v1, v2, v3, v4, v5;
	Scalar  px, py, pz, t2, t3, t4, t5;
	Point   Result;
	
	/* compute powers of t now for later efficiency, ease of reading */
	
	t2 = t * t; t3 = t2 * t; t4 = t3 * t; t5 = t4 * t;
	
	P2V(p0, WF, vp0);
	P2V(p1, WF, vp1);;

	switch (d)	  {
	      case 0:		/* these are in the .h file */
		v0 = SVMult(H5_0(t, t2, t3, t4, t5), vp0);
		v1 = SVMult(H5_1(t, t2, t3, t4, t5), d1_0);
		v2 = SVMult(H5_2(t, t2, t3, t4, t5), d2_0);
		v3 = SVMult(H5_3(t, t2, t3, t4, t5), d2_1);
		v4 = SVMult(H5_4(t, t2, t3, t4, t5), d1_1);
		v5 = SVMult(H5_5(t, t2, t3, t4, t5), vp1);
		break;
		
	      case 1:
		v0 = SVMult(dH5_0(t, t2, t3, t4), vp0);
		v1 = SVMult(dH5_1(t, t2, t3, t4), d1_0);
		v2 = SVMult(dH5_2(t, t2, t3, t4), d2_0);
		v3 = SVMult(dH5_3(t, t2, t3, t4), d2_1);
		v4 = SVMult(dH5_4(t, t2, t3, t4), d1_1);
		v5 = SVMult(dH5_5(t, t2, t3, t4), vp1);
#if 0
{Scalar x,y;
VCoords(v0,WF,&x,&y);
fprintf(stderr,"v0 = %5.5f %5.5f\t",x,y);
VCoords(v1,WF,&x,&y);
fprintf(stderr,"v1 = %5.5f %5.5f\n",x,y);
VCoords(v2,WF,&x,&y);
fprintf(stderr,"v2 = %5.5f %5.5f\t",x,y);
VCoords(v3,WF,&x,&y);
fprintf(stderr,"v3 = %5.5f %5.5f\n",x,y);
VCoords(v4,WF,&x,&y);
fprintf(stderr,"v4 = %5.5f %5.5f\t",x,y);
VCoords(v5,WF,&x,&y);
fprintf(stderr,"v5 = %5.5f %5.5f\n",x,y);
}
#endif
		break;
		
	      case 2:
		v0 = SVMult(d2H5_0(t, t2, t3), vp0);
		v1 = SVMult(d2H5_1(t, t2, t3), d1_0);
		v2 = SVMult(d2H5_2(t, t2, t3), d2_0);
		v3 = SVMult(d2H5_3(t, t2, t3), d2_1);
		v4 = SVMult(d2H5_4(t, t2, t3), d1_1);
		v5 = SVMult(d2H5_5(t, t2, t3), vp1);
		break;
	}
	
	V = VVAdd(v0, VVAdd(v1, VVAdd(v2, VVAdd(v3, VVAdd(v4, v5)))));
	return V;
}



struct hpcurve {
	Point p0,p1;
	Vector t0,t1;
	Vector d2_0, d2_1;
};


struct hpcurve* HPCreate(p0,t0,k0,p1,t1,k1)
Point p0;
Vector t0;
Scalar k0;
Point p1;
Vector t1;
Scalar k1;
{
	struct hpcurve* hp;
	Vector n0;
	Vector n1;
	Scalar x,y,z;
	
	hp = (struct hpcurve*)malloc(sizeof(struct hpcurve));
	hp->p0 = p0;
	hp->p1 = p1;
	hp->t0 = SVMult(lambda0,VNormalize(t0));
	hp->t1 = SVMult(lambda1,VNormalize(t1));
VCoords(hp->t0,WF,&x,&y);
fprintf(stderr,"t0 = %g %g,  ",x,y);
VCoords(hp->t1,WF,&x,&y);
fprintf(stderr,"t1  %g %g\n",x,y);

	VCoords(t0, WF, &x, &y);
	n0 = SVMult(k0, VCreate(WF, y, -x));
	VCoords(t1, WF, &x, &y);
	n1 = SVMult(k1, VCreate(WF, y, -x));

	hp->d2_0 = VVAdd(SVMult(lambda0 * lambda0, n0), SVMult(mu0, t0));
	hp->d2_1 = VVAdd(SVMult(lambda1 * lambda1, n1), SVMult(-mu1, t1));
VCoords(hp->d2_0,WF,&x,&y);
fprintf(stderr,"d2_0 = %g %g,  ",x,y);
VCoords(hp->d2_1,WF,&x,&y);
fprintf(stderr,"d2_1  %g %g\n",x,y);

	return hp;
}


Curve HPBezierCreate(p0,t0,k0,p1,t1,k1)
Point p0;
Vector t0;
Scalar k0;
Point p1;
Vector t1;
Scalar k1;
{
	struct hpcurve hhp, *hp= &hhp;
	Vector n0;
	Vector n1;
	Scalar x,y,z;
	Curve bcp;
	
	hp->p0 = p0;
	hp->p1 = p1;
	hp->t0 = SVMult(lambda0,VNormalize(t0));
	hp->t1 = SVMult(lambda1,VNormalize(t1));

	VCoords(t0, WF, &x, &y);
	n0 = SVMult(k0, VCreate(WF, y, -x));
	VCoords(t1, WF, &x, &y);
	n1 = SVMult(k1, VCreate(WF, y, -x));

	hp->d2_0 = VVAdd(SVMult(lambda0 * lambda0, n0), SVMult(mu0, t0));
	hp->d2_1 = VVAdd(SVMult(lambda1 * lambda1, n1), SVMult(-mu1, t1));
VCoords(hp->d2_0,WF,&x,&y);
fprintf(stderr,"d2_0 = %g %g\n",x,y);
VCoords(hp->d2_1,WF,&x,&y);
fprintf(stderr,"d2_1 = %g %g\n",x,y);

	bcp.degree = 5;
	bcp.cp[0] = p0;
	bcp.cp[1] = PVAdd(p0, SVMult(.2, hp->t0));
	bcp.cp[2] = PVAdd(bcp.cp[1], 
			  VVAdd(SVMult(.05, hp->d2_0),
				PPDiff(bcp.cp[1],p0)));
	bcp.cp[4] = PVAdd(p1, SVMult(-.2, hp->t1));
	bcp.cp[3] = PVAdd(bcp.cp[4], 
			  VVAdd(SVMult(.05, hp->d2_1),
				PPDiff(bcp.cp[4],p1)));
	bcp.cp[5] = p1;
{int i;
for(i=0;i<=5;i++){
PCoords(bcp.cp[i],WF,&x,&y);
fprintf(stderr,"cp[%d] = %g, %g\n",i,x,y);
}
}
	return bcp;
}

Point HPPosition(hp,t)
struct hpcurve* hp;
Scalar t;
{
	Vector v;
	Point p;

	v = H5Eval(0, 
		   hp->p0, hp->p1, 
		   hp->t0, hp->t1, 
		   hp->d2_0, hp->d2_1, 
		   t);
	V2P(v,WF,p);
}


Vector HPFirst(hp,t)
struct hpcurve* hp;
Scalar t;
{
	return H5Eval(1, hp->p0, hp->p1, hp->t0, hp->t1, hp->d2_0, hp->d2_1, t);
}

Vector HPSecond(hp,t)
struct hpcurve* hp;
Scalar t;
{
	return H5Eval(2, hp->p0, hp->p1, hp->t0, hp->t1, hp->d2_0, hp->d2_1, t);
}
