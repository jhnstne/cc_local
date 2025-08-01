/*
 *----------------------------------------------------------------------
 *  File:  hermite.c
 *	Construct a geometric Hermite curve (i.e., one that interpolates
 *  position and tangent direction at two points).
 *	There are lots of ways to do this.  The ones implemented in this
 *  file are:
 *	Furth : Planar cubic curve, derivative at ends is equal in
 *		length to distance between data points.
 *
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"


/*
 *----------------------------------------------------------------------
 *  Function:  UnitNormPerpTo_InPlaneOf
 *	Given vectors V1 and V2, return the vector perpendicular to V1
 *  that lies in the plane spanned by V1 and V2.  The sign of this
 *  vector is such that it is parallel to V1xV2.
 *----------------------------------------------------------------------
 */
static Vector UnitNormPerpTo_InPlaneOf(V1, V2)
Vector V1, V2;
{
	Vector R;

	R = VVCross(V1,V2);
	R = VVCross(R,V1);
	R = SVMult(1.0/VMag(R), R);
	return R;
}




/*
 *  Function:  FurthBoundary
 *	Sets tangents at either end of a single border curve, according to the
 *  notation on p. 9 of Furth
 */
void FurthBoundary(p0, n0, p1, n1, r0, r1)
Point	p0, p1;
Normal	n0, n1;
Point *r0, *r1;
{
	Vector	u1, u2, u3;
	Vector v0, v1;
	
	u1 = VNormalize(PPDiff(p1, p0));
	u3 = VNormalize(VVCross(u1, VVAdd(NDual(n0), NDual(n1))));
	u2 = VNormalize(VVCross(u3, u1));
	
	/* determine tangents at 0, 1 values along border curve i */
	/* pp0 stands for "p prime at value 0" */
	
	v0 = SVMult(VMag(PPDiff(p1, p0)), VNormalize(VVCross(NDual(n0), u3)));
	v1 = SVMult(VMag(PPDiff(p1, p0)), VNormalize(VVCross(NDual(n1), u3)));
	
	if (VVDot(VVCross(NDual(n1), u3), u1) < 0.0) {
		v0 = SVMult(-1.0, v0);
	} else {
		v1 = SVMult(-1.0, v1);
	}
	*r0 = PVAdd(p0, SVMult(1.0/3.0, v0));
	*r1 = PVAdd(p1, SVMult(1.0/3.0, v1));
	
}

/*
 *----------------------------------------------------------------------
 *  Function:  Jensen
 *----------------------------------------------------------------------
 */
void Jensen(S0,S1,T0, T3,S2,S3, T1,T2, c)
Point S0, S1, T0, T3, S2, S3;
Point *T1, *T2;
Scalar c;
{
	Vector A0, A1, A2, A3;
	Vector B0, B1, B2, B3;
	Vector C0, C1, C2;
	Scalar h0,h1,k0,k1,z;
	Frame f;
	extern int new;

	B0 = PPDiff(T0, S0);
	B3 = PPDiff(T3, S3);
	C0 = PPDiff(S1, S0);
	C1 = PPDiff(S2, S1);
	C2 = PPDiff(S3, S2);

	A0 = UnitNormPerpTo_InPlaneOf(C0, B0);
	A3 = UnitNormPerpTo_InPlaneOf(C2, B3);
	A1 = VVAdd( SVMult( 2.0/3.0, A0 ),  SVMult( 1.0/3.0, A3 ) );
	A2 = VVAdd( SVMult( 1.0/3.0, A0 ),  SVMult( 2.0/3.0, A3 ) );

	/* extract h0,h1,k0,k1 */
	f = FCreate("f1",S0,A0,C0,VVCross(A0,C0));
	VCoords(B0,f,&k0,&h0,&z);
	f = FCreate("f1",S3,A3,C2,VVCross(A3,C2));
	VCoords(B3,f,&k1,&h1,&z);

	/* calc B1, B2 */
	c = c - .5;
	B1 = VVAdd(VVAdd( SVMult( 1./3. * k0, A3 ), SVMult( 2./3.* (c*k0+c*k1), A0)),
		   VVAdd( SVMult( 2./3. * h0, C1 ), SVMult( 1./3. * h1, C0 ) ));

	B2 = VVAdd(VVAdd( SVMult( 1./3. * k1, A3 ), SVMult( 2./3.* (c*k0+c*k1), A0)),
		   VVAdd( SVMult( 1./3. * h0, C2 ), SVMult( 2./3.*h1, C1 ) ));

	*T1 = PVAdd(S1, B1);
	*T2 = PVAdd(S2, B2);

}



/*
 *----------------------------------------------------------------------
 *  Function:  ChiyokuraKimura
 *----------------------------------------------------------------------
 */
void ChiyokuraKimura(S0,S1,T0, T3,S2,S3, T1,T2,a0,a3)
Point S0, S1, T0, T3, S2, S3;
Point *T1, *T2;
Point a0,a3;
{
	Vector A0, A1, A2, A3;
	Vector B0, B1, B2, B3;
	Vector C0, C1, C2;
	Scalar h0,h1,k0,k1,z;
	Frame f;
	extern int new;

	B0 = PPDiff(T0, S0);
	B3 = PPDiff(T3, S3);
	C0 = PPDiff(S1, S0);
	C1 = PPDiff(S2, S1);
	C2 = PPDiff(S3, S2);

	if ( new ) {
		A0 = PPDiff(T0,a0);
		A0 = SVMult(1.0/VMag(A0),A0);
		A3 = PPDiff(T3,a3);
		A3 = SVMult(1.0/VMag(A3),A3);
	} else {
		A0 = UnitNormPerpTo_InPlaneOf(C0, B0);
		A3 = UnitNormPerpTo_InPlaneOf(C2, B3);
#if 0
		PrintVector(stderr,A0);
		PrintVector(stderr,A3);
#endif
	}
	A1 = VVAdd( SVMult( 2.0/3.0, A0 ),  SVMult( 1.0/3.0, A3 ) );
	A2 = VVAdd( SVMult( 1.0/3.0, A0 ),  SVMult( 2.0/3.0, A3 ) );

	/* extract h0,h1,k0,k1 */
	f = FCreate("f1",S0,A0,C0,VVCross(A0,C0));
	VCoords(B0,f,&k0,&h0,&z);
	f = FCreate("f1",S3,A3,C2,VVCross(A3,C2));
	VCoords(B3,f,&k1,&h1,&z);

	/* calc B1, B2 */
	/* the following two methods are equivalent */
#if 1
	B1 = VVAdd( VVAdd( SVMult((k1-k0)/3.0,A0), SVMult(k0, A1)),
		   VVAdd( SVMult(2.0*h0/3.0, C1), SVMult(h1/3.0,C0)));
	B2 = VVAdd( VVAdd( SVMult(k1,A2), SVMult((k0-k1)/3.0,A3)),
		   VVAdd( SVMult(h0/3.0,C2), SVMult(2.0*h1/3.0, C1)));
#else
	B1 = VVAdd( VVAdd( SVMult((k1+k0)/3.0,A0), SVMult(k0/3., A3)),
		   VVAdd( SVMult(2.0*h0/3.0, C1), SVMult(h1/3.0,C0)));
	B2 = VVAdd( VVAdd( SVMult(k1/3.,A2), SVMult((k0+k1)/3.0,A3)),
		   VVAdd( SVMult(h0/3.0,C2), SVMult(2.0*h1/3.0, C1)));
#endif


	*T1 = PVAdd(S1, B1);
	*T2 = PVAdd(S2, B2);

#if 0
	{float x;
	 x = VMag( VVDiff ( VVAdd( SVMult( 0.5, VVDiff( PPDiff(T0,S0), PPDiff(*T1,S1) ) ), PPDiff(*T1,S1) ),
			   VVAdd( SVMult( 0.5, VVDiff( PPDiff(T3,S3), PPDiff(*T2,S2) ) ), PPDiff(*T2,S2) ) ));
	 fprintf(stderr,"x = %f\n",x);
 }
#endif
		

}


/*
 *----------------------------------------------------------------------
 *  Function:  MinMaxBoundary
 *----------------------------------------------------------------------
 */
void MinMaxBoundary(s0,S1,T0, T3,S2,s3, T1,T2,a0,a3,R0,R3)
Vertex* s0;
Point S1, T0, T3, S2;
Vertex* s3;
Point *T1, *T2;
Point a0,a3;
Point R0, R3;
{
	Point S0,S3;
	Vector A0, A1, A2, A3;
	Vector B0, B1, B2, B3;
	Vector C0, C1, C2;
	Vector d0,d1,d2;
	Scalar h0,h1,k0,k1,z;
	Frame f;
	SFF sff0, sff1;
	Scalar M0a,M0b,M1a,M1b;	/* the normal component of the mixed
				   partial at the end points */
	extern int new;
	Vector N0,N1;
	Scalar N0dN1;
	Vector perp;
	Scalar t0,t1;

	GetUDPoint(s0,&S0);
	GetUDPoint(s3,&S3);

	B0 = PPDiff(T0, S0);
	B3 = PPDiff(T3, S3);
	C0 = PPDiff(S1, S0);
	C1 = PPDiff(S2, S1);
	C2 = PPDiff(S3, S2);

	if ( new ) {
		A0 = VNormalize(PPDiff(T0,a0));
		A3 = VNormalize(PPDiff(T3,a3));
	} else {
		A0 = UnitNormPerpTo_InPlaneOf(C0, B0);
		A3 = UnitNormPerpTo_InPlaneOf(C2, B3);
	}

	d0 = A0;
	d2 = A3;
	N0 = VNormalize( NDual(ReturnUDNormal(s0)) );
	N1 = VNormalize( NDual(ReturnUDNormal(s3)) );

	f = StdFrame(SpaceOf(S1));
	dGetSFF(s0->externalData,"Point.sff",f,&sff0);
	dGetSFF(s3->externalData,"Point.sff",f,&sff1);

	/* extract h0,h1,k0,k1 */
	f = FCreate("f1",S0,A0,C0,VVCross(A0,C0));
	VCoords(B0,f,&k0,&h0,&z);
	f = FCreate("f1",S3,A3,C2,VVCross(A3,C2));
	VCoords(B3,f,&k1,&h1,&z);

	t0 = .5 * EvalSFF2(&sff0,C0,A0);
	t1 = .5 * EvalSFF2(&sff1,C2,SVMult(-1.0,A3));
	N0dN1 = VVDot(N0,N1);

	perp = VNormalize(VVCross(N0,N1));
	if ( VVDot(perp, A0) < 0.0 ){
		perp = SVMult(-1.0,perp);
	}

	d1 = VVAdd( SVMult( VVDot(SVMult(.25,VVAdd(d0,d2)),perp),perp),
		   VVAdd( SVMult( (t1 - (t0*N0dN1))/(1+N0dN1*N0dN1),N1),
			 SVMult( (t0 - (t1*N0dN1))/(1+N0dN1*N0dN1),N0)));

	/*??*/  A1 = VVAdd( SVMult( 4.0/9.0, d0 ),  
			   VVAdd( SVMult( 4.0/9.0, d1 ),
				 SVMult( 1.0/9.0, d2 ) ) );
	/*??*/  A2 = VVAdd( SVMult( 1.0/9.0, d0 ),
			   VVAdd( SVMult( 4.0/9.0, d1 ),
				 SVMult( 4.0/9.0, d2 ) ) );

	/* calc B1, B2 */
	B1 = VVAdd( VVAdd( SVMult((k1-k0)/3.0,A0), SVMult(k0, A1)),
		   VVAdd( SVMult(2.0*h0/3.0, C1), SVMult(h1/3.0,C0)));
	B2 = VVAdd( VVAdd( SVMult(k1,A2), SVMult((k0-k1)/3.0,A3)),
		   VVAdd( SVMult(h0/3.0,C2), SVMult(2.0*h1/3.0, C1)));


	*T1 = PVAdd(S1, B1);
	*T2 = PVAdd(S2, B2);
}




/*
 *----------------------------------------------------------------------
 *  Function:  ComputeCrossBoundary
 *----------------------------------------------------------------------
 */
void ComputeCrossBoundary(s0,S1,T0, T3,S2,s3, T1,T2,a0,a3,A,jensen)
Vertex* s0;
Point S1, T0, T3, S2;
Vertex* s3;
Point *T1, *T2;
Point a0,a3;
Vertex* A;	/* The third vertex, on the farside of the edge */
Scalar jensen;
{
	Point S0,S3,R0,R3;
	extern double alpha,alpha1,alpha2;
	Point EdgeTangent();

	if ( jensen == 1. ){
		ChiyokuraKimura(ReturnUDPoint(s0),S1,T0, T3,S2,ReturnUDPoint(s3), 
				T1,T2,a0,a3);
	} else {
		Jensen(ReturnUDPoint(s0),S1,T0, T3,S2,ReturnUDPoint(s3), 
		       T1,T2,jensen);
	}
#if 1
#else
	S0 = ReturnUDPoint(s0);
	S3 = ReturnUDPoint(s3);
	R0 = PPac3( S0, PPac(S0,S1,.25), PPac(S0,EdgeTangent(s0,A),.25),
		   -alpha1/alpha2,1.0/alpha2,1.0/alpha2);
	R3 = PPac3( S3, PPac(S3,S2,.25), PPac(S3,EdgeTangent(s3,A),.25),
		   -alpha1/alpha2,1.0/alpha2,1.0/alpha2);
	MinMaxBoundary(s0,S1,T0, T3,S2,s3,T1,T2,a0,a3,
		       EdgeTangent(s0,A),EdgeTangent(s3,A),R0,R3);
#endif
}


int qBoundary(v1, v2, p1, p2)
Vertex* v1;
Vertex* v2;
Point* p1;
Point* p2;
{
	if ( BoundaryDegree(v1, v2) == 3 ) {
		return 0;
	}
	QuadraticBoundary(v1,v2,p1,p2);
	return 1;
}

#ifndef SQ
#define SQ(A) ((A)*(A))
#endif

static Scalar proj(p1,n1,p2,n2)
Point p1;
Vector n1;
Point p2;
Vector n2;
{
	Scalar s;
	s = VVDot(VVAdd(n1, SVMult(VVDot(n1,n2), n2)),
		  PPDiff(p2,p1));
	return s/(1-SQ(VVDot(n1,n2)));
}

static QuadraticBoundary(el, k, p1, p2)
Vertex* el;
Vertex* k;
Point* p1;
Point* p2;
{
	Vector va[3];
	Scalar w[3];
	Vector v1,v2;
	Scalar delta0, delta1;
	Scalar d;
	Point pk, pl;
	Vector nk, nl;

	pk = ReturnUDPoint(k); pl = ReturnUDPoint(el);
	nk = NDual(ReturnUDNormal(k));
	nl = NDual(ReturnUDNormal(el));
	delta0 = proj(pk, nk, pl, nl);
	delta1 = proj(pl, nl, pk, nk);
	va[0] = PPDiff(pl,pk);
	va[1] = nk;
	va[2] = nl;
	w[0] = .5; w[1] = -delta0/2; w[2] = -delta1/2;
	v1 = VVlcN(3, va, w);
	va[0] = SVMult(-1., va[0]);
	v2 = VVlcN(3, va, w);
	if ( (d=PPDist(PVAdd(pk, v1), PVAdd(pl, v2))) > 1e-7 ) {
		fprintf(stderr,"QuadraticBoundary: imp wrong. %g\n",d);
		exit(1);
	}
	*p1 = PVAdd(pl, SVMult(2./3, v2));
	*p2 = PVAdd(pk, SVMult(2./3, v1));
}




static int BoundaryDegree(r, s)
Vertex* r;
Vertex* s;
{
	float sig0,sig1;

	sig0 = NVApply(ReturnUDNormal(r), PPDiff(ReturnUDPoint(r),
						 ReturnUDPoint(s)));
	sig1 = NVApply(ReturnUDNormal(s), PPDiff(ReturnUDPoint(s),
						 ReturnUDPoint(r)));
	if ( fabs(sig0) < 1e-7  &&  fabs(sig1) < 1e-7 ) {
		return 1;
	} else if ( sig0*sig1 >= 0 ) {
		return 2;
	} else {
		return 3;
	} 
}
