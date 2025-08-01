/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  kcurve2.c
 *	An extension of the de Boor-Hollig-Sabin method to non-planar
 *  curves.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include <math.h>

#define EPS 1e-09
#ifndef SQ
#define SQ(A) ((A)*(A))
#endif


extern int steve;

/*
 *----------------------------------------------------------------------
 *  Function:  VVScalarCrossProduct
 *----------------------------------------------------------------------
 */
static Scalar VVScalarCrossProduct(Vector v1, Vector v2)
{
	Scalar x1,y1,x2,y2;

	VCoords(v1,StdFrame(SpaceOf(v1)), &x1,&y1);
	VCoords(v2,StdFrame(SpaceOf(v2)), &x2,&y2);
	return ( x1*y2 - x2*y1 );
}


/*
 *----------------------------------------------------------------------
 *  Function:  DataOrder
 *	Put an ordering on two points.  Return true if
 *  they needed to be flipped, false otherwise.
 *----------------------------------------------------------------------
 */
static int DataOrder(Point p0, Point p1)
{
	Scalar x0,y0,z0;
	Scalar x1,y1,z1;
	
	PCoords(p0, StdFrame(SpaceOf(p0)), &x0, &y0, &z0);
	PCoords(p1, StdFrame(SpaceOf(p1)), &x1, &y1, &z1);
	
	if ( x1 < x0 ){
		return 1;
	} else if ( x0 < x1 ) {
		return 0;
	}
	
	if ( y1 < y0 ){
		return 1;
	} else if ( y0 < y1 ) {
		return 0;
	}
	
	if ( z1 < z0 ){
		return 1;
	} else if ( z0 < z1 ) {
		return 0;
	}
	
	fprintf(stderr,"OrderData: points are identical!  Not swapping.  What?  You worry?\n");
	return 0;
}


/*
 *----------------------------------------------------------------------
 *  Function:  ComputeDeltas
 *----------------------------------------------------------------------
 */
static int ComputeDeltas(Point p0, Normal n0, Vector d0, Scalar k0, 
			 Point p1, Normal n1, Vector d1, Scalar k1, 
			 Scalar delta[3][2])
{
	Scalar N0_d1;
	Scalar N1_d0;
	Scalar N0_a;
	Scalar N1_a;
	Vector a;
	int order;	/* a flag to note which way we need to order the data*/
	
	order = DataOrder(p0,p1);
	
	/* flip the data if we need to */
	if ( order ) {
		Point tp; Normal tn; Vector td; Scalar tk;
		tp = p0; tn = n0; td = d0; tk = k0;
		p0 = p1; n0 = n1; d0 = SVMult(-1.,d1); k0 = k1;
		p1 = tp; n1 = tn; d1 = SVMult(-1.,td); k1 = tk;
	}
	
	a = PPDiff(p1,p0);
	N0_d1 = NVApply(n0,d1);
	N1_d0 = NVApply(n1,d0);
	N0_a = NVApply(n0,a);
	N1_a = NVApply(n1,a);
if(steve)
fprintf(stderr,"CD\n");
	/* bad things happen if tangents are parallel */
	if ( VMag(VVCross(d0,d1)) < EPS ) {
		Scalar aXd1, d0Xa;

if(steve)
fprintf(stderr,"CD a\n");
		/* Special case: straight line */
		if (fabs(k0) < EPS  &&  fabs(k1) < EPS  &&
		    fabs(N0_a) < EPS  &&  fabs(N1_a) < EPS ) {
			delta[0][1] = delta[0][0] = VMag(a)/3.;
			return 1;
		}
		/* should use epsilon */
		if ( N0_a/k0 <= EPS  ||  -N1_a/k1 <= 0.0) { 
			return 0;
		}
		delta[0][(1+order)%2] = sqrt( -N1_a/k1 * 2.0/3.0 );
		delta[0][0+order] = sqrt( N0_a/k0 * 2.0/3.0 );
		return 1;
	} else if ( fabs(k0) < EPS ) {
if(steve)
fprintf(stderr,"CD b order = %d\n",order);

		delta[0][(1+order)%2] = N0_a/N0_d1;
		delta[0][0+order] = (N1_a + 1.5*k1*SQ(delta[0][(1+order)%2]))/
				     N1_d0;
		if ( delta[0][1] > EPS  &&  delta[0][0] > EPS ) {
			if(steve)
			  fprintf(stderr,"a %g %g\n",delta[0][0],delta[0][1]);
			if ( delta[0][0+order] < VMag(a) ) {
				return 1;
			} else {
				return 0;
			}
		} else {
			return 0;
		}
	} else if ( steve && fabs(k1) < EPS ) {
if(steve)
fprintf(stderr,"CD c\n");
fprintf(stderr,"ComputeDeltas: Warning: untested code\n");

		delta[0][0+order] = N1_a/N1_d0;
		delta[0][(1+order)%2] = (N0_a - 1.5*k0*SQ(delta[0][0+order]))/
					 N0_d1;
		if ( delta[0][1] > EPS  &&  delta[0][0] > EPS ) {
if(steve)
 fprintf(stderr,"b %g %g\n",delta[0][0+order],delta[0][(1+order)%2]);
			if ( delta[0][(1+order)%2] < VMag(a) ) {
				return 1;
			} else {
				return 0;
			}
		} else {
			return 0;
		}
	} else {
		double C[5];	/* the coef of a quartic poly; C[0] is the
				   coef of x^4, and should be 1 */
		double r[4];	/* the roots of C */
		int nr;		/* the number of roots */
		int nd;		/* the number of valid deltas */
		int i;
		
		C[0] = 27./8. * k0 * SQ(k1);
		C[1] = 0.;
		C[2] = 9./2. * k0 * k1 * N1_a;
		C[3] = N0_d1 * SQ(N1_d0);
		C[4] = -N0_a * SQ(N1_d0) + 3./2.*k0*SQ(N1_a);
		
		/* We divide by C[0] to normalize the coefficients */
		for(i=1; i<=4; i++) {
			C[i] /= C[0];
		}
if(steve)
fprintf(stderr,"CD d\n");
if(steve)
fprintf(stderr,"N0_a %g, N1_a %g, N0_d1 %g, N1_d0 %g order = %d\n",
	N0_a, N1_a, N0_d1, N1_d0, order);
if(steve)
fprintf(stderr,"%g %g %g %g\n",C[1],C[2],C[3],C[4]);
		/* We don't pass C[0] as QuarticRoots expects it to be 1 */
		QuarticRoots(C+1,&nr,r);
		
		/* only return non-negative pairs; note that the
		   deltas may need to be flipped */
		nd = 0;
if(steve)
fprintf(stderr,"C = %g %g\n",k0,k1);
		for(i=0; i<nr; i++) {
			if ( r[i] > 0. ) {
				delta[nd][(1+order)%2] = r[i];
				delta[nd][0+order] = 
				  (N1_a + 3./2.*r[i]*r[i]*k1)/N1_d0;
if(steve)
fprintf(stderr," ++ %g %g\n",delta[nd][0+order],delta[nd][(1+order)%2]);
				/* invalidate deltas if negative or
				   too long in case of "line." */
				if (delta[nd][0+order] > 0.  &&
				    !( fabs(k0) < EPS  &&  fabs(k1) < EPS  &&
				      (delta[nd][0]+delta[nd][1] > 
				       PPDist(p0,p1)))) {
					nd++;
				}
			}
else
if(steve)
fprintf(stderr," ++ %g\n",r[i]);
		}
		return nd;
	}
}


/*
 *----------------------------------------------------------------------
 *  Function:  SelectDelta
 *----------------------------------------------------------------------
 */
SelectDelta(Scalar delta[3][2], int nd, Scalar* delta0, Scalar* delta1)
{
	Scalar rw, rc;
	int i;
	int which;

if(steve){
fprintf(stderr,"SelectDelta: %d\n",nd);
}
	rw = 0.0;
	which = -1;
	for(i=0;i<nd;i++){
		rc= fabs(delta[i][0]/delta[i][1]);
		if ( rc > 1.0 ){
			rc = 1.0/rc;
		}
if(steve)
fprintf(stderr,"%d %g/%g %g\n",i,delta[i][0],delta[i][1],rc);
		if ( rc > rw ){
if(steve)
fprintf(stderr,"%d looks good\n",i);
			which = i;
			rw = rc;
		}
	}
	if ( which >= 0 ) {
if(steve)
fprintf(stderr,"We'll try %d\n",which);
		*delta0 = delta[which][0];
		*delta1 = delta[which][1];
if(steve)
fprintf(stderr,"SelectDelta: %g %g\n",*delta0,*delta1);
	} else {
		fprintf(stderr,"SelectDelta: none selected (%d)(%d).  Exiting.\n",
			nd,which);
		for(i=0;i<nd;i++){
			fprintf(stderr,"%g %g\n",delta[i][0],delta[i][1]);
		}
		exit(1);
	}
} 


/*
 *----------------------------------------------------------------------
 *  Function:  dBHSM
 *	Given two vertices with normals and SFF's, construct the Bezier
 *	curve interpolating all this data.
 *  Return Value:
 *	0 if construction failed
 *	1 otherwise
 *----------------------------------------------------------------------
 */
int dBHSM(Vertex* v0, Vector t0, Vertex* v1, Vector t1, Point* p1, Point* p2)
{
	SFF sff0,sff1;		/* SFF's at vertices */
	Point q;		/* used to extract frame */
	int rv;			/* number of valid deltas */
	Scalar delta[3][2];	/* deltas */
	Scalar d0,d1;		/* best delta pair */
	Scalar k0,k1;
	
	q = ReturnUDPoint(v0);
	
	if ( !dGetSFF(v0->externalData,"Point.sff", 
		      StdFrame(SpaceOf(q)),&sff0)){
		fprintf(stderr,"dBHSM: vertex missing SFF.  Exiting.\n");
		exit(1);
	}
	
	if ( !dGetSFF(v1->externalData,"Point.sff", 
		      StdFrame(SpaceOf(q)),&sff1)){
		fprintf(stderr,"dBHSM: vertex missing SFF.  Exiting.\n");
		exit(1);
	}

	t0 = VNormalize(t0);
	t1 = VNormalize(t1);
	
	k0 = EvalSFF(&sff0, t0);
	k1 = EvalSFF(&sff1, t1);
	
	rv = ComputeDeltas(ReturnUDPoint(v0),ReturnUDNormal(v0),t0,k0,
			   ReturnUDPoint(v1),ReturnUDNormal(v1),t1,k1,delta);

	if ( rv != 0 ) {
		SelectDelta(delta,rv,&d0,&d1);
		*p1 = PVAdd(ReturnUDPoint(v0),SVMult(d0,t0));
		*p2 = PVAdd(ReturnUDPoint(v1),SVMult(-d1,t1));
	}
	
	return rv;
}
