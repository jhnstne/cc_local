/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include <stdio.h>
#include "all.h"
#include "curve.h"
#include "math.h"



Point EvalCurve(Curve *c, double t)
{
	return EvalBezier(c->degree, c->cp, t);
}


static Vector EvalFirstDerivBezier(Curve *c, double t)
{
	Point cpt[MAX];
	int i,j;
	
	for(i=0;i<=c->degree;i++){
		cpt[i] = c->cp[i];
	}
	for(i=1;i<=c->degree-1;i++){
		for(j=0;j<=c->degree-i;j++){
			cpt[j] = PPac(cpt[j],cpt[j+1],1-t);
		}
	}
	return SVMult((Scalar)c->degree,PPDiff(cpt[1],cpt[0]));
}

static Vector EvalFirstDerivative(Curve *c, double t)
{
	return EvalFirstDerivBezier(c,t);
}

static Vector EvalSecondDerivBezier(Curve *c, double t)
{
	Point cpt[MAX];
	int i,j;
	Scalar s[3];
	
	for(i=0;i<=c->degree;i++){
		cpt[i] = c->cp[i];
	}
	for(i=1;i<=c->degree-2;i++){
		for(j=0;j<=c->degree-i;j++){
			cpt[j] = PPac(cpt[j],cpt[j+1],1-t);
		}
	}
	s[0] = 1.0; s[1] = -2.0; s[2] = 1.0;
	return SVMult((Scalar)(c->degree*(c->degree-1)),PPvcN(3,cpt,s));
}

static Vector EvalSecondDerivative(Curve *c, double t)
{
	return EvalSecondDerivBezier(c,t);
}

Scalar Curvature(Curve* c, Scalar t)
{
	Vector fd, sd;
	Scalar fx,fy, sx,sy;
	double flen;
	
	fd = EvalFirstDerivative(c,t);
	sd = EvalSecondDerivative(c,t);
	if ( Dim(SpaceOf(fd)) == 2 ) {
		VCoords(fd,StdFrame(SpaceOf(fd)),&fx,&fy);
		VCoords(sd,StdFrame(SpaceOf(sd)),&sx,&sy);
		flen = VMag(fd);
#if 0
fprintf(stderr,"t = %g,  fx %g, fy %g,  sx %g, sy %g,  flen %g\n",
t,fx,fy,sx,sy,flen);
#endif
		return fabs(( fx*sy - sx*fy ) / (flen*flen*flen));
	} else {
		return VMag(VVCross(fd,sd))/(VMag(fd)*VMag(fd)*VMag(fd));
	}
}
