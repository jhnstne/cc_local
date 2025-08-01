/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */


#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "util.h"
#include "vertex.h"
#include "dstruct.h"
#include "material.h"
#include "patch.h"
#include "sff.h"

SFF PatchEvalSFF(Patch patch, Scalar r, Scalar s, Scalar t)
{
	SFF sff;
	Point pt;
	Normal n;

	PatchEvalWithNormal(patch, &pt, &n, r,s,t);
	n = VDual(VNormalize(NDual(n)));
	sff.v0 = PatchDerivEval(patch, r,s,t, -1., 0., 1.);
	sff.m[0][0] = NVApply(n, PatchDeriv2Eval(patch, r,s,t, 
						 -1.,0.,1., -1.,0.,1.));
	sff.v1 = PatchDerivEval(patch, r,s,t, -1., 1., 0.);
	sff.m[1][1] = NVApply(n, PatchDeriv2Eval(patch, r,s,t, 
					       -1.,1.,0., -1.,1.,0.));
	sff.m[0][1] = sff.m[1][0] = 
	  	NVApply(n, PatchDeriv2Eval(patch, r,s,t, 
					   -1., 0., 1., -1., 1., 0.));

	return sff;
}
