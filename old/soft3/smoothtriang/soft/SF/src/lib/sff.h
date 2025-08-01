/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  sff.h
 *----------------------------------------------------------------------
 */

#ifndef _SFF_H_
#define _SFF_H_

#include "geometry.h"
#include "dstruct.h"

typedef struct {
  Scalar m[2][2];
  Vector v0,v1;
} SFF;

Scalar EvalSFF(SFF *sff, Vector v);
Scalar EvalSFF2(SFF *sff, Vector v1, Vector v2);
SFF CreateConjugateSFF(Vector v0, Vector v1, Scalar k0, Scalar k1);
Vector SFFConjugate(SFF* s, Vector v);
Scalar SFFGaussianCurvature(SFF* sff);
Scalar SFFMeanCurvature(SFF* sff);
SFF ComputeOrthonormalSFF(SFF sff);
BOOLEAN dGetSFF(Lnode* ds, char* path, Frame f, SFF* sff);
BOOLEAN pPutSFF(Lnode** ds, char* path, SFF* sff);
void ComputePrincipalVectors(SFF sff, Vector* v1, Vector* v2, 
			     Scalar* s0, Scalar* s1);

#endif /* _SFF_H_ */
