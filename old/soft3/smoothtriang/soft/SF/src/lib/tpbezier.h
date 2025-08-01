/*
 *----------------------------------------------------------------------
 *  File:  tpbezier.h
 *----------------------------------------------------------------------
 */

#ifndef _TPBEZIER_H_
#define _TPBEZIER_H_

#include "geometry.h"

typedef struct {
  Space range;
  int degreeU, degreeV;
  Point *net;
} TPBezier;


TPBezier TPBezierCreate(int degreeU, int degreeV, Space range);
void TPBezierFree(TPBezier patch);

int TPBezierRead(TPBezier *patch, Space range);
void TPBezierWrite(TPBezier patch);

Point TPBezierGetPoint(TPBezier patch, int i1,int i2);
void TPBezierSetPoint(TPBezier patch, Point pt, int i1, int i2);

Point TPBezierEval(TPBezier patch, Scalar u, Scalar v);
void TPBezierEvalWithNormal(TPBezier patch, Point *point, Normal *normal, 
			    Scalar u, Scalar v);

int GetTPBezier(TPBezier *patch, Space range);

void TPBezierEvalWithSecondOrder(TPBezier patch, Point *point, 
				 Scalar u, Scalar v,
				 Vector *du, Vector *dv,
				 Vector *duu, Vector *duv, Vector *dvv);

void TPBezierEvalWithNormalAndGK(TPBezier patch, Point *point, Normal *normal, 
				 Scalar *gk, Scalar u, Scalar v);

void TPBezierEvalWithNormalAndSFF(TPBezier patch, Point *point, Normal *normal,
				  SFF* sff, Scalar u, Scalar v);

#endif /* _TPBEZIER_H_ */
