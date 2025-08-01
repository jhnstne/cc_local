/*
 *----------------------------------------------------------------------
 *  File:  bezier.h
 *----------------------------------------------------------------------
 */

#ifndef _BEZIER_H_
#define _BEZIER_H_

#include "geometry.h"

Point EvalBezier(int n, Point pt[], Scalar t);
Vector EvalBezierDeriv(int n, Point pt[],Scalar t);
Vector EvalBezierNthDeriv(int n, Point pt[], Scalar t, int deriv);
Scalar BezierCurvature(int n, Point p[],Scalar t);
Scalar ScalarBezier(int n, Scalar w[], Scalar t);
Scalar ScalarBezierDeriv(int n, Scalar w[], Scalar t);
Point RatBezier(int n, Point p[], Scalar w[], Scalar t);
Vector RatBezierDeriv(int n, Point p[], Scalar w[], Scalar t);

#endif /* _BEZIER_H_ */
