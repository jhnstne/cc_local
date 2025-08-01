/*
 *----------------------------------------------------------------------
 *  File:  tptess.h
 *----------------------------------------------------------------------
 */

#ifndef _TPTESS_H_
#define _TPTESS_H_

#include "geometry.h"

void SetTPTessEps(double e);

Vector FirstTPDer(void* patch, Point (*eval)(), Scalar b[3], int dir);
Normal CalcTPNormal(void* patch, Point (*eval)(), Scalar u, Scalar v);
Vector SecondTPDer(void* patch, Point (*eval)(), Scalar b[3], 
		   int dir, int dir2);
SFF TPSff(void* patch, Point (*eval)(), Scalar u, Scalar v);

void TessTPPatch(void* patch, Point (*eval)(), int samples, Frame frame);

#endif /* _TPTESS_H_ */
