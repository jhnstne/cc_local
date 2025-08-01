/*
 *----------------------------------------------------------------------
 *  Filee:  tess.c
 *----------------------------------------------------------------------
 */
#ifndef _TESS_C_
#define _TESS_C_

#include "geometry.h"
#include "material.h"
#include "patch.h"

void SetTessEps(double e)
Normal CalcNormal(void* patch, Point (*eval)(Patch*, Scalar[]), 
		  int i, int j, int k)
void TessPatch(void* patch, Point (*eval)(Patch*, Scalar[]), int samples, Frame frame)
Scalar EstGaussianCurvature(void* patch, Point (*eval)(Patch*, Scalar[]), 
			    int i, int j, int k, Normal normal)
void gkTessPatch(void* patch, Point (*eval)(Patch*, Scalar[]), int samples,Frame frame)
Vector FirstDer(void* patch, Point (*eval)(Patch*, Scalar[]), Scalar b[3], int dir)
Vector SecondDer(void* patch, Point (*eval)(Patch*, Scalar[]), 
		 Scalar b[3], int dir1, int dir2)
void CompCurvature(void* patch, Point (*eval)(Patch*, Scalar[]), 
		   Scalar b0, Scalar b1, Scalar b2)

#endif /* _TESS_C_ */
