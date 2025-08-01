/*****************************************************************/
/* Module: aputil.h                                              */
/* $Date: 1999/10/25 22:36:21 $                                  */
/* $Revision: 1.3 $                                              */
/* $Source: /p/SurfaceFitting/SF/src/Apps/ATess/RCS/aputil.h,v $ */
/*****************************************************************/
/*
** Purpose: 
*/

#ifndef _APUTIL_H_
#define _APUTIL_H_

#include "material.h"
#include "vertex.h"

void clearBuffer(char* buf, int size);
void GaussianSolve3x3(double a[3][3], double* b, double* x);
void OutputS3dMaterial(FILE* fp, Material m);
void OutputS3dVertex(FILE* fp, VERTEX v, int nspecified);
void OutputS3dMaterialVertex(FILE* fp, Material m, VERTEX v, int nspecified);
void OutputS3dPolygonTriangle(FILE* fp, VERTEX v1, VERTEX v2, VERTEX v3,
			      int nspecified);
void OutputS3dVMatPolygonTriangle(FILE* fp, Material m1, VERTEX v1,
				  Material m2, VERTEX v2, Material m3,
				  VERTEX v3, int nspecified);
void OutputS3dPolylineTriangle(FILE* fp, VERTEX v1, VERTEX v2, VERTEX v3,
			       int nspecified);
void OutputS3dVMatPolylineTriangle(FILE* fp, Material m1, VERTEX v1,
				   Material m2, VERTEX v2, Material m3,
				   VERTEX v3, int nspecified);

#endif /* _APUTIL_H_ */
