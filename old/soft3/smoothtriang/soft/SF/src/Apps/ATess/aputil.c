/******************************************************************/
/* Module: aputil.c                                               */
/* $Date: 1999/10/25 22:36:08 $                                   */
/* $Revision: 1.4 $                                               */
/* $Source: /p/SurfaceFitting/SF/src/Apps/ATess/RCS/aputil.c,v $  */
/******************************************************************/
/*
** Purpose: 
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "util.h"
#include "material.h"
#include "geometry.h"
#include "vertex.h"
#include "aputil.h"

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void clearBuffer(char* buf, int size)
{
   int i;

   for (i = 0; i < size; i++) {
      buf[i] = ' ';
   }
}

/* -------------------------------------------------------------------- *
 * Solves the 3-by-3 system of equations a*x = b.                       *
 * -------------------------------------------------------------------- */
void GaussianSolve3x3(double a[3][3], double* b, double* x)
{
   double pvt, tmp;
   int i, j, k, ipvt;
   int* pivot;
   const int n = 3;

   pivot = (int*) malloc(sizeof(int) * n);

   for (j = 0; j < n-1; j++) {
      pvt = fabs(a[j][j]);
      ipvt = j;
      pivot[j] = j;

      for (i = j+1; i < n; i++) {
         if (fabs(a[i][j]) > pvt) {
            pvt = fabs(a[i][j]);
            ipvt = i;
         }
      }

      if (pivot[j] != ipvt) {
         pivot[j] = ipvt;
         pivot[ipvt] = j;
         for (k = 0; k < n; k++) {
            tmp = a[j][k];
            a[j][k] = a[pivot[j]][k];
            a[pivot[j]][k] = tmp;
         }
         tmp = b[j];
         b[j] = b[pivot[j]];
         b[pivot[j]] = tmp;
      }

      for (i = j+1; i < n; i++) {
         if (a[j][j] == 0) fprintf(stderr, "Error: Divide by zero.\n");
         tmp = a[i][j] / a[j][j];
         for (k = j; k < n; k++) {
            a[i][k] = a[i][k] - (tmp * a[j][k]);
         }
         b[i] = b[i] - (tmp * b[j]);
      }
      /*
      for (i = j+1; i < n; i++) {
         if (a[j][j] == 0) fprintf(stderr, "Error: Divide by zero.\n");
         a[i][j] = a[i][j] / a[j][j];
      }

      for (i = j+1; i < n; i++) {
         for (k = j+1; k < n; k++) {
            a[i][k] = a[i][k] - (a[i][j] * a[j][k]);
         }
         b[i] = b[i] - (a[i][j] * b[j]);
      }
      */
   }

   /* Check for case of homogenous equation */
   /* Set x[n-1], a free parameter, to be 1 for now */
   if (a[n-1][n-1] == 0)
      x[n-1] = 1;
   else
      x[n-1] = b[n-1] / a[n-1][n-1];
   for (j = n-2; j >= 0; j--) {
      x[j] = b[j];
      for (k = n-1; k >= j+1; k--) {
         x[j] = x[j] - (x[k] * a[j][k]);
      }
      x[j] = x[j] / a[j][j];
   }
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dMaterial(FILE* fp, Material m)
{
   fprintf(fp, "d %g %g %g\n", m.diffuse.r, m.diffuse.g, m.diffuse.b);
}

/* -------------------------------------------------------------------- *
 * nspecified: 0 implies normal is not specified.                       *
 * -------------------------------------------------------------------- */
void OutputS3dVertex(FILE* fp, VERTEX v, int nspecified)
{
   Scalar x,y,z;
   Frame f;

   f = StdFrame(SpaceOf(v.position));
   if (nspecified) {
      NCoords(v.normal, f, &x, &y, &z);
      fprintf(fp, "n %lg %lg %lg\n", x, y, z);
   }
   PCoords(v.position, f, &x, &y, &z);
   fprintf(fp, "v %lg %lg %lg\n", x, y, z);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dMaterialVertex(FILE* fp, Material m, VERTEX v, int nspecified)
{
   OutputS3dMaterial(fp, m);
   OutputS3dVertex(fp, v, nspecified);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dPolygonTriangle(FILE* fp, VERTEX v1, VERTEX v2, VERTEX v3,
			      int nspecified)
{
   fprintf(fp, "P 3 0 0\n");
   OutputS3dVertex(fp, v1, nspecified);
   OutputS3dVertex(fp, v2, nspecified);
   OutputS3dVertex(fp, v3, nspecified);
   fprintf(fp, "E 0 0 0\n");
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dVMatPolygonTriangle(FILE* fp, Material m1, VERTEX v1,
				  Material m2, VERTEX v2, Material m3,
				  VERTEX v3, int nspecified)
{
   fprintf(fp, "P 3 0 0\n");
   OutputS3dMaterialVertex(fp, m1, v1, nspecified);
   OutputS3dMaterialVertex(fp, m2, v2, nspecified);
   OutputS3dMaterialVertex(fp, m3, v3, nspecified);
   fprintf(fp, "E 0 0 0\n");
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dPolylineTriangle(FILE* fp, VERTEX v1, VERTEX v2, VERTEX v3,
			       int nspecified)
{
   fprintf(fp, "L 3 3 0\n");
   OutputS3dVertex(fp, v1, nspecified);
   OutputS3dVertex(fp, v2, nspecified);
   OutputS3dVertex(fp, v3, nspecified);
   fprintf(fp, "E 0 0 0\n");
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dVMatPolylineTriangle(FILE* fp, Material m1, VERTEX v1,
				   Material m2, VERTEX v2, Material m3,
				   VERTEX v3, int nspecified)
{
   fprintf(fp, "L 3 3 0\n");
   OutputS3dMaterialVertex(fp, m1, v1, nspecified);
   OutputS3dMaterialVertex(fp, m2, v2, nspecified);
   OutputS3dMaterialVertex(fp, m3, v3, nspecified);
   fprintf(fp, "E 0 0 0\n");
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
