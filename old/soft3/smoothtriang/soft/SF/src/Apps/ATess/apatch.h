/******************************************************************/
/* Module: apatch.h                                               */
/* $Date: 1999/10/25 22:34:17 $                                   */
/* $Revision: 1.5 $                                               */
/* $Source: /p/SurfaceFitting/SF/src/Apps/ATess/RCS/apatch.h,v $  */
/******************************************************************/
/*
** Purpose: Interface module for A-patches.
*/

#ifndef _APATCH_H_
#define _APATCH_H_

#include "vertex.h"
#include "material.h"

extern Material
   APColor1,
   APColor2,
   APColor3;

enum {FACE_COLOR = 0, EDGE1_COLOR = 1, EDGE2_COLOR = 2};
enum {RIGHT_ORIENTED = 0, LEFT_ORIENTED = 1};

typedef struct {
   int degree;             /* The degree of the patch. */
   VERTEX tetrahedron[4];
   Space wspace;           /* Containing space of control vertices.*/
   int netSize;
   Scalar* net;		   /* The control net. */
   Material* netMat;
   Frame apFrame;
   int apColor;
} Apatch;

typedef struct {
   Apatch ap1;
   Apatch ap2;
} ApatchPair;

int ReadSingleApatch(Apatch* apatch, Space wspace);
int ReadApatchPair(ApatchPair* apPair, Space wspace);
Apatch CreateApatch(int degree, VERTEX* tetra, Space wspace, int color);
void WriteSingleApatch(Apatch apatch);
void WriteApatchPair(ApatchPair apPair);
void FreeApatch(Apatch apatch);
void FreeApatchPair(ApatchPair apPair);
int NetSize(int deg, int dim);
int CPIndex(int deg, int dim, int mi[]);
int CPIndexDim3(int deg, int i0, int i1, int i2, int i3);
Scalar EvalApatch(Apatch apatch, VERTEX* v);
Scalar EvalApatchGivenBCoords(Apatch apatch, VERTEX* v, Scalar u0,
                              Scalar u1, Scalar u2, Scalar u3);
Normal EvalApatchNormal(Apatch apatch, Scalar i0, Scalar i1, Scalar i2,
                        Scalar i3);
int CheckApatchOrientation(Apatch apatch);
void ConvertApatchToRightHandedOrientation(Apatch* apatch);
Material GetApatchMaterial(int apColor);

#endif /* _APATCH_H_ */

