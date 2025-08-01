/***********************************************************************/
/* Module: aptessellate.h                                              */
/* $Date: 1999/10/25 22:35:16 $                                        */
/* $Revision: 1.5 $                                                    */
/* $Source: /p/SurfaceFitting/SF/src/Apps/ATess/RCS/aptessellate.h,v $ */
/***********************************************************************/
/*
** Purpose: 
*/

#ifndef _APTESS_H_
#define _APTESS_H_

#include "vertex.h"
#include "material.h"
#include "apatch.h"

typedef struct {
   int esamples;
   int size;
   VERTEX* netV;
} FaceSampleNet;

typedef struct {
   int esamples;
   int size;
   VERTEX* zeroEV;
   VERTEX* nonzEV;
   Scalar* zEVVal;
   Scalar* nzEVVal;
} EdgeSampleNet;


void TessellateApatch(Apatch apatch);
void TessellateFaceApatch(Apatch apatch, int apVinNet[4], int apFaceVi[3],
                          int apTopVi);
void TessellateEdgeApatch(Apatch apatch, int apVinNet[4], int apZeVi[2],
                          int apNZeVi[2]);
void TessellateApatchPair(ApatchPair apPair);
void OutputS3dSampledFaceApatch(FILE* fp, VERTEX* sampledApatch, int apColor,
                                int sampleDeg, int sampleDim, int nspecified);
void OutputS3dSampledEdgeApatch(FILE* fp, VERTEX* sampledApatch, int apColor,
                                int size, int nspecified);
void OutputS3dApatchScalarNet(FILE* fp, Apatch apatch);
void OutputS3dApatchPairScalarNet(FILE* fp, ApatchPair apPair);

#endif /* _APTESS_H_ */




