/***********************************************************************/
/* Module: apconstruct.h                                               */
/* $Date: 1999/01/13 04:21:17 $                                        */
/* $Revision: 1.5 $                                                    */
/* $Source: /p/SurfaceFitting/SF/src/Apps/ATess/RCS/apconstruct.h,v $  */
/***********************************************************************/
/*
** Purpose:.
*/

#ifndef _APCONSTRUCT_H_
#define _APCONSTRUCT_H_

#include "vertex.h"
#include "apatch.h"

enum {
   ZERO_CONVEX = 0,
   POS_CONVEX  = 1,
   NEG_CONVEX  = 2,
   NON_CONVEX  = 3
};

enum {
   AVERAGING                    = 0,
   SIMPLE_INTERPOLATION         = 1,
   QUAD_PRECISION_INTERPOLATION = 2
};

void ConstructFaceApatches(Face* apFace, Space wspace);
void ConstructEdgeApatches(Edge* tedge, Space wspace);
void OutputS3dFaceApatchSimplex(FILE* fp, Apatch apatch);
void OutputS3dFaceApatchPairSimplex(FILE* fp, ApatchPair apPair);
void OutputS3dEdgeApatchSimplex(FILE* fp, Apatch apatch);

#endif /* _APCONSTRUCT_H_ */

