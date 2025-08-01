/**********************************************************************/
/* Module: apconstruct.c                                              */
/* $Date: 1999/10/25 22:34:40 $                                       */
/* $Revision: 1.7 $                                                   */
/* $Source: /p/SurfaceFitting/SF/src/Apps/ATess/RCS/apconstruct.c,v $ */
/**********************************************************************/
/*
** Purpose:.
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "geometry.h"
#include "vertex.h"
#include "dstruct.h"
#include "getset.h"
#include "material.h"
#include "libgeo.h"
#include "mesh.h"
#include "userData.h"
#include "usage.h"
#include "apconstruct.h"
#include "aputil.h"
#include "apatch.h"

#define EPS 1.0e-8

extern int ApatchDeg;
extern double LenN1FP, LenN2FP, LenN3FP;
extern double SimplexFP;
extern double FaceTopScalarFP;
extern int FPSelectionMethod;
static double eps = EPS;

/* Function Prototypes */
int TypeOfFace(Face* tface);
void ConstructSingleFaceApatchSimplex(Apatch* apatch, Face* tface,
				      int faceType, Space wspace);
void ConstructZeroConvexFaceApatch(Apatch* apatch, Face* tface,
				   int aboveOrBelow, Space wspace);
void ConstructFaceApatchPairSimplex(ApatchPair* apPair, Face* tface,
				    Space wspace);
void FitSingleCubicFaceApatch(Apatch* apatch, Face* apFace, int faceType,
			      Space wspace);
void GenerateCFAPScalarsFromNormals(Apatch* apatch);
void SetCFAPFirstFreeScalarByFarinDegElevation(Apatch* apatch);
void SetCFAPRemainingFreeScalarsByAvg(Apatch* apatch, int sset[3]);
void SetCFAPRemainingFreeScalarsBySimpleInterpolation(Apatch* apatch,
						      int sset[3]);
void SetCFAPRemainingFreeScalarsByQuadPrecisionInterpolation(Apatch* apatch,
							     int sset[3]);
void GenerateCFAPScalarsFromConvexConvexFaceDependency(Apatch* fap1,
						       Apatch* fap2,
						       int sset[3]);
void ConstructConvexConvexEdgeApatchSimplex(Apatch* eap1, Apatch* eap2, 
					    Apatch* f1ap, Apatch* f2ap,
					    Edge* tedge, Face* f1, Face* f2,
					    int f1type, int f2type, 
					    Space wspace);
void FitConvexConvexCubicEdgeApatches(Apatch* eap1, Apatch* eap2,
				      Apatch* f1ap, Apatch* f2ap,
				      Face* f1, Face* f2, 
				      int f1type, int f2type, Space wspace);

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void ConstructFaceApatches(Face* apFace, Space wspace)
{
   int faceType;

   faceType = TypeOfFace(apFace);
   /* fprintf(stderr, "type of face:%d, %s\n", faceType,ReturnName(apFace)); */
   if (faceType == ZERO_CONVEX) {
      Apatch apatch;
      ConstructZeroConvexFaceApatch(&apatch, apFace, 0, wspace);
      if (ApatchDeg == 0)
         OutputS3dFaceApatchSimplex(stdout, apatch);
      else
         WriteSingleApatch(apatch);
      FreeApatch(apatch);
   }
   else if (faceType == NON_CONVEX) {
#if 0
      ApatchPair apPair;
      ConstructFaceApatchPairSimplex(&apPair, apFace, wspace);
      if (ApatchDeg == 0) {
         OutputS3dFaceApatchPairSimplex(stdout, apPair);
      }
      else {
         FitCubicFaceApatchPair(&apPair);
         WriteApatchPair(apPair);
      }
      FreeApatchPair(apPair);
#endif
   }
   else {
      Apatch apatch;
      ConstructSingleFaceApatchSimplex(&apatch, apFace, faceType, wspace);
      if (ApatchDeg == 0) {
         OutputS3dFaceApatchSimplex(stdout, apatch);
      }
      else {
         FitSingleCubicFaceApatch(&apatch, apFace, faceType, wspace);
         WriteSingleApatch(apatch);
      }
      FreeApatch(apatch);
   }
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void ConstructEdgeApatches(Edge* tedge, Space wspace)
{
   Face* f1;
   Face* f2;
   int f1type, f2type;

   GetEdgeFaces(tedge, &f1, &f2);
   f1type = TypeOfFace(f1);
   f2type = TypeOfFace(f2);

   if (f1type != NON_CONVEX && f2type != NON_CONVEX) {
      Apatch f1ap, f2ap;
      if ((f1type != POS_CONVEX && f2type != POS_CONVEX) ||
          (f1type != NEG_CONVEX && f2type != NEG_CONVEX)) {
         Apatch eap1, eap2;
         ConstructConvexConvexEdgeApatchSimplex(&eap1, &eap2, &f1ap, &f2ap,
						tedge, f1, f2, f1type, f2type,
						wspace);
         if (ApatchDeg == 0) {
            OutputS3dEdgeApatchSimplex(stdout, eap1);
            OutputS3dEdgeApatchSimplex(stdout, eap2);
         }
         else {
            FitConvexConvexCubicEdgeApatches(&eap1, &eap2, &f1ap, &f2ap,
					     f1, f2, f1type, f2type, wspace);
            WriteSingleApatch(eap1);
            WriteSingleApatch(eap2);
         }
         FreeApatch(eap1);
         FreeApatch(eap2);
         FreeApatch(f1ap);
         FreeApatch(f2ap);
      }
   }

#if 0
   if (f1type == NON_CONVEX && f2type == NON_CONVEX) {
      /* Both adjacent faces are non-convex. */
      Apatch eap1, eap2, eap3, eap4;
      ApatchPair f1app, f2app;
      ConstructNonconvexNonconvexEdgeApatchSimplex(&eap1, &eap2, &eap3, &eap4,
						   &f1app, &f2app, tedge);
      if (ApatchDeg == 0) {
         OutputS3dEdgeApatchSimplex(stdout, eap1);
         OutputS3dEdgeApatchSimplex(stdout, eap2);
         OutputS3dEdgeApatchSimplex(stdout, eap3);
         OutputS3dEdgeApatchSimplex(stdout, eap4);
      }
      else {
         FitNonconvexNonconvexCubicEdgeApatches(&eap1, &eap2, &eap3, &eap4,
						&f1app, &f2app);
         WriteSingleApatch(eap1);
         WriteSingleApatch(eap2);
         WriteSingleApatch(eap3);
         WriteSingleApatch(eap4);
      }
      FreeApatch(eap1);
      FreeApatch(eap2);
      FreeApatch(eap3);
      FreeApatch(eap4);
      FreeApatchPair(f1app);
      FreeApatchPair(f2app);
   }
   else if (f1type != NON_CONVEX && f2type != NON_CONVEX) {
      Apatch f1ap, f2ap;
      /* Both adjacent faces are convex. */
      if (f1type == ZERO_CONVEX && f2type == ZERO_CONVEX) {
	 /* Both adjacent faces are zero-convex.            */
	 /* In this case the surface is defined directly by */
	 /* the edge's adjacent faces of the triangulation. */
         /* No edge tetrahedra need to be constructed.      */
      }
      else if ((f1type != POS_CONVEX && f2type != POS_CONVEX) ||
               (f1type != NEG_CONVEX && f2type != NEG_CONVEX)) {
         /* Both adjacent faces are non-positive or non-negative convex. */
         Apatch eap1, eap2;
         ConstructConvexConvexEdgeApatchSimplex(&eap1, &eap2, &f1ap, &f2ap,
						tedge, f1, f2, f1type, f2type,
						wspace);
         if (ApatchDeg == 0) {
            OutputS3dEdgeApatchSimplex(stdout, eap1);
            OutputS3dEdgeApatchSimplex(stdout, eap2);
         }
         else {
            FitConvexConvexCubicEdgeApatches(&eap1, &eap2, &f1ap, &f2ap,
					     f1, f2, f1type, f2type, wspace);
            WriteSingleApatch(eap1);
            WriteSingleApatch(eap2);
         }
         FreeApatch(eap1);
         FreeApatch(eap2);
      }
      else {
         /* One adjacent face is positive convex, and one negative convex. */
         Apatch eap1, eap2, fap3;
         ConstructPosConvexNegConvexEdgeApatchSimplex(&eap1, &eap2, &fap3,
						      &f1ap, &f2ap, tedge,
						      f1type, f2type);
	 /*
         if (ApatchDeg == 0) {
            OutputS3dEdgeApatchSimplex(stdout, eap1);
            OutputS3dEdgeApatchSimplex(stdout, eap2);
            OutputS3dFaceApatchSimplex(stdout, fap3);
         }
         else {
            FitPosConvexNegConvexCubicEdgeApatches(&eap1, &eap2, &fap3,
						   &f1ap, &f2ap);
            WriteSingleApatch(eap1);
            WriteSingleApatch(eap2);
            WriteSingleApatch(fap3);
         }
	 */
         FreeApatch(eap1);
         FreeApatch(eap2);
         FreeApatch(fap3);
      }
      FreeApatch(f1ap);
      FreeApatch(f2ap);
   }
   else {
      /* One adjacent face is convex and the other is non-convex. */
      Apatch eap1, eap2;
      Apatch fap;
      ApatchPair fapPair;
      ConstructConvexNonconvexEdgeApatchSimplex(&eap1, &eap2, &fap, &fapPair,
						tedge, f1type, f2type);
      if (ApatchDeg == 0) {
         OutputS3dEdgeApatchSimplex(stdout, eap1);
         OutputS3dEdgeApatchSimplex(stdout, eap2);
      }
      else {
         FitConvexNonconvexCubicEdgeApatches(&eap1, &eap2, &fap, &fapPair);
         WriteSingleApatch(eap1);
         WriteSingleApatch(eap2);
      }
      FreeApatch(eap1);
      FreeApatch(eap2);
      FreeApatch(fap);
      FreeApatchPair(fapPair);
   }
#endif
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
int TypeOfFace(Face* tface)
{
   Vertex* thisVertex;
   VERTEX faceV[3];
   Scalar n0dotp01, n1dotp10, n1dotp12, n2dotp21, n2dotp20, n0dotp02;
   int i;

   i = 0;
   ForeachFaceVertex(tface, thisVertex) {
      faceV[i].position = ReturnUDPoint(thisVertex);
      faceV[i++].normal = ReturnUDNormal(thisVertex);
   } EndForeach;
   if (i != 3) {
      fprintf(stderr, "Non-triangular face found, exiting.\n");
      exit(1);
   }

   n0dotp01 = NVApply(faceV[0].normal,
	       PPDiff(faceV[1].position, faceV[0].position));
   n1dotp10 = NVApply(faceV[1].normal,
	       PPDiff(faceV[0].position, faceV[1].position));
   n1dotp12 = NVApply(faceV[1].normal,
	       PPDiff(faceV[2].position, faceV[1].position));
   n2dotp21 = NVApply(faceV[2].normal,
	       PPDiff(faceV[1].position, faceV[2].position));
   n2dotp20 = NVApply(faceV[2].normal,
	       PPDiff(faceV[0].position, faceV[2].position));
   n0dotp02 = NVApply(faceV[0].normal,
	       PPDiff(faceV[2].position, faceV[0].position));

   if ((n0dotp01 == 0) && (n1dotp10 == 0) && (n1dotp12 == 0) &&
       (n2dotp21 == 0) && (n2dotp20 == 0) && (n0dotp02 == 0))
      return ZERO_CONVEX;
   else if ((n0dotp01 <= 0) && (n1dotp10 <= 0) && (n1dotp12 <= 0) &&
	    (n2dotp21 <= 0) && (n2dotp20 <= 0) && (n0dotp02 <= 0))
      return POS_CONVEX;
   else if ((n0dotp01 >= 0) && (n1dotp10 >= 0) && (n1dotp12 >= 0) &&
	    (n2dotp21 >= 0) && (n2dotp20 >= 0) && (n0dotp02 >= 0))
      return NEG_CONVEX;
   else
      return NON_CONVEX;
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void ConstructSingleFaceApatchSimplex(Apatch* apatch, Face* tface,
				      int faceType, Space wspace)
{
   Frame f;
   Scalar vnx, vny, vnz, vpx, vpy, vpz, cpx, cpy, cpz;
   Point tmpPt, centroid;
   Normal faceNorm;
   VERTEX tetra[4];
   double d, t[3];
   Vertex* thisVertex;
   int i;

   if (faceType == ZERO_CONVEX) {
      ConstructZeroConvexFaceApatch(apatch, tface, 0, wspace);
      return;
   }

   i = 1;
   ForeachFaceVertex(tface, thisVertex) {
      tetra[i].position = ReturnUDPoint(thisVertex);
      tetra[i++].normal = ReturnUDNormal(thisVertex);
   } EndForeach;

   f = StdFrame(wspace);
   tmpPt = PPrr(tetra[1].position, tetra[2].position, 1, 1);
   centroid = PPrr(tetra[3].position, tmpPt, 2, 1);
   if (faceType == POS_CONVEX) {
      faceNorm = PPPNormal(tetra[1].position, 
                  tetra[2].position, tetra[3].position);
      faceNorm = VDual(VNormalize(NDual(faceNorm)));
   }
   else if (faceType == NEG_CONVEX) {
      faceNorm = PPPNormal(tetra[1].position, 
                  tetra[3].position, tetra[2].position);
      faceNorm = VDual(VNormalize(NDual(faceNorm)));
   }
   else {
      fprintf(stderr, "An Apatch-Pair must be constructed for face %s.\n", ReturnName(tface));
      exit(1);
   }

   /* Get the intersection of the line through the centroid with each	*/
   /* of the vertex tangent planes.  Choose the farthest intersection	*/
   /* point in the direction of the faceNorm so that all tangent planes	*/
   /* are contained within the formed tetrahedron.			*/
   for (i = 1; i < 4; i++) {
      NCoords(tetra[i].normal, f, &vnx, &vny, &vnz);
      PCoords(tetra[i].position, f, &vpx, &vpy, &vpz);
      PCoords(centroid, f, &cpx, &cpy, &cpz);
      d = -1 * (vnx * vpx + vny * vpy + vnz * vpz);
      t[i-1] = -(vnx * cpx + vny * cpy + vnz * cpz + d) /
		(double)(NVApply(tetra[i].normal, NDual(faceNorm)));
   }

   if ((t[0] >= t[1]) && (t[0] >= t[2]))
      i = 0;
   else if ((t[1] >= t[0]) && (t[1] >= t[2]))
      i = 1;
   else
      i = 2;

   tetra[0].position = PVAdd(centroid,
			     SVMult((SimplexFP * t[i]), NDual(faceNorm)));
   *apatch = CreateApatch(ApatchDeg, tetra, wspace, FACE_COLOR);
}

/* -------------------------------------------------------------------- *
 * Construct a tetrahedral simplex over a zero convex face necessary    *
 * for tessellation purposes.                                           *
 *                                                                      *
 * aboveOrBelow : integer taking values 0 or 1.                         *
 *                0 implies that the tetrahedral simplex is constructed *
 *                on the same side of the face as the normals point,    *
 *                and 1 indicates it is constructed on the other side.  *
 * -------------------------------------------------------------------- */
void ConstructZeroConvexFaceApatch(Apatch* apatch, Face* tface,
				   int aboveOrBelow, Space wspace)
{
   Point tmpPt, centroid;
   Normal faceNorm;
   VERTEX tetra[4];
   Vertex* thisVertex;
   int v1index, i;

   i = 1;
   ForeachFaceVertex(tface, thisVertex) {
      tetra[i].position = ReturnUDPoint(thisVertex);
      tetra[i++].normal = ReturnUDNormal(thisVertex);
   } EndForeach;

   tmpPt = PPrr(tetra[1].position, tetra[2].position, 1, 1);
   centroid = PPrr(tetra[3].position, tmpPt, 2, 1);
   if (aboveOrBelow == 0) {
      faceNorm = PPPNormal(tetra[1].position,
			   tetra[2].position, tetra[3].position);
   }
   else {
      faceNorm = PPPNormal(tetra[1].position, 
			   tetra[3].position, tetra[2].position);
   }

   faceNorm = VDual(VNormalize(NDual(faceNorm)));
   tetra[0].position = PVAdd(centroid,
			     SVMult(SimplexFP, NDual(faceNorm)));
   *apatch = CreateApatch(ApatchDeg, tetra, wspace, FACE_COLOR);
   v1index = CPIndexDim3(ApatchDeg, 0, ApatchDeg, 0, 0);
   for (i = 0; i < v1index; i++)
      apatch->net[i] = 1;
   for (i = v1index; i < apatch->netSize; i++)
      apatch->net[i] = 0;
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void ConstructFaceApatchPairSimplex(ApatchPair* apPair, Face* tface,
				    Space wspace)
{
   Frame f;
   Scalar vnx, vny, vnz, vpx, vpy, vpz, cpx, cpy, cpz;
   Point tmpPt, centroid, top, bottom;
   Normal faceNorm;
   VERTEX tetra[4];
   double d, t[3], max, min;
   Vertex* thisVertex;
   int i;

   i = 1;
   ForeachFaceVertex(tface, thisVertex) {
      tetra[i].position = ReturnUDPoint(thisVertex);
      tetra[i++].normal = ReturnUDNormal(thisVertex);
   } EndForeach;

   f = StdFrame(wspace);
   tmpPt = PPrr(tetra[1].position, tetra[2].position, 1, 1);
   centroid = PPrr(tetra[3].position, tmpPt, 2, 1);
   faceNorm = PPPNormal(tetra[1].position, tetra[2].position,
			tetra[3].position);
   faceNorm = VDual(VNormalize(NDual(faceNorm)));

   /* Get the intersection of the line through the centroid with each	*/
   /* of the vertex tangent planes.  Choose the farthest intersection	*/
   /* points, one in the direction of the faceNorm defining the top 	*/
   /* tetrahedron vertex, and the other in the opposite direction for 	*/
   /* the bottom tetrahedron.  This ensures that all the tangent planes	*/
   /* are contained within the pair of tetrahedra formed.		*/
   for (i = 1; i < 4; i++) {
      NCoords(tetra[i].normal, f, &vnx, &vny, &vnz);
      PCoords(tetra[i].position, f, &vpx, &vpy, &vpz);
      PCoords(centroid, f, &cpx, &cpy, &cpz);
      d = -1 * (vnx * vpx + vny * vpy + vnz * vpz);
      t[i-1] = -(vnx * cpx + vny * cpy + vnz * cpz + d) /
		(double)(NVApply(tetra[i].normal, NDual(faceNorm)));
   }

/*
fprintf(stderr, "t[0] %f, t[1] %f, t[2] %f\n", t[0],t[1],t[2]);
NCoords(faceNorm, f, &vnx, &vny, &vnz);
fprintf(stderr, "face norm: %lg %lg %lg\n", vnx, vny, vnz);
fprintf(stderr, "centroid: %lg %lg %lg\n", cpx, cpy, cpz);
*/

   if (t[0] >= t[1]) {
      if (t[0] >= t[2]) {
         max = t[0];
         if (t[1] <= t[2])
            min = t[1];
         else
            min = t[2];
      }
      else {
         max = t[2];
         min = t[1];
      }
   }
   else {
      if (t[1] >= t[2]) {
         max = t[1];
         if (t[0] <= t[2])
            min = t[0];
         else
            min = t[2];
      }
      else {
         max = t[2];
         min = t[0];
      }
   }

   tetra[0].position = PVAdd(centroid, 
			     SVMult((SimplexFP * max), NDual(faceNorm)));
/*
PCoords(tetra[0].position, f, &vpx, &vpy, &vpz);
fprintf(stderr, "top: %lg %lg %lg\n", vpx, vpy, vpz);
*/
   apPair->ap1 = CreateApatch(ApatchDeg, tetra, wspace, FACE_COLOR);

   tetra[0].position = PVAdd(centroid, 
			     SVMult((SimplexFP * min), NDual(faceNorm)));
   apPair->ap2 = CreateApatch(ApatchDeg, tetra, wspace, FACE_COLOR);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void FitSingleCubicFaceApatch(Apatch* apatch, Face* apFace, int faceType,
			      Space wspace)
{
   Face* adjFace;
   int adjFType;
   int i;
   int sset[3]; /* indicates if the apatch scalars 1011, 1101, and 1110      */
                /* respectively are free parameters or set by the adjacent   */
                /* face continuity conditions. A value of zero indicates     */
                /* the scalar is free.                                       */

   GenerateCFAPScalarsFromNormals(apatch);
   SetCFAPFirstFreeScalarByFarinDegElevation(apatch);
   for (i = 0; i < 3; i++)
      sset[i] = 0;

   ForeachFaceFace(apFace, adjFace) {
      adjFType = TypeOfFace(adjFace);
      if (adjFType == NON_CONVEX) {
         ApatchPair adjapp;
         FreeApatchPair(adjapp);
      }
      else {
         Apatch adjap;
         if (adjFType == ZERO_CONVEX) {
            if (faceType == POS_CONVEX)
               ConstructZeroConvexFaceApatch(&adjap, adjFace, 0, wspace);
            else
               ConstructZeroConvexFaceApatch(&adjap, adjFace, 1, wspace);
         }
         else {
            ConstructSingleFaceApatchSimplex(&adjap, adjFace, adjFType,wspace);
            GenerateCFAPScalarsFromNormals(&adjap);
            SetCFAPFirstFreeScalarByFarinDegElevation(&adjap);
         }
         if ((faceType == POS_CONVEX && adjFType == NEG_CONVEX) ||
	     (faceType == NEG_CONVEX && adjFType == POS_CONVEX)) {
         }
         else {
	    GenerateCFAPScalarsFromConvexConvexFaceDependency(apatch, &adjap,
							      sset);
         }
         FreeApatch(adjap);
      }
   } EndForeach;

#if 1
   if (FPSelectionMethod == 0)
      SetCFAPRemainingFreeScalarsByAvg(apatch, sset);
   else if (FPSelectionMethod == 1)
      SetCFAPRemainingFreeScalarsBySimpleInterpolation(apatch, sset);
   else
      SetCFAPRemainingFreeScalarsByQuadPrecisionInterpolation(apatch, sset);
#endif
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void GenerateCFAPScalarsFromNormals(Apatch* apatch)
{
   Vector e[3];
   Scalar a[3], b[3], c[3], d[3];
   double mA[3][3], mX[3], mB[3];
   Scalar n1[3], n2[3], n3[3];
   Frame onframe;
   int i, i0, i1, i2, i3;

   onframe = StdFrame(apatch->wspace);
   for (i = 0; i < 3; i++) {
      e[i] = FV(onframe, i);
      VCoords(e[i], apatch->apFrame, &a[i], &b[i], &c[i]);
      d[i] = 0 - a[i] - b[i] - c[i];
   }

   NCoords(apatch->tetrahedron[1].normal, onframe,
	   &n1[0], &n1[1], &n1[2]);
   NCoords(apatch->tetrahedron[2].normal, onframe,
	   &n2[0], &n2[1], &n2[2]);
   NCoords(apatch->tetrahedron[3].normal, onframe,
	   &n3[0], &n3[1], &n3[2]);

   /* Solving the 3-by-3 system of eqs:  mA*mX = mB.			*/
   /* At vertex 1: f1200, f0210, and f0201 will be obtained.		*/
/* fprintf(stderr, "Solving matrix: \n"); */
   for (i = 0; i < 3; i++) {
      mA[i][0] = a[i];
      mA[i][1] = c[i];
      mA[i][2] = d[i];
      mB[i] = LenN1FP * n1[i];
/* fprintf(stderr,"%lg %lg %lg %lg\n", mA[i][0], mA[i][1], mA[i][2], mB[i]); */
   }
   GaussianSolve3x3(mA, mB, mX);
/* fprintf(stderr, "Result: %lg %lg %lg \n", mX[0], mX[1], mX[2]); */
   i0 = CPIndexDim3(ApatchDeg, 1, 2, 0, 0);
   i1 = CPIndexDim3(ApatchDeg, 0, 3, 0, 0);
   i2 = CPIndexDim3(ApatchDeg, 0, 2, 1, 0);
   i3 = CPIndexDim3(ApatchDeg, 0, 2, 0, 1);
   apatch->net[i0] = mX[0];
   apatch->net[i1] = 0;
   apatch->net[i2] = mX[1];
   apatch->net[i3] = mX[2];

   /* At vertex 2: f1020, f0120, and f0021 will be obtained.		*/
/* fprintf(stderr, "Solving matrix: \n"); */
   for (i = 0; i < 3; i++) {
      mA[i][0] = a[i];
      mA[i][1] = b[i];
      mA[i][2] = d[i];
      mB[i] = LenN2FP * n2[i];
/* fprintf(stderr,"%lg %lg %lg %lg\n", mA[i][0], mA[i][1], mA[i][2], mB[i]); */
   }
   GaussianSolve3x3(mA, mB, mX);
/* fprintf(stderr, "Result: %lg %lg %lg \n", mX[0], mX[1], mX[2]); */
   i0 = CPIndexDim3(ApatchDeg, 1, 0, 2, 0);
   i1 = CPIndexDim3(ApatchDeg, 0, 1, 2, 0);
   i2 = CPIndexDim3(ApatchDeg, 0, 0, 3, 0);
   i3 = CPIndexDim3(ApatchDeg, 0, 0, 2, 1);
   apatch->net[i0] = mX[0];
   apatch->net[i1] = mX[1];
   apatch->net[i2] = 0;
   apatch->net[i3] = mX[2];

   /* At vertex 3: f1002, f0102, and f0012 will be obtained.		*/
/* fprintf(stderr, "Solving matrix: \n");*/
   for (i = 0; i < 3; i++) {
      mA[i][0] = a[i];
      mA[i][1] = b[i];
      mA[i][2] = c[i];
      mB[i] = LenN3FP * n3[i];
/* fprintf(stderr,"%lg %lg %lg %lg\n", mA[i][0], mA[i][1], mA[i][2], mB[i]);*/
   }
   GaussianSolve3x3(mA, mB, mX);
/* fprintf(stderr, "Result: %lg %lg %lg \n", mX[0], mX[1], mX[2]);*/
   i0 = CPIndexDim3(ApatchDeg, 1, 0, 0, 2);
   i1 = CPIndexDim3(ApatchDeg, 0, 1, 0, 2);
   i2 = CPIndexDim3(ApatchDeg, 0, 0, 1, 2);
   i3 = CPIndexDim3(ApatchDeg, 0, 0, 0, 3);
   apatch->net[i0] = mX[0];
   apatch->net[i1] = mX[1];
   apatch->net[i2] = mX[2];
   apatch->net[i3] = 0;
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void SetCFAPFirstFreeScalarByFarinDegElevation(Apatch* apatch)
{
   /* The array index at which to store the generated	                */
   /* control scalars can be determined easily, and are therefore 	*/
   /* referred to directly (the control scalars are stored in reverse	*/
   /* lexographic order within a linear array).				*/

   apatch->net[14] = (1.0 / 4.0) * (apatch->net[11] + apatch->net[12] +
				    apatch->net[13] + apatch->net[15] +
				    apatch->net[17] + apatch->net[18]);
}

/* -------------------------------------------------------------------- *
 * A simple averaging scheme is used for this purpose.
 * -------------------------------------------------------------------- */
void SetCFAPRemainingFreeScalarsByAvg(Apatch* apatch, int sset[3])
{
   /* We need to determine the remaining control scalars which form the	*/
   /* free parameters of our construction; used to control shape of the	*/
   /* fitted Apatch.  The array index at which to store the generated	*/
   /* control scalars can be determined easily, and are therefore 	*/
   /* referred to directly (the control scalars are stored in reverse	*/
   /* lexographic order within a linear array).				*/

   if (sset[0] == 0)
      apatch->net[5] = (1.0 / 2.0) * (apatch->net[4] + apatch->net[7]);
   if (sset[1] == 0)
      apatch->net[6] = (1.0 / 2.0) * (apatch->net[4] + apatch->net[9]);
   if (sset[2] == 0)
      apatch->net[8] = (1.0 / 2.0) * (apatch->net[7] + apatch->net[9]);

   apatch->net[1]  = (1.0 / 3.0) * (apatch->net[4] + apatch->net[5] +
				    apatch->net[6]);
   apatch->net[2]  = (1.0 / 3.0) * (apatch->net[5] + apatch->net[7] +
				    apatch->net[8]);
   apatch->net[3]  = (1.0 / 3.0) * (apatch->net[6] + apatch->net[8] +
				    apatch->net[9]);
   apatch->net[0]  = (1.0 / 3.0) * (apatch->net[1] + apatch->net[2] +
				    apatch->net[3]);

   /* Modify a free parameter - the scalar at the top vertex. */
   apatch->net[0]  = FaceTopScalarFP * apatch->net[0];
#if 0
   apatch->net[1]  = FaceTopScalarFP/2.0 * apatch->net[1];
   apatch->net[2]  = FaceTopScalarFP/2.0 * apatch->net[2];
   apatch->net[3]  = FaceTopScalarFP/2.0 * apatch->net[3];
#endif
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void SetCFAPRemainingFreeScalarsBySimpleInterpolation(Apatch* apatch,
						      int sset[3])
{
   if (sset[0] == 0)
      apatch->net[5] = (1.0 / 2.0) * (apatch->net[4] + apatch->net[7]);
   if (sset[1] == 0)
      apatch->net[6] = (1.0 / 2.0) * (apatch->net[4] + apatch->net[9]);
   if (sset[2] == 0)
      apatch->net[8] = (1.0 / 2.0) * (apatch->net[7] + apatch->net[9]);

   apatch->net[1] = 2 * apatch->net[4];
   apatch->net[2] = 2 * apatch->net[7];
   apatch->net[3] = 2 * apatch->net[9];
   apatch->net[0] = apatch->net[4] + apatch->net[7] + apatch->net[9];
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void SetCFAPRemainingFreeScalarsByQuadPrecisionInterpolation(Apatch* apatch,
							     int sset[3])
{
   if (sset[0] == 0)
      apatch->net[5] = (1.0 / 2.0) * (apatch->net[4] + apatch->net[7]);
   if (sset[1] == 0)
      apatch->net[6] = (1.0 / 2.0) * (apatch->net[4] + apatch->net[9]);
   if (sset[2] == 0)
      apatch->net[8] = (1.0 / 2.0) * (apatch->net[7] + apatch->net[9]);

   apatch->net[0] = apatch->net[4] + apatch->net[7] + apatch->net[9];
   apatch->net[1] = (1.0/3.0 * apatch->net[0]) + apatch->net[4];
   apatch->net[2] = (1.0/3.0 * apatch->net[0]) + apatch->net[7];
   apatch->net[3] = (1.0/3.0 * apatch->net[0]) + apatch->net[9];
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void GenerateCFAPScalarsFromConvexConvexFaceDependency(Apatch* fap1,
						       Apatch* fap2,
						       int sset[3])
{
   int ef1i[2], nef1i, ef2i[2], nef2i;
   int i, j, k;
   Scalar sf1[4], sf2[4], se;
   Point pt;
   Scalar a[4], b[4];
   int cpi[4];
   double mA[3][3], mX[3], mB[3];
   Vector tmpVec;

   k = 0;
   for (i = 1; i < 4; i++) {
      for (j = 1; j < 4; j++) {
         tmpVec = PPDiff(fap1->tetrahedron[i].position, 
			 fap2->tetrahedron[j].position);
 	 if ( VVDot(tmpVec, tmpVec) == 0 ) {
            ef1i[k] = i;
            ef2i[k] = j;
            k++;
         }
      }
   }
   if (k > 2) {
      fprintf("Invalid edge vertex count, exiting.\n");
      exit(1);
   }
   for (i = 1; i < 4; i++) {
      if (i != ef1i[0] && i != ef1i[1])
         nef1i = i;
      if (i != ef2i[0] && i != ef2i[1])
         nef2i = i;
   }

   cpi[0] = 0;  cpi[nef1i] = 0;  cpi[ef1i[0]] = 2;  cpi[ef1i[1]] = 1;
   sf1[ef1i[0]] = fap1->net[CPIndex(ApatchDeg, 3, cpi)];
   cpi[0] = 0;  cpi[nef1i] = 0;  cpi[ef1i[0]] = 1;  cpi[ef1i[1]] = 2;
   sf1[ef1i[1]] = fap1->net[CPIndex(ApatchDeg, 3, cpi)];
   cpi[0] = 0;  cpi[nef1i] = 1;  cpi[ef1i[0]] = 1;  cpi[ef1i[1]] = 1;
   sf1[nef1i] = fap1->net[CPIndex(ApatchDeg, 3, cpi)];

   cpi[0] = 0;  cpi[nef2i] = 0;  cpi[ef2i[0]] = 2;  cpi[ef2i[1]] = 1;
   sf2[ef2i[0]] = fap2->net[CPIndex(ApatchDeg, 3, cpi)];
   cpi[0] = 0;  cpi[nef2i] = 0;  cpi[ef2i[0]] = 1;  cpi[ef2i[1]] = 2;
   sf2[ef2i[1]] = fap2->net[CPIndex(ApatchDeg, 3, cpi)];
   cpi[0] = 0;  cpi[nef2i] = 1;  cpi[ef2i[0]] = 1;  cpi[ef2i[1]] = 1;
   sf2[nef2i] = fap2->net[CPIndex(ApatchDeg, 3, cpi)];

   pt = PPrr(fap1->tetrahedron[0].position, 
	     fap2->tetrahedron[0].position, 1, 1);
   PCoords(pt, fap1->apFrame, &a[0], &a[1], &a[2]);
   a[3] = 1- a[0] - a[1] - a[2];
/*
fprintf(stderr, "Bcoords pt wrt fap1: %lg, %lg, %lg, %lg\n", a[0],a[1],a[2],a[3]);
*/
   PCoords(pt, fap2->apFrame, &b[0], &b[1], &b[2]);
   b[3] = 1- b[0] - b[1]- b[2];
/*
fprintf(stderr, "Bcoords pt wrt fap2: %lg, %lg, %lg, %lg\n", b[0],b[1],b[2],b[3]);
*/
   mA[0][0] = 1;   mA[0][1] = -a[0];   mA[0][2] = 0;
   mA[1][0] = 1;   mA[1][1] = 0;       mA[1][2] = -b[0];
   mA[2][0] = 1;   mA[2][1] = -0.5;    mA[2][2] = -0.5;

   mB[0] = a[1]*sf1[1] + a[2]*sf1[2] + a[3]*sf1[3];
   mB[1] = b[1]*sf2[1] + b[2]*sf2[2] + b[3]*sf2[3];
   mB[2] = 0;

/*
fprintf(stderr, "Solving matrix: \n");
for (i=0; i<3; i++)
fprintf(stderr, "%lf %lf %lf   %lf \n", mA[i][0], mA[i][1], mA[i][2], mB[i]);
fprintf(stderr, "Matrix again: \n");
for (i=0; i<3; i++)
fprintf(stderr, "%lg %lg %lg   %lg \n", mA[i][0], mA[i][1], mA[i][2], mB[i]);
*/

   GaussianSolve3x3(mA, mB, mX);
/*
fprintf(stderr, "Result: %lg %lg %lg \n", mX[0], mX[1], mX[2]);
*/

   se     = mX[0];
   sf1[0] = mX[1];
   sf2[0] = mX[2];

   cpi[0] = 1;  cpi[nef1i] = 0;  cpi[ef1i[0]] = 1;  cpi[ef1i[1]] = 1;
   fap1->net[CPIndex(ApatchDeg, 3, cpi)] = sf1[0];
   sset[nef1i - 1] = 1;
   cpi[0] = 1;  cpi[nef2i] = 0;  cpi[ef2i[0]] = 1;  cpi[ef2i[1]] = 1;
   fap2->net[CPIndex(ApatchDeg, 3, cpi)] = sf2[0];
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void ConstructConvexConvexEdgeApatchSimplex(Apatch* eap1, Apatch* eap2, 
					    Apatch* f1ap, Apatch* f2ap,
					    Edge* tedge, Face* f1, Face* f2,
					    int f1type, int f2type, 
					    Space wspace)
{
   VERTEX etetra[4];
   Vertex* edgeV[2];

   fprintf(stderr, "CCEdgeFaces: %s, %s\n",ReturnName(f1),ReturnName(f2));

   if (f1type == ZERO_CONVEX && f2type == NEG_CONVEX) {
      ConstructZeroConvexFaceApatch(f1ap, f1, 1, wspace);
      ConstructSingleFaceApatchSimplex(f2ap, f2, f2type, wspace);
   }
   else if (f1type == NEG_CONVEX && f2type == ZERO_CONVEX) {
      ConstructSingleFaceApatchSimplex(f1ap, f1, f1type, wspace);
      ConstructZeroConvexFaceApatch(f2ap, f2, 1, wspace);
   }
   else if ((f1type == POS_CONVEX && f2type == NEG_CONVEX) ||
            (f1type == NEG_CONVEX && f2type == POS_CONVEX)) {
      fprintf(stderr, "Edge faces %s, %s must be handled separately by ConstructPosConvexNegConvexEdgeApatchSimplex().\n", ReturnName(f1), ReturnName(f2));
      exit(1);
   }
   else {
      ConstructSingleFaceApatchSimplex(f1ap, f1, f1type, wspace);
      ConstructSingleFaceApatchSimplex(f2ap, f2, f2type, wspace);
   }

   etetra[1].position = PPrr(f1ap->tetrahedron[0].position,
                             f2ap->tetrahedron[0].position, 1, 1);
   GetEdgeVertices(tedge, &edgeV[0], &edgeV[1]);
   etetra[2].position = ReturnUDPoint(edgeV[0]);
   etetra[2].normal   = ReturnUDNormal(edgeV[0]);
   etetra[3].position = ReturnUDPoint(edgeV[1]);
   etetra[3].normal   = ReturnUDNormal(edgeV[1]);

   etetra[0].position = f1ap->tetrahedron[0].position;
   *eap1 = CreateApatch(ApatchDeg, etetra, wspace, EDGE1_COLOR);
   etetra[0].position = f2ap->tetrahedron[0].position;
   *eap2 = CreateApatch(ApatchDeg, etetra, wspace, EDGE2_COLOR);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void FitConvexConvexCubicEdgeApatches(Apatch* eap1, Apatch* eap2,
				      Apatch* f1ap, Apatch* f2ap,
				      Face* f1, Face* f2, 
				      int f1type, int f2type, Space wspace)
{
   int i, j, k, l;
   int e1p2, e1p3, e2p2, e2p3, f1ne1, f2ne2;
   int f1cpi[4], f2cpi[4], cpi[4];
   int a0, a1, a2, a3;
   Scalar be1[4], be2[4];
   Vector tmpvec;
   Scalar tmp;

   FitSingleCubicFaceApatch(f1ap, f1, f1type, wspace);
   FitSingleCubicFaceApatch(f2ap, f2, f2type, wspace);

   /* Get the face-edge tetrahedra common edge vertex indices. */
   for (i = 1; i < 4; i++) {
      tmpvec = PPDiff(f1ap->tetrahedron[i].position, 
		      eap1->tetrahedron[2].position);
      if ( VVDot(tmpvec, tmpvec) == 0 ) {
         e1p2 = i;
      } 
      else {
         tmpvec = PPDiff(f1ap->tetrahedron[i].position, 
			 eap1->tetrahedron[3].position);
         if ( VVDot(tmpvec, tmpvec) == 0 ) {
            e1p3 = i;
         } 
         else {
            f1ne1 = i;
         }
      }
      tmpvec = PPDiff(f2ap->tetrahedron[i].position, 
		      eap2->tetrahedron[2].position);
      if ( VVDot(tmpvec, tmpvec) == 0 ) {
         e2p2 = i;
      }
      else {
         tmpvec = PPDiff(f2ap->tetrahedron[i].position, 
			 eap2->tetrahedron[3].position);
         if ( VVDot(tmpvec, tmpvec) == 0 ) {
            e2p3 = i;
         }
         else {
            f2ne2 = i;
         }
      }
   }

fprintf(stderr,"e1p2: %d, e1p3: %d, f1ne1: %d, e2p2: %d, e2p3: %d, f2ne2: %d\n",e1p2,e1p3,f1ne1,e2p2,e2p3,f2ne2);

   /* Set face-edge tetrahedra C0 continuity. */
   j = 0;
   for (i = 0; i <= ApatchDeg; i++) {
      for (k = 0; k <= (ApatchDeg - i); k++) {
         l = ApatchDeg - i - k;
         f1cpi[0] = i;  f1cpi[f1ne1] = 0;  f1cpi[e1p2] = k;  f1cpi[e1p3] = l;
         eap1->net[CPIndexDim3(ApatchDeg, i, 0, k, l)] =
            f1ap->net[CPIndex(ApatchDeg, 3, f1cpi)];
         f2cpi[0] = i;  f2cpi[f2ne2] = 0;  f2cpi[e2p2] = k;  f2cpi[e2p3] = l;
         eap2->net[CPIndexDim3(ApatchDeg, i, 0, k, l)] =
            f2ap->net[CPIndex(ApatchDeg, 3, f2cpi)];
      }
   }

   /* Set face-edge tetrahedra C1 continuity. */
   PCoords(eap1->tetrahedron[1].position, f1ap->apFrame, 
	   &be1[0], &be1[1], &be1[2]);
   PCoords(eap2->tetrahedron[1].position, f2ap->apFrame, 
	   &be2[0], &be2[1], &be2[2]);
   be1[3] = 1 - be1[0] - be1[1] - be1[2];
   be2[3] = 1 - be2[0] - be2[1] - be2[2];
   j = 1;
   for (i = 0; i <= (ApatchDeg - j); i++) {
      for (k = 0; k <= (ApatchDeg - i - j); k++) {
         l = ApatchDeg - i - j - k;
         cpi[0] = i+1;  cpi[f1ne1] = j-1;  cpi[e1p2] = k;    cpi[e1p3] = l;
         a0 = CPIndex(ApatchDeg, 3, cpi);
         cpi[0] = i;    cpi[f1ne1] = j;    cpi[e1p2] = k;    cpi[e1p3] = l;
         a1 = CPIndex(ApatchDeg, 3, cpi);
         cpi[0] = i;    cpi[f1ne1] = j-1;  cpi[e1p2] = k+1;  cpi[e1p3] = l;
         a2 = CPIndex(ApatchDeg, 3, cpi);
         cpi[0] = i;    cpi[f1ne1] = j-1;  cpi[e1p2] = k;    cpi[e1p3] = l+1;
         a3 = CPIndex(ApatchDeg, 3, cpi);
         eap1->net[CPIndexDim3(ApatchDeg,i,j,k,l)] = 
	   ( (be1[0] * f1ap->net[a0]) + (be1[f1ne1] * f1ap->net[a1]) +
	     (be1[e1p2] * f1ap->net[a2]) + (be1[e1p3] * f1ap->net[a3]) );

         cpi[0] = i+1;  cpi[f2ne2] = j-1;  cpi[e2p2] = k;    cpi[e2p3] = l;
         a0 = CPIndex(ApatchDeg, 3, cpi);
         cpi[0] = i;    cpi[f2ne2] = j;    cpi[e2p2] = k;    cpi[e2p3] = l;
         a1 = CPIndex(ApatchDeg, 3, cpi);
         cpi[0] = i;    cpi[f2ne2] = j-1;  cpi[e2p2] = k+1;  cpi[e2p3] = l;
         a2 = CPIndex(ApatchDeg, 3, cpi);
         cpi[0] = i;    cpi[f2ne2] = j-1;  cpi[e2p2] = k;    cpi[e2p3] = l+1;
         a3 = CPIndex(ApatchDeg, 3, cpi);
         eap2->net[CPIndexDim3(ApatchDeg,i,j,k,l)] = 
	   ( (be2[0] * f2ap->net[a0]) + (be2[f2ne2] * f2ap->net[a1]) +
	     (be2[e2p2] * f2ap->net[a2]) + (be2[e2p3] * f2ap->net[a3]) );
      }
   }

   /* Check if edge-edge tetrahedra C0 continuity is satisfied so far. */
   i = 0;
   for (j = 0; j <= (ApatchDeg - i); j++) {
      for (k = 0; k <= (ApatchDeg - i - j); k++) {
         l = ApatchDeg - i - j - k;
         if ( fabs(eap1->net[CPIndexDim3(ApatchDeg,i,j,k,l)] -
		   eap2->net[CPIndexDim3(ApatchDeg,i,j,k,l)]) > eps ) {
            fprintf(stderr, "Edge-edge tetrahedra C0 continuity not satisfied by construction, exiting.\n");
            exit(1);
         }
      }
   }

   /* Check if scalars determined for the edge tetrahedra are consistent   */
   /* with the given normals (i.e. they interpolate them) at the vertices. */
   /* Since edge-edge tetrahedra C0 continuity is satisfied, we only need  */
   /* to examine the scalars along one of the edge tetrahedra.             */


   /* Check if edge-edge tetrahedra C1 continuity is satisfied so far. */
   PCoords(eap2->tetrahedron[0].position, eap1->apFrame,
	   &be2[0], &be2[1], &be2[2]);
   be2[3] = 1 - be2[0] - be2[1] - be2[2];
   i = 1;
   for (j = 0; j <= (ApatchDeg - i); j++) {
     for (k = 0; k <= (ApatchDeg - i - j); k++) {
        l = ApatchDeg - i -j- k;
        a0 = CPIndexDim3(ApatchDeg, i,   j,   k,   l  );
        a1 = CPIndexDim3(ApatchDeg, i-1, j+1, k,   l  );
        a2 = CPIndexDim3(ApatchDeg, i-1, j,   k+1, l  );
        a3 = CPIndexDim3(ApatchDeg, i-1, j,   k,   l+1);
        switch (j) {
        case 0:
	  /* Check for C1 continuity across edge-edge tetrahedra. */
          tmp = ( (be2[0] * eap1->net[a0]) + (be2[1] * eap1->net[a1]) +
		  (be2[2] * eap1->net[a2]) + (be2[3] * eap1->net[a3]) );
fprintf(stderr, "tmp is: %lf, eap2net[a0]: %lf \n", tmp, eap2->net[a0]);
          if ( fabs(eap2->net[a0] - tmp) > eps ) {
             fprintf(stderr, "Edge-edge tetrahedra C1 continuity not satisfied by construction.\n");
          }
          break;
        case 1:
	  /* Set edge-edge control scalars (e0210 and e0201)   */
	  /* to ensure C1 continuity.                          */
          if (be2[1] == 0) {
             fprintf(stderr, "Divide by zero error in computing edge-edge scalars.\n");
             exit(1);
          }
          eap1->net[a1] = 
	    (1.0 / be2[1]) * (eap2->net[a0] - ((be2[0] * eap1->net[a0]) +
					       (be2[2] * eap1->net[a2]) +
					       (be2[3] * eap1->net[a3])));
          eap2->net[a1] = eap1->net[a1];
          break;
        case 2:
	  /* Set edge-edge tetrahedra free parameters and ensure continuity. */
          /* eap1->net[a0] and eap2->net[a0] are the free parameters, and    */
          /* eap1/2->net[a1] is set by continuity constraints.               */
          eap1->net[a0] = 
	    (1.0 / 3.0) * (eap1->net[CPIndexDim3(ApatchDeg,i+1,j-1,k,l)] +
			   eap1->net[CPIndexDim3(ApatchDeg,i,j-1,k+1,l)] +
			   eap1->net[CPIndexDim3(ApatchDeg,i,j-1,k,l+1)]);
          eap2->net[a0] = 
	    (1.0 / 3.0) * (eap2->net[CPIndexDim3(ApatchDeg,i+1,j-1,k,l)] +
			   eap2->net[CPIndexDim3(ApatchDeg,i,j-1,k+1,l)] +
			   eap2->net[CPIndexDim3(ApatchDeg,i,j-1,k,l+1)]);
          if (be2[1] == 0) {
             fprintf(stderr, "Divide by zero error in computing edge-edge scalars.\n");
             exit(1);
          }
          eap1->net[a1] = 
	    (1.0 / be2[1]) * (eap2->net[a0] - ((be2[0] * eap1->net[a0]) +
					       (be2[2] * eap1->net[a2]) +
					       (be2[3] * eap1->net[a3])));
          eap2->net[a1] = eap1->net[a1];
          break;
        }
     }
   }
}


#if 0

/* -------------------------------------------------------------------- *
 * Check if the fitted A-patch gives correct normals at	                *
 * the tetrahedral corners lying on the triangulation.  	        *
 * Returns 0 if the evaluation is inconsistent with the data,           *
 * returns 1 otherwise.                                                 *
 * -------------------------------------------------------------------- */
int IsFittedFaceApatchConsistentWithData(Apatch apatch, 
					 TriangulationFace tface)
{
   VERTEX chkV;
   Scalar chkVVal, chkx, chky, chkz;
   Vector chkVec;

   chkV.position = apatch.tetrahedron[1].position;
   chkVVal = EvalApatch(apatch, &chkV);
   if (fabs(chkVVal) > eps) {
      fprintf(stderr, "Face %s: Scalar evaluated at vertex1 is not zero.\n", tface.name);
      return 0;
   }
   /*   if (chkVVal != 0) {
      fprintf(stderr, "Scalar evaluated at P1 is not zero.\n");
      return 0;
   }
   */
   /*
NCoords(chkV.normal, StdFrame(tface.wspace), &chkx, &chky, &chkz);
fprintf(stderr, "chkV.normal: %lg %lg %lg\n", chkx, chky, chkz);
   */
   chkVec = VVCross(NDual(chkV.normal), NDual(tface.vertices[0].normal));
   VCoords(chkVec, StdFrame(apatch.wspace), &chkx, &chky, &chkz);
   if (chkx != 0 || chky != 0 || chkz != 0) {
      fprintf(stderr, "Face %s: Normal evaluated at vertex1 inconsistent with data.\n", tface.name);
      return 0;
   }
   if (VVDot(NDual(chkV.normal), NDual(tface.vertices[0].normal)) < 0) {
      fprintf(stderr, "Face %s: Normal evaluated at vertex1 should point in the opposite direction.\n", tface.name);
      return 0;
   }

   chkV.position = apatch.tetrahedron[2].position;
   chkVVal = EvalApatch(apatch, &chkV);
   if (fabs(chkVVal) > eps) {
      fprintf(stderr, "Face %s: Scalar evaluated at vertex2 is not zero.\n", tface.name);
      return 0;
   }
   /*   if (chkVVal != 0) {
      fprintf(stderr, "Scalar evaluated at P2 is not zero.\n");
      return 0;
   }
   */
   chkVec = VVCross(NDual(chkV.normal), NDual(tface.vertices[1].normal));
   VCoords(chkVec, StdFrame(apatch.wspace), &chkx, &chky, &chkz);
   if (chkx != 0 || chky != 0 || chkz != 0) {
      fprintf(stderr, "Face %s: Normal evaluated at vertex2 inconsistent with data.\n", tface.name);
      return 0;
   }
   if (VVDot(NDual(chkV.normal), NDual(tface.vertices[1].normal)) < 0) {
      fprintf(stderr, "Face %s: Normal evaluated at vertex2 should point in the opposite direction.\n", tface.name);
      return 0;
   }

   chkV.position = apatch.tetrahedron[3].position;
   chkVVal = EvalApatch(apatch, &chkV);
   if (fabs(chkVVal) > eps) {
      fprintf(stderr, "Face %s: Scalar evaluated at vertex3 is not zero.\n", tface.name);
      return 0;
   }
   /*   if (chkVVal != 0) {
      fprintf(stderr, "Scalar evaluated at P3 is not zero.\n");
      return 0;
   }
   */
   chkVec = VVCross(NDual(chkV.normal), NDual(tface.vertices[2].normal));
   VCoords(chkVec, StdFrame(apatch.wspace), &chkx, &chky, &chkz);
   if (chkx != 0 || chky != 0 || chkz != 0) {
      fprintf(stderr, "Face %s: Normal evaluated at vertex3 inconsistent with data.\n", tface.name);
      return 0;
   }
   if (VVDot(NDual(chkV.normal), NDual(tface.vertices[2].normal)) < 0) {
      fprintf(stderr, "Face %s: Normal evaluated at vertex3 should point in the opposite direction.\n", tface.name);
      return 0;
   }

   return 1;
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void FitCubicFaceApatchPair(ApatchPair* apPair)
{
   FitSingleCubicFaceApatch(&apPair->ap1);
   SetCubicFaceApatchPairContinuity(apPair);
   SetCubicFaceApatchPairFreeScalarsByAvg(apPair);

   /*
   if (IsFittedFaceApatchConsistentWithData(apPair->ap1, tface) == 0) {
      fprintf(stderr, "Face %s: Invalid apPair->ap1. Exiting.\n", tface.name);
      exit(1);
   }
   if (IsFittedFaceApatchConsistentWithData(apPair->ap2, tface) == 0) {
      fprintf(stderr, "Face %s: Invalid apPair->ap2. Exiting.\n", tface.name);
      exit(1);
   }
   */
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void SetCubicFaceApatchPairContinuity(ApatchPair* apPair)
{
   Scalar b0, b1, b2, b3;
   int v1index, i, j, k, l, a0, a1, a2, a3;

   /* Ensure C0 continuity across the shared face. */
   v1index = CPIndexDim3(ApatchDeg, 0, ApatchDeg, 0, 0);
   for (i = v1index; i < apPair->ap1.netSize; i++) {
      apPair->ap2.net[i] = apPair->ap1.net[i];
   }

   /* Ensure C1 continuity across the shared face. */
   /* Get barycentric coords of ap2.tetrahedron[0] wrt ap1. */
   PCoords(apPair->ap2.tetrahedron[0].position, apPair->ap1.apFrame,
		&b0, &b1, &b2);
   b3 = 1 - b0 - b1 - b2;
   i = 1;
   for (j = 0; j <= (ApatchDeg - i); j++) {
      for (k = 0; k <= (ApatchDeg - i - j); k++) {
         l = ApatchDeg - i - j - k;
         a0 = CPIndexDim3(ApatchDeg, 1, j, k, l);
         a1 = CPIndexDim3(ApatchDeg, 0, j+1, k, l);
         a2 = CPIndexDim3(ApatchDeg, 0, j, k+1, l);
         a3 = CPIndexDim3(ApatchDeg, 0, j, k, l+1);
         apPair->ap2.net[a0] = (b0 * apPair->ap1.net[a0]) +
			       (b1 * apPair->ap1.net[a1]) +
			       (b2 * apPair->ap1.net[a2]) +
			       (b3 * apPair->ap1.net[a3]);
      }
   }
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void SetCubicFaceApatchPairFreeScalarsByAvg(ApatchPair* apPair)
{
   /* We need to determine the remaining control scalars which form the	*/
   /* free parameters of our construction; used to control shape of the	*/
   /* fitted Apatch.  The array index at which to store the generated	*/
   /* control scalars can be determined easily, and are therefore 	*/
   /* referred to directly (the control scalars are stored in reverse	*/
   /* lexographic order within a linear array).				*/

   apPair->ap2.net[1] = (1.0 / 3.0) * (apPair->ap2.net[4] +
			apPair->ap2.net[5] + apPair->ap2.net[6]);
   apPair->ap2.net[2] = (1.0 / 3.0) * (apPair->ap2.net[5] +
			apPair->ap2.net[7] + apPair->ap2.net[8]);
   apPair->ap2.net[3] = (1.0 / 3.0) * (apPair->ap2.net[6] +
			apPair->ap2.net[8] + apPair->ap2.net[9]);
   apPair->ap2.net[0] = (1.0 / 3.0) * (apPair->ap2.net[1] +
			apPair->ap2.net[2] + apPair->ap2.net[3]);

}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void ConstructNonconvexNonconvexEdgeApatchSimplex(Apatch* eap1, Apatch* eap2, 
						  Apatch* eap3, Apatch* eap4,
						  ApatchPair* f1app, 
						  ApatchPair* f2app,
						  TriangulationEdge tedge)
{
   VERTEX e1tetra[4], e2tetra[4], e3tetra[4], e4tetra[4];
   Scalar chk;
   Point tmpP2, tmpP3;
   int i;

   fprintf(stderr, "Nonconvex Nonconvex Edge Faces: %s, %s\n", tedge.f1.name,tedge.f2.name);

   ConstructFaceApatchPairSimplex(f1app, tedge.f1);
   ConstructFaceApatchPairSimplex(f2app, tedge.f2);

   /* Assuming all faces are given according to a right hand system */
   /* and all normals point to the same side.                       */
   /* So f1app->ap1 and f2app->ap1 should lie on the same side.     */
   e1tetra[0].position = PPrr(f1app->ap1.tetrahedron[0].position,
                              f2app->ap1.tetrahedron[0].position, 1, 1);
   e2tetra[0].position = e1tetra[0].position;
   e1tetra[1].position = f1app->ap1.tetrahedron[0].position;
   e2tetra[1].position = f2app->ap1.tetrahedron[0].position;
   e3tetra[0].position = PPrr(f1app->ap2.tetrahedron[0].position,
                              f2app->ap2.tetrahedron[0].position, 1, 1);
   e4tetra[0].position = e3tetra[0].position;
   e3tetra[1].position = f1app->ap2.tetrahedron[0].position;
   e4tetra[1].position = f2app->ap2.tetrahedron[0].position;
   tmpP2 = tedge.vertices[0].position;
   tmpP3 = tedge.vertices[1].position;
   chk = VVDot( 
            VVCross( 
               PPDiff(tmpP2, e1tetra[1].position), 
               PPDiff(tmpP3, tmpP2) ),
            PPDiff(e1tetra[0].position, e1tetra[1].position) );
   if (chk >= 0) {
      e1tetra[2] = tedge.vertices[0];   e1tetra[3] = tedge.vertices[1];
      e2tetra[2] = tedge.vertices[1];   e2tetra[3] = tedge.vertices[0];
      e3tetra[2] = tedge.vertices[1];   e3tetra[3] = tedge.vertices[0];
      e4tetra[2] = tedge.vertices[0];   e4tetra[3] = tedge.vertices[1];
   }
   else {
      e1tetra[2] = tedge.vertices[1];   e1tetra[3] = tedge.vertices[0];
      e2tetra[2] = tedge.vertices[0];   e2tetra[3] = tedge.vertices[1];
      e3tetra[2] = tedge.vertices[0];   e3tetra[3] = tedge.vertices[1];
      e4tetra[2] = tedge.vertices[1];   e4tetra[3] = tedge.vertices[0];
   }

   *eap1 = CreateApatch(ApatchDeg, e1tetra, tedge.wspace, EDGE1_COLOR);
   *eap2 = CreateApatch(ApatchDeg, e2tetra, tedge.wspace, EDGE2_COLOR);
   *eap3 = CreateApatch(ApatchDeg, e3tetra, tedge.wspace, EDGE1_COLOR);
   *eap4 = CreateApatch(ApatchDeg, e4tetra, tedge.wspace, EDGE2_COLOR);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void FitNonconvexNonconvexCubicEdgeApatches(Apatch* eap1, Apatch* eap2,
					    Apatch* eap3, Apatch* eap4,
					    ApatchPair* f1app, 
					    ApatchPair* f2app)
{
   int i;

   FitCubicFaceApatchPair(f1app);
   FitCubicFaceApatchPair(f2app);

   for (i = 0; i < NetSize(ApatchDeg,3); i++) {
      eap1->net[i] = 0;
      eap2->net[i] = 0;
      eap3->net[i] = 0;
      eap4->net[i] = 0;
   }

   /*Now set C0 continuity and set scalars to interpolate normals*/

}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void ConstructPosConvexNegConvexEdgeApatchSimplex(Apatch* eap1, Apatch* eap2, 
						  Apatch* fap3, 
						  Apatch* f1ap, Apatch* f2ap,
						  TriangulationEdge tedge, 
						  int f1type, int f2type)
{
   fprintf(stderr, "Edge faces %s, %s are adjacent positive and negative convex. Not handled (no edge A-patches are constructed.\n", tedge.f1.name, tedge.f2.name);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void FitPosConvexNegConvexCubicEdgeApatches(Apatch* eap1, Apatch* eap2, 
					    Apatch* fap3, 
					    Apatch* f1ap, Apatch* f2ap)
{

}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void ConstructConvexNonconvexEdgeApatchSimplex(Apatch* eap1, Apatch* eap2, 
					       Apatch* fap, 
					       ApatchPair* fapPair,
					       TriangulationEdge tedge, 
					       int f1type, int f2type)
{
   VERTEX e1tetra[4], e2tetra[4];
   Scalar chk;
   Point tmpP2, tmpP3;
   int i;

   fprintf(stderr, "Convex Nonconvex Edge Faces: %s, %s\n", tedge.f1.name, tedge.f2.name);

   if (f1type != NON_CONVEX && f2type == NON_CONVEX) {
      ConstructSingleFaceApatchSimplex(fap, tedge.f1, f1type);
      ConstructFaceApatchPairSimplex(fapPair, tedge.f2);
   }
   else if (f1type == NON_CONVEX && f2type != NON_CONVEX) {
      ConstructFaceApatchPairSimplex(fapPair, tedge.f1);
      ConstructSingleFaceApatchSimplex(fap, tedge.f2, f2type);
   }
   else {
      fprintf(stderr, "Edge without convex & nonconvex adjacent faces.\n");
      exit(1);
   }

   if (f1type == NEG_CONVEX || f2type == NEG_CONVEX) {
      e1tetra[0].position = PPrr(fap->tetrahedron[0].position,
                                 fapPair->ap2.tetrahedron[0].position, 1, 1);
      e2tetra[1].position = fapPair->ap2.tetrahedron[0].position;
   }
   else {
      e1tetra[0].position = PPrr(fap->tetrahedron[0].position,
                                 fapPair->ap1.tetrahedron[0].position, 1, 1);
      e2tetra[1].position = fapPair->ap1.tetrahedron[0].position;
   }
   e2tetra[0].position = e1tetra[0].position;
   e1tetra[1].position = fap->tetrahedron[0].position;
   /*The following part should work.*/
   tmpP2 = tedge.vertices[0].position;
   tmpP3 = tedge.vertices[1].position;
   chk = VVDot( 
            VVCross( 
               PPDiff(tmpP2, e1tetra[1].position), 
               PPDiff(tmpP3, tmpP2) ),
            PPDiff(e1tetra[0].position, e1tetra[1].position) );
   if (chk >= 0) {
      e1tetra[2] = tedge.vertices[0];   e1tetra[3] = tedge.vertices[1];
      e2tetra[2] = tedge.vertices[1];   e2tetra[3] = tedge.vertices[0];
   }
   else {
      e1tetra[2] = tedge.vertices[1];   e1tetra[3] = tedge.vertices[0];
      e2tetra[2] = tedge.vertices[0];   e2tetra[3] = tedge.vertices[1];
   }

   *eap1 = CreateApatch(ApatchDeg, e1tetra, tedge.wspace, EDGE1_COLOR);
   *eap2 = CreateApatch(ApatchDeg, e2tetra, tedge.wspace, EDGE2_COLOR);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void FitConvexNonconvexCubicEdgeApatches(Apatch* eap1, Apatch* eap2,
					 Apatch* fap, ApatchPair* fapPair)
{
   int i;

   FitSingleCubicFaceApatch(fap);
   FitCubicFaceApatchPair(fapPair);

   for (i = 0; i < eap1->netSize; i++)
      eap1->net[i] = 0;
   for (i = 0; i < eap2->netSize; i++)
      eap2->net[i] = 0;

   /*Now set C0 continuity and set scalars to interpolate normals*/
}

/***************************/
#endif

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dFaceApatchSimplex(FILE* fp, Apatch apatch)
{
   Material matTriangulation;
   Material matSimplex;

   matTriangulation = APColor1;
   matSimplex = APColor2;

   OutputS3dVMatPolygonTriangle(fp, matSimplex, apatch.tetrahedron[0],
				matSimplex, apatch.tetrahedron[1],
				matSimplex, apatch.tetrahedron[2], 0);
   OutputS3dVMatPolygonTriangle(fp, matSimplex, apatch.tetrahedron[0],
				matSimplex, apatch.tetrahedron[2],
				matSimplex, apatch.tetrahedron[3], 0);
   OutputS3dVMatPolygonTriangle(fp, matSimplex, apatch.tetrahedron[0],
				matSimplex, apatch.tetrahedron[3],
				matSimplex, apatch.tetrahedron[1], 0);
   OutputS3dVMatPolygonTriangle(fp, matTriangulation, apatch.tetrahedron[1],
				matTriangulation, apatch.tetrahedron[2],
				matTriangulation, apatch.tetrahedron[3], 0);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dFaceApatchPairSimplex(FILE* fp, ApatchPair apPair)
{
   OutputS3dFaceApatchSimplex(fp, apPair.ap1);
   OutputS3dFaceApatchSimplex(fp, apPair.ap2);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
void OutputS3dEdgeApatchSimplex(FILE* fp, Apatch apatch)
{
   Material matTriangulation;
   Material matSimplex;

   matTriangulation = APColor1;
   matSimplex = APColor2;

   OutputS3dVMatPolygonTriangle(fp, matSimplex, apatch.tetrahedron[0], 
				matTriangulation, apatch.tetrahedron[2],
				matSimplex, apatch.tetrahedron[3], 0);
   OutputS3dVMatPolygonTriangle(fp, matSimplex, apatch.tetrahedron[1], 
				matTriangulation, apatch.tetrahedron[3],
				matSimplex, apatch.tetrahedron[2], 0);
   OutputS3dVMatPolygonTriangle(fp, matSimplex, apatch.tetrahedron[0],
				matSimplex, apatch.tetrahedron[1],
				matSimplex, apatch.tetrahedron[2], 0);
   OutputS3dVMatPolygonTriangle(fp, matSimplex, apatch.tetrahedron[0],
				matSimplex, apatch.tetrahedron[3],
				matSimplex, apatch.tetrahedron[1], 0);
}

/* -------------------------------------------------------------------- *
 * -------------------------------------------------------------------- */
