/******************************************************************/
/* Module: AFitter.c                                              */
/* $Date: 1999/10/25 22:06:16 $                                   */
/* $Revision: 1.5 $                                               */
/* $Source: /p/SurfaceFitting/SF/src/Apps/ATess/RCS/AFitter.c,v $ */
/******************************************************************/
/*
 * Copyright (c) 1998, Computer Graphics Lab, University of Waterloo.
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "mesh.h"
#include "userData.h"
#include "commandline.h"
#include "usage.h"
#include "apconstruct.h"

/* Forward declarations for option handlers */
extern void Usage();

Space World;
Frame WorldF;
int ApatchDeg = 3;
double LenN1FP = 1.0;
double LenN2FP = 1.0;
double LenN3FP = 1.0;
double SimplexFP = 2.0;
double FaceTopScalarFP = 1.0;
int FPSelectionMethod = QUAD_PRECISION_INTERPOLATION;

main(int argc, char **argv)
{
   Mesh* theMesh;
   Face* thisFace;
   Edge* thisEdge;

   ParseCommandLine( argc, argv);

   /* Read in the mesh */
   theMesh = MeshParse(stdin);
   World = SCreate( "World", 3);
   WorldF = StdFrame(World);
   AddGeometry(World, theMesh);

   ForeachMeshFace(theMesh, thisFace) {
      ConstructFaceApatches(thisFace, World);
   } EndForeach;

#if 1
   ForeachMeshEdge(theMesh, thisEdge) {
      if (!BoundaryEdge(thisEdge)) {
         ConstructEdgeApatches(thisEdge, World);
      }
   } EndForeach;
#endif
}

/*------------------------------------------------------------*/
/*                   Command Line Handlers                    */
/*------------------------------------------------------------*/

void SetApatchDeg(char *num[])
{
   ApatchDeg = atoi(num[0]);
   if (ApatchDeg != 0 && ApatchDeg != 3) {
      fprintf(stderr, "Only cubic Apatch construction is handled currently; defaulting Apatch degree to 3.\n");
      ApatchDeg = 3;
   }
}

void SetVNormLenFP(char* arg[])
{
   LenN1FP = atof(arg[0]);
   LenN2FP = atof(arg[1]);
   LenN3FP = atof(arg[2]);
}

void SetTetraSimplexFP(char* arg[])
{
   SimplexFP = atof(arg[0]);
   if (SimplexFP <= 1.0) {
      fprintf(stderr, "SimplexFP must be greater than 1.0; defaulting to 2.0.\n");
      SimplexFP = 2.0;
   }
}

void SetFaceTopScalarFP(char* arg[])
{
   FaceTopScalarFP = atof(arg[0]);
   if (FaceTopScalarFP <= 0.0) {
      fprintf(stderr, "FaceTopScalarFP must be greater than 0.0; defaulting to 1.0.\n");
      FaceTopScalarFP = 1.0;
   }
}

void SetFPSelectionMethod(char *num[])
{
   FPSelectionMethod = atoi(num[0]);
   if (FPSelectionMethod != AVERAGING &&
       FPSelectionMethod != SIMPLE_INTERPOLATION &&
       FPSelectionMethod != QUAD_PRECISION_INTERPOLATION) {
      fprintf(stderr, "Invalid value for free parameter selection method.\n");
      exit(1);
   }
}

/* Table of available options -- register new options here */
/* For full details on how to add options, see the file    */
/* commandline.h.                                          */
Option Options[] = {
/* Name Handler          #args   helpstring  */
   "h", Usage,             0,    ": Print available options.",
   "d", SetApatchDeg,      1,    "<ApatchDegree> : Specify the degree of the Apatch to be fitted.  A value of zero outputs the simplicial hull over the mesh.  The default value is 3, constructing a cubic Apatch.", 
   "n", SetVNormLenFP,     3,    "<lenN1 lenN2 lenN3>: Set the length of the normals at each of the vertices of the surface triangulation.  These are free parameters controlling the shape of the Apatch.  The default lengths are all 1.",
   "s", SetTetraSimplexFP, 1,    "<simplex-scale-factor>: Set the multiplicative factor to vary the height of the simplex constructed; a value greater than 1.0 must be chosen to ensure tangent plane containment, the default is 2.0.",
   "f", SetFaceTopScalarFP,1,    "<topScalar-multiplier>: If using averaging as the free parameter selection method, set the multiplicative factor to vary the value of the control scalar at the top of the face simplex; a value greater than 0.0 must be chosen to ensure the scalar maintains the correct sign, the default is 1.0.",
   "m", SetFPSelectionMethod, 1, "<selection-method>: Set an integer value indicating the free parameter selection method to be used. Values: 0 = Averaging, 1 = SimpleInterpolation, 2 = QuadraticPrecisionInterpolation (default = 2).\n",

/*  Do not delete the next line */
    NULL,    NULL,       0,      NULL,      NULL
  };

/* Global variables go here */
char *Banner  = "AFitter, Version 1";
char *UsageString = "AFitter [options] < mesh";

