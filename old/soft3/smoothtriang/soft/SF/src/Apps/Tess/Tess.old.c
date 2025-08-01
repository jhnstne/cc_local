/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 * Author: Tony DeRose
 *
 * Purpose: Surface patch tessellator for the Surface Fitting Testbed.
 *          Currently the only supported types of patches are BezierTriangle
 *	and TPBezierPatch.
 *
 * Tessellation is done by uniformly sampling in parameter space.
 *
 */

#include <stdio.h>
#include "all.h"

#define	K_UNDEF 0
#define	K_GAUSSIAN 1
#define K_MEAN 2


extern	Scalar	KGauss(), KMean();

int EdgeSamples = 0;
Space World;
void sgpTriangleOutput();
void (*OutputTriangle)() = sgpTriangleOutput;
int DoGaussian = 0;
int DoMean = 0;
int IsoLines=0;
Scalar offsetDist=0.;
int s3d=0;
Scalar s3dBoundaries=0;
int s3dWidth=1;

main(argc, argv)
int argc;
char **argv;
{

  ParseCommandLine( argc, argv);
  
  /* Create the space where patches map into */
  World = SCreate( "World", 3);
  
  
  /* Main Loop.  Read, tessellate, and output triangles. */
  while (ReadDstruct() != EOF) {
    if ( QueryDstructPath("BezierTriangle") ) {
      Patch ThisPatch;

      GetPatch(&ThisPatch,World);
      TessellateBezierTriangle(ThisPatch);
      PatchFree( ThisPatch);

    } else if ( QueryDstructPath("TPBezier") ) {
      TPBezier tp;

      GetTPBezier(&tp,World);
      TPBezierTessellate(tp);
      TPBezierFree(tp);

    } else {
      fprintf(stderr,"%s: unknown patch type.  Exiting.\n",argv[0]);
      exit(1);
    }
  }
  exit(0);
}





void Samples(num)
char *num[];
{
  EdgeSamples = atoi( num[0] );
}



void WireFrame()
{
  extern void wireframeOutput();

  OutputTriangle = wireframeOutput;
}


void	SetGauss()
   {
   DoGaussian = 1;
   }

void	SetMean()
   {
   DoMean = 1;
   }


void LineMode()
{
  IsoLines = 1;
  if ( EdgeSamples == 0 ){
    EdgeSamples = 2;
  }
}

void SetOffset(arg)
char* arg[];
{
	double atof();
	offsetDist = atof(arg[0]);
}


void SetS3d()
{
	s3d = 1;
}

void SetBoundaries(arg)
char* arg[];
{
	s3dBoundaries = atof(arg[0]);
	s3dWidth = atoi(arg[1]);
}

Option Options[] = {
/* Name		Handler		#Args	HelpString */
   "h",		Usage,		0,	": \t\t\tPrint available options.",
   "s", 	Samples,	1,	"<edge-samples>: \tThis is the number of samples along each edge.\n\t\t\t\tThe default is to output the control net.",
   "w",		WireFrame,	0,	":\t\t\tGenerates output for wireframe.",
   "kg",	SetGauss,	0,	":\t\t\tGenerates Gaussian curvature data.",
   "km",	SetMean,	0,	":\t\t\tGenerates Mean curvature data.",
   "g3d",	LineMode,	0,	":\t\t\tGenerate isolines in g3d form.",
   "s3d",	SetS3d,		0,	":\t\t\tOutput in S3d form.",
   "boundaries",SetBoundaries,	2,	"offset width:\t\t\tOutput patch boundaries (s3d).\n",
   "os",	SetOffset,	1,	"offset: Tessellate the offset surface.\n",

/* Do not delete this next line */
   NULL,	NULL,		0,	NULL
   };

char* Banner = "Tess";
char* UsageString = "Tess [options] < bezier-triangle-stream > dstruct-triangle-stream";

