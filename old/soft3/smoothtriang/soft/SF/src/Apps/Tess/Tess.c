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
#include <math.h>
#include <stdlib.h>
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "libgeo.h"
#include "commandline.h"
#include "usage.h"
#include "patch.h"
#include "sff.h"
#include "tpbezier.h"

/* #include "spatch.h" */

#define	K_UNDEF 0
#define	K_GAUSSIAN 1
#define K_MEAN 2


extern	Scalar	KGauss(), KMean();

int EdgeSamples = 0;
Space World;
void sgpTriangleOutput();
void g3dTriangleOutput();
void psTriangleOutput();
void (*OutputTriangle)() = sgpTriangleOutput;
int DoGaussian = 0;
int DoMean = 0;
int WhichFunctional = -1;
int IsoLines=0;
Scalar offsetDist=0.;
int boundary=0;
int split=0;
int g3d=0;
int s3d=0;
int ps=0;
double psScale=1.0;
int iv=0;
Scalar s3dBoundaries=0;
int s3dWidth=1;
Scalar cblen= -1.0;

main(int argc, char **argv)
{
	
	ParseCommandLine( argc, argv);
	
	if ( ps ) {
		printf("%%!\n");
		printf("%g %g scale\n",psScale,psScale);
		printf("%g setlinewidth\n",1/psScale);
	}
	/* Create the space where patches map into */
	World = SCreate( "World", 3);
	
	
	/* Main Loop.  Read, tessellate, and output triangles. */
	while (ReadDstruct() != EOF) {
		if ( QueryDstructPath("BezierTriangle") ) {
			Patch ThisPatch;
			
			GetPatch(&ThisPatch,World);
			TessellateBezierTriangle(ThisPatch);
			PatchFree( ThisPatch);
			
		} else if ( QueryDstructPath("RationalBezierTriangle") ) {
			TessellateRationalBezierTriangle(din);
		} else if ( QueryDstructPath("TPBezier") ) {
			TPBezier tp;
			
			GetTPBezier(&tp,World);
			TPBezierTessellate(tp);
			TPBezierFree(tp);
			
#ifdef SPATCH
		} else if (QueryDstructPath("Spatch") ) {
			Spatch sp;

			dGetSpatch(din, NULL, &sp, StdFrame(World));
			SpatchTessellate(sp, EdgeSamples, StdFrame(World));
			free(sp.net);
/*			fTessSpatch(stdout, sp, EdgeSamples);*/
#endif
		} else {
			fprintf(stderr,"%s: unknown patch type.  Exiting.\n",argv[0]);
			exit(1);
		}
	}
	if ( ps ) {
		printf("showpage\n");
	}
	exit(0);
}





void Samples(char *num[])
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

void	SetMean(char *num[])
{
	DoMean = 1;
	WhichFunctional = atoi( num[0]);
}


void LineMode()
{
	IsoLines = 1;
	if ( EdgeSamples == 0 ){
		EdgeSamples = 2;
	}
}

void SetOffset(char* arg[])
{
	double atof();
	offsetDist = atof(arg[0]);
}

void SetS3d()
{
	s3d = 1;
}

void SetBoundaries(char* arg[])
{
	s3dBoundaries = atof(arg[0]);
	s3dWidth = atoi(arg[1]);
}


void SetBoundary()
{
	boundary = 1;
}

void DoSplit()
{
	split = 1;
}

void OutG3d()
{
	OutputTriangle = g3dTriangleOutput;
	g3d = 1;
}


void OutPS(char* arg[])
{
	OutputTriangle = psTriangleOutput;
	g3d = 0;
	s3d = 0;
	ps = 1;
	psScale = atof(arg[0]);
}

void SetIV()
{
	printf("#Inventor V2.0 ascii\n\n");
	printf("Material {\n");
	printf("  ambientColor  0.09 0.04 0.04\n");
	printf("  diffuseColor  0.9 0.4 0.4\n");
	printf("  specularColor  0.3 0.3 0.3\n");
	printf("  shininess 0.2\n");
	printf("  transparency   0\n");
	printf("}\n");
	iv = 1;
}

void SetCB(arg)
char* arg[];
{
	double atof();

	cblen = atof(arg[0]);
}

Option Options[] = {
/* Name		Handler		#Args	HelpString */
   "h",		Usage,		0,	": \t\t\tPrint available options.",
   "s", 	Samples,	1,	"<edge-samples>: \tThis is the number of samples along each edge.\n\t\t\t\tThe default is to output the control net.",
   "w",		WireFrame,	0,	":\t\t\tGenerates output for wireframe.",
   "kg",	SetGauss,	0,	":\t\t\tGenerates Gaussian curvature data.",
   "km",	SetMean,	1,
":\t\t\t(-1) Generates Misc curvature data:\n\t\t\t\t-1: Mean, 0: Thin plate, 1: PrincipalK",
   "iso",		LineMode,	0,	":\t\t\tGenerate isolines in g3d form.",
   "os",	SetOffset,	1,	"offset: \t\tTessellate the offset surface.",
   "boundary",	SetBoundary, 	0,	": \t\tOnly sample the boundaries.",
   "split",	DoSplit,	0,	": \t\tSplit spatches at center.",
   "g3d",	OutG3d,		0,	": \t\tOutput a3d.",
   "a3d",	OutG3d,		0,	": \t\tOutput a3d.",
   "s3d",	SetS3d,		0,	": \t\tOutput in S3d form.",
   "iv",	SetIV,		0,	": \t\tOutput in OpenInventor iv form.",
   "cb",	SetCB,		1,	"s:\t\tOutput s3d crossboundaries.",
   "boundaries",SetBoundaries,	2,	"offset width:\t\t\tOutput patch boundaries (s3d).\n",
   "ps",	OutPS,		1,	"s: Output ps projected in x-y plane.",

/* Do not delete this next line */
   NULL,	NULL,		0,	NULL
   };

char* Banner = "Tess";
char* UsageString = "Tess [options] < bezier-triangle-stream > dstruct-triangle-stream";

