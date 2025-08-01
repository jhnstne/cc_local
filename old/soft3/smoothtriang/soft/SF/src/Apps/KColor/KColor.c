/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 ** KColor.c
 **
 ** Author: Michael Lounsbery
 ** Created: Fri Aug 11, 1989 at 01:11:48 PM
 **
 ** Purpose: Tags vertices already tagged with Gaussian or Mean curvature
 **		with more visually-oriented colors.
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

#define	K_GAUSS	1
#define K_MEAN	2

#define	HalfHueRange	(HueRange / 2.0)
#define kdiff		(kmax - kmin)


void		KColor(Lnode* l, int WhichKurvature);
void InsertMat(Material mat, Lnode *Vertex);

static int	KurveType = K_GAUSS;	/* default is Gaussian */
static int	PrintInfo = 0;		/* Says whether to print info */
static Scalar	HueRange = 240.0;	/* probably won't change */
static Scalar	Gamma = 0.625;		/* shape of hue interpolation */
static Scalar	kmax = 4.0;		/* max curvature value */
static Scalar	kmin = -4.0;		/* min curvature value */
static Scalar	MaxK;
static Scalar	MinK;
static int 	yack = 0;		/* non-zero if yack */
static int	s3d = 0;



void ColorS3d(Scalar* mink, Scalar* maxk)
{
	char buf[1000];
	Material mat;
	float kval;

	while ( fgets(buf, 1000, stdin) != NULL ) {
		if (sscanf(buf, "# gk %f", &kval)==1) {
			fputs(buf,stdout);
			mat = KMat(kval, kmin, kmax, Gamma);
			fprintf(stdout, "d %f %f %f\n",
				mat.diffuse.r,mat.diffuse.g,mat.diffuse.b);
			fprintf(stdout, "s %f %f %f\n",
				mat.specular.r,mat.specular.g,mat.specular.b);
			fprintf(stdout, "g %d 0 0\n",mat.specularity);
			if (kval < *mink)
			  *mink = kval;
		
			if (kval > *maxk)
			  *maxk = kval;
		} else {
			fputs(buf, stdout);
		}
	}
}


void ColorDstruct(Scalar* mink, Scalar* maxk)
{
	while( ReadDstruct() != EOF ){
		CopyDstructFields("Triangle","Triangle");
		
		if ( yack ) {
			char* buf;
			GetString("Triangle.name", &buf);
			fprintf(stderr,"%s\n",buf);
		}
		
		KColor(dout, KurveType);
		
		FlushDstruct();
	}
}

main(int argc, char* argv[])
{
	MaxK = kmin;		/* initialize to silly values */
	MinK = kmax;
	
	ParseCommandLine(argc,argv);
	
	if ( s3d ) {
		ColorS3d(&MinK, &MaxK);
	} else {
		ColorDstruct(&MinK, &MaxK);
	}
	
	if (PrintInfo) {
		fprintf(stderr, 
			"\nMaximum was %lg, minimum was %lg\n", MaxK, MinK);
	}
	exit(0);
}


/*
 ** Given a triangle lnode already tagged with curvature info, adds an
 ** appropriate RGB material property to each vertex of the triangle
 */

void	KColor(Lnode* l, int WhichKurvature)
{
	Lnode	*Cur, *ThisVertex;
	int		i;
	Material	mat;
	double	Kval;
	char		*Field, FieldMem[81], *KStr, KStrMem[81];
	
	Field = FieldMem;
	KStr = KStrMem;
	
	if ((l == NULL) || (l->car == NULL) || (strcmp(l->car->name, "Triangle")))      {
		fprintf(stderr, "ERROR in KColor: Lnode isn't a valid triangle\n");
		exit(1);
	}
	
	switch(WhichKurvature)      {
	      case K_GAUSS:
		strcpy(KStr, "curvature.gaussian");
		break;
		
	      case K_MEAN:
		strcpy(KStr, "curvature.mean");
		break;
		
	      default:
		fprintf(stderr, "ERROR in KColor: Unknown curvature type: %d\n", WhichKurvature);
		exit(1);
	}
	
	for (i = 0; i <= 2; i++)      {
		sprintf(Field, "Triangle.vertex%d", i + 1);
		
		if (!dLookUpLnode(l, Field, Cur))
		  {
			  fprintf(stderr, "ERROR in KColor: vertex%d not found\n", i + 1);
			  exit(1);
		  }
		
		ThisVertex = Cur->cdr;
		
		if (!dGetScalar(ThisVertex, KStr, &Kval))	 {
			fprintf(stderr, "ERROR in KColor: \"%s\" not found on vertex\n", KStr);
			exit(1);
		}
		
		if (Kval < MinK)
		  MinK = Kval;
		
		if (Kval > MaxK)
		  MaxK = Kval;
		
		mat = KMat(Kval, kmin, kmax, Gamma);

		InsertMat(mat, ThisVertex);
	}
}



void		InsertMat(Material	mat, Lnode		*Vertex)
{
	dPutScalar(Vertex, "material.mat.diffuse[0]", mat.diffuse.r);
	dPutScalar(Vertex, "material.mat.diffuse[1]", mat.diffuse.g);
	dPutScalar(Vertex, "material.mat.diffuse[2]", mat.diffuse.b);
	
	dPutScalar(Vertex, "material.mat.specular[0]", mat.specular.r);
	dPutScalar(Vertex, "material.mat.specular[1]", mat.specular.g);
	dPutScalar(Vertex, "material.mat.specular[2]", mat.specular.b);
	
	dPutScalar(Vertex, "material.mat.specularity", (Scalar) mat.specularity);
}


/* command line routines */

void	SetMean()
{
	KurveType = K_MEAN;
}


void	SetGaussian()
{
	KurveType = K_GAUSS;
}


void	DoInfo()
{
	PrintInfo = 1;
}


void	SetMin(char	**a)
{
	kmin = atof(a[0]);
}


void	SetMax(char	**a)
{
	kmax = atof(a[0]);
}


void	SetGamma(char	**a)
{
	Gamma = atof(a[0]);
}

void	DoYack()
{
	yack = 1;
}

void DoS3d()
{
	s3d = 1;
}

char* Banner="KColor";
char* UsageString = "KColor [options] < triangle-string";
Option Options[]={
    "h", Usage, 0, ": print available options.",
    "kg", SetGaussian, 0, ": Color by Gaussian curvature value (default)",
    "km", SetMean, 0, ": Color by mean curvature",
    "g", SetGamma, 1, "gamma : set hue interpolation degree (default = 0.625)",
    "max", SetMax, 1, "max k value : set maximum curvature value (default = 4.0)",
    "min", SetMin, 1, "min k value : set minimum curvature value (default = -4.0)",
    "info", DoInfo, 0, ": Give excess value warnings, print max, min info at end",
    "s3d", DoS3d, 0,	": Read/output s3d",
    "v", DoInfo, 0, ": Give excess value warnings, print max, min info at end",
    "yack",  DoYack, 0, ": Yack, yack, yack",
	
    NULL,NULL,     0, NULL
  };
