/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** radial.c
**
** Purpose: Set color based on radial distance from surface of sphere.
*/


#include <math.h>
#include <stdio.h>
#include "all.h"


void	Mat1(), InsertMat();

Material	MatCombo();

#define	HalfHueRange	(HueRange / 2.0)
static Scalar	HueRange = 240.0;	/* probably won't change */
static Scalar	Gamma = 0.625;		/* shape of hue interpolation */

double sphereRadius = 1.0;
Space world;
Frame stdframe;

#define SPHERE 1
#define TORUS 2
#define CAPSULE 3

static int verbose = 0;
static int type=SPHERE;
static int deltadist=0;

double maxDist = -1000;
double minDist = 1000;
double maxNdot = -2;
double minNdot = 2;
double dmax = 1.1;
double dmin = .9;




/*
 ** Routines from Charles for manipulating HLS
 */

/*
 *----------------------------------------------------------------------
 *  Function:  VALUE
 *----------------------------------------------------------------------
 */
static double VALUE(double n1, double n2, double hue)
{
	while (hue>360.0) hue = hue-360.0;
	while (hue<  0.0) hue = hue+360.0;
	if (hue< 60.0) return (n1 + (n2-n1)*hue/60.0);
	if (hue<180.0) return (n2);
	if (hue<240.0) return (n1 + (n2-n1)*(240.0-hue)/60.0);
	return (n1);
}

/*
 *----------------------------------------------------------------------
 *  Function:  HLStoRGB
 *----------------------------------------------------------------------
 */
static void HLStoRGB(double h, double l, double s, 
		     double *r, double  *g, double *b)
{
	double m1,m2;
	if (l<=0.5) m2 = l*(l+s); else m2 = l+s-(l*s);  
	m1 = 2*l - m2;
	if (s==0.0) {
		*r = l; *g = l; *b = l;
	} else {
		*r = VALUE(m1,m2,h+120.0);
		*g = VALUE(m1,m2,h);
		*b = VALUE(m1,m2,h-120.0);
	}
}

/*
 ** Given a distance d, this maps it into HLS space, and from there
 ** into RGB space.
 **
 ** Stolen heavily from Michael, who said it was "Stolen heavily from 
** Charles Loop" (although there it was used for Gaussian Curvature )
*/
/*
 *----------------------------------------------------------------------
 *  Function:  DMat
 *----------------------------------------------------------------------
 */
Material DMat(double	d)
{
	double	Oldd = d;
	Material	mat;
	Scalar	r, g, b;

	/* map k to hue */
  
	if ( d > dmax ){
		d = dmax;
	} else if ( d < dmin ) {
		d = dmin;
	}

	d = sphereRadius - d;

	/* map to -1:1 */
	if ( d > 0.0 ) {
		d = d/(dmax-sphereRadius);
	} else {
		d = d/(sphereRadius-dmin);
	}

	if (d < 0.0) {
		d = HalfHueRange*(1.0 - pow(-d, Gamma));
	} else {
		d = HalfHueRange*(1.0 + pow(d, Gamma));
	}

	HLStoRGB(d, 0.5, 0.9, &r, &g, &b);
  
	mat.diffuse.r  = r;
	mat.diffuse.g  = g;
	mat.diffuse.b  = b;
  
	mat.specular.r  = 0.0;
	mat.specular.g  = 0.0;
	mat.specular.b  = 0.0;
  
	mat.specularity = 1;
  
	return	mat;
}


/*
 *----------------------------------------------------------------------
 *  Function:  CalcDistance
 *----------------------------------------------------------------------
 */
double CalcDistance(Point p, Normal n)
{
	Scalar x,y,z;
	double m;
	Point c;
	Scalar dist, ndot;

	switch(type){
	      case SPHERE:
		ndot = NVApply(n, VNormalize(PPDiff(p, FOrg(stdframe))));
		if ( deltadist == 0 ) {
			dist = PPDist( p, FOrg(stdframe) );
		} else {
			dist = fabs(1-PPDist( p, FOrg(stdframe) ));
		}
		break;
	      case TORUS:
		PCoords(p,stdframe,&x,&y,&z);
		m = sqrt(x*x + y*y);
		c = PCreate(stdframe,x/m,y/m,0.0);
		dist = 2*PPDist(p, c);
		ndot = NVApply(n, VNormalize(PPDiff(p,c)));
		break;
	      case CAPSULE:
		{
			static first=1;
			Normal n1,n2;
			Point p1,p2;

			ndot = 0; /* too lazy to implement this */
			/* build data to help us figure distance */
			if ( first ) {
				n1 = NCreate(stdframe,0.0,0.0,1.0);
				p1 = PCreate(stdframe,0.0,0.0,2.0);
				n2 = NCreate(stdframe,0.0,0.0,-1.0);
				p2 = PCreate(stdframe,0.0,0.0,-2.0);
			}

			/* find region point is in */
			if ( NVApply(n1,PPDiff(p,p1)) > 0.0 ) {
				return PPDist( p, p1 );
			} else if ( NVApply(n2,PPDiff(p,p2)) > 0.0 ) {
				return PPDist( p, p2 );
			} else {
				PCoords(p,stdframe,&x,&y,&z);
				return (sqrt(x*x + y*y));
			}
		}
	      default:
		fprintf(stderr,"Radial: Type %d not implemented.  Exiting.\n",type);
		exit(1);
	}
	if ( maxDist < dist ) maxDist = dist;
	if ( minDist > dist ) minDist = dist;

	if ( maxNdot < ndot ) maxNdot = ndot;
	if ( minNdot > ndot ) minNdot = ndot;
}



/*
 *----------------------------------------------------------------------
 *  Function:  AssignRadialColor
 *----------------------------------------------------------------------
 */
void AssignRadialColor()
{
	Point p;
	Normal n;
	double dist;
	int i;
	char vname[100];
	Material M;
  
	for(i=1;i<=3;i++){
		sprintf(vname,"Triangle.vertex%d",i);
		if ( !dGetVertexPosition(din,vname,stdframe,&p) ){
			fprintf(stderr,"AssignRadialColor: invalid triangle.  Exiting.\n");
			exit(1);
		}
		sprintf(vname,"Triangle.vertex%d",i);
		if ( !dGetVertexNormal(din,vname,stdframe,&n) ) {
			fprintf(stderr,"AssignRadialColor: No normals.  Exiting.\n");
			exit(1);
		}
		dist = CalcDistance(p, n);
 
		M = DMat(dist);
		pSetRGBMaterial(&dout,vname,&M);
	}
}


main(int argc, char* argv[])
{
	char outputString[1000];

	ParseCommandLine(argc,argv);
	world = SCreate("world",3);
	stdframe = StdFrame(world);

	while( ReadDstruct() != EOF ){
		CopyDstructFields("Triangle","Triangle");
		AssignRadialColor();
		FlushDstruct();
	}
	if ( verbose ) {
		fprintf(stderr,"max normalized dist = %g, min normalized dist = %g\n",maxDist,minDist);
		fprintf(stderr," max ndot = %g, min ndot = %g\n",
			maxNdot, minNdot);
	}
	exit(0);
}




void SetVerbose()
{
	verbose = 1;
}

void SetMax(char* a[])
{
	dmax = atof(a[0]);
}


void SetMin(char* a[])
{
	dmin = atof(a[0]);
}

void SetGamma(char* a[])
{
	Gamma = atof(a[0]);
}

void TypeTorus()
{
	type = TORUS;
}

void TypeCapsule()
{
	type = CAPSULE;
}

void TypeSphere()
{
	type = SPHERE;
}

void SetDelta()
{
	deltadist = 1;
}

char* Banner="Radial";
char* UsageString = "Radial [options] < triangle-string";
Option Options[]={
	"help", Usage,	0, ": print available options.",
	"h",  Usage, 		0, ": print available options.",
	"v",	SetVerbose,	0, ": verbose.",
	"max",SetMax,		1, "float : set the max value (default 1.1).",
	"min",SetMin,		1, "float : set the min value (default 0.9).",
	"gamma",SetGamma,	1, "float : set the gamma value (default .625).",
	"sphere",TypeSphere,  0, ": calc distance to sphere (default).",
	"torus",TypeTorus,	0, ": calc distance to torus.",
	"capsule",TypeCapsule,0, ": calc distance to capsule.",
	"delta", SetDelta,	0, ": Output distance to unit.",

	NULL,	NULL,     	0, NULL
};
