/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  toSticks.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include "stick.h"
#include "math.h"

/* global variables */
int intype = I_SGP;
int outtype = I_SGP;

double sphereRadius;
double stickRadius;
double tangentRadius;

Material sphereMaterial;
Material stickMaterial;
Material normalFrontMaterial;
Material normalBackMaterial;

Space world;
Frame worldF;

int noBalls=0;
int noSticks=0;
int noNormals=0;
int keep=0;
int autoSize=0;
float autoScale=1.;


/*
 *----------------------------------------------------------------------
 *  Function:  SetDefaults
 *  NOTES:
 *	Should make up a material if it can't find one in material 
 *  library instead of exiting.
 *----------------------------------------------------------------------
 */
void SetDefaults()
{
  sphereRadius = 0.15;
  tangentRadius = 0.45;
  stickRadius = 0.05;

  if ( !MaterialLookup( "RedPlastic", &sphereMaterial ) ) {
    fprintf(stderr,"SetDefaults: can't find material RedPlatic!  Exiting.\n");
    exit(1);
  }

  if ( !MaterialLookup( "BluePlastic", &stickMaterial ) ){
    fprintf(stderr,"SetDefaults: can't find material BluePlastic!  Exiting.\n");
    exit(1);
  }

  if ( !MaterialLookup( "YellowPlastic", &normalFrontMaterial ) ){
    fprintf(stderr,"SetDefaults: can't find material YellowPlastic!  Exiting.\n");
    exit(1);
  }

  if ( !MaterialLookup( "DarkGreenPlastic", &normalBackMaterial ) ){
    fprintf(stderr,"SetDefaults: can't find material DarkGreenPlastic!  Exiting.\n");
    exit(1);
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  OutputSphere
 *----------------------------------------------------------------------
 */
void OutputSphere(Point p)
{
  Sphere s;
  
  s.c = p;
  s.r = sphereRadius;
  s.m = sphereMaterial;

  TriangulateSphere(s);
}



/*
 *----------------------------------------------------------------------
 *  Function:  OutputNormal
 *----------------------------------------------------------------------
 */
void OutputNormal(Normal n, Point p)
{
  Square sq;

  sq.n = n;
  sq.p = p;
  sq.r = tangentRadius;
  sq.m = normalFrontMaterial;
  TriangulateNormal(sq);

  sq.n = VDual( SVMult( -1.0, NDual(sq.n) ) );
  sq.m = normalBackMaterial;
  TriangulateNormal(sq);
}


/*
 *----------------------------------------------------------------------
 *  Function:  OutputStick
 *----------------------------------------------------------------------
 */
void OutputStick(Point p1,Point p2)
{
  Stick s;

  s.p1 = p1;
  s.p2 = p2;
  s.r = stickRadius;
  s.m = stickMaterial;
  if ( quality == LOW_QUALITY  &&  outtype == I_A3D ) {
    PrintA3DLine(s);
  } else {
    TriangulateStick(s);
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  OutputColorStick
 *----------------------------------------------------------------------
 */
void OutputColorStick(Point p1, Point p2,
		      double dr, double dg, double db,
		      double sr,double sg, double sb,
		      double ph)
{
  Stick s;

  s.p1 = p1;
  s.p2 = p2;
  s.r = stickRadius;
  s.m.specularity = dr;
  s.m.diffuse.r = dr;
  s.m.diffuse.g = dg;
  s.m.diffuse.b = db;
  s.m.specular.r = sr;
  s.m.specular.g = sg;
  s.m.specular.b = sb;

  if ( quality == LOW_QUALITY  &&  outtype == I_A3D ) {
    PrintA3DLine(s);
  } else {
    TriangulateStick(s);
  }
}



/*
 *----------------------------------------------------------------------
 *  Function:  OutputTriangle
 *----------------------------------------------------------------------
 */
void OutputTriangle(Triangle t)
{
  int i;

  if ( autoSize ) {
    Scalar d;

    d = PPDist(t.p[0], t.p[1]);
    sphereRadius = autoScale*d/10.;
    stickRadius = autoScale*d/30.;
    tangentRadius = autoScale*d/4.;
fprintf(stderr,"scale %g %g %g\n",sphereRadius,stickRadius,tangentRadius);
    autoSize = 0;
  }
  for( i=0; i<3; i++){
    if ( CheckAndAddPoint(t.p[i], sphereRadius/10.0) ) {
      if ( !noBalls )
	OutputSphere(t.p[i]);
      if ( !noNormals )
	OutputNormal(t.n[i],t.p[i]);
    }
    if ( CheckAndAddStick(t.p[i], t.p[(i+1)%3], stickRadius/10.0 ) ) {
      if ( !noSticks )
	OutputStick(t.p[i],t.p[(i+1)%3]);
    }
  }
}



/*
 *----------------------------------------------------------------------
 *  Function:  ReadTriangle
 *----------------------------------------------------------------------
 */
int ReadTriangle(Triangle* t)
{
  switch(intype){
  case I_DSTRUCT:
    return ReadDstructTriangle(t);
    break;
  case I_SGP:
    return ReadSGPTriangle(t);
    break;
  default:
    fprintf(stderr,"ReadTriangle: unknown input format %d.  Exiting.\n",
	    intype);
    exit(1);
  }
}



main(int argc, char* argv[])
{
  SetDefaults();
  InitDataStructure();

  ParseCommandLine(argc, argv);

  world = SCreate("triangle world",3);
  worldF = StdFrame(world);

  InitDataStructure();

  switch( intype ) {
    case I_SGP:
    case I_DSTRUCT:
      ProcessTriangleStream();
      break;
    case I_SEGMENT:
      ProcessSegmentStream();
      break;
    default:
      fprintf(stderr,"%s: unknown input type %d.\n",argv[0]);
      exit(1);
  }

  exit(0);
}


ProcessTriangleStream()
{
  Triangle t;

  while( ReadTriangle(&t) != EOF ) {
    OutputTriangle(t);
  }
}


ProcessSegmentStream()
{
	char line[1000];
	Scalar x1,y1,z1;
	Scalar x2,y2,z2;
	double dr,dg,db;
	double sr,sg,sb;
	double ph;

	while( fgets(line, 1000, stdin) ) {
		if ( strlen(line) == 999 ) {
			fprintf(stderr,"ProcessSegmentStream: line too long.\n");
			exit(1);
		}
		if ( sscanf(line," %lf %lf %lf  %lf %lf %lf  %lf %lf %lf %lf %lf %lf  %lf",
			    &x1,&y1,&z1,&x2,&y2,&z2,&dr,&dg,&db,&sr,&sg,&sb,&ph) == 13 ) {
			OutputColorSegment(x1,y1,z1,x2,y2,z2,dr,dg,db,sr,sg,sb,ph);
		} else if ( sscanf(line," %lf %lf %lf  %lf %lf %lf  %lf %lf %lf",
			    &x1,&y1,&z1,&x2,&y2,&z2,&dr,&dg,&db) == 9 ) {
			OutputColorSegment(x1,y1,z1,x2,y2,z2,dr,dg,db,0.,0.,0.,0.);
		} else if ( sscanf(line," %lf %lf %lf  %lf %lf %lf",
			    &x1,&y1,&z1,&x2,&y2,&z2) == 6 ) {
			OutputSegment(x1,y1,z1,x2,y2,z2);
		}
	}
}

OutputColorSegment(Scalar x1,Scalar y1,Scalar z1,
		   Scalar x2,Scalar y2,Scalar z2,
		   double dr,double dg,double db,
		   double sr,double sg,double sb,
		   double ph)
{
    Point p1;
    Point p2;

    p1 = PCreate(worldF,x1,y1,z1);
    p2 = PCreate(worldF,x2,y2,z2);

    if ( autoSize ) {
	    double d;
	    d = PPDist(p1,p2);
	    sphereRadius = autoScale*d/10.;
	    stickRadius = autoScale*d/30.;
	    tangentRadius = autoScale*d/4.;
fprintf(stderr,"scale %g %g %g\n",sphereRadius,stickRadius,tangentRadius);
	    autoSize = 0;
    }
    if ( CheckAndAddPoint(p1, sphereRadius/10.0) ) {
      if ( !noBalls )
	OutputSphere(p1);
    }
    if ( CheckAndAddPoint(p2, sphereRadius/10.0) ) {
      if ( !noBalls )
	OutputSphere(p2);
    }
    if ( CheckAndAddStick(p1, p2, stickRadius/10.0 ) ) {
      if ( !noSticks )
	OutputColorStick(p1, p2, dr,dg,db, sr,sg,sb, ph);
    }
}


OutputSegment(Scalar x1,Scalar y1,Scalar z1, 
	      Scalar x2,Scalar y2,Scalar z2)
{
    Point p1;
    Point p2;

    p1 = PCreate(worldF,x1,y1,z1);
    p2 = PCreate(worldF,x2,y2,z2);

    if ( autoSize ) {
	    double d;
	    d = PPDist(p1,p2);
	    sphereRadius = autoScale*d/10.;
	    stickRadius = autoScale*d/30.;
	    tangentRadius = autoScale*d/4.;
fprintf(stderr,"scale %g %g %g\n",sphereRadius,stickRadius,tangentRadius);
	    autoSize = 0;
    }

    if ( CheckAndAddPoint(p1, sphereRadius/10.0) ) {
      if ( !noBalls )
	OutputSphere(p1);
    }
    if ( CheckAndAddPoint(p2, sphereRadius/10.0) ) {
      if ( !noBalls )
	OutputSphere(p2);
    }
    if ( CheckAndAddStick(p1, p2, stickRadius/10.0 ) ) {
      if ( !noSticks )
	OutputStick(p1, p2);
    }
}



/*------------------------------------------------------------*/
/*                   Command Line Handlers                    */
/*------------------------------------------------------------*/


void InputType(char* argv[])
{
  if ( strcmp(argv[0],"sgp") == 0 ) {
    intype = I_SGP;
  } else if ( strcmp(argv[0],"dstruct") == 0 ) {
    intype = I_DSTRUCT;
  } else if ( strcmp(argv[0],"segment") == 0 ) {
    intype = I_SEGMENT;
  } else {
    fprintf(stderr,"Unknown input type '%s'.  Exiting.\n",argv[0]);
    exit(1);
  }
    
}


void  Method(char* argv[])
{
  if ( strcmp(argv[0],"sgp") == 0 ) {
    outtype = I_SGP;
  } else if ( strcmp(argv[0],"dstruct") == 0 ) {
    outtype = I_DSTRUCT;
  } else if ( strcmp(argv[0],"a3d") == 0 ) {
    outtype = I_A3D;
  } else if ( strcmp(argv[0],"s3d") == 0 ) {
    outtype = I_S3D;
  } else {
    fprintf(stderr,"Unknown output type '%s'.  Exiting.\n",argv[0]);
    exit(1);
  }
}


void Quality(char* argv[])
{
  extern int delta;

  delta = 0;
}

void LowQuality(char* argv[])
{
	quality = LOW_QUALITY;
}


void NoBalls()
{
  noBalls = 1;
}


void BallRadius(char* argv[])
{
  sphereRadius = atof(argv[0]);
}


void BallColor(char* argv[])
{
  double atof();

  sphereMaterial.specularity = atof(argv[0]);
  sphereMaterial.diffuse.r = atof(argv[1]);
  sphereMaterial.diffuse.g = atof(argv[2]);
  sphereMaterial.diffuse.b = atof(argv[3]);
  sphereMaterial.specular.r = atof(argv[4]);
  sphereMaterial.specular.g = atof(argv[5]);
  sphereMaterial.specular.b = atof(argv[6]);
  sphereMaterial.name = NULL;
}

void BallMaterial(char* argv[])
{
  if ( !MaterialLookup( argv[0], &sphereMaterial) ){
    fprintf(stderr,"BallMaterial: material `%s' doesn't exist.  Exiting.\n",
	    argv[0]);
    exit(1);
  }
}

void NoSticks()
{
  noSticks = 1;
}

void SetStickRadius(char* argv[])
{
  stickRadius = atof(argv[0]);
}

void StickColor(char* argv[])
{
  double atof();

  stickMaterial.specularity = atof(argv[0]);
  stickMaterial.diffuse.r = atof(argv[1]);
  stickMaterial.diffuse.g = atof(argv[2]);
  stickMaterial.diffuse.b = atof(argv[3]);
  stickMaterial.specular.r = atof(argv[4]);
  stickMaterial.specular.g = atof(argv[5]);
  stickMaterial.specular.b = atof(argv[6]);
  stickMaterial.name = NULL;
}

void StickMaterial(char* argv[])
{
  if ( !MaterialLookup( argv[0], &stickMaterial) ){
    fprintf(stderr,"StickMaterial: material `%s' doesn't exist.  Exiting.\n",
	    argv[0]);
    exit(1);
  }
}

void NoNormals()
{
  noNormals = 1;
}

void NormalRadius(char* argv[])
{
  tangentRadius = atof(argv[0]);
}

void PCLNormalColor(char* argv[])
{
  double atof();

  normalFrontMaterial.specularity = atof(argv[0]);
  normalFrontMaterial.diffuse.r = atof(argv[1]);
  normalFrontMaterial.diffuse.g = atof(argv[2]);
  normalFrontMaterial.diffuse.b = atof(argv[3]);
  normalFrontMaterial.specular.r = atof(argv[4]);
  normalFrontMaterial.specular.g = atof(argv[5]);
  normalFrontMaterial.specular.b = atof(argv[6]);
  normalFrontMaterial.name = NULL;
}
void NormalMaterial(char* argv[])
{
  if ( !MaterialLookup( argv[0], &normalFrontMaterial) ){
    fprintf(stderr,"NormalMaterial: material `%s' doesn't exist.  Exiting.\n",
	    argv[0]);
    exit(1);
  }
}
void PCLBackNormalColor(char* argv[])
{
  double atof();

  normalBackMaterial.specularity = atof(argv[0]);
  normalBackMaterial.diffuse.r = atof(argv[1]);
  normalBackMaterial.diffuse.g = atof(argv[2]);
  normalBackMaterial.diffuse.b = atof(argv[3]);
  normalBackMaterial.specular.r = atof(argv[4]);
  normalBackMaterial.specular.g = atof(argv[5]);
  normalBackMaterial.specular.b = atof(argv[6]);
  normalBackMaterial.name = NULL;
}

void BackNormalMaterial(char* argv[])
{
  if ( !MaterialLookup( argv[0], &normalBackMaterial) ){
    fprintf(stderr,"BackNormalMaterial: material `%s' doesn't exist.  Exiting.\n",
	    argv[0]);
    exit(1);
  }
}


void SetAuto(char* arg[])
{
	autoScale = atof(arg[0]);
fprintf(stderr,"aS = %g\n",autoScale);
	autoSize = 1;
}



       /* Table of available options -- register new options here */
Option Options[] = {
/*  Name     Handler     #args   helpstring  */
    "help",  Usage,      0,      ": Print available options.",
    "i",     InputType,  1,	 "x: Input is of type x, where is 'sgp', 'dstruct' or 'segment' [sgp].",
    "o",     Method,     1,	 "x: Set output method x, \n\
               where 'x' is one of 'sgp', 'a3d', 'dstruct', or 's3d' [sgp].",
    "b",     NoBalls,  0,      ": Don't output balls.",
    "br",    BallRadius, 1,	 "radius: Set ball radius to 'radius'.  [.15]",
    "bm",    BallMaterial, 1,    "material: Set ball material to 'material'. [RedPlastic]",
    "bc",    BallColor,  7,      "s rd gd bd rs gs bs: set ball color.",
    "s",     NoSticks,  0,      ": Don't output sticks.",
    "sr",    SetStickRadius, 1,	 "radius: Set stick radius to 'radius'. [.05]",
    "sm",    StickMaterial, 1,    "material: Set stick material to 'material'. [BluePlastic]",
    "sc",    StickColor,  7,      "s rd gd bd rs gs bs: set stick color.",
    "n",     NoNormals,    0,     ": Don't output normals.",
    "nr",    NormalRadius, 1,     "radius: Set normal Radius to 'radius'. [.45]",
    "nm",    NormalMaterial, 1,   "material: Set normal material to 'material'. [YellowPlastic]",
    "nbm",   BackNormalMaterial, 1, "material: Set back normal material to 'material'. [DarkGreenPlastic]",
    "nc",    PCLNormalColor,  7,      "s rd gd bd rs gs bs: set normal color.",
    "nbc",   PCLBackNormalColor, 7, "s rd gd bd rs gs bs: set back normal color.",
    "Q",     Quality,    0,      ": Output 16 triangles for cylinders instead of 8.",
    "L",     LowQuality, 0,	 ": Output a octahedron instead of icosahedron for spheres.",
    "a",     SetAuto, 1,	 "scale: automatically guess a size based on first.\n\
			Scale the guess by 'scale'.",


/*  Do not delete the next line */
    NULL,    NULL,       0,      NULL,      NULL
  };


/* Global variables go here */
char *Banner  = " Last Modified: Thu Apr 12 13:52:59 PDT 1990 ";
char *CommandName;

char* UsageString = "Stick [options] < stream-of-triangles";
