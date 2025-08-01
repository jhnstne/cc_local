/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  tG1.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

Space world;
Frame worldF;

double tol=0.0001;
int verbose = 1;
int gk=0;

void CheckAndAddPoint(Point p, Normal n, double tol, double gk);

void ProcessTriangle()
{
  Point p1,p2,p3;
  Normal n1,n2,n3;
  Scalar gk0,gk1,gk2;

  if ( !GetVertexPosition("Triangle.vertex1",worldF,&p1) ||
       !GetVertexPosition("Triangle.vertex2",worldF,&p2) ||
       !GetVertexPosition("Triangle.vertex3",worldF,&p3) ||
       !GetVertexNormal("Triangle.vertex1",worldF,&n1) ||
       !GetVertexNormal("Triangle.vertex2",worldF,&n2) ||
       !GetVertexNormal("Triangle.vertex3",worldF,&n3) ) {
    fprintf(stderr,"Invalid triangle.  Exiting.\n");
    exit(1);
  }

  if ( gk ) {
    if ( !GetScalar("Triangle.vertex1.curvature.gaussian",&gk0) ||
	 !GetScalar("Triangle.vertex2.curvature.gaussian",&gk1) ||
	 !GetScalar("Triangle.vertex3.curvature.gaussian",&gk2) ) {
      fprintf(stderr,"No curvature on triangle.  Exiting.\n");
      exit(1);
    }
  } else {
    gk0 = 0.;
    gk1 = 0.;
    gk2 = 0.;
  }

  CheckAndAddPoint(p1,n1,tol,gk0);
  CheckAndAddPoint(p2,n2,tol,gk1);
  CheckAndAddPoint(p3,n3,tol,gk2);
}



int findMin=0;
double minDot;
double maxGk;

main(int argc,char* argv[])
{
  int triCount=0;

  InitDataStructure();

  ParseCommandLine(argc, argv);

  world = SCreate("triangle world",3);
  worldF = StdFrame(world);

  minDot = 1.0;

  InitDataStructure();

  while( ReadDstruct() != EOF ) {
    triCount++;
    ProcessTriangle();
  }

  if ( findMin ) {
    fprintf(stderr,"Minimum dot product == %g, angle = %g radians = %g degrees.\n",minDot,
	    acos(minDot),acos(minDot)*180.0/3.14159);
    if ( gk ) {
      fprintf(stderr,"Max gk diff == %g\n",maxGk);
    }
  } else if ( verbose ){
    fprintf(stderr,"%d triangles, all with common tangent plane at common vertices.\n");
  }

  exit(0);
}







/*------------------------------------------------------------*/
/*                   Command Line Handlers                    */
/*------------------------------------------------------------*/

void SetTol(char* a[])
{
  tol = atof(a[0]);
  if ( tol < 0.0 ) {
    fprintf(stderr,"SetTol: tolerance less than zero.  Exiting.\n");
    exit(1);
  }
}

void SetVerbose()
{
  verbose = 1;
}

void SetFindMin()
{
  findMin = 1;
}

void SetGK()
{
  gk = 1;
}


       /* Table of available options -- register new options here */
Option Options[] = {
/*  Name     Handler     #args   helpstring  */
    "help",  Usage,      0,      ": Print available options.",
    "t",     SetTol,	 1,	 "tolerance: set tolerance.",
    "v",     SetVerbose, 0,	 ": Set verbose mode.",
    "m",     SetFindMin, 0,	 ": Find the minimum dot product between two normals.",
    "gk",    SetGK,	 0,	 ": Check Gaussian Curvature as well as TP.",

/*  DO not delete the next line */
    NULL,    NULL,       0,      NULL,      NULL
  };


/* Global variables go here */
char *Banner  = "";
char *CommandName;

char* UsageString = "TG1 < triangle-stream";
