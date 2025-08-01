/*
 * Copyright (c) 1996, Computer Graphics Laboratory, University of Waterloo
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  sphere.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include <math.h>

Scalar radius =1.0;
int numSegs1 =5;	/* from top to bottom */
int numSegs2 =10;	/* around the center */
int curvature=1;

#define MAX 100

void PrintPt (char* name, Point point, Vector norm);

main(int argc,char* argv[])
{
  int i,j;
  Space w;
  Frame f;
  Point p1;
  Point p2;
  Point o;
  int current=0;
  Vector dz;
  Scalar x,y,z;
  char name[20];
  double r;

  w = SCreate("world",3);
  dz = FV( StdFrame(w), 2);
  o = FOrg( StdFrame(w) );

  ParseCommandLine(argc,argv);

  /* the top and bottom */
  PrintPt("Ptop",PCreate(StdFrame(w),0.0,0.0,radius),
	     VCreate(StdFrame(w),0.0,0.0,1.0));
  PrintPt("Pbot",PCreate(StdFrame(w),0.0,0.0,-radius),
	     VCreate(StdFrame(w),0.0,0.0,-1.0));
  for(i=1; i<numSegs1; i++){
    z = radius*sin( (float)i/(float)numSegs1 * M_2_PI/2 - M_2_PI/4 ); 
    r = radius*cos( (float)i/(float)numSegs1 * M_2_PI/2 - M_2_PI/4 );
    for(j=0; j<numSegs2; j++){
      p1 = PCreate( StdFrame(w), r*cos(M_2_PI*(float)j/(float)numSegs2),
		                 r*sin(M_2_PI*(float)j/(float)numSegs2), z);
      sprintf(name,"P%dx%d",i,j);
      PrintPt(name,p1,PPDiff(p1,o));
    }
  }

  /* print the faces */
  for(j=0; j<numSegs2; j++){
    printf("FBx%d = [ Pbot, P1x%d, P1x%d ];\n",j,(j+1)%(numSegs2), j );
  }
  for(i=1; i<numSegs1-1; i++){
    for(j=0; j<numSegs2; j++){
      printf("Fa%dx%d = [ P%dx%d, P%dx%d, P%dx%d ];\n", i,j,
	     i,j,  i+1, (j+1)%(numSegs2), i+1,j );
    }
  }
  for(j=0; j<numSegs2; j++){
    printf("FTx%d = [ Ptop, P%dx%d, P%dx%d ];\n", j, 
	                  numSegs1-1, j, numSegs1-1, (j+1)%(numSegs2) );
  }
  for(i=1; i<numSegs1-1; i++){
    for(j=0; j<numSegs2; j++){
      printf("Fb%dx%d = [ P%dx%d, P%dx%d, P%dx%d ];\n", i,j,
	     i,j,  i,(j+1)%numSegs2, i+1, (j+1)%numSegs2 );
    }
  }

  /* print the mesh */
  printf("mesh = {\n");
  for(j=0; j<numSegs2; j++){
    printf("\tFBx%d, FTx%d,\n",j,j);
  }
  for(i=1; i<numSegs1-1; i++){
    for(j=0; j<numSegs2; j++){
      printf("\tFa%dx%d, Fb%dx%d",i,j,i,j);
      if ( i != numSegs1-2  ||  j != numSegs2-1 )
	printf(",\n");
      else
	printf("\n");
    }
  }
  printf("};\n");
}


void Radius(a)
char* a[];
{
  radius = atof(a[0]);
}


void NumSegs1(a)
char* a[];
{
  numSegs1 = atoi(a[0]);
}


void NumSegs2(a)
char* a[];
{
  numSegs2 = atoi(a[0]);
}

void SetNoCurvature()
{
  curvature = 0;
}

/* Table of available options -- register new options here */
Option Options[] = {
/*  Name     Handler     #args   helpstring  */
    "help",  Usage,      0,      ": Print available options.",
    "r",     Radius,     1,      "r: set radius of the sphere.",
    "ntb",     NumSegs1,    1,      "n1: set number of segments from top to bottom",
    "nc",     NumSegs2,    1,      "n1: set number of samples around the center",
    "nk",    SetNoCurvature,0,    ": don't tag vertices with second fundemental form.",

/*  Do not delete the next line */
    NULL,    NULL,       0,      NULL,      NULL
  };


/* Global variables go here */
char *Banner  = "";
char *CommandName;
char *UsageString = "Sphere [args]";
