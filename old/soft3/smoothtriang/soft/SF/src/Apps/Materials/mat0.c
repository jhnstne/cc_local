/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  mat0.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

char* matName="RedPlastic";
Material m;

#define MAT_NAME 1
#define MAT_RGB 2
#define MAT_HSV 4
#define MAT_TRI 8

int matFlag=MAT_NAME;



main(int argc, char* argv[])
{
  char outputString[1000];

  ParseCommandLine(argc,argv);

  while( ReadDstruct() != EOF ){
    if ( !QueryDstructPath( "Triangle" ) ) {
      fprintf(stderr,"%s: dstruct not Triangle.  Exiting.\n",argv[0]);
      exit(1);
    }
    CopyDstructFields("Triangle","Triangle");

    switch(matFlag){
    case MAT_NAME:
      if ( ! QueryDstructPath( "Triangle.vertex1.material.name" ) )
	  SetMaterialName("Triangle.vertex1",matName);
      if ( ! QueryDstructPath( "Triangle.vertex2.material.name" ) )
	  SetMaterialName("Triangle.vertex2",matName);
      if ( ! QueryDstructPath( "Triangle.vertex3.material.name" ) )
	  SetMaterialName("Triangle.vertex3",matName);
      break;
    case MAT_NAME|MAT_TRI:
      SetMaterialName("Triangle",matName);
      break;
    case MAT_RGB:
      SetRGBMaterial("Triangle.vertex1",&m);
      SetRGBMaterial("Triangle.vertex2",&m);
      SetRGBMaterial("Triangle.vertex3",&m);
      break;
    case MAT_RGB|MAT_TRI:
      SetRGBMaterial("Triangle",&m);
      break;
    case MAT_HSV:
    case MAT_HSV|MAT_TRI:
      fprintf(stderr,"%s: MAT_HSV: not implemented yet.  Exiting.\n",argv[0]);
      exit(1);
    default:
      fprintf(stderr,"%s: case %d unknown.  Exiting.\n",argv[0],matFlag);
      exit(1);
    }
    FlushDstruct();
  }
  exit(0);
}


void Mat(char **a)
{
  matName = (char *)malloc( strlen(*a) +1 );
  strcpy( matName, *a);
  matFlag = MAT_NAME | (matFlag & MAT_TRI);
}


void SetColor(char **a)
{
  matFlag = MAT_RGB | (matFlag & MAT_TRI);

  m.diffuse.r   = atof(a[0]); 
  m.diffuse.g   = atof(a[1]); 
  m.diffuse.b   = atof(a[2]); 
  m.specular.r  = atof(a[3]); 
  m.specular.g  = atof(a[4]); 
  m.specular.b  = atof(a[5]); 
  m.specularity = atof(a[6]);  /* ugh!  specularity is type int :-( */
}


void SetTri()
{
  matFlag |= MAT_TRI;
}


char* Banner="Mat0";
char* UsageString = "Mat0 [options] < triangle-string";
Option Options[]={
  "h", Usage, 0, ": print available options.",
  "m", Mat, 1, "material: get a material from material library.",
  "t", SetTri, 0, ": associate a color per triangle instead of per vertex.",
  "rgb", SetColor, 7, "rd gd bd  rs gs bs p: set color.",
  NULL,NULL,     0, NULL
  };
