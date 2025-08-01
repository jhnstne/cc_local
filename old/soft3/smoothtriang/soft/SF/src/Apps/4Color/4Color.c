/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  stellate.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"

Space world;
int maxColors=0;
int verbose=0;

void WriteVertex(v)
Vertex* v;
{
	printf("Nv%s = ",v->name);
	fdWriteDstructOneLine(stdout,v->externalData);
	printf(";\n");
}


#define RED 1
#define GREEN 2
#define BLUE 4
#define PURPLE 8

/*     luminosity = .299 red + .587 green + .114 blue */

void ColorFace(f)
Face* f;
{
	Face* fn;
	int c;
	char* color;
	
	c = 0;
	ForeachFaceFace(f,fn){
		if ( dGetString(fn->externalData,
				"Face.material.name",
				&color) ){
			if ( strcmp("RedPlastic",color) == 0 ){
				c |= RED;
			} else if ( strcmp("NewBlue",color) == 0 ){
				c |= GREEN;
			} else if ( strcmp("NewGreen",color) == 0 ){
				c |= BLUE;
			} else if ( strcmp("NewPurple",color) == 0 ){
				c |= PURPLE;
			}
		}
	} EndForeach;

	if ( !(c & RED) ){
		dSetMaterialName(f->externalData,"Face","RedPlastic");
		if ( maxColors < 1 ) maxColors=1;
	} else   if ( !(c & GREEN) ){
		dSetMaterialName(f->externalData,"Face","NewBlue");
		if ( maxColors < 2 ) maxColors=2;
	} else   if ( !(c & BLUE) ){
		dSetMaterialName(f->externalData,"Face","NewGreen");
		if ( maxColors < 3 ) maxColors=3;
	} else   if ( !(c & PURPLE) ){
		dSetMaterialName(f->externalData,"Face","NewPurple");
		if ( maxColors < 4 ) maxColors=4;
	} else {
		fprintf(stderr,"4Color: mesh not 4 colorable.  Exiting.\n");
		exit(1);
	}
}




main(argc,argv)
int argc;
char* argv[];
{
	Mesh* m;
	Face* f;
	Face* f2;
	Vertex* v;
	
	ParseCommandLine(argc,argv);
	
	m = MeshParse(stdin);
	
	/* NOTE: should really delete all the old colors first */
	
	ForeachMeshFace(m,f){
		ForeachFaceFace(f,f2){
			ColorFace(f2);
		} EndForeach;
	}EndForeach;
	
	WriteMesh(m,stdout);
	if ( verbose ) {
		fprintf(stderr, "4Color: used %d colors.\n",maxColors);
	}
}


char* Banner = "4Color";
char* UsageString = "4Color [options] < mesh > mesh";

void Verbose()
{
	verbose = 1;
}

Option Options[]={
  "h",Usage,0,": \tprint help message.",
  "v",Verbose,0,": \t verbose.",
  NULL,NULL,0,NULL
  };
