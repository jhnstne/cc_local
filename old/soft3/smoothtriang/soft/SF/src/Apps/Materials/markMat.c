/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  markMat.c
 *	Color based on the mark of the object.
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
int colorBar=0;


#define NUM_COLORS 14

static char* colorTable[NUM_COLORS] = {
	"RedPlastic",
	"NewBlue",
	"NewPurple",
	"NewGreen",
	"DarkGreenPlastic",
	"YellowPlastic",
	"PurplePlastic",
	"RedMatte",
	"GreenPlastic",
	"GreenMatte",
	"BluePlastic",
	"BlueMatte",
	"YellowMatte",
	"PurpleMatte"
  };

static numMaterials = NUM_COLORS;
static Material colorMaterial[NUM_COLORS];
static short colorUsedTable[NUM_COLORS];

AddMaterial(int mark, int type)
{
	if ( mark < numMaterials ) {
		colorUsedTable[mark] = 1;
		if ( type == MAT_NAME ) {
			SetMaterialName("Triangle.vertex1", colorTable[mark]);
			SetMaterialName("Triangle.vertex2", colorTable[mark]);
			SetMaterialName("Triangle.vertex3", colorTable[mark]);
		} else if ( type == MAT_RGB ) {
			SetRGBMaterial("Triangle.vertex1", 
				       &colorMaterial[mark]);
			SetRGBMaterial("Triangle.vertex2", 
				       &colorMaterial[mark]);
			SetRGBMaterial("Triangle.vertex3", 
				       &colorMaterial[mark]);
		} else {
			fprintf(stderr, "matFlag = %d not imp.\n", matFlag);
			exit(1);
		}
	} else {
		SetMaterialName("Triangle.vertex1", "NullMaterial");
		SetMaterialName("Triangle.vertex2", "NullMaterial");
		SetMaterialName("Triangle.vertex3", "NullMaterial");
	}
}

PrintColorsUsed()
{
	int i;

	for(i=0; i<NUM_COLORS; i++){
		if ( colorUsedTable[i] ) {
			switch (matFlag) {
			      case MAT_NAME:
				fprintf(stderr, "Color %d = %s\n", i, 
					colorTable[i]);
				break;
			      case MAT_RGB:
				fprintf(stderr, "Color %d\n", i);
				break;
			}
		}
	}	
}

#if 0
fOutputColor(FILE* fp, Material mat, int i)
{
	fprintf(fp, "%g %g %g d\n",mat.diffuse.r,mat.diffuse.g,mat.diffuse.b);
	fprintf(fp, "%g %g %g s\n",
		mat.specular.r,mat.specular.g,mat.specular.b);
	fprintf(fp, "%d 0 0 p\n",mat.specularity);
	fprintf(fp, "0 %g %g m\n", .1, i+.1);
	fprintf(fp, "0 %g %g l\n", .1, i+.9);
	fprintf(fp, "0 %g %g l\n", .9, i+.9);
	fprintf(fp, "0 %g %g l\n", .9, i+.1);
}
#else
fOutputColor(FILE* fp, Material mat, int i)
{
	fprintf(fp, "d %g %g %g\n",mat.diffuse.r,mat.diffuse.g,mat.diffuse.b);
	fprintf(fp, "s %g %g %g\n",
		mat.specular.r,mat.specular.g,mat.specular.b);
	fprintf(fp, "g %d %d %d\n",mat.specularity,mat.specularity,mat.specularity);
	fprintf(fp, "P 4 0 0\n");
	fprintf(fp, "n 1 0 0\n");
	fprintf(fp, "v 0 %g %g\n", .1, i+.1);
	fprintf(fp, "n 1 0 0\n");
	fprintf(fp, "v 0 %g %g\n", .1, i+.9);
	fprintf(fp, "n 1 0 0\n");
	fprintf(fp, "v 0 %g %g\n", .9, i+.9);
	fprintf(fp, "n 1 0 0\n");
	fprintf(fp, "v 0 %g %g\n", .9, i+.1);
	fprintf(fp, "E 0 0 0\n");
}
#endif

fOutputA3dColorBar(FILE* fp, int n)
{
	int i;

	for (i=0; i<=n; i++){
		if ( matFlag == MAT_NAME ) {
			MaterialLookup(colorTable[i], &(colorMaterial[i]));
		}
		fOutputColor(fp, colorMaterial[i], i);
	}
}

main(int argc, char* argv[])
{
	int mark;
	Scalar fmark;
	
	ParseCommandLine(argc, argv);
	
	if ( colorBar > 0 ) {
		fOutputA3dColorBar(stdout, colorBar);
		exit(0);
	}
	while( ReadDstruct() != EOF ){
		if ( !QueryDstructPath( "Triangle" ) ) {
			fprintf(stderr, "\
%s: dstruct not Triangle.  Exiting.\n", argv[0]);
			exit(1);
		}
		CopyDstructFields("Triangle", "Triangle");
		
		GetScalar("Triangle.mark", &fmark);
		mark = fmark;
		AddMaterial(mark, matFlag);
		FlushDstruct();
	}
	PrintColorsUsed();
	exit(0);
}


#define MAX_LINE 1000

void SetColorTable(char* arg[])
{
	char* file = arg[0];
	FILE* fp;
	char buf[MAX_LINE];
	int i;

	matFlag = MAT_RGB;
	if ( ! (fp = fopen(file, "r")) ) {
		fprintf(stderr, "SetColorTable: can't open file '%s'.\n",
			file);
		exit(1);
	}

	for ( i=0; i<NUM_COLORS; i++ ) {
		if ( !MaterialRead(fp, &(colorMaterial[i])) ) {
			break;
		}
	}
	numMaterials = i;

	fclose(fp);
}

void ColorBar(char* arg[])
{
	colorBar = atoi(arg[0]);
	if ( colorBar < 1 ) {
		fprintf(stderr, "ColorBar: # colors < 1.  Ignoring.\n");
	}
}

char* Banner="MarkMat";
char* UsageString = "MarkMat [options] < triangle-stream";
Option Options[]={
	"h", Usage, 0, ": print available options.",
	"ct", SetColorTable, 1, "file: read the color table from 'file'",
	"cb", ColorBar, 1, "n: output an a3d color bar of colors 0 to n.",

	NULL,NULL,     0, NULL
  };
