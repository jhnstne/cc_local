/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  torus.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

Scalar radius1 =1.0;
Scalar radius2 =0.5;
int numSeg1 =10;
int numSeg2 =10;
double piFrac=1.0;

#define MAX 100
#define PI2 6.283185308
#define EPS 0.00001

/*
 *----------------------------------------------------------------------
 *  Function:  ComputeSFF
 *	Compute the second fundemental form at a point on the torus.
 *	NOTE: the vectors v0 and v1 are not unit length.  The curvatures
 *	in the matrix m in these direction reflect this.
 *----------------------------------------------------------------------
 */
void ComputeSFF(SFF* sff, Point p1, Point p2, 
		int i, int n1, double r1, int j, int n2, double r2)
{
	Scalar x,y,z;
	Scalar dx,dy,dz;
	Frame f;
	Vector vnormal;
	double rc;
	Scalar cosi,cosj,sini,sinj;

	f = StdFrame(SpaceOf(p1));
	vnormal = VNormalize(PPDiff(p2,p1));

	cosi = cos(piFrac*PI2*(float)i/(float)n1);
	cosj = cos(PI2*(float)j/(float)n2);
	sini = sin(piFrac*PI2*(float)i/(float)n1);
	sinj = sin(PI2*(float)j/(float)n2);
  
	sff->v0 = VCreate(f, 
			  -( r2*cosj + r1 )*sini, 
			  ( r2*cosj + r1 )*cosi, 
			  0.0);
	sff->v1 = VCreate(f, -r2 * sinj * cosi, -r2 * sinj * sini, r2*cosj );

	sff->m[0][0] = VVDot(vnormal, 
			     VCreate(f, -( r2*cosj + r1 )*cosi, 
				     -( r2*cosj + r1 )*sini, 0.0));

	sff->m[1][1] = VVDot(vnormal,
			     VCreate(f, -r2*cosj*cosi, -r2*cosj*sini, -r2*sinj));

	sff->m[0][1] = sff->m[1][0] =
	  VVDot(vnormal, VCreate(f, r2*sinj*sini, -r2*sinj*cosi, 0.0 ));
}




main(int argc, char* argv[])
{
	int i,j;
	Space w;
	Frame f;
	Point p1;
	Point p2;
	int current=0;
	Vector dz;
	Scalar x,y,z;
	SFF sff;

	w = SCreate("world",3);
	dz = FV( StdFrame(w), 2);

	ParseCommandLine(argc,argv);

	for(i=0; i<numSeg1; i++){
		for(j=0; j<numSeg2; j++){
			p1 = PCreate( StdFrame(w), radius1*cos(piFrac*PI2*(float)i/(float)numSeg1),
				     radius1*sin(piFrac*PI2*(float)i/(float)numSeg1), 0.0);
			f = FCreate("temp", p1, PPDiff(p1,FOrg(StdFrame(w))),
				    dz,
				    VVCross( dz, PPDiff(p1,FOrg(StdFrame(w))))
				    );
			p2 = PCreate(f, radius2*cos(PI2*(float)j/(float)numSeg2),
				     radius2*sin(PI2*(float)j/(float)numSeg2), 0.0);

			/* print the vertex */
			printf("P%dR%d = ",j,i);
			pPutVertexPosition(&dout,"Point",p2);
			pPutVertexNormal(&dout,"Point",VDual(VNormalize(PPDiff(p2,p1))));
			ComputeSFF(&sff,p1,p2,i,numSeg1,radius1,j,numSeg2,radius2);
			if ( VVDot(sff.v0,PPDiff(p2,p1)) > 1e-4  || VVDot(sff.v0,PPDiff(p2,p1)) > 1e-4){
				fprintf(stderr,"Oops!  %g %g\n",
					VVDot(sff.v0,PPDiff(p2,p1)),
					VVDot(sff.v1,PPDiff(p2,p1)));
			}
			pPutSFF(&dout,"Point.sff",&sff);
			WriteDstruct();
		}
	}

	/* print the faces */
	/* i has funny index range to allow for cleaner face traversal */
	/* each time in the inner loop we print the two faces adjacent
	   to edge j on ring i */
	for(i=1; i<=numSeg1; i++){
		for(j=0; j<numSeg2; j++){
			if ( i != numSeg1  ||  piFrac == 1.0 ) {
				printf("FaP%dR%d = [ P%dR%d, P%dR%d, P%dR%d ];\n", j,i%numSeg1, 
				       j,i%numSeg1, (j+1)%numSeg2,i%numSeg1, j,(i-1)%numSeg1);
			}
			if ( i != numSeg1-1  ||  piFrac == 1.0 ) {
				printf("FbP%dR%d = [ P%dR%d, P%dR%d, P%dR%d ];\n", j,i%numSeg1,
				       j,i%numSeg1, (j+1)%numSeg2,(i+1)%numSeg1, (j+1)%numSeg2,i%numSeg1);
			}
		}
	}

	/* print the mesh */
	printf("mesh = {\n");
	for(i=1; i<=numSeg1; i++){
		for(j=0; j<numSeg2; j++){
			int needComma;

			needComma = 0;

			if ( i != numSeg1  ||  piFrac == 1.0 ) {
				printf("\tFaP%dR%d", j,i%numSeg1 );
				needComma = 1;
			}
			if ( i != numSeg1-1  ||  piFrac == 1.0 ) {
				if ( needComma )
				  printf(", ");
				else
				  printf("\t");
				printf("FbP%dR%d", j,i%numSeg1);
			}
			if ( i !=numSeg1 || j!=numSeg2-1 )
			  printf(",\n");
			else
			  printf("\n");
		}
	}
	printf("};\n");
}


void Radius1(char* a[])
{
	radius1 = atof(a[0]);
}

void Radius2(char* a[])
{
	radius2 = atof(a[0]);
}

void NumSegs1(char* a[])
{
	numSeg1 = atoi(a[0]);
}

void NumSegs2(char* a[])
{
	numSeg2 = atoi(a[0]);
}

void SetFrac(char* a[])
{
	piFrac = atof(a[0]);
	if ( piFrac < 0.0 | piFrac > 1.0 ){
		fprintf(stderr,"piFrac must be between 0 and 1 (is %g).  Using 1.0.\n",
			piFrac);
		piFrac = 1.0;
	}
}


/* Table of available options -- register new options here */
Option Options[] = {
	/*  Name     Handler     #args   helpstring  */
	"help",  Usage,      0,      ": Print available options [default].",
	"r1",    Radius1,    1,      "r1: set radius of larger circle [1.0].",
	"r2",    Radius2,    1,      "r2: set radius of smaller circle [0.5].",
	"n1",    NumSegs1,   1,      "n1: set number of samples of larger circle [10].",
	"n2",    NumSegs2,   1,      "n2: set number of samples of smaller circle [10].",
	"f",     SetFrac,    1,      "frac: fraction of torus printed [1].",

	/*  Do not delete the next line */
	NULL,    NULL,       0,      NULL,      NULL
};


/* Global variables go here */
char *Banner  = "";
char *CommandName;
char *UsageString = "Torus [args]";
