/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  func.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "all.h"

int nx=11;
int ny=11;
double xmin = -1.;
double ymin = -1.;
double xmax = 1.;
double ymax = 1.;
static double randomP = 0.;
int curvature=1;
int tri=1;

#define MAX 100
#define PI2 6.283185308

Space w;
Frame wf;

static int fmesh=1;

int Fmesh()
{
	return fmesh;
}

ItoEV(Vertex* v)
{
	UDItoEV(v);
	if ( curvature ) {
		pPutSFF(&(v->externalData), 
			"Point.sff",&(v->internalData->sff));
	}
}


void* CreateGrid(int n1, int n2, int size)
{
	int i;
	void** v;

	v = (void**)malloc(n1*sizeof(int*));

	for (i=0; i<n1; i++){
		v[i] = (void*)malloc(n2*size);
	}
	return v;
}

void DestroyGrid(Vertex*** p, int n1, int n2)
{
	int i;

	for (i=0; i<n1; i++) {
		free(p[i]);
	}
	free(p);
}

void SetPoints(Mesh* m, Vertex*** va, double xmin, double ymin, 
	       double xmax, double ymax, int nx, int ny, 
	       double (*fp)(), Normal (*fn)(), SFF (*fs)())
{
	int i,j;
	Point p,p2;
	Normal n;
	double x,y,z;
	Vertex* CreateVertex();
	char s[100];
	SFF sff;

	for (i=0; i<nx; i++) {
		for (j=0; j<ny; j++) {
			va[i][j] = CreateVertex(m);

			x = (xmax-xmin)*i/(nx-1) + xmin + randomP*random()/0xFFFFFFFF;
			y = (ymax-ymin)*j/(ny-1) + ymin + randomP*random()/0xFFFFFFFF;;
			z = fp(x, y);

			p = PCreate(wf, x, y, z);
			SetUDPoint(va[i][j], p);

			if ( fn ) {
				n = fn(x, y);
				SetUDNormal(va[i][j], n);
			}

			if ( curvature ) {
				va[i][j]->internalData->sff = fs(x,y);
			}
			sprintf(s,"v%dX%d",i,j);
			SetName(va[i][j],s);
		}
	}
}

void DstructTri(Vertex* v1, Vertex* v2, Vertex* v3) {
	PutVertexPosition("Triangle.vertex1", ReturnUDPoint(v1));
	PutVertexNormal("Triangle.vertex1", ReturnUDNormal(v1));
	PutScalar("Triangle.vertex1.curvature.gaussian",
		  SFFGaussianCurvature(&(v1->internalData->sff)));
	PutScalar("Triangle.vertex1.curvature.mean",
		  SFFMeanCurvature(&(v1->internalData->sff)));
	
	PutVertexPosition("Triangle.vertex2", ReturnUDPoint(v2));
	PutVertexNormal("Triangle.vertex2", ReturnUDNormal(v2));
	PutScalar("Triangle.vertex2.curvature.gaussian",
		  SFFGaussianCurvature(&(v2->internalData->sff)));
	PutScalar("Triangle.vertex2.curvature.mean",
		  SFFMeanCurvature(&(v2->internalData->sff)));
	
	PutVertexPosition("Triangle.vertex3", ReturnUDPoint(v3));
	PutVertexNormal("Triangle.vertex3", ReturnUDNormal(v3));
	PutScalar("Triangle.vertex3.curvature.gaussian",
		  SFFGaussianCurvature(&(v3->internalData->sff)));
	PutScalar("Triangle.vertex3.curvature.mean",
		  SFFMeanCurvature(&(v3->internalData->sff)));
	FlushDstruct();
}

void WriteTri(Vertex*** va, int nx, int n1)
{
	int i,j;

	for (i=0; i<nx-1; i++) {
		for (j=0; j<ny-1; j++) {
			DstructTri(va[i][j], va[i+1][j], va[i+1][j+1]);
			DstructTri(va[i][j], va[i+1][j+1], va[i][j+1]);
		}
	}
}

Face* CreateQuadFace(Mesh* m, Vertex* v0, Vertex* v1, Vertex* v2, Vertex* v3)
{
	Face* f;
	Vertex* va[4];
	Face* CreateNFace();

	va[0] = v0;	va[1] = v1;	va[2] = v2;	va[3] = v3;

	f = CreateNFace(m, va, 4);

	return f;
}


Face* CreateTriFace(Mesh* m, Vertex* v0, Vertex* v1, Vertex* v2)
{
	Face* f;
	Vertex* va[3];
	Face* CreateNFace();

	va[0] = v0;	va[1] = v1;	va[2] = v2;

	f = CreateNFace(m, va, 3);

	return f;
}


Mesh* GridToMesh(Mesh* m, Vertex*** va, int nx, int ny)
{
	int i,j;
	Face* f;
	char s[100];

	for (i=0; i<nx-1; i++) {
		for (j=0; j<ny-1; j++) {
			if ( !tri ) {
				f = CreateQuadFace(m,
						   va[i][j],     va[i+1][j],
						   va[i+1][j+1], va[i][j+1]);
				sprintf(s,"F%dX%d",i,j);
				SetName(f, s);
			} else {
				f = CreateTriFace(m,
						  va[i][j],     va[i+1][j],
						  va[i][j+1]);
				sprintf(s,"F%dX%da",i,j);
				SetName(f, s);

				f = CreateTriFace(m,
						  va[i+1][j],
						  va[i+1][j+1], va[i][j+1]);
				sprintf(s,"F%dX%db",i,j);
				SetName(f, s);
			}
		}
	}
}

PrintHeader()
{
	printf("%% Sampled domain: %g %g %g %g\n",xmin,ymin,xmax,ymax);
	printf("%% Number of samples: %d %d\n",nx,ny);
	printf("\n");
}

mfunction(int argc, char* argv[], Space s, double (*fp)(), 
	  Normal (*fn)(), SFF (*fs)())
{
	Vector dz;
	Scalar x,y,z;
	char name[20];
	Vertex*** vgrid;
	Mesh* m;

	if ( fp == NULL ) {
		fprintf(stderr, "mfunction: fp == NULL\n");
		exit(1);
	}
	if ( fn == NULL  &&  fs != NULL ) {
		fprintf(stderr, "mfunction: fn == NULL buf fs != NULL\n");
		exit(1);
	}
	w = s;
	wf = StdFrame(w);
	
	ParseCommandLine(argc, argv);

	if ( fmesh == 1 ) {
		PrintHeader();
	}

	if ( fs == NULL ) {
		curvature = 0;
	}

	vgrid = CreateGrid(nx, ny, sizeof(Vertex*));

	m = NewMesh();

	SetName(m, "mesh");

	SetPoints(m, vgrid, xmin, ymin, xmax, ymax, nx, ny, fp, fn, fs);

	if ( fmesh == 1 ) {
		GridToMesh(m, vgrid, nx, ny);

		WriteItoEMesh(m, stdout, ItoEV, NULL, NULL);

		DestroyGrid(vgrid,nx, ny);
	} else {
		WriteTri(vgrid, nx, ny);
	}
}


void SetXY(char* a[])
{
	xmin = atof(a[0]);
	ymin = atof(a[1]);
	xmax = atof(a[2]);
	ymax = atof(a[3]);
}

void SetSamples(char* a[])
{
	nx = atoi(a[0]);
	ny = atoi(a[1]);
	if ( nx < 2 ) {
		fprintf(stderr,"nx too small.  Using 2.\n");
		nx = 2;
	}
	if ( ny < 2 ) {
		fprintf(stderr,"ny too small.  Using 2.\n");
		ny = 2;
	}
}


void SetNoCurvature()
{
	curvature = 0;
}

void SetQuad()
{
	tri = 0;
}

void SetRandom(char* argv[])
{
	randomP = atof(argv[0]);
	fprintf(stderr,"Random set to %f\n",randomP);
	srandom(time(0));
}

void SetTri()
{
	fmesh = 0;
}


/* Table of available options -- register new options here */
Option Options[] = {
     /*  Name    Handler     #args   helpstring  */
	"help",  Usage,      0,  ": Print available options.",
	"xy",    SetXY,      4,	 "xmin ymin xmax ymax: Set the sampling bounds [-1 -1 1 1].",
	"s",     SetSamples, 2,	 "nx ny: The number of samples in x and y [11 11].",
	"quad",	 SetQuad,    0,  ": output quads instead of tris.",
	"nk",    SetNoCurvature,0,    ": don't tag vertices with second fundemental form.",
	"r",	SetRandom, 1, "#: perturb randomly by #\n",
	"tri",	SetTri,	0,	": Output triangles rather than a mesh\n",
	  
	/*  Do not delete the next line */
	NULL,    NULL,       0,      NULL,      NULL
    };


/* Global variables go here */
char *Banner  = "";
char *CommandName;
char *UsageString = "Monkey [args]";
