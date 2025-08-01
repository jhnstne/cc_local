/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Stephen Mann
*/

#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "mesh.h"
#include "userData.h"
#include "commandline.h"
#include "usage.h"
#include "patch.h"
#include "sff.h"


       /* Forward declarations for option handlers */
extern void Usage();

       /* Table of available options -- register new options here */
       /* For full details on how to add options, see the file    */
       /* commandline.h.                                          */
Option Options[] = {
/*  Name     Handler     #args   helpstring  */
    "h",     Usage,      0,      ": Print available options.",

/*  Do not delete the next line */
    NULL,    NULL,       0,      NULL,      NULL
  };


/* Global variables go here */
char *Banner  = "Fitter0, Version 2";
char *UsageString = "Fitter0 [options] < mesh";
Space World;

main(argc, argv)
int argc;
char **argv;
{
    Mesh *theMesh;
    Face *thisFace;
    Vertex *vertices[3], *thisVertex;
    int i;
    Lnode* l=NULL;

    ParseCommandLine( argc, argv);

    /* Read in the mesh */
    theMesh = MeshParse(stdin);
    World = SCreate( "world", 3);
    AddGeometry(World, theMesh);

    ForeachMeshFace(theMesh, thisFace) {

	/* Put the vertices of thisFace into an array */
	i = 0;
	ForeachFaceVertex( thisFace, thisVertex) {
	    vertices[i++] = thisVertex;
	} EndForeach;
	if (i > 3) {
	    fprintf(stderr, "Non-triangular face found...exiting.\n");
	    exit(1);
	}

	pCopyDstructFields(&(thisFace->externalData),"Face",
			   &l,"Triangle");

	SetPoint(&l, StdFrame(World), "Triangle.vertex1", vertices[0]);
	SetPoint(&l, StdFrame(World), "Triangle.vertex2", vertices[1]);
	SetPoint(&l, StdFrame(World), "Triangle.vertex3", vertices[2]);

	
	/* Output the triangle */
	dWriteDstruct(l);
	pDeleteLnode(&l);

    } EndForeach;
}


SetPoint(l, f, p, v)
Lnode** l;
Frame f;
char* p;
Vertex* v;
{
	char buf[100];
	Scalar x,y,z;
	SFF sff;

	PCoords(ReturnUDPoint(v), StdFrame(World), &x, &y, &z);
	sprintf(buf,"%s.pos[0]",p);
	pPutScalar(l, buf, x);
	sprintf(buf,"%s.pos[1]",p);
	pPutScalar(l, buf, y);
	sprintf(buf,"%s.pos[2]",p);
	pPutScalar(l, buf, z);

	NCoords(ReturnUDNormal(v), StdFrame(World), &x, &y, &z);
	sprintf(buf,"%s.norm[0]",p);
	pPutScalar(l, buf, x);
	sprintf(buf,"%s.norm[1]",p);
	pPutScalar(l, buf, y);
	sprintf(buf,"%s.norm[2]",p);
	pPutScalar(l, buf, z);

	sprintf(buf,"%s.curvature.gaussian",p);
	dGetSFF(v->externalData,"Point.sff",f,&sff);
	pPutScalar(l, buf, SFFGaussianCurvature(&sff));
}
