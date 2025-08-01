/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: Wed Jul 05, 1989 at 04:37:14 PM
** Purpose: Version 2 of a C0 linear surface fitter.
**   This version takes as input a mesh rather than
**   a stream of face descriptions.
**
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

main(int argc, char **argv)
{
	Mesh *theMesh;
	Face *thisFace;
	Patch thisPatch;
	Vertex *vertices[3], *thisVertex;
	int i;

	ParseCommandLine( argc, argv);

	/* Read in the mesh */
#if 1
	theMesh = MeshParse(stdin);
	World = SCreate( "world", 3);
	AddGeometry(World, theMesh);
#else
	World = SCreate( "world", 3);
	theMesh = MeshUDParse(stdin,World);
#endif

	ForeachMeshFace(theMesh, thisFace) {
	/* Create a linear patch to fill in this face (assuming no normals) */
		thisPatch = PatchCreate( 1, World, 0);

		/* Put the vertices of thisFace into an array */
		i = 0;
		ForeachFaceVertex( thisFace, thisVertex) {
			vertices[i++] = thisVertex;
		} EndForeach;
		if (i > 3) {
			fprintf(stderr, 
				"Non-triangular face found...exiting.\n");
			exit(1);
		}

		pCopyDstructFields(&(thisFace->externalData),"Face",
				   &(thisPatch.ln),"BezierTriangle");

		/* Set the control net of the patch to the vertices of the face */
		PatchSetPoint( &thisPatch, ReturnUDPoint(vertices[0]),
			      1, 0, 0);
		PatchSetPoint( &thisPatch, ReturnUDPoint(vertices[1]),
			      0, 1, 0);
		PatchSetPoint( &thisPatch, ReturnUDPoint(vertices[2]),
			      0, 0, 1);

#if 0
		if ( ReturnUDGeoFlags(theMesh) & G_NORMAL ){
			PatchSetNormal( &thisPatch, 
				       ReturnUDNormal(vertices[0]), 1, 0, 0);
			PatchSetNormal( &thisPatch, 
				       ReturnUDNormal(vertices[1]), 0, 1, 0);
			PatchSetNormal( &thisPatch, 
				       ReturnUDNormal(vertices[2]), 0, 0, 1);
			thisPatch.normaldim = 3;
		}
#endif
		/* Output the patch */
		PatchWrite( stdout, thisPatch);

		/* Free the storage */
		PatchFree( thisPatch);
	} EndForeach;
}


