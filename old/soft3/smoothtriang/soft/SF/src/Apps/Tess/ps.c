/*
 * Copyright (c) 1997, Computer Graphics Laboratory, University of Waterloo
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  ps.c
 *	Output a PostScript triangle
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "commandline.h"
#include "usage.h"
#include "patch.h"
#include "ratpatch.h"
#include "sff.h"
#include "libgeo.h"
#include "tpbezier.h"


/*
 *----------------------------------------------------------------------
 *  Function:  g3dTriangleOutput
 *----------------------------------------------------------------------
 */
void psTriangleOutput(FILE *fp, VERTEX v1, VERTEX v2, VERTEX v3, Lnode* ds)
{
	static int count=0;
	Frame F;
	Scalar x,y,z;
	
	F = StdFrame(SpaceOf( v1.position));
	
	PCoords(v1.position, F, &x,&y,&z);
	fprintf(fp, "%g %g moveto\n", x, y);
	
	PCoords(v2.position, F, &x,&y,&z);
	fprintf(fp, "%g %g lineto\n", x, y);
	
	PCoords(v3.position, F, &x,&y,&z);
	fprintf(fp, "%g %g lineto\n", x, y);
	fprintf(fp, "closepath stroke\n");

	count++;
}
