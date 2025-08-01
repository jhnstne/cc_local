/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  tptess.c
 *	Tessellate tensor product bezier patches with triangles.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "commandline.h"
#include "usage.h"
#include "patch.h"
#include "sff.h"
#include "libgeo.h"
#include "tpbezier.h"



extern int EdgeSamples;
extern Space World;

extern Scalar offsetDist;

static Point EvalOffset(TPBezier* tp, Scalar b[2])
{
	Point p;
	Normal n;

	TPBezierEvalWithNormal(*tp, &p, &n, b[0], b[1]);
	return PVAdd(p, SVMult(offsetDist, NDual(n)));
}


/*
 *----------------------------------------------------------------------
 *  Function:  WriteTriangleWithNormalsAndGK
 *	Output a triangle with normals and Gaussian attached.
 *----------------------------------------------------------------------
 */
void WriteTriangleWithNormalsAndGK( Point v1, Point v2, Point v3,
				   Normal n1, Normal n2, Normal n3,
				   Scalar g1, Scalar g2, Scalar g3)
{
  static int count=0;
  Frame F;
  char outputString[1000];
  char Access[81], VertexStr[81], Field[81];

  F = StdFrame(SpaceOf( v1 ));

  sprintf(outputString,"tri-%d",count);
  PutString("Triangle.name", outputString);
  PutVertexPosition("Triangle.vertex1",v1);
  PutVertexNormal("Triangle.vertex1",n1);
  PutScalar("Triangle.vertex1.curvature.gaussian",g1);

  PutVertexPosition("Triangle.vertex2",v2);
  PutVertexNormal("Triangle.vertex2",n2);
  PutScalar("Triangle.vertex2.curvature.gaussian",g2);

  PutVertexPosition("Triangle.vertex3",v3);
  PutVertexNormal("Triangle.vertex3",n3);
  PutScalar("Triangle.vertex3.curvature.gaussian",g3);

  FlushDstruct();
  count++;
}


/*
 *----------------------------------------------------------------------
 *  Function:  WriteTriangleWithNormals
 *	Output a triangle with normals attached.
 *----------------------------------------------------------------------
 */
void WriteTriangleWithNormals(Point v1, Point v2, Point v3, 
			      Normal n1, Normal n2, Normal n3)
{
  static int count=0;
  Frame F;
  char outputString[1000];
  char Access[81], VertexStr[81], Field[81];

  F = StdFrame(SpaceOf( v1 ));

  sprintf(outputString,"tri-%d",count);
  PutString("Triangle.name", outputString);
  PutVertexPosition("Triangle.vertex1",v1);
  PutVertexNormal("Triangle.vertex1",n1);
  PutVertexPosition("Triangle.vertex2",v2);
  PutVertexNormal("Triangle.vertex2",n2);
  PutVertexPosition("Triangle.vertex3",v3);
  PutVertexNormal("Triangle.vertex3",n3);

  FlushDstruct();
  count++;
}


/*
 *----------------------------------------------------------------------
 * Function: WriteTriangle
 *
 * Output a triangle dstruct.  The normal attached to each vertex is the
 * normal of plane containing the vertices.
 *----------------------------------------------------------------------
 */
void WriteTriangle(Point v1, Point v2, Point v3)
{
    Normal n;

    n = VDual( VVCross( PPDiff( v2, v1), PPDiff( v3, v1)));
    WriteTriangleWithNormals( v1, v2, v3, n, n, n);
}


/*
 *----------------------------------------------------------------------
 * Function: WriteNet
 *
 * The 2D array of points represents a sampling of a surface.  For each
 * quadrilateral grid cell two triangular facets are built.  The array
 * of normal vector can be NULL.
 *----------------------------------------------------------------------
 */
void WriteNet(Point point[], Normal normal[], Scalar gk[], int rows, int cols)
{
    int i, j;

    for (i=0; i<rows-1; i++) {
	for (j=0; j<cols-1; j++) {
	    int i1 = i*cols+j;
	    int i2 = i1 + 1;
	    int i3 = i1 + cols;
	    int i4 = i3 + 1;
	    
	    if (normal == NULL) {
		WriteTriangle( point[i1], point[i3], point[i2]);
		WriteTriangle( point[i3], point[i4], point[i2]);
	    } else if ( gk == NULL ) {
		WriteTriangleWithNormals( point[i1], point[i3], point[i2],
				  normal[i1], normal[i3], normal[i2]);
		WriteTriangleWithNormals( point[i3], point[i4], point[i2],
				  normal[i3], normal[i4], normal[i2]);
	    } else {
		WriteTriangleWithNormalsAndGK( point[i1], point[i3], point[i2],
				  normal[i1], normal[i3], normal[i2],
				  gk[i1], gk[i3], gk[i2]);
		WriteTriangleWithNormalsAndGK( point[i3], point[i4], point[i2],
				  normal[i3], normal[i4], normal[i2],
				  gk[i3], gk[i4], gk[i2]);
	    }
	}
    }
}



/*
 *----------------------------------------------------------------------
 * Function: TPBezierTessellate( tp)
 *
 * Tessellate the tensor product patch "EdgeSamples+1" times in each parametric
 * direction, then output the triangles.
 *
 * If EdgeSamples=0, then output the Bezier net.
 *----------------------------------------------------------------------
 */
void TPBezierTessellate( TPBezier tp )
{
    Scalar u, v, deltau, deltav;
    int iu, iv, offset=0;
    Point *positions;
    Normal *normals;
    Scalar *gk;
    Scalar testGK;
    SFF TPSff();
SFF sff1;
    SFF sff2;
    extern int boundary;

    if (EdgeSamples == 0) {
	WriteNet( tp.net, NULL, NULL, tp.degreeU+1, tp.degreeV+1);
	return;
    }

    if ( boundary ) {
	    OutputTPG3dBoundary(tp);
	    return;
    }

    /* Allocate temp storage for points and normals of evaluation */
    positions = (Point  *) malloc(sizeof(Point)*
				  (EdgeSamples+1)*(EdgeSamples+1));
    normals   = (Normal *) malloc(sizeof(Point)*
				  (EdgeSamples+1)*(EdgeSamples+1));
    gk	      = (Scalar *) malloc(sizeof(Scalar)*
				  (EdgeSamples+1)*(EdgeSamples+1));

    /* Sample uniformly in parameter space, being             */
    /* careful that the edges of the unit square are sampled. */
    u = 0.0;
    deltau = 1.0/((double) EdgeSamples);
    deltav = deltau;
    for (iu = 0; iu < EdgeSamples; iu++) {
	v = 0.0;
	for (iv = 0; iv < EdgeSamples; iv++) {
	    TPBezierEvalWithNormalAndGK( tp, positions+offset, normals+offset,
					 &testGK, u,v);
	    *(normals+offset) = VDual(VNormalize(NDual(*(normals+offset))));
	    if ( offsetDist != 0. ) {
		    *(positions+offset) = PVAdd(*(positions+offset),
						SVMult(offsetDist,
						       NDual(*(normals+offset))));
	    }
	    *(gk+offset) = testGK;
	    offset++;
	    v += deltav;
	}
	TPBezierEvalWithNormalAndGK( tp, positions+offset, normals+offset,
			        &testGK, u,1.0);
	*(normals+offset) = VDual(VNormalize(NDual(*(normals+offset))));
	if ( offsetDist != 0. ) {
		*(positions+offset) = PVAdd(*(positions+offset),
					    SVMult(offsetDist,
						   NDual(*(normals+offset))));
		sff2 = TPSff(&tp,EvalOffset, u, 1.);
		testGK = SFFGaussianCurvature(&sff2);
	}
	*(gk+offset) = testGK;
	offset++;
	u += deltau;
    }

    /* The u=1 line must still be sampled */
    v = 0.0;
    for (iv = 0; iv < EdgeSamples; iv++) {
	TPBezierEvalWithNormalAndGK( tp, positions+offset, normals+offset,
			        &testGK, 1.0,v);
	*(normals+offset) = VDual(VNormalize(NDual(*(normals+offset))));
	if ( offsetDist != 0. ) {
		*(positions+offset) = PVAdd(*(positions+offset),
					    SVMult(offsetDist,
						   NDual(*(normals+offset))));
		sff2 = TPSff(&tp,EvalOffset, 1., v);
		testGK = SFFGaussianCurvature(&sff2);
	}
	*(gk+offset) = testGK;
	offset++;
	v += deltav;
    }
    TPBezierEvalWithNormalAndGK( tp, positions+offset, normals+offset,
			    &testGK, 1.0,1.0);
    *(normals+offset) = VDual(VNormalize(NDual(*(normals+offset))));
    if ( offsetDist != 0. ) {
	    *(positions+offset) = PVAdd(*(positions+offset),
					SVMult(offsetDist,
					       NDual(*(normals+offset))));
		sff2 = TPSff(&tp,EvalOffset, 1., 1.);
		testGK = SFFGaussianCurvature(&sff2);
    }
    *(gk+offset) = testGK;
    
    WriteNet( positions, normals, gk, EdgeSamples+1, EdgeSamples+1 );

    free( positions );
    free( normals );
}

OutputTPG3dBoundary(TPBezier tp)
{
	int i;
	Point p;
	Normal n;
	Scalar x,y,z;

	for (i=0; i<=EdgeSamples; i++) {
		TPBezierEvalWithNormal(tp, &p, &n, 0., 
					    (double)i/(double)EdgeSamples);
		if ( offsetDist > 0 ) {
			p = PVAdd(p, SVMult(offsetDist, NDual(n)));
		}
		if ( i==0 ) {
			G3DMoveTo(p, n);
		} else {
			G3DDrawTo(p, n);
		}
	}

	for (i=0; i<=EdgeSamples; i++) {
		TPBezierEvalWithNormal(tp, &p, &n, 1., 
					    (double)i/(double)EdgeSamples);
		if ( offsetDist > 0 ) {
			p = PVAdd(p, SVMult(offsetDist, NDual(n)));
		}
		if ( i==0 ) {
			G3DMoveTo(p, n);
		} else {
			G3DDrawTo(p, n);
		}
	}

	for (i=0; i<=EdgeSamples; i++) {
		TPBezierEvalWithNormal(tp, &p, &n, 
					    (double)i/(double)EdgeSamples, 0.);
		if ( offsetDist > 0 ) {
			p = PVAdd(p, SVMult(offsetDist, NDual(n)));
		}
		if ( i==0 ) {
			G3DMoveTo(p, n);
		} else {
			G3DDrawTo(p, n);
		}
	}

	for (i=0; i<=EdgeSamples; i++) {
		TPBezierEvalWithNormal(tp, &p, &n, 
					    (double)i/(double)EdgeSamples, 1.);
		if ( offsetDist > 0 ) {
			p = PVAdd(p, SVMult(offsetDist, NDual(n)));
		}
		if ( i==0 ) {
			G3DMoveTo(p, n);
		} else {
			G3DDrawTo(p, n);
		}
	}
}
