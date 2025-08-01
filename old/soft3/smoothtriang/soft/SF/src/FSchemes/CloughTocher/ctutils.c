/*
 * Copyright (c) 1997, Computer Graphics Laboratory, University of Waterloo
 */

/*
 *----------------------------------------------------------------------
 *  File:  ctutils.c
 *	Some routines used by several methods in this directory.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"
#include "ctutils.h"

/*
** Compute a point in the plane through the hyperplane described by 
** vertex.
*/
Point EdgeTangent( Vertex* vertex, Point point )
{
	Point p1;
	Vector v1;

	p1 = PPac(ReturnUDPoint(vertex), point, 2.0/3.0);
	v1 = FV( StdFrame(SpaceOf(p1)), 2 );
	return LineIntersectPlane(p1,v1,ReturnUDPoint(vertex),ReturnUDNormal(vertex));
}


/*
 * GetBaryCoords
 *
 * - project four points in 3-space onto the plane and calculate 
 *   barycentric co-ordinates of the 4th wrt the first 3
 */
void GetBaryCoords (Point p0, Point p1, Point p2, Point p, Scalar b[3])
{
    Frame	std_frame, frame;
    Vector	v1, v2;
    Scalar	px, py, pz;

    std_frame = StdFrame(SpaceOf(p0));
    PCoords (p0, std_frame, &px, &py, &pz);
    p0 = PCreate (std_frame, px, py, 0);

    PCoords (p1, std_frame, &px, &py, &pz);
    p1 = PCreate (std_frame, px, py, 0);

    PCoords (p2, std_frame, &px, &py, &pz);
    p2 = PCreate (std_frame, px, py, 0);

    v1 = PPDiff (p1, p0);
    v2 = PPDiff (p2, p0);
    frame = FCreate ("PPPBary", p0, v1, v2, VVCross (v1, v2));

    PCoords (p, frame, &px, &py, &pz);

    b [0] = 1 - px - py;
    b [1] = px;
    b [2] = py;
} /* GetBaryCoords  */

/*
 * ConstructBoundary
 *
 * Given: a face
 * Construct: - boundary of a degree 3 bezier patch
 * - works only with functional data
 * - each edge subdivided evenly
 * - edge points intersect normal plane of closest vertex
 * - middle point not set
 */
void ConstructBoundary (Vertex* vr[3], Patch* patch)
{
    Point	Pr, Ps, Pt;
    Vertex	*Vr, *Vs, *Vt;
    Space World;

    World = SpaceOf(ReturnUDPoint(vr[0]));

    /* Create the cubic patch */
    *patch = PatchCreate( 3, World, 0);

    Vr = vr [0];
    Vs = vr [1];
    Vt = vr [2];

    /* Set the corner points of the net to the vertices of the face */
    PatchSetPoint( patch, ReturnUDPoint(Vr), 3, 0, 0); 
    PatchSetPoint( patch, ReturnUDPoint(Vs), 0, 3, 0); 
    PatchSetPoint( patch, ReturnUDPoint(Vt), 0, 0, 3); 

    /* Do the rs edge points */
    PatchSetPoint( patch, EdgeTangent( Vr, ReturnUDPoint(Vs) ), 2, 1, 0);
    PatchSetPoint( patch, EdgeTangent( Vs, ReturnUDPoint(Vr) ), 1, 2, 0);

    /* Do the st edge points */
    PatchSetPoint( patch, EdgeTangent( Vs, ReturnUDPoint(Vt) ), 0, 2, 1);
    PatchSetPoint( patch, EdgeTangent( Vt, ReturnUDPoint(Vs) ), 0, 1, 2);

    /* Do the tr edge points */
    PatchSetPoint( patch, EdgeTangent( Vt, ReturnUDPoint(Vr) ), 1, 0, 2);
    PatchSetPoint( patch, EdgeTangent( Vr, ReturnUDPoint(Vt) ), 2, 0, 1);
} /* ConstructBoundary  */


