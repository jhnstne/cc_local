/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: 03/12/89 at 00:22:04
** Purpose: Random Utilities.
**
*/

#ifndef _GEOM_H_
#define _GEOM_H_


/*
 *  Function:  LineIntersectPlane
 *
 *  Calling Sequence:
 *	Point LineIntersectPlane(p1, v1, p2, n2)
 *	Point p1; Vector v1; Point p2; Normal n2;
 *
 *  Description:
 *	Return the Point that results from intersecting the line described
 *  point p1/vector v1 with the plane described by point p2/normal n2.
 *
 *  Notes:
 *	No check is made to see if the line is parallel to the plane.
 */

Point LineIntersectPlane(Point p1, Vector v1, Point p2, Normal n2);


/*
 *  Function:  LineIntersectLine
 *
 *  Calling Sequence:
 *	BOOLEAN LineIntersectLine(p1,v1, p2,v2, pr)
 *	Point p1; Vector v1; Point p2; Vector v2; Point* pr;
 *
 *  Description:
 *	Find the intersection of the two lines determined by p1,v1 and
 *  p2,v2.  If intersection exists, return it in pr, and return TRUE.
 *  Otherwise return FALSE.
 */
BOOLEAN LineIntersectLine(Point p1, Vector v1, Point p2, Vector v2, Point* pr);


/*
 *----------------------------------------------------------------------
 *  Function:  ConstructQuadratic(p)
 *      Construct the quadratic precision point of a cubic patch.
 *  Parameters:
 *	Patch* p	- a cubic Bezier patch with all control points
 *			  assigned, except possibly the center point.
 *  Return Value:
 *	Point		- A point which if used as the center point of
 *			  the cubic would yield a quadratic patch if
 *			  the boundaries are degree raised quadratics.
 *----------------------------------------------------------------------
 */
Point ConstructQuadratic(Patch* p);


/*
 *----------------------------------------------------------------------
 *  Function:  GaussianCurvature(d2u, d2uv, d2v, du, dv, n)
 *	Compute the Gaussian curvature given the 2 deriv in two
 *	directions, the mixed partial in those directions, 
 *	the partials in those directions, and the outward pointing
 *	unit normal.
 *  Notes:
 *	du and dv should be linearly independent.
 *----------------------------------------------------------------------
 */
Scalar GaussianCurvature(Vector d2u, Vector d2uv, Vector d2v, 
			 Vector du, Vector dv, Normal n);

Scalar CircArcLen(Point p1, Vector v1, Point p2);

/*
 *----------------------------------------------------------------------
 *  Function:  ComputePlanarTangents
 *	Compute tangent vectors that point lie in a common plane with
 *  both specified points and each other, and lie in the given tangent
 *  planes.
 *	Vectors should both be "oriented" from p0 to p1.
 *----------------------------------------------------------------------
 */
void ComputePlanarTangents(Point p0, Normal n0, Point p1,Normal n1,
		      Vector* v0, Vector* v1, Vector* plane);

#endif /* _GEOM_H_ */
