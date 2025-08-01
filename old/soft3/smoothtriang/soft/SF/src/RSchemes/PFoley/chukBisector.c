
/*
 * Copyright (c) 1994, Computer Graphics Labratory, University of Waterloo
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Matthew Davidchuk
** 
** Purpose: To implement a parametric surface construction scheme
**
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "all.h"
#include "PFoley.h"

extern int useBisector;
extern int bisectorMethod;
extern int useNormals;
extern int count_bad_normals;
extern int count_coplanar;
extern int badBorderFound;
extern double BorderThreshold;

/*
 * GetBisector
 *
 * Returns the normalized vector which bisects the angle
 * formed by vectors u and v
 *
 * Note: Will not handle four coplanar points correctly.
 */
RetVal GetBisector (Point* p0, Point* p1,  
		    Point* p2, Point* p, Vector* result)
{
    Vector	bisector0, bisector1;
    Vector	v20, vp0, v21, vp1, v10, v01;
    Vector	proj0, proj1;

    v10 = PPDiff (*p1, *p0);
    v01 = PPDiff (*p0, *p1);

    v20 = PPDiff (*p2, *p0);
    vp0 = PPDiff (*p, *p0);
    v21 = PPDiff (*p2, *p1);
    vp1 = PPDiff (*p, *p1);

    v20 = VNormalize (v20);
    vp0 = VNormalize (vp0);
    v21 = VNormalize (v21);
    vp1 = VNormalize (vp1);

    bisector0 = VVAdd (v20, vp0);
    bisector1 = VVAdd (v21, vp1);

    if (VMag (bisector0) < EPSILON_SMALL_VECTOR) {
	return RV_BAD_FRAME;
    } else {
	bisector0 =  VNormalize (bisector0);
    } /* if  */

    if (VMag (bisector1) < EPSILON_SMALL_VECTOR) {
	return RV_BAD_FRAME;
    } else {
	bisector1 =  VNormalize (bisector1);
    } /* if  */

#if 0
    if (0 == bisectorMethod) {
	fprintf (stderr, "Bisector method not defined!\n");
	exit (1);
    } else if (1 == bisectorMethod) {
	*result = VVAdd (bisector0, bisector1);
	*result = VNormalize (*result);
    } else if (2 == bisectorMethod) {
	bisector0 = VVCross (v10, bisector0);
	bisector0 = VVCross (bisector0, v10);
	bisector1 = VVCross (v10, bisector1);
	bisector1 = VVCross (bisector1, v10);

	*result = VVAdd (bisector0, bisector1);
	*result = VNormalize (*result);
    } else if (3 == bisectorMethod) {
	static int	first = 1;
	if (first) {
	    first = 0;
	    fprintf (stderr, "Using bisector method 3\n");
	} /* if  */
	proj0 = SVMult ( VVDot (bisector0, v10) / VMag (v10), v10);
	proj1 = SVMult ( VVDot (bisector1, v01) / VMag (v01), v01);
	bisector0 = VVDiff (proj0, bisector0);
	bisector1 = VVDiff (proj1, bisector1);

	bisector0 = VNormalize (bisector0);
	bisector1 = VNormalize (bisector1);

	*result = VVAdd (bisector0, bisector1);
	*result = VNormalize (*result);
    } else if (4 == bisectorMethod) {
	fprintf (stderr, "Bisector method not defined!\n");
	exit (1);
    } /* if  */
#else
    proj0 = SVMult ( VVDot (bisector0, v10) / VMag (v10), v10);
    proj1 = SVMult ( VVDot (bisector1, v01) / VMag (v01), v01);
    bisector0 = VVDiff (proj0, bisector0);
    bisector1 = VVDiff (proj1, bisector1);

    bisector0 = VNormalize (bisector0);
    bisector1 = VNormalize (bisector1);

    *result = VVAdd (bisector0, bisector1);
    *result = VNormalize (*result);
#endif

    return RV_OK;

} /* GetBisector  */

