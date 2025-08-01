
#include <stdio.h>
#include "all.h"
#include "PFoley.h"

extern double BorderThreshold;

BOOLEAN BadCenter (Patch* patch)
{
    double	d01, d12, d20, d0, d1, d2;

    d01 = PPDist (	PatchGetPoint (*patch, 3, 0, 0),
		    PatchGetPoint (*patch, 0, 3, 0));
    d12 = PPDist (	PatchGetPoint (*patch, 0, 3, 0),
		    PatchGetPoint (*patch, 0, 0, 3));
    d20 = PPDist (	PatchGetPoint (*patch, 3, 0, 0),
		    PatchGetPoint (*patch, 0, 0, 3));
    d0 = PPDist ( PatchGetPoint (*patch, 3, 0, 0), 
		    PatchGetPoint (*patch, 1, 1, 1));
    d1 = PPDist ( PatchGetPoint (*patch, 0, 3, 0), 
		    PatchGetPoint (*patch, 1, 1, 1));
    d2 = PPDist ( PatchGetPoint (*patch, 0, 0, 3), 
		    PatchGetPoint (*patch, 1, 1, 1));
    if ( ((d0 > (BorderThreshold * d01)) && (d0 > (BorderThreshold * d20))) ||
	 ((d1 > (BorderThreshold * d12)) && (d1 > (BorderThreshold * d01))) ||
	 ((d2 > (BorderThreshold * d12)) && (d2 > (BorderThreshold * d20))) ) {
	return TRUE;
    } else {
	return FALSE;
    } /* if  */
} /* BadCenter  */

BOOLEAN BadBoundaries (Patch* patch)
{
    double	d, d1, d2;

    d = PPDist (    PatchGetPoint (*patch, 3, 0, 0),
		    PatchGetPoint (*patch, 0, 3, 0));
    d1 = PPDist (   PatchGetPoint (*patch, 3, 0, 0),
		    PatchGetPoint( *patch, 1, 2, 0));
    d2 = PPDist (   PatchGetPoint (*patch, 0, 3, 0),
		    PatchGetPoint( *patch, 2, 1, 0));
    if ((d1 > (BorderThreshold * d)) ||
        (d2 > (BorderThreshold * d))) {
	return TRUE;
    } /* if  */

    d = PPDist (    PatchGetPoint (*patch, 0, 3, 0),
		    PatchGetPoint (*patch, 0, 0, 3));
    d1 = PPDist (   PatchGetPoint (*patch, 0, 3, 0),
		    PatchGetPoint( *patch, 0, 1, 2));
    d2 = PPDist (   PatchGetPoint (*patch, 0, 0, 3),
		    PatchGetPoint( *patch, 0, 2, 1));
    if ((d1 > (BorderThreshold * d)) ||
        (d2 > (BorderThreshold * d))) {
	return TRUE;
    } /* if  */

    d = PPDist (    PatchGetPoint (*patch, 3, 0, 0),
		    PatchGetPoint (*patch, 0, 0, 3));
    d1 = PPDist (   PatchGetPoint (*patch, 3, 0, 0),
		    PatchGetPoint( *patch, 1, 0, 2));
    d2 = PPDist (   PatchGetPoint (*patch, 0, 0, 3),
		    PatchGetPoint( *patch, 2, 0, 1));
    if ((d1 > (BorderThreshold * d)) ||
        (d2 > (BorderThreshold * d))) {
	return TRUE;
    } /* if  */

    return FALSE;

} /* BadBoundaries  */

