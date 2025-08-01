
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

/* #define EPSILON_SMALL_VECTOR    1e-04 */
#define EPSILON_COPLANAR	1e-04
/* #define EPSILON_DOT_PRODUCT     1e-06 */

extern Space World;    /* Space where faces and patches reside */


#ifdef NOPFOLEY
    int useBisector;
    int useNormals;
    int count_bad_normals;
    int count_coplanar;
    int boundBorder;
    int cutPatches;
    double BorderThreshold;
#else
    extern int useBisector;
    extern int bisectorMethod;
    extern int useNormals;
    extern int zNormal;
    extern int count_bad_normals;
    extern int count_coplanar;
    extern int badBorderFound;
    extern double BorderThreshold;
    extern int boundBorder;
    extern int cutPatches;
#endif

/*
 * EdgeTangent
 * 
 * Find intersection of vertical vector through 2/3 * p1 + 1/3 * p2
 * with xy-plane defined by f.
 */
static Point EdgeTangent (Vertex* v1, Vertex* v2, Frame fr)
{
    Point	p, p1, p2, presult;
    Vector	v;
    Normal	n;

    p1 = ReturnUDPoint(v1);
    p2 = ReturnUDPoint(v2);

    p = PPac( p1, p2, 2.0/3.0 );

    if (GetUDNormal (v1, &n)) {
	n = ReturnUDNormal (v1);
    } else {
	fprintf (stderr, "PFoley: no normal\n");
	return p;
    } /* GetUDNormal  */
    v = FV( fr, 2 );
    presult = LineIntersectPlane(p, v, p1, n);

#if 0
    if (boundBorder) {
	static int first = 1;

	if (first) {
	    fprintf (stderr, "Checking for bad borders and replacing with Gregory patches\n");
	    first = 0;
	} /* if  */
	if (VMag (PPDiff (presult, p)) > 3 * VMag (PPDiff (p1, p2))) {
	    presult = PVAdd (p, SVMult (3.0 * VMag (PPDiff (p1, p2)), 
			VNormalize(v)));
	} /* if  */
    } /* if  */
#endif

    return presult;
} /* EdgeTangent  */

/*
 * ConstructBoundary
 *
 * Given: a face
 * Construct: - boundary of a degree 3 bezier patch
 * - works only with functional data
 * - each edge subdivided evenly
 * - edge points intersect normal plane of closest vertex
 * - middle point not set
 *
 * return 
 * TRUE  if vr[1]vr[2] boundary exceeds threshold
 * FALSE otherwise
 */
BOOLEAN ConstructBoundary (Vertex* vr[3], Patch* patch, Frame fr)
{
    Vertex	*Vr, *Vs, *Vt;

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
    PatchSetPoint( patch, EdgeTangent( Vr, Vs, fr ), 2, 1, 0);
    PatchSetPoint( patch, EdgeTangent( Vs, Vr, fr ), 1, 2, 0);

    /* Do the st edge points */
    PatchSetPoint( patch, EdgeTangent( Vs, Vt, fr ), 0, 2, 1);
    PatchSetPoint( patch, EdgeTangent( Vt, Vs, fr ), 0, 1, 2);

    /* Do the tr edge points */
    PatchSetPoint( patch, EdgeTangent( Vt, Vr, fr ), 1, 0, 2);
    PatchSetPoint( patch, EdgeTangent( Vr, Vt, fr ), 2, 0, 1);

    if (boundBorder || cutPatches) {
	return BadBoundaries (patch);
    } else {
	return FALSE;
    } /* if  */
} /* ConstructBoundary  */

/*
 * AreCoPlanar - are the four points coplanar?
 */
static RetVal AreCoPlanar (Point* p0, Point* p1, Point* p2, Point* p)
{
    Vector	v0, v1, v2;
    Frame	f;
    Scalar	x,y,z;

    v0 = PPDiff (*p0,*p1);
    v1 = PPDiff (*p0,*p2);
    v2 = VVCross (v0, v1);
    v2 = VNormalize (v2);
    f = FCreate ("AreCoPlanar", *p0, v0, v1, v2);
    PCoords (*p, f, &x, &y, &z);

    if (fabs(z) < EPSILON_COPLANAR) {
	return RV_POINTS_COPLANAR;
    } else {
	return RV_OK;
    } /* if  */
} /* AreCoPlanar  */

/*
 * IsGoodProjection
 *
 * Two triangles (p0,p1,p2) and (p0,p1,p).
 * Want vector v to be on same side of both triangles
 */
static RetVal IsGoodProjection (Point* p0, Point* p1, 
		    Point* p2, Point* p, Vector* v)
{
    Vector	vec10, vec20, vecv0;    /* vectors (p1,p0) (p2,p0) (p,p0) */
    Vector	cross12;		/* cross product (v1,v0)x(v2,v0) */
    Vector	crossv1;		/* cross product (v1,v0)x(v,v0) */
    Scalar	dot_12_v, dot_1v_v;

    vec10 = PPDiff (*p1, *p0);
    vec20 = PPDiff (*p2, *p0);
    vecv0 = PPDiff (*p, *p0);

    cross12 = VVCross (vec10, vec20);
    crossv1 = VVCross (vecv0, vec10);
    dot_12_v = VVDot (cross12, *v);
    dot_1v_v = VVDot (crossv1, *v);

    if ( ( (dot_12_v > 0) && (dot_1v_v < 0) ) ||
         ( (dot_12_v < 0) && (dot_1v_v > 0) ) ) {
	/* bad news! projected triangles overlap! */
	return RV_BAD_PROJECTION;
    } else {
	return RV_OK;
    } /* if  */
} /* IsGoodProjection  */


/*
 * GetFrame
 *
 * v0, v1, v2 define one triangle
 * v0, v1, v  define the other
 *
 * Calculate frame used for projecting triangle pairs.
 * Won't handle degenerate triangles.
 */
RetVal GetFrame (Vertex* v0, Vertex* v1, Vertex* v2, Vertex* v, Frame* f)
{
    Frame	proj_frame;
    Point	p0, p1, p2, p;
    Vector	vec0, vec1, vec2;    /* frame basis vectors */
    Vector	bisector;
    Vector	normal0, normal1;
    Vector	vec01, vec10, vec20, vecv0;  
		    /* vectors (v1,v0) (v2,v0) (v,v0) */

    /* 
     * - get the bisector of the vectors (v2,v0) and (v,v0)
     * - get the bisector of the vectors (v2,v1) and (v,v1)
     * - average them
     */
    p0 = ReturnUDPoint (v0);
    p1 = ReturnUDPoint (v1);
    p2 = ReturnUDPoint (v2);
    p = ReturnUDPoint (v);

    if (RV_POINTS_COPLANAR == AreCoPlanar (&p0, &p1, &p2, &p)) {
	Vector	vec10, vec20, vecp0, vec01, vec21, vecp1, cross2, crossp, cross;
	vec10 = PPDiff (p1, p0);
	vec20 = PPDiff (p2, p0);
	vecp0 = PPDiff (p,  p0);
	vec01 = PPDiff (p0, p1);
	vec21 = PPDiff (p2, p1);
	vecp1 = PPDiff (p,  p1);

	cross2 = VVCross (vec20, vec10);
	cross2 = VNormalize (cross2);
	crossp = VVCross (vec10, vecp0);
	crossp = VNormalize (crossp);

	cross = VVAdd (cross2, crossp);

	cross2 = VVCross (vec01, vec21);
	cross2 = VNormalize (cross2);
	crossp = VVCross (vecp1, vec01);
	crossp = VNormalize (crossp);

	cross = VVAdd (cross, crossp);
	cross = VVAdd (cross, cross2);

	cross = VNormalize (cross);
	vec1 = VVCross (vec10, cross);
	cross = VVCross (vec1, vec10);

	vec10 = VNormalize (vec10);
	vec1 = VNormalize (vec1);
	cross = VNormalize (cross);

	proj_frame = FCreate ("Projection frame", p0, vec10, vec1, cross);
	*f = proj_frame;
	count_coplanar++;
	return RV_OK;
    } /* AreCoPlanar  */

    /*
     * - add normal vectors at v0 and v1, then normalize
     * - if new vector lies on the wrong side of the planes
     *   defined by both triangles then use bisector
     */
    if (zNormal) {
	    vec2 = FV(StdFrame(SpaceOf(p0)), 2);
    } else if (useNormals) {
	normal0 = NDual (ReturnUDNormal (v0));
	normal1 = NDual (ReturnUDNormal (v1));
	vec2 = VVAdd (normal0, normal1);
	if (VMag (vec2) < EPSILON_SMALL_VECTOR) {
	    return RV_BAD_FRAME;
	} else {
	    vec2 =  VNormalize (vec2);
	} /* if  */
    } else {
	if (RV_OK != GetBisector (&p0, &p1, &p2, &p, &bisector)) {
	    return RV_BAD_FRAME;
	} /* if  */
	if (VMag (bisector) < EPSILON_SMALL_VECTOR) {
	    return RV_BAD_FRAME;
	} /* if  */
	if (useBisector) {
	    vec2 = bisector;
	} else {
	    normal0 = NDual (ReturnUDNormal (v0));
	    normal1 = NDual (ReturnUDNormal (v1));

	    vec2 = VVAdd (bisector, VVAdd (normal0, normal1));

	    if (RV_BAD_PROJECTION == 
		    IsGoodProjection(&p0, &p1, &p2, &p, &vec2)) {
		count_bad_normals++;
		vec2 = bisector;
		fprintf (stderr, "chukBoundary: Throwing out normals.\n");
	    } /* if  */
	} /* if  */
    } /* if  */

    vec0 = PPDiff (p1, p0);
    vec1 = VVCross (vec0, vec2);
    vec2 = VVCross (vec1, vec0);

    vec0 = VNormalize (vec0);
    vec1 = VNormalize (vec1);
    vec2 = VNormalize (vec2);

    proj_frame = FCreate ("Projection frame", p0, vec0, vec1, vec2);
    *f = proj_frame;
    return RV_OK;
} /* GetFrame  */



/*
 * chukBoundary
 *
 */
Point chukBoundary(Vertex *v1, Vertex *v2)
{
    Vertex*	vr [3];
    Vertex	*v3, *v;
    Patch	patch;
    Frame	fr;
    Edge*	e;
    Face	*f1, *f2;
    RetVal	retval;

    v3 = VertexPath(v1,v2,"-1");
    v  = VertexPath(v1,v2,"1");

    if ((NULL == v3) || (NULL == v)) {
	fprintf (stderr, "chukBoundary: VertexPath returned NULL\n");
    } /* if  */

    vr [0] = v1;
    vr [1] = v2;
    vr [2] = v3;

#if 0
    /* find out if we are on the edge of the mesh */
    GetVertexEdge (v1, v2, &e);
    if (NULL == e) {
        GetVertexEdge (v2, v1, &e);
        if (NULL == e) {
            fprintf (stderr, "chukBoundary: GetVertexEdge returned NULL\n");
	    exit (1);
        } /* if  */
    } /* if  */
    GetEdgeFaces (e, &f1, &f2);

    if ((NULL == f1) || (NULL == f2)) {
	return PPac(ReturnUDPoint(v1),
		    ReturnUDPoint(v2), 2.0/3.0);
    } /* if  */
#endif

    retval = GetFrame (v1, v2, v3, v, &fr);
    /* get frame */
    if (RV_OK != retval) {
	fprintf (stderr, "chukBoundary: Coouldn't construct frame\n");
	exit (1);
    } /* if  */

    ConstructBoundary (vr, &patch, fr);

    return PatchGetPoint( patch, 2, 1, 0); 
} /* chukBoundary */

