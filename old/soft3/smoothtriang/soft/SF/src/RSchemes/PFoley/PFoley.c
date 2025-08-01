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
** To do: write some reasonable docs so that this spaghetti code
**        can be deciphered by some poor grad student in the future
**
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "all.h"
#include <sys/time.h>
#include <sys/resource.h>
#include <limits.h>
#include "PFoley.h"

/* Forward declarations */
void ProcessFace();
Point EdgeTangent();
void TweakBoundary();
void AdjustPatch();
Point LineIntersectPlane(Point, Vector, Point, Normal);
void GregoryProcessFace( Patch* patch0, Patch* patch1, Patch* patch2, 
	Vertex* vr[3], 
        BOOLEAN bb0, BOOLEAN bb1, BOOLEAN bb2, int samples );
BOOLEAN ConstructBoundary (Vertex* vr[3], Patch* patch, Frame fr);



/* Global variables */
int verbose=0;
int samples=1;
int gaussian=0;
int cubicPrecision=0;
int useStandardFrame = 0;
int boundBorder = 0;
int useBisector = 0;
int useNormals = 0;
int zNormal = 0;
int useFoleyBlend = 1;
int bisectorMethod = 0;
int cutPatches = 0;
double BorderThreshold = 3.0;
int linearDerivative = 0;


int	patch_count = 0;
int	count_no_frame = 0;
int	count_bad_normals = 0;
int	count_coplanar = 0;
int	count_gregory = 0;

Space World;                 /* Space where faces and patches reside */

main(int argc, char **argv)
{
	Mesh* m;
	Face* f;
	int num;
	Vertex* v=NULL;
	
	/* Interpret command line options */
	ParseCommandLine( argc, argv);
	
	/* Create the world space */
	World = SCreate( "World", 3);
	
	m = MeshParse(stdin);
	AddGeometry(World,m);
	
	/* If no normals, complain and quit. */
	if ( !(ReturnUDGeoFlags(m) & G_NORMAL) ) {
		fprintf(stderr,"Foley: Normals not given.  Exiting.\n");
		exit(1);
	} 
	
	/* Main Loop */
	num=0;
	ForeachMeshFace(m,f){
	
		if ( verbose  &&  f->name != NULL ) 
		  fprintf(stderr,"process face %s\n",f->name);
		ProcessFace(f);
		num += 1;

	} EndForeach;

	if (verbose){
		struct rusage u;

		getrusage(RUSAGE_SELF, &u);
		fprintf(stderr,"Foley: finished.  ");
		fprintf(stderr,
		    "%d neighborhoods processed in %d cpu seconds.\n",
			num, u.ru_utime);
	}
	fprintf (stderr, "%d patches\n", patch_count);
	fprintf (stderr, "%d couldn't construct frame\n", count_no_frame);
	fprintf (stderr, "%d bad normals\n", count_bad_normals);
	fprintf (stderr, "%d coplanar triangle pairs\n", count_coplanar);
	fprintf (stderr, "%d Gregory triangles constructed\n", count_gregory);
	exit(0);
}

/*
 * GetBaryCoords
 *
 * - project four points in onto the plane defined by frame 'f'
 *   and calculate barycentric co-ordinates of the 4th point 
 *   with respect to the first 3 points
 */
BOOLEAN GetBaryCoords (Point p0, Point p1, Point p2, Point p, 
	    Scalar* b0, Scalar* b1, Scalar* b2, Frame f)
{
    Frame	frame;
    Vector	v1, v2, cross12;
    Scalar	px, py, pz;
    Point	np0, np1, np2;

    PCoords (p0, f, &px, &py, &pz);
    np0 = PCreate (f, px, py, 0);

    PCoords (p1, f, &px, &py, &pz);
    np1 = PCreate (f, px, py, 0);

    PCoords (p2, f, &px, &py, &pz);
    np2 = PCreate (f, px, py, 0);

    v1 = PPDiff (np1, np0);
    v2 = PPDiff (np2, np0);
    cross12 = VVCross (v1, v2);

    if (VMag (cross12) < EPSILON_SMALL_VECTOR) {
	/* something wrong! */
	fprintf (stderr, 
	    "GetBaryCoords: degenerate triangle: %d\n", patch_count);
#if 1
    PCoords (np0, f, &px, &py, &pz);
    fprintf (stderr, "%e\t%e\t%e\n", px, py, pz);
    PCoords (np1, f, &px, &py, &pz);
    fprintf (stderr, "%e\t%e\t%e\n", px, py, pz);
    PCoords (np2, f, &px, &py, &pz);
    fprintf (stderr, "%e\t%e\t%e\n\n", px, py, pz);
#endif
	return TRUE;
    } /* if  */
#if 0
    fprintf (stderr, "v1: %e    v2: %e    cross12: %e\n", 
	VMag (v1), VMag (v2), VMag (cross12));
    fprintf (stderr, "%lf\t%lf\n", VVDot (v1, v2), VMag (v1) * VMag (v2));
#endif
#if 0
v1 = VNormalize (v1);
v2 = VNormalize (v2);
    cross12 = VNormalize (cross12);
#endif

    frame = FCreate ("PPPBary", np0, v1, v2, cross12);

    PCoords (p, frame, &px, &py, &pz);

    *b0 = 1 - px - py;
    *b1 = px;
    *b2 = py;

    return FALSE;
} /* GetBaryCoords  */


/*
 * GetInterior
 *
 * Given: - face
 *        - 3 vertices defining a face
 *          1st 2 vertices define edge under consideration
 * Calc:  interior point of face as per Foley
 *
 *
 * Return: true if neighbouring face doesn't exist, false otherwise
 */
BOOLEAN GetInterior (Face* face, Vertex* v0, Vertex* v1, Vertex* v2, 
	    Patch* patch, BOOLEAN* badcenter, BOOLEAN* badboundary, 
	    BOOLEAN first)
{
    Face	*nface, *f1, *f2;
    Edge*	e;
    Vertex	*v, *vr[3], *nvr[3];
    int		i;
    Patch	npatch;
    Frame	proj_frame;
    Scalar	px, py, pz, dummy;
    Scalar	c102, c012, b300, b030, 
		    b210, b120, b201, b102, b021, b012,
		    b111x, b111y, b111z;
    Scalar	r, s, t;
    Point	p111;
    RetVal	retval;

    /* get and order vertices for both patches
     *  patch vertices -> v0, v1, v2
     * npatch vertices -> v0, v1, v
     */

    /* find and process neighbouring face
     * get edge associated with v0 and v1 */
    GetVertexEdge (v0, v1, &e);

    /* try reverse order, maybe order was wrong */
    if (NULL == e) {
	GetVertexEdge (v1, v0, &e);
	if (NULL == e) {
	    fprintf (stderr, "Foley: GetVertexEdge returned NULL\n");
	    return TRUE;
	} /* if  */
    } /* if  */

    /* get faces associated with e */
    GetEdgeFaces (e, &f1, &f2);

    if (f1 != face) {
	nface = f1;
    } else {
	nface = f2;
    } /* if  */

    /* neighbour doesn't exist! use triangle as projection plane */
    if (NULL == nface) {
	Vector	vec, vec1, vec2;
	Point	p0, p1, p2;
	Point P[9];
	Scalar w[9];

	p0 = ReturnUDPoint (v0);
	p1 = ReturnUDPoint (v1);
	p2 = ReturnUDPoint (v2);
	vr [0] = v0;
	vr [1] = v1;
	vr [2] = v2;
	vec1 = PPDiff (p1, p0);
	vec2 = PPDiff (p2, p0);
	vec = VVCross (vec1,vec2);
	proj_frame = FCreate ("GetInterior", p0, vec1, vec2, vec);

	ConstructBoundary (vr, patch, proj_frame);

	P[6] = PatchGetPoint(*patch, 3, 0, 0);
	P[7] = PatchGetPoint(*patch, 0, 3, 0);
	P[8] = PatchGetPoint(*patch, 0, 0, 3);
	P[0] = PatchGetPoint(*patch, 2, 1, 0);
	P[1] = PatchGetPoint(*patch, 1, 2, 0);
	P[2] = PatchGetPoint(*patch, 0, 2, 1);
	P[3] = PatchGetPoint(*patch, 0, 1, 2);
	P[4] = PatchGetPoint(*patch, 1, 0, 2);
	P[5] = PatchGetPoint(*patch, 2, 0, 1);

        /* Set the middle point to the centroid of the edge points */
        w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
        w[6] = w[7] = w[8] = -(1.0/6.0);
        PatchSetPoint( patch, PPacN( 9, P, w), 1, 1, 1);

	return FALSE;
    } /* if  */
	
    i = 0;
    ForeachFaceVertex(nface,v){
      nvr[i++] = v;
    } EndForeach;

    /* assign v to be the vertex on neighbour which is not v0 or v1 */
    if (v0 == nvr[0]) {
	if (v1 == nvr[1]) v = nvr[2];
	    else v = nvr[1];
    } else if (v0 == nvr[1]) {
	if (v1 == nvr[0]) v = nvr[2];
	    else v = nvr[0];
    } else {
	if (v1 == nvr[0]) v = nvr[1];
	    else v = nvr[0];
    } /* if  */

    vr [0] = v0;
    vr [1] = v1;
    vr [2] = v2;

    nvr [0] = v0;
    nvr [1] = v1;
    nvr [2] = v;	

    if (useStandardFrame) {
	proj_frame = StdFrame(World);
    } else {
	retval = GetFrame (v0, v1, v2, v, &proj_frame);
	if (RV_OK != retval) {
	    fprintf (stderr, "GetFrame: couldn't construct frame\n");
	    return TRUE;
	} /* if  */
    } /* GetFrame  */

    if (ConstructBoundary (vr, patch, proj_frame)) {
	*badboundary = TRUE;
    } /* if  */
    ConstructBoundary (nvr, &npatch, proj_frame);

    if (linearDerivative) {
	Point p1;
	Point ptA,ptB;
	Vector tempv1, tempv2;
	Normal n;
	Point point;

	/* normal Clough Tocher
	 * - take average of points (0,2,1) and (2,0,1)
	 *   i.e. two point closest to boundary
	 */
	ptA = PPac( PatchGetPoint(npatch,0,2,1), 
		    PatchGetPoint(npatch,2,0,1), 1.0/2.0);
	ptB = PPac(	PatchGetPoint(*patch,0,2,1), 
			PatchGetPoint(*patch,2,0,1), 1.0/2.0);

	tempv1 = PPDiff(ptA,ptB);
	tempv2 = PPDiff( PatchGetPoint(*patch,2,1,0),
		  PatchGetPoint(*patch,1,2,0));
	n = VDual(VVCross(tempv1,tempv2));
	point = PatchGetPoint(*patch,1,2,0);

	p1 = PPac3( PatchGetPoint (*patch, 3, 0, 0),
		    PatchGetPoint (*patch, 0, 3, 0),
		    PatchGetPoint (*patch, 0, 0, 3),
		 1.0/3.0, 1.0/3.0, 1.0/3.0);

	tempv1 = FV( proj_frame, 2 );
	p111 = LineIntersectPlane(p1,tempv1,point,n);
	PatchSetPoint (patch, p111, 1, 1, 1);

    } else {
	PCoords (PatchGetPoint (*patch,3,0,0), proj_frame, &px, &py, &pz);
	b300 = pz;
	PCoords (PatchGetPoint (*patch,0,3,0), proj_frame, &px, &py, &pz);
	b030 = pz;
	PCoords (PatchGetPoint (*patch,2,1,0), proj_frame, &px, &py, &pz);
	b210 = pz;
	PCoords (PatchGetPoint (*patch,1,2,0), proj_frame, &px, &py, &pz);
	b120 = pz;
	PCoords (PatchGetPoint (*patch,2,0,1), proj_frame, &px, &py, &pz);
	b201 = pz;
	PCoords (PatchGetPoint (*patch,1,0,2), proj_frame, &px, &py, &pz);
	b102 = pz;
	PCoords (PatchGetPoint (*patch,0,2,1), proj_frame, &px, &py, &pz);
	b021 = pz;
	PCoords (PatchGetPoint (*patch,0,1,2), proj_frame, &px, &py, &pz);
	b012 = pz;
	PCoords (PatchGetPoint (npatch,1,0,2), proj_frame, &px, &py, &pz);
	c102 = pz;
	PCoords (PatchGetPoint (npatch,0,1,2), proj_frame, &px, &py, &pz);
	c012 = pz;

	if (GetBaryCoords (	PatchGetPoint (*patch,3,0,0),
			PatchGetPoint (*patch,0,3,0),
			PatchGetPoint (*patch,0,0,3),
			PatchGetPoint (npatch,0,0,3), 
			&r, &s, &t, proj_frame)) {
	    return TRUE;
	} /* if  */

	if ((1.0 + s + r != 1.0) && (1.0 + t != 1.0)) {
	    b111z = ( c102 + c012 - r*r*(b300+b210) - 2*r*s*(b210+b120) - 
		2*r*t*b201 - 2*s*t*b021 - s*s*(b120+b030) - 
		t*t*(b102+b012) ) / 2.0 / (r + s) / t;
	} else {
	    fprintf (stderr, "Foley: (%s, line %d) unable to compute "
			"interior patch point\n", __FILE__, __LINE__);
	    exit (1);
	} /* if */

	/* give *p x,y coordinates in center of triangle */
	p111 = PPac3( PatchGetPoint (*patch, 3, 0, 0),
		    PatchGetPoint (*patch, 0, 3, 0),
		    PatchGetPoint (*patch, 0, 0, 3),
		    1.0/3.0, 1.0/3.0, 1.0/3.0);
	PCoords (p111, proj_frame, &b111x, &b111y, &dummy);

	p111 = PCreate (proj_frame, b111x, b111y, b111z);
	PatchSetPoint (patch, p111, 1, 1, 1);

	PatchFree (npatch);
    } /* if */

    if (boundBorder) {
	if (BadCenter (patch)) {
	    if (first) {
		    BOOLEAN bc_tmp = FALSE, bb_tmp = FALSE;
		    Patch patch_tmp;

		    GetInterior (nface, nvr[0], nvr[1], nvr[2], 
			&patch_tmp, &bc_tmp, &bb_tmp, FALSE);
		    if (bc_tmp) {
			*badcenter = TRUE;
		    } else {
			*badcenter = FALSE;
		    } /* if  */
		    PatchFree (patch_tmp);
	    } else {
		*badcenter = TRUE;
	    } /* if  */
	} /* if  */
    } else if (cutPatches) {
	if (BadCenter (patch)) {
	    *badcenter = TRUE;
	} /* if  */
    } /* if  */

    return FALSE;
} /* GetInterior  */

void RotatePatch (Patch* patch)
{
    Point	p300, p030, p003, p210, p120, p201, p102, p021, p012, p111;

    p300 = PatchGetPoint (*patch, 3, 0, 0);
    p030 = PatchGetPoint (*patch, 0, 3, 0);
    p003 = PatchGetPoint (*patch, 0, 0, 3);
    p210 = PatchGetPoint (*patch, 2, 1, 0);
    p120 = PatchGetPoint (*patch, 1, 2, 0);
    p201 = PatchGetPoint (*patch, 2, 0, 1);
    p102 = PatchGetPoint (*patch, 1, 0, 2);
    p021 = PatchGetPoint (*patch, 0, 2, 1);
    p012 = PatchGetPoint (*patch, 0, 1, 2);
    p111 = PatchGetPoint (*patch, 1, 1, 1);

    PatchSetPoint (patch, p300, 0, 3, 0);
    PatchSetPoint (patch, p030, 0, 0, 3);
    PatchSetPoint (patch, p003, 3, 0, 0);
    PatchSetPoint (patch, p210, 0, 2, 1);
    PatchSetPoint (patch, p120, 0, 1, 2);
    PatchSetPoint (patch, p201, 1, 2, 0);
    PatchSetPoint (patch, p102, 2, 1, 0);
    PatchSetPoint (patch, p021, 1, 0, 2);
    PatchSetPoint (patch, p012, 2, 0, 1);
    PatchSetPoint (patch, p111, 1, 1, 1);
} /* RotatePatch  */

/*
 * struct FoleyStruct
 *
 * p210_2 is weighted by u2 and p210_1 by u1.
 * Same pattern for other points (pxyz_i weighted by ui).
 */
typedef struct FoleyStruct {
    Patch	patch0, patch1, patch2;
} FoleyStruct;

Point EvalPatch (FoleyStruct* data, Scalar b[3]);

void ProcessFace (Face* face)
{
    FoleyStruct	data;
    Vertex	*vr[3];
    int		notexist0, notexist1, notexist2; 
    BOOLEAN	bb0 = FALSE, bb1 = FALSE, bb2 = FALSE;
    BOOLEAN	bc0 = FALSE, bc1 = FALSE, bc2 = FALSE;

patch_count++;

    {
	int		i = 0;
	Vertex*		v;

	ForeachFaceVertex(face,v){
	  vr[i++] = v;
	} EndForeach;
    }

    notexist0 = GetInterior (face, vr[1], vr[2], vr [0], 
		    &data.patch0, &bc0, &bb0, TRUE);
    notexist1 = GetInterior (face, vr[2], vr[0], vr [1], 
		    &data.patch1, &bc1, &bb1, TRUE);
    notexist2 = GetInterior (face, vr[0], vr[1], vr [2], 
		    &data.patch2, &bc2, &bb2, TRUE);
    
    /* could we build the patch ? */
    if (notexist0 || notexist1 || notexist2) {
	return;
    } /* if  */

    RotatePatch (&data.patch0);
    RotatePatch (&data.patch1); 
    RotatePatch (&data.patch1);

    if (boundBorder && (bc0 || bb0 || bc1 || bb1 || bc2 || bb2)) {
	/* pass the patch to another scheme - Gregory */
	GregoryProcessFace (&data.patch0, &data.patch1, &data.patch2,
	    vr, bc0, bc1, bc2, samples);
	PatchFree (data.patch0);
	PatchFree (data.patch1);
	PatchFree (data.patch2);
	count_gregory ++;
	return;
    } else if (cutPatches && (bc0 || bb0 || bc1 || bb1 || bc2 || bb2)) {
	PatchFree (data.patch0);
	PatchFree (data.patch1);
	PatchFree (data.patch2);
	count_gregory ++;
	return;
    } /* if  */

    if (!cubicPrecision) {
	Point	p021_0, p012_0, p201_1, p102_1, p210_2, p120_2;

	p012_0 = PatchGetPoint (data.patch0, 0, 1, 2);
	PatchSetPoint (&data.patch2, p012_0, 0, 1, 2);
	p021_0 = PatchGetPoint (data.patch0, 0, 2, 1);
	PatchSetPoint (&data.patch1, p021_0, 0, 2, 1);

	p102_1 = PatchGetPoint (data.patch1, 1, 0, 2);
	PatchSetPoint (&data.patch2, p102_1, 1, 0, 2);
	p201_1 = PatchGetPoint (data.patch1, 2, 0, 1);
	PatchSetPoint (&data.patch0, p201_1, 2, 0, 1);

	p120_2 = PatchGetPoint (data.patch2, 1, 2, 0);
	PatchSetPoint (&data.patch1, p120_2, 1, 2, 0);
	p210_2 = PatchGetPoint (data.patch2, 2, 1, 0);
	PatchSetPoint (&data.patch0, p210_2, 2, 1, 0);
    } /* if  */

    SetTessEps (M_EPSILON);

    /* tesselate patch */
    if (gaussian) {
	    SetTessMethod("s3d");
	    gkTessPatch(&data,EvalPatch,samples,StdFrame(World));
    } else {
	    SetTessMethod("iv");
	    SetTessMethod("s3d");
	    TessPatch(&data,EvalPatch,samples,StdFrame(World));
    } /* if */


    PatchFree (data.patch0);
    PatchFree (data.patch1);
    PatchFree (data.patch2);

} /* ProcessFace */

Point EvalPatch (FoleyStruct* data, Scalar u[3])
{
    /*
     * u[i] == 1.0 - EPSILON indicates the tesselator is trying
     *      to approximate a derivative to calculate the normal at vertex i
     * Hack: We know the normal at the vertices so figure out which 
     *	    edge the tesselator is interested in and return the vector
     *	    defined by the vertex and an adjacent edge control point
     */
    if (u[0] == 1.0 - M_EPSILON) {
	if (u[1] == M_EPSILON) {
	    return PatchGetPoint (data->patch0, 2, 1, 0);
	} else {
	    return PatchGetPoint (data->patch0, 2, 0, 1);
	} /* if  */
    } else if (u[1] == 1.0 - M_EPSILON) {
	if (u[0] == M_EPSILON) {
	    return PatchGetPoint (data->patch0, 1, 2, 0);
	} else {
	    return PatchGetPoint (data->patch0, 0, 2, 1);
	} /* if  */
    } else if (u[2] == 1.0 - M_EPSILON) {
	if (u[0] == M_EPSILON) {
	    return PatchGetPoint (data->patch0, 1, 0, 2);
	} else {
	    return PatchGetPoint (data->patch0, 0, 1, 2);
	} /* if  */

    /*
     * Also check if the tesselator is asking for the vertices.
     * There are singularities at these points.
     */
    } else if (u[0] == 1.0) {
	return PatchGetPoint (data->patch0, 3, 0, 0);
    } else if (u[1] == 1.0) {
	return PatchGetPoint (data->patch0, 0, 3, 0);
    } else if (u[2] == 1.0) {
	return PatchGetPoint (data->patch0, 0, 0, 3);
    } else if (u[0] == 0.0) {
	return PatchEval (data->patch0, 0.0, u[1], u[2]);
    } else if (u[1] == 0.0) {
	return PatchEval (data->patch1, u[0], 0.0, u[2]);
    } else if (u[2] == 0.0) {
	return PatchEval (data->patch2, u[0], u[1], 0.0);
    } else {
	if (cubicPrecision) {
	    Patch	result;
	    Point	p;
	    Scalar	w0, w1, w2, denom;
	    Scalar	w021_1, w021_2, w012_1, w012_2, w210_1, 
		w210_2, w120_1, w120_2, w201_1, w201_2, w102_1, w102_2;
	    Scalar	denom_01, denom_02, denom_12;
	    Scalar	u0term, u1term, u2term;
	    static int	first = 1;

#define PW1(x)	((x)*(x)*(x))
#define PW2(x)	(x)

	    if (first) {
		fprintf (stderr, "Using Foley cubic precision\n");
		first = 0;
	    } /* if  */

	    u0term = PW1 (u[0]);
	    u1term = PW1 (u[1]);
	    u2term = PW1 (u[2]);
	    denom = u0term*u1term + u0term*u2term + u1term*u2term;

	    w0 = (u1term * u2term) / denom;
	    w1 = (u0term * u2term) / denom;
	    w2 = (u0term * u1term) / denom;

	    denom_01 = (1 - u[1]) * PW1(u[0]) + 
		(1-u[0]) * PW1(u[1]) +  PW2(u[2])*PW1(u[0])*PW1(u[1]);
	    denom_02 = (1-u[2]) * PW1(u[0]) +
		(1-u[0]) * PW1(u[2]) +  PW2(u[1])*PW1(u[0])*PW1(u[2]);
	    denom_12 = (1-u[2]) * PW1(u[1]) + 
		(1-u[1]) * PW1(u[2]) +  PW2(u[0])*PW1(u[1])*PW1(u[2]);

	    w201_1 = (1 - u[1]) * PW1(u[2]) / denom_12;
	    w201_2 = (1 - u[2]) * PW1(u[1]) / denom_12;
	    w210_1 = (1 - u[2]) * PW1(u[1]) / denom_12;
	    w210_2 = (1 - u[1]) * PW1(u[2]) / denom_12;

	    w021_1 = (1 - u[0]) * PW1(u[2]) / denom_02;
	    w021_2 = (1 - u[2]) * PW1(u[0]) / denom_02;
	    w120_1 = (1 - u[2]) * PW1(u[0]) / denom_02;
	    w120_2 = (1 - u[0]) * PW1(u[2]) / denom_02;

	    w012_1 = (1 - u[0]) * PW1(u[1]) / denom_01;
	    w012_2 = (1 - u[1]) * PW1(u[0]) / denom_01;
	    w102_1 = (1 - u[1]) * PW1(u[0]) / denom_01;
	    w102_2 = (1 - u[0]) * PW1(u[1]) / denom_01;

	    result = PatchCreate( 3, World, 0);

	    PatchSetPoint (&result, 
		PatchGetPoint (data->patch0, 3, 0, 0), 3, 0, 0);

	    PatchSetPoint (&result, 
		PatchGetPoint (data->patch0, 0, 3, 0), 0, 3, 0);

	    PatchSetPoint (&result, 
		PatchGetPoint (data->patch0, 0, 0, 3), 0, 0, 3);

	    p = PPac3 (	PatchGetPoint (data->patch1, 2, 0, 1),
			    PatchGetPoint (data->patch2, 2, 0, 1),
			    PatchGetPoint (data->patch0, 2, 0, 1),
			    w201_1, w201_2, 1-w201_1-w201_2);
	    PatchSetPoint( &result, p, 2, 0, 1); 
	    p = PPac3 (	PatchGetPoint (data->patch2, 2, 1, 0),
			    PatchGetPoint (data->patch1, 2, 1, 0),
			    PatchGetPoint (data->patch0, 2, 1, 0),
			    w210_1, w210_2, 1-w210_1-w210_2);
	    PatchSetPoint( &result, p, 2, 1, 0); 

	    p = PPac3 (	PatchGetPoint (data->patch0, 0, 2, 1),
			    PatchGetPoint (data->patch2, 0, 2, 1),
			    PatchGetPoint (data->patch1, 0, 2, 1),
			    w021_1, w021_2, 1 - w021_1 - w021_2);
	    PatchSetPoint( &result, p, 0, 2, 1); 
	    p = PPac3 (	PatchGetPoint (data->patch2, 1, 2, 0),
			    PatchGetPoint (data->patch0, 1, 2, 0),
			    PatchGetPoint (data->patch1, 1, 2, 0),
			    w120_1, w120_2, 1-w120_1-w120_2);
	    PatchSetPoint( &result, p, 1, 2, 0); 

	    p = PPac3 (	PatchGetPoint (data->patch0, 0, 1, 2),
			    PatchGetPoint (data->patch1, 0, 1, 2),
			    PatchGetPoint (data->patch2, 0, 1, 2),
			    w012_1, w012_2, 1-w012_1-w012_2);
	    PatchSetPoint( &result, p, 0, 1, 2); 
	    p = PPac3 (	PatchGetPoint (data->patch1, 1, 0, 2),
			    PatchGetPoint (data->patch0, 1, 0, 2),
			    PatchGetPoint (data->patch1, 1, 0, 2),
			    w102_1, w102_2, 1-w102_1-w102_2);
	    PatchSetPoint( &result, p, 1, 0, 2); 

	    p = PPac3 (	PatchGetPoint (data->patch0, 1, 1, 1),
			    PatchGetPoint (data->patch1, 1, 1, 1),
			    PatchGetPoint (data->patch2, 1, 1, 1),
			    w0, w1, w2);
	    PatchSetPoint( &result, p, 1, 1, 1); 

	    p = PatchEval (result, u[0], u[1], u[2]);

	    PatchFree (result);

	    return p;
	} else {
	    Patch	result;
	    Point	p;
	    Scalar	w0, w1, w2, denom;
	    Scalar	w021, w012, w210, w120, w201, w102;
	    Scalar	denom_01, denom_02, denom_12;
	    Scalar	u0term, u1term, u2term;

	    if (useFoleyBlend) {
		u0term = u[0]; u1term = u[1]; u2term = u[2]; 
	    } else {
		u0term = u[0]*u[0]; u1term = u[1]*u[1]; u2term = u[2]*u[2];
	    } /* if  */
	    denom = u0term*u1term + u0term*u2term + u1term*u2term;

	    w0 = (u1term * u2term) / denom;
	    w1 = (u0term * u2term) / denom;
	    w2 = (u0term * u1term) / denom;

	    denom_01 = (1 - u[1]) * u[0] * u[0] + 
			    (1-u[0]) * u[1] * u[1];
	    denom_02 = (1-u[2]) * u[0] * u[0] +
			    (1-u[0]) * u[2] * u[2];
	    denom_12 = (1-u[2]) * u[1] * u[1] + 
			    (1-u[1]) * u[2] * u[2];

	    w021 = (1 - u[0]) * u[2] * u[2] / denom_02;
	    w012 = (1 - u[0]) * u[1] * u[1] / denom_01;
	    w210 = (1 - u[2]) * u[1] * u[1] / denom_12;
	    w120 = (1 - u[2]) * u[0] * u[0] / denom_02;
	    w201 = (1 - u[1]) * u[2] * u[2] / denom_12;
	    w102 = (1 - u[1]) * u[0] * u[0] / denom_01;

	    result = PatchCreate( 3, World, 0);

	    PatchSetPoint (&result, 
		PatchGetPoint (data->patch0, 3, 0, 0), 3, 0, 0);

	    PatchSetPoint (&result, 
		PatchGetPoint (data->patch0, 0, 3, 0), 0, 3, 0);

	    PatchSetPoint (&result, 
		PatchGetPoint (data->patch0, 0, 0, 3), 0, 0, 3);

	    p = PPac (	PatchGetPoint (data->patch0, 0, 2, 1),
			    PatchGetPoint (data->patch2, 0, 2, 1),
			    w021);
	    PatchSetPoint( &result, p, 0, 2, 1); 
	    p = PPac (	PatchGetPoint (data->patch0, 0, 1, 2),
			    PatchGetPoint (data->patch1, 0, 1, 2),
			    w012);
	    PatchSetPoint( &result, p, 0, 1, 2); 
	    p = PPac (	PatchGetPoint (data->patch2, 2, 1, 0),
			    PatchGetPoint (data->patch1, 2, 1, 0),
			    w210);
	    PatchSetPoint( &result, p, 2, 1, 0); 
	    p = PPac (	PatchGetPoint (data->patch2, 1, 2, 0),
			    PatchGetPoint (data->patch0, 1, 2, 0),
			    w120);
	    PatchSetPoint( &result, p, 1, 2, 0); 
	    p = PPac (	PatchGetPoint (data->patch1, 2, 0, 1),
			    PatchGetPoint (data->patch2, 2, 0, 1),
			    w201);
	    PatchSetPoint( &result, p, 2, 0, 1); 
	    p = PPac (	PatchGetPoint (data->patch1, 1, 0, 2),
			    PatchGetPoint (data->patch0, 1, 0, 2),
			    w102);
	    PatchSetPoint( &result, p, 1, 0, 2); 
	    p = PPac3 (	PatchGetPoint (data->patch0, 1, 1, 1),
			    PatchGetPoint (data->patch1, 1, 1, 1),
			    PatchGetPoint (data->patch2, 1, 1, 1),
			    w0, w1, w2);
	    PatchSetPoint( &result, p, 1, 1, 1); 

	    p = PatchEval (result, u[0], u[1], u[2]);

	    PatchFree (result);

	    return p;
	} /* if  */
    } /* if  */
} /* EvalPatch */



char* Banner = "Foley";
char* UsageString = "Foley [option] < mesh ";

extern void Usage();

void Verbose()
{
	verbose = 1;
}

void SetThreshold(arg)
char* arg[];
{
  BorderThreshold = atof(arg[0]);
}

void SetSamples(arg)
char* arg[];
{
  samples = atoi(arg[0]);
}

void Gaussian()
{
    gaussian = 1;
} /* Gaussian */

void UseCubic()
{
    cubicPrecision = 1;
} /* Blend0 */

void UseStdFr ()
{
    useStandardFrame = 1;
} /* UseStandardFrame  */

void BoundBorder ()
{
    boundBorder = 1;
} /* BoundBorder  */

void UseNormals ()
{
    useNormals = 1;
} /* UseNormals  */

void ZNormals ()
{
    zNormal = 1;
} /* ZNormals  */

void UseBisector ()
{
    useBisector = 1;
} /* UseBisector  */

void UseFoleyBlend ()
{
    useFoleyBlend = 1;
} /* UseFoleyBlend */

void CutPatches ()
{
    cutPatches = 1;
} /* NoFoleyBlend  */

void NoFoleyBlend ()
{
    useFoleyBlend = 0;
} /* NoFoleyBlend  */

void SetZ1 ()
{
    bisectorMethod = 1;
} /* SetZ1  */

void SetZ2 ()
{
    bisectorMethod = 2;
} /* SetZ2  */

void SetZ3 ()
{
    bisectorMethod = 3;
} /* SetZ3  */

void SetZ4 ()
{
    bisectorMethod = 4;
} /* SetZ4  */

void SetLinearDeriv ()
{
    linearDerivative = 1;
} /* SetZ4  */


Option Options[] = {
/* Name,        Routine,        #args,  Help String */
   "h",         Usage,          0,      ":      Print available options.",
   "v",         Verbose,        0,      ":      Verbose mode.",
   "s",         SetSamples,     1,      "#:     Number of samples per side.",
   "k",		Gaussian,	0,	":      Compute Gaussian curvature.",
   "cubic",	UseCubic,	0,	":      Use Foley cubic precision.",
   "fr-std",	UseStdFr,	0,	":      Use standard frame for projections.",
   "bb",	BoundBorder,	0,	":      Border heights are bounded by 3 x length of triangle side.",
   "cut",	CutPatches,	0,	":      Don't draw bad patches.",
   "thres #",	SetThreshold,	1,	":      Set bound/cut threshold.",
   "n",		UseNormals,	0,	":      Use normals only to get projection frame.",
   "z",		ZNormals,	0,	":      Use x-y plane as projection plane.",
   "bi",	UseBisector,	0,	":      Use bisectors only to get projection frame.",
   "fb",	UseFoleyBlend,	0,	":      Use Foley blend to weight patch interior (and C^2 control points). Default.",
   "nfb",	NoFoleyBlend,	0,	":      Use higher degree blend to weight patch interior (and C^2 control points).",
   "z1",	SetZ1,		0,	":      use bisector method 1.",
   "z2",	SetZ2,		0,	":      use bisector method 2.",
   "z3",	SetZ3,		0,	":      use bisector method 3.",
   "z4",	SetZ4,		0,	":      use bisector method 4.",
   "ld",	SetLinearDeriv,	0,	":      Use linearly varying derivatives.",

   NULL,        NULL,           0,      NULL
   };

