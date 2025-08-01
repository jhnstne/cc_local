/*
 * Copyright (c) 1994, Computer Graphics Labratory, University of Waterloo
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Matthew Davidchuk
** 
** Purpose: To implement Tom Foley's scattered data interpolation scheme
**
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "all.h"
#include <sys/time.h>
#include <sys/resource.h>
#include <limits.h>

#define M_EPSILON	1.0e-04
#define OUTPUT_PATCH	0

/* Forward declarations */
void ProcessFace();
Point EdgeTangent();
void TweakBoundary();
void AdjustPatch();
Point LineIntersectPlane();

/* Global variables */
int verbose=0;
int samples=1;
int gaussian=0;
int furthBoundary=0;
int goodBoundary=0;
int chiyokuraBoundary=0;
int cloughTocher=0;

Point LineIntersectPlane(Point, Vector, Point, Normal);


static char *RoutineName;
static Space World;                 /* Space where faces and patches reside */

static Vertex* vp=NULL;

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
	exit(0);
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

    PCoords (p0, StdFrame(World), &px, &py, &pz);
    p0 = PCreate (StdFrame(World), px, py, 0);

    PCoords (p1, StdFrame(World), &px, &py, &pz);
    p1 = PCreate (StdFrame(World), px, py, 0);

    PCoords (p2, StdFrame(World), &px, &py, &pz);
    p2 = PCreate (StdFrame(World), px, py, 0);

    v1 = PPDiff (p1, p0);
    v2 = PPDiff (p2, p0);
    frame = FCreate ("PPPBary", p0, v1, v2, VVCross (v1, v2));

    PCoords (p, frame, &px, &py, &pz);

    b [0] = 1 - px - py;
    b [1] = px;
    b [2] = py;
} /* GetBaryCoords  */


/*
 *  Function:  FurthBoundary
 *      Sets tangents at either end of a single border curve, according to the
 *  notation on p. 9 of Furth
 */
void FurthBoundary(p0, n0, p1, n1, r0, r1)
Point   p0, p1;
Normal  n0, n1;
Point *r0, *r1;
{
        Vector  u1, u2, u3;
        Vector v0, v1;

        u1 = VNormalize(PPDiff(p1, p0));
        u3 = VNormalize(VVCross(u1, VVAdd(NDual(n0), NDual(n1))));
        u2 = VNormalize(VVCross(u3, u1));

        /* determine tangents at 0, 1 values along border curve i */
        /* pp0 stands for "p prime at value 0" */

        v0 = SVMult(VMag(PPDiff(p1, p0)), VNormalize(VVCross(NDual(n0), u3)));
        v1 = SVMult(VMag(PPDiff(p1, p0)), VNormalize(VVCross(NDual(n1), u3)));

        if (VVDot(VVCross(NDual(n1), u3), u1) < 0.0) {
                v0 = SVMult(-1.0, v0);
        } else {
                v1 = SVMult(-1.0, v1);
        }
        *r0 = PVAdd(p0, SVMult(1.0/3.0, v0));
        *r1 = PVAdd(p1, SVMult(1.0/3.0, v1));
} /* FurthBoundary */


/*
 * EdgeTangent
 * 
 * Find intersection of vertical vector through 2/3 * p1 + 1/3 * p2
 * with plane defined by p1 and n1.
 */
Point EdgeTangent (Vertex* v1, Vertex* v2)
{
    static int	first = 1;
    Point	p1, p2;

    if ( goodBoundary ) {
	if ( first ) {
		fprintf(stderr,"Using dBHS boundaries\n");
		first=0;
	}
	if ( dBHS(v1,v2,&p1,&p2) ) {
		return p1;
	} else {
		Vector v;
		fprintf(stderr,"no dBHS boundary; using SS boundary\n");
		p1 = PPac(ReturnUDPoint(v1),
			  ReturnUDPoint(v2), 4.0/9.0);
		p2 = LineIntersectPlane(p1,NDual(ReturnUDNormal(v1)),
					ReturnUDPoint(v1),
					ReturnUDNormal(v1));
		v = PPDiff(p2, ReturnUDPoint(v1));
		v = SVMult(VMag(PPDiff(ReturnUDPoint(v1),
				       ReturnUDPoint(v2)))/(3.0*VMag(v)),
			   v);
		p2 = PVAdd(ReturnUDPoint(v1),v);
		return p2;
	} /* if  */
    } else if ( furthBoundary ) {
	if ( first ) {
	    fprintf(stderr,"Using Furth boundaries\n");
	    first=0;
	}
	FurthBoundary(ReturnUDPoint(v1),
		      ReturnUDNormal(v1),
		      ReturnUDPoint(v2),
		      ReturnUDNormal(v2),
		      &p1, &p2);
	return p1;
    } else {
	Scalar	nx, ny, nz;
	Scalar	px, py, pz;
	Scalar	p1x, p1y, p1z;
	Scalar	vx, vy, vz;
	Vector	pdiff;
	Point	p;

	if ( first ) {
	    fprintf(stderr,"Using simple boundaries\n");
	    first=0;
	}
	NCoords (ReturnUDNormal(v1), StdFrame (World), &nx, &ny, &nz);

	p1 = ReturnUDPoint(v1);
	p2 = ReturnUDPoint(v2);

	p = PPac( p1, p2, 2.0/3.0 );
	pdiff = PPDiff (p, p1);
	VCoords (pdiff, StdFrame (World), &vx, &vy, &vz);
	PCoords (p, StdFrame (World), &px, &py, &pz);
	PCoords (p1, StdFrame (World), &p1x, &p1y, &p1z);
	if (1.0 != 1.0 + nz) {
	    p = PCreate (StdFrame (World), px, py, p1z - (vx * nx + vy * ny) / nz);
	} else {
	    p = PCreate (StdFrame (World), px, py, FLT_MAX);
	} /* if  */
	return p;
    } /* if  */
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
 */
void ConstructBoundary (Vertex* vr[3], Patch* patch)
{
    Point	Pr, Ps, Pt;
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
    PatchSetPoint( patch, EdgeTangent( Vr, Vs ), 2, 1, 0);
    PatchSetPoint( patch, EdgeTangent( Vs, Vr ), 1, 2, 0);

    /* Do the st edge points */
    PatchSetPoint( patch, EdgeTangent( Vs, Vt ), 0, 2, 1);
    PatchSetPoint( patch, EdgeTangent( Vt, Vs ), 0, 1, 2);

    /* Do the tr edge points */
    PatchSetPoint( patch, EdgeTangent( Vt, Vr ), 1, 0, 2);
    PatchSetPoint( patch, EdgeTangent( Vr, Vt ), 2, 0, 1);
} /* ConstructBoundary  */

/*
 * Gethk
 *
 * When using Chiyokura-Kimura boundary conditions
 * need h0, h1, k0, k1 scalar quantities.
 */

void Gethk (Patch *patch, Patch *npatch, 
		Scalar *k0, Scalar *k1, Scalar *h0, Scalar *h1)
{
    Vector	F0, C0, H0;
    Vector	F1, C1, H1;
    Frame	frame0, frame1;
    Scalar	s0, s1;

    F1 = PPDiff (	PatchGetPoint (*patch, 2, 0, 1),
			PatchGetPoint (*patch, 3, 0, 0));
    F0 = PPDiff (	PatchGetPoint (*patch, 0, 2, 1),
			PatchGetPoint (*patch, 0, 3, 0));
    C1 = PPDiff (	PatchGetPoint (*npatch, 2, 0, 1),
			PatchGetPoint (*patch, 2, 0, 1));
    C0 = PPDiff (	PatchGetPoint (*npatch, 0, 2, 1),
			PatchGetPoint (*patch, 0, 2, 1));
    C0 = VNormalize (C0);
    C1 = VNormalize (C1);

    H1 = PPDiff (	PatchGetPoint (*patch, 3, 0, 0),
			PatchGetPoint (*patch, 2, 1, 0));
    H1 = SVMult (3.0, H1);
    H0 = PPDiff (	PatchGetPoint (*patch, 1, 2, 0),
			PatchGetPoint (*patch, 0, 3, 0));
    H0 = SVMult (3.0, H0);

    frame0 = FCreate ("Gethk frame 0", PatchGetPoint (*patch, 0, 3, 0),
		    C0, H0, VVCross (C0, H0));
    frame1 = FCreate ("Gethk frame 1", PatchGetPoint (*patch, 3, 0, 0),
		    C1, H1, VVCross (C1, H1));
    
    VCoords (F0, frame0, k0, h0, &s0);
    VCoords (F1, frame1, k1, h1, &s1);

#if 0
    {
	/* verify h's and k's */

	Vector	testF0, testF1;
	Vector	testC0, testC1, testH0, testH1;
	Scalar	fx, fy, fz;

	testC0 = SVMult (*k0, C0);
	testC1 = SVMult (*k1, C1);
	testH0 = SVMult (*h0, H0);
	testH1 = SVMult (*h1, H1);

	testF0 = VVAdd (testC0, testH0);
	testF1 = VVAdd (testC1, testH1);

	testF0 = VVDiff (testF0, F0);
	testF1 = VVDiff (testF1, F1);

	VCoords (testF0, StdFrame(World), &fx, &fy, &fz);
	fprintf (stderr, "F0: %lf %lf %lf\n", fx, fy ,fz);
	VCoords (testF1, StdFrame(World), &fx, &fy, &fz);
	fprintf (stderr, "F1: %lf %lf %lf\n", fx, fy ,fz);
    }
#endif
} /* Gethk  */

/*
 * GetInterior
 *
 * Given: - face
 *        - 3 vertices defining a face
 *          1st 2 vertices define edge under consideration
 * Calc:  interior point of face as per Foley
 *
 *
 * Return: true if neighbouring face exists, false otherwise
 */
BOOLEAN GetInterior (Face* face, Vertex* v0, Vertex* v1, Vertex* v2, Point* p)
{
    Face	*nface, *f1, *f2;
    Edge*	e;
    Vertex	*v, *vr[3], *nvr[3];
    int		i;
    Patch	patch, npatch;
    Frame	std_frame;
    Scalar	px, py, pz, dummy;
    Scalar	c102, c012, b300, b030, b003, 
		    b210, b120, b201, b102, b021, b012,
		    b111x, b111y, b111z;
    Scalar	rst [3]; /* r==rst[0], s=rst[1], t=rst[1] */
    Scalar	r, s, t;
    Point	tmp_pt;

    /* construct patch with vertices in order v0,v1,v2 */
    {
	vr [0] = v0;
	vr [1] = v1;
	vr [2] = v2;

	ConstructBoundary (vr, &patch);
    }

    /* find and process neighbouring face */
    {
	/* get edge associated with v0 and v1 */
	GetVertexEdge (v0, v1, &e);

	/* try reverse order, maybe order was wrong */
	if (NULL == e) {
	    GetVertexEdge (v1, v0, &e);
	    if (NULL == e) {
		fprintf (stderr, "Foley: GetVertexEdge returned NULL\n");
		return FALSE;
	    } /* if  */
	} /* if  */

	/* get faces associated with e */
	GetEdgeFaces (e, &f1, &f2);

	if (f1 != face) {
	    nface = f1;
	} else {
	    nface = f2;
	} /* if  */

	/* neighbour doesn't exist! */
	if (NULL == nface) return FALSE;

	if (!cloughTocher) {
	    i = 0;
	    ForeachFaceVertex(nface,v){
	      vr[i++] = v;
	    } EndForeach;

	    /* assign v to be the vertex on neighbour which is not v0 or v1 */
	    if (v0 == vr[0]) {
		if (v1 == vr[1]) v = vr[2];
		    else v = vr[1];
	    } else if (v0 == vr[1]) {
		if (v1 == vr[0]) v = vr[2];
		    else v = vr[0];
	    } else {
		if (v1 == vr[0]) v = vr[1];
		    else v = vr[0];
	    } /* if  */


	    /* order points so that vertex with multi-index (0,0,3)
	     * is found in the neighbouring patch
	     */
	    nvr [0] = v0;
	    nvr [1] = v1;
	    nvr [2] = v;

	    ConstructBoundary (nvr, &npatch);
	} /* if  */
    } 

    if (!cloughTocher) {
	std_frame = StdFrame (World);

	PCoords (PatchGetPoint (patch,3,0,0), std_frame, &px, &py, &pz);
	b300 = pz;
	PCoords (PatchGetPoint (patch,0,3,0), std_frame, &px, &py, &pz);
	b030 = pz;
	PCoords (PatchGetPoint (patch,0,0,3), std_frame, &px, &py, &pz);
	b003 = pz;
	PCoords (PatchGetPoint (patch,2,1,0), std_frame, &px, &py, &pz);
	b210 = pz;
	PCoords (PatchGetPoint (patch,1,2,0), std_frame, &px, &py, &pz);
	b120 = pz;
	PCoords (PatchGetPoint (patch,2,0,1), std_frame, &px, &py, &pz);
	b201 = pz;
	PCoords (PatchGetPoint (patch,1,0,2), std_frame, &px, &py, &pz);
	b102 = pz;
	PCoords (PatchGetPoint (patch,0,2,1), std_frame, &px, &py, &pz);
	b021 = pz;
	PCoords (PatchGetPoint (patch,0,1,2), std_frame, &px, &py, &pz);
	b012 = pz;
    } /* if  */

    if (chiyokuraBoundary) {
	Scalar	a, b, c, d, e, h0, h1, k0, k1, c0, c1, cx, cy;
	Vector	vecC0, vecC1;
	static int	first = 1;

	/*
	 * Solve system of equations:
	 *  ax + by = d
	 *  cx + by = e
	 *
	 *  a = 4/3 * c1
	 *  c = 4/3 * c0
	 *  x = k        ( k(t) = k0*u0(t) +  k*u1(t) + k1*u2(t) )
	 *  y = b111     ( the value we are interested in )
	 */

	if ( first ) {
	    fprintf(stderr,"Using Chiyokura-Kimura conditions\n");
	    first=0;
	}

	vecC0 = PPDiff (	PatchGetPoint (npatch, 0, 2, 1),
				PatchGetPoint (patch, 0, 2, 1));
	vecC1 = PPDiff (	PatchGetPoint (npatch, 2, 0, 1),
				PatchGetPoint (patch, 2, 0, 1));
	vecC0 = VNormalize (vecC0);
	vecC1 = VNormalize (vecC1);

	VCoords (vecC0, std_frame, &cx, &cy, &c0);
	VCoords (vecC1, std_frame, &cx, &cy, &c1);

	Gethk (&patch, &npatch, &k0, &k1, &h0, &h1);
	/* fprintf (stderr, "k0(%lf), k1(%lf), h0(%lf), h1(%lf)\n",
	    k0,k1,h0,h1); */

	a = 4.0 * c1 / 3.0;
	b = 2.0;
	c = 4.0 * c0 / 3.0;
	d = - h0 * (b300-b210) - 2 * h1 * (b210-b120) - k1 * c0 / 3.0
	    + k0 * c1 / 3.0 + k1 * c1 / 3.0 + 3 * b210 - b201;
	e = -2 * h0 * (b210-b120) - h1 * (b120-b030) + k0 * c0/3.0
	    + k1 * c0 / 3.0 - k0 * c1 / 3.0 + 3 * b120 - b021;

#define MATTHEWS_EPS	0.0001
	if (fabs(a-c) < MATTHEWS_EPS) {
	    if (fabs(d-e) < MATTHEWS_EPS) {
		/* set k = 1 (ie x=1) to get     b111z = (d - 4/3 * c1)/b */
		b111z = (d - a * (k0+k1)/2.0) / b;
		/* fprintf (stderr, "here\n"); */
	    } else {
		fprintf (stderr, "No solution\n");
		fprintf (stderr, "a(%lf), c(%lf), d(%lf), e(%lf)\n",a,c,d,e);
		exit(1);
	    } /* if  */
	} else {
	    b111z = (d - a*(d-e)/(a-c)) / b;
	    /* fprintf (stderr, "a(%lf), c(%lf), d(%lf), e(%lf)\n",a,c,d,e); */
	    /* fprintf (stderr, "fsbs(a-c) != 0\n"); */
	} /* if  */
    } else if (cloughTocher) {
	Vertex *r,*s,*t;
	Vertex *vtx1, *vtx2;
	Point p1,p2;
	Point ptA,ptB;
	Vector v1, v2;
	Normal n;
	Point p;
	Point rcc;
	Scalar	x, y, z;

	r = v0; s = v1; t = v2;

	vtx1 = s;
	vtx2 = t;
	p1 = EdgeTangent( vtx1, ReturnUDPoint(VertexPath(vtx1,vtx2,"1")) );
	p2 = EdgeTangent( vtx2, ReturnUDPoint(VertexPath(vtx1,vtx2,"1")) );
	ptA = PPac(p1,p2,1.0/2.0);
	ptB = PPac(	PatchGetPoint(patch,1,2,0), 
		    PatchGetPoint(patch,1,0,2), 1.0/2.0);
	v1 = PPDiff(ptA,ptB);
	v2 = PPDiff( PatchGetPoint(patch,0,1,2),
		  PatchGetPoint(patch,0,2,1));
	n = VDual(VVCross(v1,v2));
	p = PatchGetPoint(patch,0,1,2);
	p1 = PPac3( ReturnUDPoint(r),ReturnUDPoint(s),ReturnUDPoint(t),
		 1.0/9.0, 4.0/9.0, 4.0/9.0);
	v1 = FV( StdFrame(SpaceOf(p1)), 2 );
	rcc = LineIntersectPlane(p1,v1,p,n);

	/* use rcc to calculate z component for centroid of triangle */

	/* find intersection of plane defined by (p, (0,2,1)-p, rcc-p)
	 * with vertical line through the centroid 
	 */
	v1 = PPDiff (PatchGetPoint (patch, 0,2,1), PatchGetPoint (patch,0,1,2));
	v2 = PPDiff (p, PatchGetPoint (patch,0,1,2));
	n = VDual(VVCross(v1,v2));
	v1 = FV( StdFrame(SpaceOf(p1)), 2 );

	*p = LineIntersectPlane (p1, v1, p, n);

	return TRUE;

    } else {

	PCoords (PatchGetPoint (npatch,1,0,2), std_frame, &px, &py, &pz);
	c102 = pz;
	PCoords (PatchGetPoint (npatch,0,1,2), std_frame, &px, &py, &pz);
	c012 = pz;

	GetBaryCoords (	PatchGetPoint (patch,3,0,0),
			PatchGetPoint (patch,0,3,0),
			PatchGetPoint (patch,0,0,3),
			PatchGetPoint (npatch,0,0,3), rst);

	r = rst[0];
	s = rst[1];
	t = rst[2];

	if ((1.0 + s + r != 1.0) && (1.0 + t != 1.0)) {
	    b111z = ( c102 + c012 - r*r*(b300+b210) - 2*r*s*(b210+b120) - 
		2*r*t*b201 - 2*s*t*b021 - s*s*(b120+b030) - 
		t*t*(b102+b012) ) / 2.0 / (r + s) / t;
	} else {
	    fprintf (stderr, "Foley: (%s, line %d) unable to compute "
			"interior patch point\n", __FILE__, __LINE__);
	    exit (1);
	} /* if */
    } /* if  */

    /* give *p x,y coordinates in center of triangle */
    *p = PPac3( PatchGetPoint (patch, 3, 0, 0),
		PatchGetPoint (patch, 0, 3, 0),
		PatchGetPoint (patch, 0, 0, 3),
		1.0/3.0, 1.0/3.0, 1.0/3.0);
    PCoords (*p, std_frame, &b111x, &b111y, &dummy);

    *p = PCreate (std_frame, b111x, b111y, b111z);

    return TRUE;
} /* GetInterior  */

BOOLEAN GetCloughTocherPoints (Patch *patch

typedef struct FoleyStruct {
    Patch	patch0, patch1, patch2;
} FoleyStruct;

Point EvalPatch (FoleyStruct* data, Scalar b[3]);

void ProcessFace (Face* face)
{
    FoleyStruct	data;
    Vertex	*v, *vr[3];
    int		i;
    int		exist0, exist1, exist2; /* does face 0, 1, 2 exist? */
    Point	p0, p1, p2;

    i = 0;
    ForeachFaceVertex(face,v){
      vr[i++] = v;
    } EndForeach;

    ConstructBoundary (vr, &data.patch0);
    ConstructBoundary (vr, &data.patch1);
    ConstructBoundary (vr, &data.patch2);

    exist0 = GetInterior (face, vr[1], vr[2], vr [0], &p0);
    exist1 = GetInterior (face, vr[2], vr[0], vr [1], &p1);
    exist2 = GetInterior (face, vr[0], vr[1], vr [2], &p2);

    /* handle cases where one or more boundaries are missing */
    if (!exist0 && !exist1 && !exist2) {
	/* no neighbours! take affine combination of boundary */
	Point P[9];
	Scalar w[9];

	P[0] = PatchGetPoint (data.patch0, 2, 1, 0);
	P[1] = PatchGetPoint (data.patch0, 1, 2, 0);
	P[2] = PatchGetPoint (data.patch0, 0, 2, 1);
	P[3] = PatchGetPoint (data.patch0, 0, 1, 2);
	P[4] = PatchGetPoint (data.patch0, 1, 0, 2);
	P[5] = PatchGetPoint (data.patch0, 2, 0, 1);
	P[6] = PatchGetPoint (data.patch0, 3, 0, 0);
	P[7] = PatchGetPoint (data.patch0, 0, 3, 0);
	P[8] = PatchGetPoint (data.patch0, 0, 0, 3);

	/* Set the middle point to the centroid of the edge points */
	w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
	w[6] = w[7] = w[8] = -(1.0/6.0);
	p0 = p1 = p2 = PPacN( 9, P, w);
    } else if (!exist0) {
	if (!exist1) {
	    p0 = p1 = p2;
	} else if (!exist2) {
	    p0 = p2 = p1;
	} else {
	    p0 = PPac (p1, p2, 0.5);
	} /* if  */
    } else if (!exist1) {
	if (!exist0) {
	    p0 = p1 = p2;
	} else if (!exist2) {
	    p1 = p2 = p0;
	} else {
	    p1 = PPac (p0, p2, 0.5);
	} /* if  */
    } else if (!exist2) {
	if (!exist0) {
	    p0 = p2 = p1;
	} else if (!exist1) {
	    p1 = p2 = p0;
	} else {
	    p2 = PPac (p0, p1, 0.5);
	} /* if  */
    } /* if  */

    PatchSetPoint (&data.patch0, p0, 1, 1, 1);
    PatchSetPoint (&data.patch1, p1, 1, 1, 1);
    PatchSetPoint (&data.patch2, p2, 1, 1, 1);

#if OUTPUT_PATCH
    PatchWrite (stdout, data.patch0);
    /* PatchWrite (stdout, data.patch1); */
    /* PatchWrite (stdout, data.patch2); */
#else
    SetTessEps (M_EPSILON);

    /* tesselate patch */
    if (gaussian) {
	gkTessPatch(&data,EvalPatch,samples,StdFrame(World));
    } else {
	TessPatch(&data,EvalPatch,samples,StdFrame(World));
    } /* if */
#endif

    PatchFree(data.patch0);
    PatchFree(data.patch1);
    PatchFree(data.patch2);
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
	/* fprintf (stderr, "Foley: (EvalPatch) u[0]=1-EPSILON\n"); */
	if (u[1] == M_EPSILON) {
	    return PatchGetPoint (data->patch0, 2, 1, 0);
	} else {
	    return PatchGetPoint (data->patch0, 2, 0, 1);
	} /* if  */
    } else if (u[1] == 1.0 - M_EPSILON) {
	/* fprintf (stderr, "Foley: (EvalPatch) u[1]=1-EPSILON\n"); */
	if (u[0] == M_EPSILON) {
	    return PatchGetPoint (data->patch0, 1, 2, 0);
	} else {
	    return PatchGetPoint (data->patch0, 0, 2, 1);
	} /* if  */
    } else if (u[2] == 1.0 - M_EPSILON) {
	/* fprintf (stderr, "Foley: (EvalPatch) u[2]=1-EPSILON\n"); */
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
    } else {
	Scalar	result, x, y, z, w0, w1, w2, denom;

	denom = u[0]*u[1] + u[0]*u[2] + u[1]*u[2];

	/* fprintf (stderr, "denom (%le)\n", denom); */
	/* fprintf (stderr, "EP: (%lf)\n", (double)EPSILON); */

	w0 = (u[1] * u[2]) / denom;
	w1 = (u[0] * u[2]) / denom;
	w2 = (u[0] * u[1]) / denom;
	PCoords (PatchEval (data->patch0, u[0], u[1], u[2]), 
		    StdFrame (World), &x, &y, &z);
	result = w0 * z;
	PCoords (PatchEval (data->patch1, u[0], u[1], u[2]), 
		    StdFrame (World), &x, &y, &z);
	result += w1 * z;
	PCoords (PatchEval (data->patch2, u[0], u[1], u[2]), 
		    StdFrame (World), &x, &y, &z);
	result += w2 * z;
	return PCreate (StdFrame (World), x, y, result);
    } /* if  */
} /* EvalPatch */



char* Banner = "Foley";
char* UsageString = "Foley [option] < mesh ";

extern void Usage();

void Verbose()
{
	verbose = 1;
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


void SetFurthBoundary()
{
    furthBoundary = 1;
} /* SetFurthBoundary */

void SetGoodBoundary()
{
    goodBoundary=1;
} /* SetGoodBoundary */

void SetChiyokura()
{
    chiyokuraBoundary = 1;
} /* SetChiyokura */

void SetClough()
{
    cloughTocher = 1;
} /* SetClough */



Option Options[] = {
/* Name,        Routine,        #args,  Help String */
   "h",         Usage,          0,      ":      Print available options.",
   "v",         Verbose,        0,      ":      Verbose mode.",
   "s",         SetSamples,     1,      "n :    Number of samples per side.",
   "k",		Gaussian,	0,	":      Compute Gaussian curvature.",
   "dbhs",      SetGoodBoundary, 0,     ":   Use deBoor, Hollig, Sabin boundaries.",
   "fb",        SetFurthBoundary,0,     ":     Use Furth Boundaries [default].",
   "chiyo",	SetChiyokura,	0,	":  Use Chiyokura-Kimura boundaries",
   "clough",	SetClough,	0,	":  Use Clough-Tocher centre point",

   NULL,        NULL,           0,      NULL
   };

