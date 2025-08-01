/*
 * Copyright (c) 1997, Computer Graphics Laboratory, University of Waterloo
 * Copyright (c) 1996, CGL, University of Waterloo
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  MFarin.c
 *----------------------------------------------------------------------
 */

/*
** Authors: Steve Mann
** Purpose: An implementation of Clough-Toucher using a modification
**	of Farin's boundary curves.  While Farin minimized the C^2
**	discontinuity between mini-triangles, in this variation
**	we minimize the C^2 discontinuity between marco-triangles.
**
*/
#include <stdio.h>
#include <math.h>
#include "all.h"
#include "ctutils.h"
#include <sys/time.h>
#include <sys/resource.h>

/* Forward declarations */
extern void SetNormals();
extern Normal ComputeNormal();
extern Patch BuildPatch();
static BOOLEAN GetInterior(Vertex* v0, Vertex* v1, Vertex* v2, Point* p);

Point LineIntersectPlane(Point,Vector,Point,Normal);


/* Global variables */
static char *RoutineName;
static Space World;                 /* Space where nhoods and patches reside */




/*
 *    FIRST OF THREE CASES:
 *
 *
 *               t=003 = vtx2
 *                 V
 *                /|\
 *               / | \
 *          102 x  |  x p2
 *             /|  x  |\          <- 012
 *            x |  |  | \
 *           /  |  |  |  \
 *   r=300  V  ptB | ptA  V Pt[2]
 *           \  |  |  |  /
 *            x |  |  | /
 *             \|  x  |/          <- 021
 *          120 x  |  x p1
 *               \ | /
 *                \|/
 *                 V
 *               s=030 = vtx1
 */




void AdjustPatch(Vertex* vr[3],Patch patch,
		 Patch* pr, Patch* ps, Patch* pt)
{
	Vertex *r,*s,*t;
	Vertex *vtx1, *vtx2;
	Point p1,p2;
	Point ptA,ptB;
	Vector v1, v2;
	Normal n;
	Point p;
	Point rcc,scc,tcc;
	Point prs, pst, ptr;
	Point centroid;
	Point q;
	
	r = vr[0];
	s = vr[1];
	t = vr[2];
	
	
	q = PatchGetPoint(patch,1,1,1);
	GetInterior(r,s,t,&p);
	PatchSetPoint(&patch,p,1,1,1);
	PatchSplit(patch, pr, ps, pt, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	tcc = PatchGetPoint(*pt, 1,1,1);
	GetInterior(s,t,r,&p);
	PatchSetPoint(&patch,p,1,1,1);
	PatchSplit(patch, pr, ps, pt, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	rcc = PatchGetPoint(*pr, 1,1,1);
	GetInterior(t,r,s,&p);
	PatchSetPoint(&patch,p,1,1,1);
	PatchSplit(patch, pr, ps, pt, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	scc = PatchGetPoint(*ps, 1,1,1);
	
	PatchSplit(patch, pr, ps, pt, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	
	PatchSetPoint(pr,rcc,1,1,1);
	PatchSetPoint(ps,scc,1,1,1);
	PatchSetPoint(pt,tcc,1,1,1);
	
	prs = PPac3( rcc, scc, PatchGetPoint(*pr, 1, 0, 2), 
		    1.0/3.0, 1.0/3.0, 1.0/3.0);
	pst = PPac3( scc, tcc, PatchGetPoint(*ps, 2, 1, 0), 
		    1.0/3.0, 1.0/3.0, 1.0/3.0);
	ptr = PPac3( tcc, rcc, PatchGetPoint(*pt, 0, 2, 1), 
		    1.0/3.0, 1.0/3.0, 1.0/3.0);
	
	PatchSetPoint(pr, prs, 2, 0, 1);
	PatchSetPoint(ps, prs, 0, 2, 1);
	
	PatchSetPoint(ps, pst, 1, 2, 0);
	PatchSetPoint(pt, pst, 1, 0, 2);
	
	PatchSetPoint(pt, ptr, 0, 1, 2);
	PatchSetPoint(pr, ptr, 2, 1, 0);
	
	centroid = PPac3(prs,pst,ptr,1.0/3.0,1.0/3.0,1.0/3.0);
	PatchSetPoint(pr, centroid, 3,0,0);
	PatchSetPoint(ps, centroid, 0,3,0);
	PatchSetPoint(pt, centroid, 0,0,3);
	
}

void ProcessFace( Face* f )
{
    Patch patch;
    Patch pr,ps,pt;
    Vertex *vr[3],*v;
    Point P[9];
    Scalar w[9];
    int i;

    /* Create the cubic patch */
    patch = PatchCreate( 3, World, 0);

    i = 0;
    ForeachFaceVertex(f, v) {
	    if ( i >= 3 ) {
		    fprintf(stderr,"MFarin: ProcessFace(%s): too many vertices\n",ReturnName(f));
		    exit(1);
	    }
	    vr[i++] = v;
    } EndForeach;

    /* Set the corner points of the net to the vertices of the nhood */
    PatchSetPoint( &patch, ReturnUDPoint(vr[0]), 3, 0, 0); 
    P[6]=ReturnUDPoint(vr[0]);
    PatchSetPoint( &patch, ReturnUDPoint(vr[1]), 0, 3, 0); 
    P[7]=ReturnUDPoint(vr[1]);
    PatchSetPoint( &patch, ReturnUDPoint(vr[2]), 0, 0, 3); 
    P[8]=ReturnUDPoint(vr[2]);

    /* Set the boundary points based on tangent planes */
    /* at nhood vertices using the method described in  */
    /* Clough-Toucher                                  */

    /* Do the rs edge points */
    P[0]=EdgeTangent( vr[0], ReturnUDPoint(vr[1]) );
    P[1]=EdgeTangent( vr[1], ReturnUDPoint(vr[0]) );
    PatchSetPoint( &patch, P[0], 2, 1, 0);
    PatchSetPoint( &patch, P[1], 1, 2, 0);

    /* Do the st edge points */
    P[2]=EdgeTangent( vr[1], ReturnUDPoint(vr[2]));
    P[3]=EdgeTangent( vr[2], ReturnUDPoint(vr[1]));
    PatchSetPoint( &patch, P[2], 0, 2, 1);
    PatchSetPoint( &patch, P[3], 0, 1, 2);

    /* Do the tr edge points */
    P[4]=EdgeTangent( vr[2], ReturnUDPoint(vr[0]));
    P[5]=EdgeTangent( vr[0], ReturnUDPoint(vr[2]));
    PatchSetPoint( &patch, P[4], 1, 0, 2);
    PatchSetPoint( &patch, P[5], 2, 0, 1);

    /* Set the middle point to the centroid of the edge points */
    w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
    w[6] = w[7] = w[8] = -(1.0/6.0);
    PatchSetPoint( &patch, PPacN( 9, P, w), 1, 1, 1);

    AdjustPatch(vr,patch,&pr,&ps,&pt);

    PatchWrite(stdout,pr);
    PatchWrite(stdout,ps);
    PatchWrite(stdout,pt);
}


int verbose=0;

main(int argc, char **argv)
{
	Mesh* m;
	Face* f;
	int num;

	ParseCommandLine(argc, argv);
	World = SCreate("World", 3);

	m = MeshParse(stdin);
	AddGeometry(World, m);

        /* If no normals, complain and quit. */
        if ( !(ReturnUDGeoFlags(m) & G_NORMAL) ) {
                fprintf(stderr,"MFarin: normals not given.  Exiting.\n");                exit(1);
        } 
        ForeachMeshFace(m,f){
        
                if ( ! BoundaryFace(f) ) {
                        if ( verbose  &&  f->name != NULL ) { 
				fprintf(stderr,"process face %s\n",f->name);
			}
                        ProcessFace(f);
                }
                num += 1;

        } EndForeach;

        if (verbose){
                struct rusage u;

                getrusage(RUSAGE_SELF, &u);
                fprintf(stderr,"MFarin: finished.  ");
                fprintf(stderr,"%d neighborhoods processed in %d cpu seconds.\n",
                        num, u.ru_utime);
        }
        exit(0);
}



char* Banner = "MFarin";
char* UsageString = "MFarin [option] < mesh";
Option Options[] = {
/* Name,        Routine,        #args,  Help String */
   "h",         Usage,          0,      ":      Print available options.",
};


static BOOLEAN GetInterior(Vertex* v0, Vertex* v1, Vertex* v2, Point* p)
{
    Face* face;
    Face	*nface, *f1, *f2;
    Edge*	e;
    Vertex	*vv, *vr[3], *nvr[3];
    int		i;
    Patch	patch, npatch;
    Frame	std_frame;
    Scalar	px, py, pz, dummy;
    Scalar	b111x, b111y, b111z;
    Scalar	rst [3]; /* r==rst[0], s=rst[1], t=rst[1] */
    Scalar	r, s, t;
    Scalar 	u,v,w;
    Point	tmp_pt;

    /* construct patch with vertices in order v0,v1,v2 */
    vr [0] = v0;
    vr [1] = v1;
    vr [2] = v2;

    ConstructBoundary (vr, &patch);

    /* find and process neighbouring face */
    {
	/* get edge associated with v0 and v1 */
	GetVertexEdge (v0, v1, &e);

	/* try reverse order, maybe order was wrong */
	if (NULL == e) {
	    GetVertexEdge (v1, v0, &e);
	    if (NULL == e) {
		fprintf (stderr, "MFarin: GetVertexEdge returned NULL\n");
		return FALSE;
	    } /* if  */
	} /* if  */

	/* get faces associated with e */
	GetEdgeFaces (e, &f1, &f2);

	{ Vertex* vv;
	  face = f2;
	ForeachFaceVertex(f1, vv) {
		if ( vv == v2 ) {
			face = f1;
		}
	} EndForeach;
        }

	if (f1 != face) {
	    nface = f1;
	} else {
	    nface = f2;
	} /* if  */

	if ( nface == face ) {
		fprintf(stderr,"MFarin -- GetInterior: You goofed!\n");
		exit(1);
	}

	/* neighbour doesn't exist! */
	if (NULL == nface) return FALSE;

	i = 0;
	ForeachFaceVertex(nface,vv){
	  vr[i++] = vv;
	} EndForeach;

	/* assign v to be the vertex on neighbour which is not v0 or v1 */
	if (v0 == vr[0]) {
	    if (v1 == vr[1]) vv = vr[2];
		else vv = vr[1];
	} else if (v0 == vr[1]) {
	    if (v1 == vr[0]) vv = vr[2];
		else vv = vr[0];
	} else {
	    if (v1 == vr[0]) vv = vr[1];
		else vv = vr[0];
	} /* if  */


	/* order points so that vertex with multi-index (0,0,3)
	 * is found in the neighbouring patch
	 */
	nvr [0] = v0;
	nvr [1] = v1;
	nvr [2] = vv;

	ConstructBoundary (nvr, &npatch);
    } 

    std_frame = StdFrame (World);

    GetBaryCoords (	PatchGetPoint (patch,3,0,0),
		   PatchGetPoint (patch,0,3,0),
		   PatchGetPoint (patch,0,0,3),
		   PatchGetPoint (npatch,0,0,3), rst);
    
    r = rst[0];
    s = rst[1];
    t = rst[2];

    GetBaryCoords (PatchGetPoint (patch,3,0,0),
		   PatchGetPoint (patch,0,3,0),
		   PatchGetPoint (npatch,0,0,3),
		   PatchGetPoint (patch,0,0,3), rst);
    u = rst[0];
    v = rst[1];
    w = rst[2];

    {
	Scalar a1,a2,a3,a4;
	Scalar b1,b2, c1,c2;
	Scalar d1,d2, e1,e2;

	Scalar s1, a12,s2,r3,a11,r1,r2,a22,D;

	Scalar u,v,w, uh,vh,wh;
	Scalar y;

	/* C1 = patch,0,0,3
	   P1 = patch,3,0,0
	   P2 = patch,0,3,0
	   C2 = npatch,0,0,3
	 */
	/* C1 = uh*C2 + vh*P1 * wh*P2 */
	GetBaryCoords (PatchGetPoint (patch,3,0,0),
		       PatchGetPoint (patch,0,3,0),
		       PatchGetPoint (npatch,0,0,3),
		       PatchGetPoint (patch,0,0,3), rst);
	uh = rst[2];
	vh = rst[0];
	wh = rst[1];

	/* C2 = u*C1 + v*P2 * w*P1 */
	GetBaryCoords (PatchGetPoint (patch,0,3,0), 
		       PatchGetPoint (patch,3,0,0),
		       PatchGetPoint (patch,0,0,3),
		       PatchGetPoint (npatch,0,0,3),rst);
	u = rst[2];
	v = rst[0];
	w = rst[1];

	PCoords (PatchGetPoint (patch,3,0,0), std_frame, &px, &py, &pz);
	/*b300*/a1 = pz;
	PCoords (PatchGetPoint (patch,0,3,0), std_frame, &px, &py, &pz);
	/*b030*/a4 = pz;
	PCoords (PatchGetPoint (patch,2,1,0), std_frame, &px, &py, &pz);
	/*b210*/a2 = pz;
	PCoords (PatchGetPoint (patch,1,2,0), std_frame, &px, &py, &pz);
	/*b120*/a3 = pz;
	PCoords (PatchGetPoint (patch,2,0,1), std_frame, &px, &py, &pz);
	/*b201*/b1 = pz;
	PCoords (PatchGetPoint (patch,1,0,2), std_frame, &px, &py, &pz);
	/*b102*/d1 = pz;
	PCoords (PatchGetPoint (patch,0,2,1), std_frame, &px, &py, &pz);
	/*b021*/b2 = pz;
	PCoords (PatchGetPoint (patch,0,1,2), std_frame, &px, &py, &pz);
	/*b012*/d2 = pz;
	
	PCoords (PatchGetPoint (npatch,1,0,2), std_frame, &px, &py, &pz);
	/*c102*/e1 = pz;
	PCoords (PatchGetPoint (npatch,0,1,2), std_frame, &px, &py, &pz);
	/*c012*/e2 = pz;
	
	PCoords (PatchGetPoint (npatch,2,0,1), std_frame, &px, &py, &pz);
	/*c201*/c1 = pz;
	PCoords (PatchGetPoint (npatch,0,2,1), std_frame, &px, &py, &pz);
	/*c021*/c2 = pz;

	r1 = uh*e1 + vh*c1 - u*d1 - w*b1;
	r2 = uh*e2 + wh*c2 - u*d2 - v*b2;
	r3 = v*a3 + w*a2;
	a11 = 2.*(v*v + w*w);
	a12 = -2.*(v*wh + w*vh);
	a22 = 2.*(wh*wh + vh*vh);
	s1 = 2.*(v*r1 + w*r2);
	s2 = -2.*(wh*r1 + vh*r2);
	D = 2.*u*a12 + a22*u*u + a11;

	{Scalar x;
	y = (u*s1 + u*a12*r3 + u*u*s2 + r3*a11)/D;
	x = (y - v*a3 - w*a2)/u;

	 y = x;}

	/* give *p x,y coordinates in center of triangle */
	*p = PPac3( PatchGetPoint (patch, 3, 0, 0),
		   PatchGetPoint (patch, 0, 3, 0),
		   PatchGetPoint (patch, 0, 0, 3),
		   1.0/3.0, 1.0/3.0, 1.0/3.0);
	PCoords (*p, std_frame, &b111x, &b111y, &dummy);
    
	*p = PCreate (std_frame, b111x, b111y, y);
	return TRUE;
    }

} /* GetInterior  */
