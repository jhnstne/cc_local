/*
 * Copyright (c) 1997, Computer Graphics Laboratory, University of Waterloo
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  Farin.c
 *----------------------------------------------------------------------
 */

/*
** Authors: Tony DeRose and Steve Mann
** Last Modified: May 30, 1997
** Purpose: An implementation of Farin's modification to the
**	Clough-Toucher scheme for constructing C1 cubic surface.
**	Farin's method minimizes the C^2 discontinuity between
**	mini-triangles.
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

Point LineIntersectPlane(Point,Vector,Point,Normal);

/* Global variables */
static char *RoutineName;
static Space World;                 /* Space where nhoods and patches reside */



Patch QuadraticInterpolant(Vertex* v0, Vertex* v1, Vertex* v2)
{
	Patch patch;
	Point P[9];
	Scalar w[9];
	/* Create the cubic patch */
	patch = PatchCreate( 3, World, 0);

	/* Set the corner points of the net to the vertices */
	PatchSetPoint( &patch, ReturnUDPoint(v0), 3, 0, 0); 
	P[6]=ReturnUDPoint(v0);
	PatchSetPoint( &patch, ReturnUDPoint(v1), 0, 3, 0); 
	P[7]=ReturnUDPoint(v1);
	PatchSetPoint( &patch, ReturnUDPoint(v2), 0, 0, 3); 
	P[8]=ReturnUDPoint(v2);

	/* Set the boundary points based on tangent planes */
	/* Do the rs edge points */
	P[0]=EdgeTangent( v0, ReturnUDPoint(v1) );
	P[1]=EdgeTangent( v1, ReturnUDPoint(v0) );
	PatchSetPoint( &patch, P[0], 2, 1, 0);
	PatchSetPoint( &patch, P[1], 1, 2, 0);

	/* Do the st edge points */
	P[2]=EdgeTangent( v1, ReturnUDPoint(v2));
	P[3]=EdgeTangent( v2, ReturnUDPoint(v1));
	PatchSetPoint( &patch, P[2], 0, 2, 1);
	PatchSetPoint( &patch, P[3], 0, 1, 2);

	/* Do the tr edge points */
	P[4]=EdgeTangent( v2, ReturnUDPoint(v0));
	P[5]=EdgeTangent( v0, ReturnUDPoint(v2));
	PatchSetPoint( &patch, P[4], 1, 0, 2);
	PatchSetPoint( &patch, P[5], 2, 0, 1);

	/* Set the middle point to quadratic precision ala Farin */
	w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
	w[6] = w[7] = w[8] = -(1.0/6.0);
	PatchSetPoint( &patch, PPacN( 9, P, w), 1, 1, 1);

	return patch;
}

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
	Point p1,p2;
	Point ptA,ptB;
	Vector v1, v2;
	Normal n;
	Point p;
	Point rcc,scc,tcc;
	Point prs, pst, ptr;
	Point centroid;
	Scalar x,y,z;
	Patch op, or, os, ot;
	/* Scalar's for Farin's scheme */
	Scalar a1,a2,a3,a4;
	Scalar b1,b2, c1,c2;
	Scalar d1,d2, e1,e2;
	Scalar s1, a12,s2,r3,a11,r1,r2,a22,D;
	Scalar u,v,w, uh,vh,wh;
	Scalar rst[3];
	Scalar px, py, pz;
	Frame std_frame;

	std_frame = StdFrame(World);

	r = vr[0];
	s = vr[1];
	t = vr[2];

	PatchSplit(patch, pr, ps, pt, 1.0/3.0, 1.0/3.0, 1.0/3.0);

	/* st boundary */
	/* ot is our neighbor, with split point 0,0,3 
	   pr is our current, with split point 3,0,0  */
	op = QuadraticInterpolant(s, t, (VertexPath(s,t,"1")));
	PatchSplit(op, &or, &os, &ot, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	GetBaryCoords(PatchGetPoint(ot, 0,0,3), 
		      PatchGetPoint(ot, 3,0,0), 
		      PatchGetPoint(ot, 0,3,0), 
		      PatchGetPoint(*pr, 3,0,0), rst);
	uh = rst[0]; vh = rst[1]; wh = rst[2];
	GetBaryCoords(PatchGetPoint(*pr, 3,0,0), 
		      PatchGetPoint(*pr, 0,3,0), 
		      PatchGetPoint(*pr, 0,0,3),
		      PatchGetPoint(ot, 0,0,3), rst);
	u = rst[0]; v = rst[1]; w = rst[2];
	PCoords(PatchGetPoint(*pr, 0,0,3), std_frame, &px, &py, &pz);
	/*b300*/a1 = pz;
	PCoords (PatchGetPoint(*pr, 0,3,0), std_frame, &px, &py, &pz);
	/*b030*/a4 = pz;
	PCoords (PatchGetPoint(*pr, 0,1,2), std_frame, &px, &py, &pz);
	/*b210*/a2 = pz;
	PCoords (PatchGetPoint(*pr, 0,2,1), std_frame, &px, &py, &pz);
	/*b120*/a3 = pz;
	PCoords (PatchGetPoint(*pr, 1,0,2), std_frame, &px, &py, &pz);
	/*b201*/b1 = pz;
	PCoords (PatchGetPoint(*pr, 2,0,1), std_frame, &px, &py, &pz);
	/*b102*/d1 = pz;
	PCoords (PatchGetPoint(*pr, 1,2,0), std_frame, &px, &py, &pz);
	/*b021*/b2 = pz;
	PCoords (PatchGetPoint(*pr, 2,1,0), std_frame, &px, &py, &pz);
	/*b012*/d2 = pz;
	PCoords (PatchGetPoint(ot, 1,0,2), std_frame, &px, &py, &pz);
	/*c102*/e1 = pz;
	PCoords (PatchGetPoint(ot, 0,1,2), std_frame, &px, &py, &pz);
	/*c012*/e2 = pz;
	
	PCoords (PatchGetPoint(ot, 2,0,1), std_frame, &px, &py, &pz);
	/*c201*/c1 = pz;
	PCoords (PatchGetPoint(ot, 0,2,1), std_frame, &px, &py, &pz);
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

	p1 = PPac3( ReturnUDPoint(r),ReturnUDPoint(s),ReturnUDPoint(t), 
		   1.0/9.0, 4.0/9.0, 4.0/9.0);
	PCoords(p1, std_frame, &px, &py, &pz);
	rcc = PCreate(std_frame, px, py, y);
	PatchFree(op);	PatchFree(or);	PatchFree(os);	PatchFree(ot);

	/* tr boundary */
	op = QuadraticInterpolant(t,r,(VertexPath(t,r,"1")));
	PatchSplit(op, &or, &os, &ot, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	GetBaryCoords(PatchGetPoint(ot, 0,0,3), 
		      PatchGetPoint(ot, 3,0,0), 
		      PatchGetPoint(ot, 0,3,0), 
		      PatchGetPoint(*ps, 0,3,0), rst);
	uh = rst[0]; vh = rst[1]; wh = rst[2];
	GetBaryCoords(PatchGetPoint(*ps, 0,3,0), 
		      PatchGetPoint(*ps, 0,0,3), 
		      PatchGetPoint(*ps, 3,0,0),
		      PatchGetPoint(ot, 0,0,3), rst);
	u = rst[0]; v = rst[1]; w = rst[2];
	PCoords(PatchGetPoint(*ps, 3,0,0), std_frame, &px, &py, &pz);
	/*b300*/a1 = pz;
	PCoords (PatchGetPoint(*ps,0,0,3), std_frame, &px, &py, &pz);
	/*b030*/a4 = pz;
	PCoords (PatchGetPoint(*ps,2,0,1), std_frame, &px, &py, &pz);
	/*b210*/a2 = pz;
	PCoords (PatchGetPoint(*ps,1,0,2), std_frame, &px, &py, &pz);
	/*b120*/a3 = pz;
	PCoords (PatchGetPoint(*ps,2,1,0), std_frame, &px, &py, &pz);
	/*b201*/b1 = pz;
	PCoords (PatchGetPoint(*ps,1,2,0), std_frame, &px, &py, &pz);
	/*b102*/d1 = pz;
	PCoords (PatchGetPoint(*ps,0,1,2), std_frame, &px, &py, &pz);
	/*b021*/b2 = pz;
	PCoords (PatchGetPoint(*ps,0,2,1), std_frame, &px, &py, &pz);
	/*b012*/d2 = pz;
	PCoords (PatchGetPoint(ot,1,0,2), std_frame, &px, &py, &pz);
	/*c102*/e1 = pz;
	PCoords (PatchGetPoint(ot,0,1,2), std_frame, &px, &py, &pz);
	/*c012*/e2 = pz;
	
	PCoords (PatchGetPoint(ot,2,0,1), std_frame, &px, &py, &pz);
	/*c201*/c1 = pz;
	PCoords (PatchGetPoint(ot,0,2,1), std_frame, &px, &py, &pz);
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

	p1 = PPac3( ReturnUDPoint(r),ReturnUDPoint(s),ReturnUDPoint(t), 
		   4.0/9.0, 1.0/9.0, 4.0/9.0);
	PCoords(p1, std_frame, &px, &py, &pz);
	scc = PCreate(std_frame, px, py, y);
	PatchFree(or);	PatchFree(os);	PatchFree(ot);	PatchFree(op);

	/* rs boundary */
	op = QuadraticInterpolant(r,s,(VertexPath(r,s,"1")));
	PatchSplit(op, &or, &os, &ot, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	GetBaryCoords(PatchGetPoint(ot, 0,0,3), 
		      PatchGetPoint(ot, 3,0,0), 
		      PatchGetPoint(ot, 0,3,0), 
		      PatchGetPoint(*pt, 0,0,3), rst);
	uh = rst[0]; vh = rst[1]; wh = rst[2];
	GetBaryCoords(PatchGetPoint(*pt, 0,0,3), 
		      PatchGetPoint(*pt, 3,0,0), 
		      PatchGetPoint(*pt, 0,3,0),
		      PatchGetPoint(ot, 0,0,3), rst);
	u = rst[0]; v = rst[1]; w = rst[2];
	PCoords(PatchGetPoint(*pt, 0,3,0), std_frame, &px, &py, &pz);
	/*b300*/a1 = pz;
	PCoords (PatchGetPoint(*pt, 3,0,0), std_frame, &px, &py, &pz);
	/*b030*/a4 = pz;
	PCoords (PatchGetPoint(*pt, 1,2,0), std_frame, &px, &py, &pz);
	/*b210*/a2 = pz;
	PCoords (PatchGetPoint(*pt, 2,1,0), std_frame, &px, &py, &pz);
	/*b120*/a3 = pz;
	PCoords (PatchGetPoint(*pt, 0,2,1), std_frame, &px, &py, &pz);
	/*b201*/b1 = pz;
	PCoords (PatchGetPoint(*pt, 0,1,2), std_frame, &px, &py, &pz);
	/*b102*/d1 = pz;
	PCoords (PatchGetPoint(*pt, 2,0,1), std_frame, &px, &py, &pz);
	/*b021*/b2 = pz;
	PCoords (PatchGetPoint(*pt, 1,0,2), std_frame, &px, &py, &pz);
	/*b012*/d2 = pz;
	PCoords (PatchGetPoint(ot, 1,0,2), std_frame, &px, &py, &pz);
	/*c102*/e1 = pz;
	PCoords (PatchGetPoint(ot, 0,1,2), std_frame, &px, &py, &pz);
	/*c012*/e2 = pz;
	
	PCoords (PatchGetPoint(ot, 2,0,1), std_frame, &px, &py, &pz);
	/*c201*/c1 = pz;
	PCoords (PatchGetPoint(ot, 0,2,1), std_frame, &px, &py, &pz);
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

	p1 = PPac3( ReturnUDPoint(r),ReturnUDPoint(s),ReturnUDPoint(t), 
		   4.0/9.0, 4.0/9.0, 1.0/9.0);
	PCoords(p1, std_frame, &px, &py, &pz);
	tcc = PCreate(std_frame, px, py, y);
	PatchFree(or);	PatchFree(os);	PatchFree(ot);	PatchFree(op);

	/* set the cross boundary points */
	PatchSetPoint(pr,rcc,1,1,1);
	PatchSetPoint(ps,scc,1,1,1);
	PatchSetPoint(pt,tcc,1,1,1);

	/* set the internal boundary point */
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
	
	/* set the split point */
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

    i = 0;
    ForeachFaceVertex(f, v) {
	    if ( i >= 3 ) {
		    fprintf(stderr,"Farin: ProcessFace(%s): too many vertices\n",ReturnName(f));
		    exit(1);
	    }
	    vr[i++] = v;
    } EndForeach;

    patch = QuadraticInterpolant(vr[0], vr[1], vr[2]);

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
                fprintf(stderr,"Farin: normals not given.  Exiting.\n");                exit(1);
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
                fprintf(stderr,"Farin: finished.  ");
                fprintf(stderr,"%d neighborhoods processed in %d cpu seconds.\n",
                        num, u.ru_utime);
        }
        exit(0);
}


char* Banner = "Farin";
char* UsageString = "Farin [option] < mesh";
Option Options[] = {
/* Name,        Routine,        #args,  Help String */
   "h",         Usage,          0,      ":      Print available options.",
};
