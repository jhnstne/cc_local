/*
 * Copyright (c) 1997, Computer Graphics Laboratory, University of Waterloo
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  CloughTocher.c
 *----------------------------------------------------------------------
 */

/*
** Authors: Tony DeRose and Steve Mann
** Last Modified: May 30, 1997
** Purpose: An implementation of Clough-Toucher for
**          constructing C1 cubic surface.
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
static Space World;



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
	Scalar x,y,z;
	
	r = vr[0];
	s = vr[1];
	t = vr[2];
	
	vtx1 = s;
	vtx2 = t;
	p1 = EdgeTangent( vtx1, ReturnUDPoint(VertexPath(vtx1,vtx2,"1")) );
	p2 = EdgeTangent( vtx2, ReturnUDPoint(VertexPath(vtx1,vtx2,"1")) );
	ptA = PPac(p1,p2,1.0/2.0);
	ptB = PPac( PatchGetPoint(patch,1,2,0), 
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
	
	vtx1 = t;
	vtx2 = r;
	p1 = EdgeTangent( vtx1, ReturnUDPoint(VertexPath(vtx1,vtx2,"1")) );
	p2 = EdgeTangent( vtx2, ReturnUDPoint(VertexPath(vtx1,vtx2,"1")) );
	ptA = PPac(p1,p2,1.0/2.0);
	ptB = PPac( PatchGetPoint(patch,2,1,0), 
		   PatchGetPoint(patch,0,1,2), 1.0/2.0);
	v1 = PPDiff(ptA, ptB);
	v2 = PPDiff( PatchGetPoint(patch,2,0,1),
		    PatchGetPoint(patch,1,0,2));
	n = VDual(VVCross(v1,v2));
	p = PatchGetPoint(patch,2,0,1);
	p1 = PPac3( ReturnUDPoint(r),ReturnUDPoint(s),ReturnUDPoint(t), 
		   4.0/9, 1.0/9.0, 4.0/9.0);
	v1 = FV( StdFrame(SpaceOf(p1)), 2 );
	scc = LineIntersectPlane(p1,v1,p,n);
	
	vtx1 = r;
	vtx2 = s;
	p1 = EdgeTangent( vtx1, ReturnUDPoint(VertexPath(vtx1,vtx2,"1")) );
	p2 = EdgeTangent( vtx2, ReturnUDPoint(VertexPath(vtx1,vtx2,"1")) );
	ptA = PPac(p1,p2,1.0/2.0);
	ptB = PPac( PatchGetPoint(patch,0,2,1), 
		   PatchGetPoint(patch,2,0,1), 1.0/2.0);
	v1 = PPDiff(ptA, ptB);
	v2 = PPDiff( PatchGetPoint(patch,1,2,0),
		    PatchGetPoint(patch,2,1,0));
	n = VDual(VVCross(v1,v2));
	p = PatchGetPoint(patch,1,2,0);
	p1 = PPac3( ReturnUDPoint(r),ReturnUDPoint(s),ReturnUDPoint(t), 
		   4.0/9.0, 4.0/9, 1.0/9.0);
	v1 = FV( StdFrame(SpaceOf(p1)), 2 );
	tcc = LineIntersectPlane(p1,v1,p,n);
	
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
		    fprintf(stderr,"CloughTocher: ProcessFace(%s): too many vertices\n",ReturnName(f));
		    exit(1);
	    }
	    vr[i++] = v;
    } EndForeach;

    /* Set the corner points of the net to the data points */
    PatchSetPoint( &patch, ReturnUDPoint(vr[0]), 3, 0, 0); 
    P[6]=ReturnUDPoint(vr[0]);
    PatchSetPoint( &patch, ReturnUDPoint(vr[1]), 0, 3, 0); 
    P[7]=ReturnUDPoint(vr[1]);
    PatchSetPoint( &patch, ReturnUDPoint(vr[2]), 0, 0, 3); 
    P[8]=ReturnUDPoint(vr[2]);

    /* Set the boundary points based on tangent planes */
    /* at vertices using the method described in  */
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
    /* This is the quadratic precision point of Farin */
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
                fprintf(stderr,"CloughTocher: normals not given.  Exiting.\n");                exit(1);
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
                fprintf(stderr,"ShirmanSequin: finished.  ");
                fprintf(stderr,"%d neighborhoods processed in %d cpu seconds.\n",
                        num, u.ru_utime);
        }
        exit(0);
}



char* Banner = "CloughTocher";
char* UsageString = "CloughTocher [option] < mesh";
Option Options[] = {
/* Name,        Routine,        #args,  Help String */
   "h",         Usage,          0,      ":      Print available options.",
};
