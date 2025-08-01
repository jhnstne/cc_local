/*
 * Copyright (c) 1994, Computer Graphics Labratory, University of Waterloo
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Steve Mann
** 
** Purpose: To test a simple C^0 cubic patch scheme.
**
*/
#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"
#include "patch.h"
#include "commandline.h"
#include "usage.h"

#include <sys/time.h>
#include <sys/resource.h>

/* Forward declarations */
void ProcessFace(Face* );
Point EdgeTangent(Vertex*, Vertex*);
void TweakBoundary(Point p0, Point p1, Normal n0, Normal n1, 
		   Point *r0, Point *r1);
void AdjustPatch();
Point LineIntersectPlane();

/* Global variables */
int fourNinths=0;
int verbose=0;
int new=0;
int furthBoundary=1;
int tweakBoundary=0;
int goodBoundary=0;
int quadBoundary=0;
int kolbBoundary=0;
int s3dBoundaries=1;
double tweak=0.5858;
Scalar jensen=1.;

static char *RoutineName;
static Space World;                 /* Space where faces and patches reside */
static Scalar ShapeParameter = 0.25;/* Controls magnitude of edge tangents  */


main(int argc, char **argv)
{
	Mesh* m;
	Face* f;
	int num;
	
	/* Interpret command line options */
	ParseCommandLine( argc, argv);
	
	/* Create the world space */
	World = SCreate( "World", 3);
	
	m = MeshParse(stdin);
	AddGeometry(World,m);
	
	/* If no normals, complain and quit. */
	if ( !(ReturnUDGeoFlags(m) & G_NORMAL) ) {
		fprintf(stderr,"Cubic: normals not given.  Exiting.\n");
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
		fprintf(stderr,"Cubic: finished.  ");
		fprintf(stderr,"%d neighborhoods processed in %d cpu seconds.\n",
			num, u.ru_utime);
	}
	exit(0);
}

extern int OutputBoundary(Vertex* v0, Point p1, Point p2, Vertex* v3);

void ProcessFace( face )
Face* face;
{
    Patch patch;
    Patch pr,ps,pt;
    Point P[9];
    Scalar w[9];
    static int normalFlag = 0;
    Vertex *v, *vr[3];
    int i;
    Vertex* r;
    Vertex* s;
    Vertex* t;

    i = 0;
    ForeachFaceVertex(face,v){
      vr[i++] = v;
    } EndForeach;

    r = vr[0];
    s = vr[1];
    t = vr[2];

    /* Create the cubic patch */
    patch = PatchCreate( 3, World, 0);

    /* Set the corner points of the net to the vertices of the face */
    PatchSetPoint( &patch, ReturnUDPoint(r), 3, 0, 0); 
    P[6]=ReturnUDPoint(r);
    PatchSetPoint( &patch, ReturnUDPoint(s), 0, 3, 0); 
    P[7]=ReturnUDPoint(s);
    PatchSetPoint( &patch, ReturnUDPoint(t), 0, 0, 3); 
    P[8]=ReturnUDPoint(t);

    /* Do the rs edge points */
    P[0]=EdgeTangent( r, s );
    P[1]=EdgeTangent( s, r );
    PatchSetPoint( &patch, P[0], 2, 1, 0);
    PatchSetPoint( &patch, P[1], 1, 2, 0);

    /* Do the st edge points */
    P[2]=EdgeTangent( s, t );
    P[3]=EdgeTangent( t, s );
    PatchSetPoint( &patch, P[2], 0, 2, 1);
    PatchSetPoint( &patch, P[3], 0, 1, 2);

    /* Do the tr edge points */
    P[4]=EdgeTangent( t, r );
    P[5]=EdgeTangent( r, t );
    PatchSetPoint( &patch, P[4], 1, 0, 2);
    PatchSetPoint( &patch, P[5], 2, 0, 1);

    /* Set the middle point to the centroid of the edge points */
    w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
    w[6] = w[7] = w[8] = -(1.0/6.0);
    PatchSetPoint( &patch, PPacN( 9, P, w), 1, 1, 1);

    PatchWrite(stdout,patch);

    PatchFree(patch);
}

int newBoundary=0;

/*
 *  Function:  EdgeTangent
 *	Compute a point in the plane through the hyperplane described by 
 *  vertex.
 */
Point EdgeTangent( v1, v2 )
Vertex *v1;
Vertex *v2;
{
	Point p1;
	Point p2;
	void FurthBoundary();
	static int first=1;
	
	if ( quadBoundary ) {
		if ( qBoundary(v1, v2, &p1, &p2) ) {
			return p1;
		}
	}
	if ( kolbBoundary ) {
		Vector d1, d2, pl;

		if ( first ) {
			"Using Kolb boundaries\n";
			first = 0;
		}
		p1 = ReturnUDPoint(v1);
		p2 = ReturnUDPoint(v2);

		ComputePlanarTangents(p1, ReturnUDNormal(v1),
				      p2, ReturnUDNormal(v2),
				      &d1, &d2, &pl);
		return PVAdd(p1, SVMult(CircArcLen(p1, d1, p2)/3, d1));
	}
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
		}
	}
	if ( tweakBoundary ) {
		if ( first ) {
			fprintf(stderr,"Using tweak boundaries\n");
			first=0;
		}
		TweakBoundary(ReturnUDPoint(v1), 
			      ReturnUDPoint(v2), 
			      ReturnUDNormal(v1),
			      ReturnUDNormal(v2),
			      &p1, &p2);
		return p1;
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
	} 
	
	if ( !fourNinths ) {
		p1 = PPac(ReturnUDPoint(v1), 
			  ReturnUDPoint(v2), 2.0/3.0);
	} else {
		p1 = PPac(ReturnUDPoint(v1), 
			  ReturnUDPoint(v2), 4.0/9.0);
	}
	p2 = LineIntersectPlane(p1,NDual(ReturnUDNormal(v1)), 
				ReturnUDPoint(v1),
				ReturnUDNormal(v1));
	
	if ( newBoundary ) {
		Vector v;
		if ( first ) {
			fprintf(stderr,"Using SS boundaries\n");
			first=0;
		}
		v = PPDiff(p2, ReturnUDPoint(v1));
		v = SVMult(VMag(PPDiff(ReturnUDPoint(v1),
				       ReturnUDPoint(v2)))/(3.0*VMag(v)), v);
		p2 = PVAdd(ReturnUDPoint(v1),v);
	}
	return p2;
}





/*
 *  Function:  FurthBoundary
 *	Sets tangents at either end of a single border curve, according to the
 *  notation on p. 9 of Furth
 */
void FurthBoundary(p0, n0, p1, n1, r0, r1)
Point	p0, p1;
Normal	n0, n1;
Point *r0, *r1;
{
	Vector	u1, u2, u3;
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
	
}



/*
 *----------------------------------------------------------------------
 *  Function:  TweakBoundary
 *----------------------------------------------------------------------
 */
void TweakBoundary(Point p0, Point p1, Normal n0, Normal n1, 
		   Point *r0, Point *r1)
{
  Vector u1, u2, u3;
  Vector v0, v1;
  Point pr;
  double nv,nv2;

  if ( (nv=VMag(VVCross(NDual(n0), NDual(n1)))) < 0.0000000001 ) {
/* not correct */fprintf(stderr,"TweakBoundary: assumed linear\n");
    *r0 = PPac(p0,p1,1.0/3.0);
    *r1 = PPac(p0,p1,2.0/3.0);
    return;
  }

  u1 = PPDiff(p1, p0);
  u3 = VVCross(u1, VVAdd(NDual(n0), NDual(n1)));

  /* make sure u1 not parallel to n0+n1 */
  if ( VMag( u3 ) < 0.000000001 ) {
    fprintf(stderr,"TweakBoundary: u1 and u3 parallel.  Using Furth.\n");
    FurthBoundary(p0, n0, p1, n1, r0, r1);
    return;
  }

  /* determine tangents at 0, 1 values along border curve i */
  /* pp0 stands for "p prime at value 0" */

  v0 = VVCross(NDual(n0), u3);
  v1 = VVCross(NDual(n1), u3);

  /* make sure v0 and v1 lie in tangent planes */
  /* SHOULD NEVER BE TRUE */
  if ( NVApply(n0,v0) > 0.0000000001  ||
       NVApply(n1,v1) > 0.0000000001 ) {
    fprintf(stderr,"TweakBoundary: Wierdness -- v0=v1 but don't lie in tangent planes!.  Calling FurthBoundary().\n");
    FurthBoundary(p0, n0, p1, n1, r0, r1);
  }

  /* special case: normals NOT parallel, but v0=v1 common to both */
  if ( (nv2=VMag(VVCross(v0, v1))) < 0.00000001 ) {
    Scalar x,y,z;
fprintf(stderr,"TweakBoundary: (2) assumed linear\n");
    NCoords(n0,StdFrame(SpaceOf(n0)),&x,&y,&z);
    fprintf(stderr,"n0 = (%g,%g,%g), ",x,y,z);

    NCoords(n1,StdFrame(SpaceOf(n1)),&x,&y,&z);
    fprintf(stderr,"n1 = (%g,%g,%g)\n",x,y,z);

    PCoords(p0,StdFrame(SpaceOf(p0)),&x,&y,&z);
    fprintf(stderr,"p0 = (%g,%g,%g), ",x,y,z);

    VCoords(v0,StdFrame(SpaceOf(v0)),&x,&y,&z);
    fprintf(stderr,"v0 = (%g,%g,%g), ",x,y,z);

    PCoords(p1,StdFrame(SpaceOf(p1)),&x,&y,&z);
    fprintf(stderr,"p1 = (%g,%g,%g), ",x,y,z);

    VCoords(v1,StdFrame(SpaceOf(v1)),&x,&y,&z);

    fprintf(stderr,"v1 = (%g,%g,%g).\n",x,y,z);

    *r0 = PPac(p0,p1,1.0/3.0);
    *r1 = PPac(p0,p1,2.0/3.0);

    return;
  }

  if ( !LineIntersectLine(p0,v0, p1,v1, &pr) ){
    Scalar x,y,z;

    fprintf(stderr,"TweakBoundary: non-planar lines! nv = %g,nv2=%g\n",nv,nv2);

    NCoords(n0,StdFrame(SpaceOf(n0)),&x,&y,&z);
    fprintf(stderr,"n0 = (%g,%g,%g), ",x,y,z);

    NCoords(n1,StdFrame(SpaceOf(n1)),&x,&y,&z);
    fprintf(stderr,"n1 = (%g,%g,%g)\n",x,y,z);

    PCoords(p0,StdFrame(SpaceOf(p0)),&x,&y,&z);
    fprintf(stderr,"p0 = (%g,%g,%g), ",x,y,z);

    VCoords(v0,StdFrame(SpaceOf(v0)),&x,&y,&z);
    fprintf(stderr,"v0 = (%g,%g,%g), ",x,y,z);

    PCoords(p1,StdFrame(SpaceOf(p1)),&x,&y,&z);
    fprintf(stderr,"p1 = (%g,%g,%g), ",x,y,z);

    VCoords(v1,StdFrame(SpaceOf(v1)),&x,&y,&z);
    fprintf(stderr,"v1 = (%g,%g,%g).\nCalling Furth boundaries\n",x,y,z);

    FurthBoundary(p0, n0, p1, n1, r0, r1);
    return;
  }

  *r0 = PPac(p0,pr,1.0-tweak);
  *r1 = PPac(p1,pr,1.0-tweak);

}







    


char* Banner = "Cubic";
char* UsageString = "Cubic [option] < mesh ";

void SetFour()
{
	fourNinths = 1;
}

extern void Usage();

void Verbose()
{
	verbose = 1;
}


void NewBoundary()
{
	newBoundary=1;
	furthBoundary=0;
}

void SetFurthBoundary()
{
	furthBoundary = 1;
}


void SetTweakBoundary(a)
char* a[];
{
	tweakBoundary=1;
	tweak = atof(a[0]);
fprintf(stderr,"Tweak = %f\n",tweak);
}


void SetGoodBoundary()
{
	goodBoundary=1;
	furthBoundary=0;
}


void SetKolbBoundary()
{
	kolbBoundary = 1;
	furthBoundary = 0;
}

void SetQuad()
{
	quadBoundary=1;
}


void SetS3dBoundaries()
{
	s3dBoundaries=1;
}



Option Options[] = {
/* Name, 	Routine,	#args,	Help String */
   "h",		Usage,		0,	": 	Print available options.",
   "4",		SetFour,	0,	":	Set edge tangents to be 4/9.",
   "v",		Verbose,	0,	":	Verbose mode.",
   "nb",	NewBoundary,    0,      ":	New method of computing boundary curves.",
   "fb",	SetFurthBoundary,0,     ":	Use Furth Boundaries [default].",
   "tb",	SetTweakBoundary,  1,	"tweak:	Use 'tweak' frac of dist from\
 vertices for boundary.",
   "dbhs",	SetGoodBoundary, 0,	":	Use deBoor, Hollig, Sabin boundaries.",
   "kolb",	SetKolbBoundary, 0,	":	Use Kolb boundaries.",
   "q",		SetQuad,	0,	":	Use quadratic boundaries when possible.",

   NULL,	NULL,		0,	NULL
   };
