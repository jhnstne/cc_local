/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Steve Mann
** 
** Purpose: An implementation of the surface interpolation
**	described in the paper by Shirman-Sequin.
**
*/
#include <stdio.h>
#include <math.h>
#include "all.h"
#include "../../lib/KCurve/curve.h"
#include <sys/time.h>
#include <sys/resource.h>

/* Forward declarations */
void ProcessFace(Face*);
Point EdgeTangent(Vertex *v1, Vertex *v2);
void TweakBoundary(Point p0, Normal n0, Point p1, Normal n1, 
		   Point* r0, Point* r1);
void AdjustPatch(Patch patch, Patch *pat1, Patch *pat2, Patch *pat3,
		 Vertex* r, Vertex* s, Vertex* t);

#define LOOP 1
#define CENTROID 2
#define CROSS 3
#define SLOAN 4
#define BISECT 5
#define PERP 6

/* Global variables */
int normalType=LOOP;
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
int foley=0;
int samples=0;

static char *RoutineName;
static Space World;                 /* Space where faces and patches reside */
static Scalar ShapeParameter = 0.25;/* Controls magnitude of edge tangents  */







/* centroid of cubic points */
double alpha1 = -.25;
double alpha2 = 2.25;
double alpha3 = -.75;
double alpha4 = 2.75;

/* centroid of quartic points 
double alpha1 = -1.0;
double alpha2 =  3.0;
double alpha3 = -.75;
double alpha4 = 2.75;
*/

double alpha;


main(int argc, char **argv)
{
	Mesh* m;
	Face* f;
	int num;
	
	/* Interpret command line options */
	ParseCommandLine( argc, argv);
	alpha = 1.0 - alpha1 -alpha2;
	
	/* Create the world space */
	World = SCreate( "World", 3);
	
	m = MeshParse(stdin);
	AddGeometry(World,m);
	
	/* If no normals, complain and quit. */
	if ( !(ReturnUDGeoFlags(m) & G_NORMAL) ) {
		fprintf(stderr,"ShirmanSequin: normals not given.  Exiting.\n");
		exit(1);
	} 
	
	/* Main Loop */
	num=0;
	ForeachMeshFace(m,f){
	
		if ( ! BoundaryFace(f) || 1 ) {
			if ( verbose  &&  f->name != NULL ) 
			  fprintf(stderr,"process face %s\n",f->name);
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

extern int OutputBoundary(Vertex* v0, Point p1, Point p2, Vertex* v3);

void ProcessFace(Face* face)
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

	if ( foley ) {
		void ConstructBoundary (Vertex* vr[3], Patch* patch);
		ConstructBoundary(vr, &patch);
		P[6] = PatchGetPoint(patch, 3,0,0);
		P[7] = PatchGetPoint(patch, 0, 3, 0);
		P[8] = PatchGetPoint(patch, 0, 0, 3);
		P[0] = PatchGetPoint(patch, 2, 1, 0);
		P[1] = PatchGetPoint(patch, 1, 2, 0);
		P[2] = PatchGetPoint(patch, 0, 2, 1);
		P[3] = PatchGetPoint(patch, 0, 1, 2);
		P[4] = PatchGetPoint(patch, 1, 0, 2);
		P[5] = PatchGetPoint(patch, 2, 0, 1);
	} else {
		/* Set the corner points of the net to the vertices 
		   of the face */
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
	}

	/* Set the middle point to the centroid of the edge points */
	w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
	w[6] = w[7] = w[8] = -(1.0/6.0);
	PatchSetPoint( &patch, PPacN( 9, P, w), 1, 1, 1);

	AdjustPatch(patch,&pr,&ps,&pt,r,s,t);

	dCopyDstructFields(face->externalData,"Face",pr.ln,"BezierTriangle");
	dCopyDstructFields(face->externalData,"Face",ps.ln,"BezierTriangle");
	dCopyDstructFields(face->externalData,"Face",pt.ln,"BezierTriangle");
	if ( samples == 0 ) {
	    PatchWrite(stdout,pr);
	    PatchWrite(stdout,ps);
	    PatchWrite(stdout,pt);
	} else {
	    Point EvalPatch(void*, Scalar u[3]);
	    TessPatch(&pr,EvalPatch,samples,StdFrame(World));
	    TessPatch(&ps,EvalPatch,samples,StdFrame(World));
	    TessPatch(&pt,EvalPatch,samples,StdFrame(World));
	}
	if (s3dBoundaries) {
		if (OutputBoundary(r, P[0], P[1], s)==0) {
			s3dBoundaries = 0;
		} else {
			OutputBoundary(s, P[2], P[3], t);
			OutputBoundary(t, P[4], P[5], r);
		}
	}

	PatchFree(patch);
	PatchFree(pr);
	PatchFree(ps);
	PatchFree(pt);
}

Point EvalPatch(void* P, Scalar u[3])
{
	return PatchEval(*((Patch*)P), u[0], u[1], u[2]);
}


/*
 *                                                                      
 *                     s=T2=030
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *      r=T1=300                     t=T3=003
 *                                                                      
 */
void AdjustPatch(Patch patch, Patch *pat1, Patch *pat2, Patch *pat3,
		 Vertex* r, Vertex* s, Vertex* t)
{
	Point T1,T2,T3;
	Point B1,B2,B3;
	Point C1,C2,C3;
	Point D1,D2,D3;
	Point S1,S2,S3;
	Point S1a, S2a, S3a, S1b, S2b, S3b;
	Point L12,L13;
	Point M13,M23;
	Point K23,K12;
	Point N12,N13,N23;
	Point P1,P2,P3;
	Point Z;
	int n;
	Point p[10];
	Scalar sc[10];
	void ComputeCrossBoundary();
	Patch tempPatch;
	
	/* extract the known points from the cubic patch */
	T1 = PatchGetPoint(patch,3,0,0); /* == r */
	T2 = PatchGetPoint(patch,0,3,0); /* == s */
	T3 = PatchGetPoint(patch,0,0,3); /* == t */
	B1 = PatchGetPoint(patch,2,0,1);
	B2 = PatchGetPoint(patch,1,0,2);
	C1 = PatchGetPoint(patch,2,1,0);
	C2 = PatchGetPoint(patch,1,2,0);
	D1 = PatchGetPoint(patch,0,1,2);
	D2 = PatchGetPoint(patch,0,2,1);
	
	/* NOTE: The .25's come in because we need to convert B_i and C_i from
	   cubic to quartic control points */
	/* compute S's */
	S1 = PPac3( T1,             PPac(T1,B1,.25), PPac(T1,C1,.25), 
		   -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	S2 = PPac3( T2, PPac(T2,C2,.25), PPac(T2,D2,.25),
		   -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	S3 = PPac3( T3, PPac(T3,D1,.25), PPac(T3,B2,.25),
		   -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	
	/* BLEAH!  This will be a real mess! */
	S1a = PPac3(T1, 
		    PPac(T1, EdgeTangent(r,VertexPath(s,r,"-1")),.25),
		    PPac(T1,C1,.25), 
		    -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	S2a = PPac3(T2,
		    PPac(T2, EdgeTangent(s,VertexPath(t,s,"-1")),.25),
		    PPac(T2,D2,.25),
		    -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	S3a = PPac3( T3, 
		    PPac(T3, EdgeTangent(t,VertexPath(r,t,"-1")),.25),
		    PPac(T3,B2,.25),
		    -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	
	S1b = PPac3(T1,             
		    PPac(T1,B1,.25), 
		    PPac(T1,EdgeTangent(r,VertexPath(t,r,"1")),.25), 
		    -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	S2b = PPac3(T2, 
		    PPac(T2,C2,.25), 
		    PPac(T2,EdgeTangent(s,VertexPath(r,s,"1")),.25),
		    -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	S3b = PPac3(T3,
		    PPac(T3,D1,.25),
		    PPac(T3,EdgeTangent(t,VertexPath(s,t,"1")),.25),
		    -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
	
	
	if ( foley ) {
		Point p;
		BOOLEAN GetInterior (Vertex* v0, Vertex* v1, 
				     Vertex* v2, Point* p);

		if ( !GetInterior(r,s,t,&p) ) {
			fprintf(stderr,"Help!  I've been shot!\n");
		}
		PatchSetPoint(&patch, p, 1,1,1);
		PatchSplit(patch, pat1, pat2, pat3, 1.0/3.0, 1.0/3.0, 1.0/3.0);
		tempPatch = PatchDegreeRaise(*pat3);
		L12 = PatchGetPoint(tempPatch, 2,1,1);
		K12 = PatchGetPoint(tempPatch, 1,2,1);

		if ( !GetInterior(t,r,s,&p) ) {
			fprintf(stderr,"Help!  I've been shot!\n");
		}
		PatchSetPoint(&patch, p, 1,1,1);
		PatchSplit(patch, pat1, pat2, pat3, 1.0/3.0, 1.0/3.0, 1.0/3.0);
		tempPatch = PatchDegreeRaise(*pat2);
		L13 = PatchGetPoint(tempPatch, 2,1,1);
		M13 = PatchGetPoint(tempPatch, 1,1,2);

		if ( !GetInterior(s,t,r,&p) ) {
			fprintf(stderr,"Help!  I've been shot!\n");
		}
		PatchSetPoint(&patch, p, 1,1,1);
		PatchSplit(patch, pat1, pat2, pat3, 1.0/3.0, 1.0/3.0, 1.0/3.0);
		tempPatch = PatchDegreeRaise(*pat1);
		K23 = PatchGetPoint(tempPatch, 1,2,1);
		M23 = PatchGetPoint(tempPatch, 2,1,1);

	} else {
		/* compute K's, L's, and M's */
		/* rs boundary */
		ComputeCrossBoundary(r,C1,PPac(T1,S1,.25), 
				     PPac(T2,S2,.25),C2,s, &L12,&K12,
				     PPac(T1,S1a,.25), 
				     PPac(T2,S2b,.25),
				     VertexPath(r,s,"-1"),jensen);
		/* rt boundary */
		ComputeCrossBoundary(r,B1,PPac(T1,S1,.25),
				     PPac(T3,S3,.25),B2,t,&L13,&M13,
				     PPac(T1,S1b,.25),
				     PPac(T3,S3a,.25),
				     VertexPath(r,t,"1"),jensen);
		/* st boundary */
		ComputeCrossBoundary(s,D2,PPac(T2,S2,.25),
				     PPac(T3,S3,.25),D1,t,&K23,&M23,
				     PPac(T2,S2a,.25),
				     PPac(T3,S3b,.25),
				     VertexPath(s,t,"-1"),jensen);
	
	}	
	
	
	/* compute P's */
	sc[0] = 1.0/6.0; 
	sc[1] = -1.0/6.0; 
	sc[2] = 3.0/6.0; 
	sc[3] = 3.0/6.0;
	sc[0] = -alpha3/(2.0*alpha2);    
	sc[1] = -(alpha1/alpha2 +alpha4/(2.0*alpha2));
	sc[2] = 3.0/(2.0*alpha2);        
	sc[3] = 3.0/(2.0*alpha2);
	p[0] = T1;      p[1] = S1;       p[2] = L13;     p[3] = L12;
	
	P1 = PPacN(4, p, sc);
	
	p[0] = T2;      p[1] = S2;       p[2] = K23;     p[3] = K12;
	P2 = PPacN(4, p, sc);
	
	p[0] = T3;      p[1] = S3;       p[2] = M23;     p[3] = M13;
	P3 = PPacN(4, p, sc);
	
	
	/* Compute N's */
	sc[0] =       sc[1] = alpha3/3.0;    sc[2] = -alpha3/3.0;
	sc[3] = sc[4] = alpha2/18.0+(alpha1+2.0*alpha4)/6.0;
	sc[5] =        alpha2/18.0-(alpha1+2.0*alpha4)/6.0;
	
	p[0] = S1;       p[1] = S2;       p[2] = S3;      
	p[3] = P1;       p[4] = P2;       p[5] = P3;
	N12 = PPacN(6, p, sc);
	
	p[0] = S3;       p[1] = S1;       p[2] = S2;
	p[3] = P3;       p[4] = P1;       p[5] = P2;
	N13 = PPacN(6, p, sc);
	
	p[0] = S2;       p[1] = S3;       p[2] = S1;
	p[3] = P2;       p[4] = P3;       p[5] = P1;
	N23 = PPacN(6, p, sc);
	
	
	Z = PPac3( P1, P2, P3, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	
	
	/* Contruct Three New Patches */
	PatchSplit(patch, pat1, pat2, pat3, 1.0/3.0, 1.0/3.0, 1.0/3.0);
	
	PatchSetPoint( pat1, S3, 1, 0, 2 );
	PatchSetPoint( pat1, P3, 2, 0, 1 );
	PatchSetPoint( pat1, Z,  3, 0, 0 );
	PatchSetPoint( pat1, P2, 2, 1, 0 );
	PatchSetPoint( pat1, S2, 1, 2, 0 );
	tempPatch = PatchDegreeRaise(*pat1); PatchFree(*pat1); *pat1 = tempPatch;
	PatchSetPoint( pat1, K23, 1, 2, 1 );
	PatchSetPoint( pat1, N23, 2, 1, 1 );
	PatchSetPoint( pat1, M23, 1, 1, 2 );
	
	PatchSetPoint( pat2, S1, 2, 1, 0 );
	PatchSetPoint( pat2, P1, 1, 2, 0 );
	PatchSetPoint( pat2, Z,  0, 3, 0 );
	PatchSetPoint( pat2, P3, 0, 2, 1 );
	PatchSetPoint( pat2, S3, 0, 1, 2 );
	tempPatch = PatchDegreeRaise(*pat2); PatchFree(*pat2); *pat2 = tempPatch;
	PatchSetPoint( pat2, L13, 2, 1, 1 );
	PatchSetPoint( pat2, N13, 1, 2, 1 );
	PatchSetPoint( pat2, M13, 1, 1, 2 );
	
	PatchSetPoint( pat3, S2, 0, 2, 1 );
	PatchSetPoint( pat3, P2, 0, 1, 2 );
	PatchSetPoint( pat3, Z,  0, 0, 3 );
	PatchSetPoint( pat3, P1, 1, 0, 2 );
	PatchSetPoint( pat3, S1, 2, 0, 1 );
	tempPatch = PatchDegreeRaise(*pat3); PatchFree(*pat3); *pat3 = tempPatch;
	PatchSetPoint( pat3, L12, 2, 1, 1 );
	PatchSetPoint( pat3, N12, 1, 1, 2 );
	PatchSetPoint( pat3, K12, 1, 2, 1 );
	
}





int newBoundary=0;

/*
 *  Function:  EdgeTangent
 *	Compute a point in the plane through the hyperplane described by 
 *  vertex.
 */
Point EdgeTangent(Vertex *v1, Vertex *v2)
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
	if ( foley ) {
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
			p = PCreate (StdFrame (World), px, py, MAXFLOAT);
		} /* if  */
		return p;
	}
	if ( tweakBoundary ) {
		if ( first ) {
			fprintf(stderr,"Using tweak boundaries\n");
			first=0;
		}
		TweakBoundary(ReturnUDPoint(v1), 
			      ReturnUDNormal(v1),
			      ReturnUDPoint(v2), 
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
 *----------------------------------------------------------------------
 *  Function:  TweakBoundary
 *----------------------------------------------------------------------
 */
void TweakBoundary(Point p0, Normal n0, Point p1, Normal n1, 
		   Point* r0, Point* r1)
{
	Vector	u1, u2, u3;
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







    


char* Banner = "ShirmanSequin";
char* UsageString = "ShirmanSequin [option] < mesh ";

void SetAlpha(char* a[])
{
  alpha1 = atof(a[0]);
  alpha2 = atof(a[1]);
  if ( alpha1 + alpha2 != alpha3 + alpha4 ) {
    fprintf(stderr,"Bad alpha values.  Exiting.\n");
    exit(1);
  }
}

void SetFour()
{
	fourNinths = 1;
}

extern void Usage();

void Verbose()
{
	verbose = 1;
}


void New()
{
	new = 1;
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


void SetTweakBoundary(char* a[])
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


void SetJensen(char* argv[])
{
  jensen = atof(argv[0]);
}


void SetS3dBoundaries()
{
	s3dBoundaries=1;
}

void SetFoley()
{
	foley = 1;
}

void SetS(char* argv[])
{
  samples = atoi(argv[0]);
}

Option Options[] = {
/* Name, 	Routine,	#args,	Help String */
   "h",		Usage,		0,	": 	Print available options.",
   "a",		SetAlpha,	2,	"alpha1 alpha2: Set alpha values.\n\
		alpha1+alpha2 must equal 2.0.",
   "4",		SetFour,	0,	":	Set edge tangents to be 4/9.",
   "v",		Verbose,	0,	":	Verbose mode.",
   "n",		New,		0,	":	New method of continuity cond.",
   "nb",	NewBoundary,    0,      ":	New method of computing boundary curves.",
   "fb",	SetFurthBoundary,0,     ":	Use Furth Boundaries [default].",
   "tb",	SetTweakBoundary,  1,	"tweak:	Use 'tweak' frac of dist from\
 vertices for boundary.",
   "dbhs",	SetGoodBoundary, 0,	":	Use deBoor, Hollig, Sabin boundaries.",
   "kolb",	SetKolbBoundary, 0,	":	Use Kolb boundaries.",
   "q",		SetQuad,	0,	":	Use quadratic boundaries when possible.",
   "j",		SetJensen,	1,	"c:	Use Jensen crossboundaries.",
   "sb",	SetS3dBoundaries,0,	":	Output s3d boundaries in ss.boundaries.s3d.",
   "foley",	SetFoley,	0,	":	Use Foley's functional construction",
   "s",		SetS,		1,	"s:	Tesselate with numerical normals",

   NULL,	NULL,		0,	NULL
   };


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
BOOLEAN GetInterior (Vertex* v0, Vertex* v1, Vertex* v2, Point* p)
{
    Face* face;
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
		fprintf(stderr,"ShirmanSequin -- GetInterior: You goofed!\n");
		exit(1);
	}

	/* neighbour doesn't exist! */
	if (NULL == nface) return FALSE;

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
    } 

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

    { /* the else clause in Foley.c */
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
    }

    /* give *p x,y coordinates in center of triangle */
    *p = PPac3( PatchGetPoint (patch, 3, 0, 0),
		PatchGetPoint (patch, 0, 3, 0),
		PatchGetPoint (patch, 0, 0, 3),
		1.0/3.0, 1.0/3.0, 1.0/3.0);
    PCoords (*p, std_frame, &b111x, &b111y, &dummy);

    *p = PCreate (std_frame, b111x, b111y, b111z);

    return TRUE;
} /* GetInterior  */
