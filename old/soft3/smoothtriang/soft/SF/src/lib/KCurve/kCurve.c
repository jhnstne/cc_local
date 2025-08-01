/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include <stdio.h>
#include "all.h"
#include "curve.h"
#include <math.h>

Space w;
Frame WF;

typedef Curve BezierCurve;

typedef struct {
	Point p;
	Vector v;
	double k; /* curvature */
} Kdata;


/*
 *----------------------------------------------------------------------
 *  Function:  ReadData
 *----------------------------------------------------------------------
 */
void ReadData(kd)
Kdata *kd;
{
	Scalar x,y;
	Scalar dx,dy;
	double curvature;
	
	if ( scanf(" %F %F  %F %F  %F",&x,&y,&dx,&dy,&curvature) != 5 ) {
		fprintf(stderr,"ReadData(): bad data.  Exiting.\n");
		exit(1);
	}
	if ( Dim(w) == 2 ){
		kd->p = PCreate(StdFrame(w),x,y);
		kd->v = VNormalize(VCreate(StdFrame(w),dx,dy));
	} else {
		kd->p = PCreate(StdFrame(w),x,y,0.0);
		kd->v = VNormalize(VCreate(StdFrame(w),dx,dy,0.0));
	}
	kd->k = curvature;
}



/*
 *----------------------------------------------------------------------
 *  Function:  ReadBezierCurve
 *----------------------------------------------------------------------
 */
void ReadBezierCurve(c)
BezierCurve* c;
{
	int n;
	int i;
	Scalar x,y;

	scanf(" %d",&n);
	if ( n >= MAX ) {
		fprintf(stderr,"Degree of Bezier curve too high.\n");
		fprintf(stderr," Must be no more than %d\n",MAX-1);
		exit(1);
	}

	c->degree = n;
	for(i=0; i<=n; i++){
		if ( scanf(" %lf %lf",&x,&y) != 2 ) {
			fprintf(stderr,"Error in reading bezier curve.\n");
			exit(1);
		}
		c->cp[i] = PCreate(WF, x, y);
	}
}

/*
 *----------------------------------------------------------------------
 *  Function:  Read3DBezier
 *----------------------------------------------------------------------
 */
void Read3dBezierCurve(c)
BezierCurve* c;
{
	int n;
	int i;
	Scalar x,y,z;
	Space s3;
	Frame wf3;
	Frame wf3p;
	AffineMap am;
	Vector v1,v2,v3;

	scanf(" %d",&n);
	if ( n >= MAX ) {
		fprintf(stderr,"Degree of Bezier curve too high.\n");
		fprintf(stderr," Must be no more than %d\n",MAX-1);
		exit(1);
	}

	s3 = SCreate("3Space",3);
	wf3 = StdFrame(s3);
	c->degree = n;
	for(i=0; i<=n; i++){
		if ( scanf(" %lf %lf %lf",&x,&y,&z) != 3 ) {
			fprintf(stderr,"Error in reading bezier curve.\n");
			exit(1);
		}
		c->cp[i] = PCreate(wf3, x, y, z);
	}

	v1 = VNormalize(PPDiff(c->cp[n],c->cp[0]));
	v2 = VNormalize(PPDiff(c->cp[1],c->cp[0]));
	if ( VVDot(v1, v2) < 1e-4 ) {
		fprintf(stderr,"Sorry, first, second, and last must be non-collinear\n");
		exit(1);
	}
	v3 = VNormalize(VVCross(v1,v2));
	v2 = VNormalize(VVCross(v3,v1));
	wf3p = FCreate("Local",c->cp[0],v1,v2,v3);
	am = ACreate( wf3p, FOrg(WF), FV(WF, 0), FV(WF, 1), VZero(w) );
	for(i=0; i<=n; i++){
		c->cp[i] = PAxform(c->cp[i], am);
	}
}




void PrintPoint(fp,p)
FILE* fp;
Point p;
{
	Scalar x,y,z;
	
	if ( Dim(SpaceOf(p)) == 2 ){
		PCoords(p,StdFrame(SpaceOf(p)),&x,&y);
		fprintf(fp,"%g %g ",x,y);
	} else {
		PCoords(p,StdFrame(SpaceOf(p)),&x,&y,&z);
		fprintf(fp,"%g %g %g ",x,y,z);
	}
}

void PrintVector(fp,v)
FILE* fp;
Vector v;
{
	Scalar x,y,z;
	
	if ( Dim(SpaceOf(v)) == 2 ) {
		VCoords(v,StdFrame(SpaceOf(v)),&x,&y);
		fprintf(fp,"%g %g ",x,y);
	} else {
		VCoords(v,StdFrame(SpaceOf(v)),&x,&y,&z);
		fprintf(fp,"%g %g %g ",x,y,z);
	}
}


void PrintKdata(fp,kd)
FILE* fp;
Kdata* kd;
{
	fprintf(stderr,"(");
	PrintPoint(stderr,kd->p);
	fprintf(stderr,") <");
	PrintVector(stderr,kd->v);
	fprintf(stderr,"> k = %f\n",kd->k);
}


void CheckRoots(p,r,nr)
double p[];
double r[];
int nr;
{
	int i,j;
	double sum;
	
	for(i=0;i<nr;i++){
		sum = 0.0;
		for(j=0;j<=4;j++){
			sum = r[i]*sum + p[j];
		}
		printf("root %d = %+12.12e yields %+12.12e\n",i,r[i],sum);
	}
}



void PrintCurve(c)
BezierCurve* c;
{
	Scalar Curvature();
	
	fprintf(stderr,"(");
	PrintPoint(stderr,c->cp[0]);
	fprintf(stderr,") (");
	PrintPoint(stderr,c->cp[1]);
	fprintf(stderr,") <");
	PrintVector(stderr,PPDiff(c->cp[1],c->cp[0]));
	if ( Dim(SpaceOf(c->cp[0])) == 2 ) {
		fprintf(stderr,"> k=%g\n",Curvature(c,0.0));
	} else
	  fprintf(stderr,">\n");
	
	fprintf(stderr,"(");
	PrintPoint(stderr,c->cp[2]);
	fprintf(stderr,") (");
	PrintPoint(stderr,c->cp[3]);
	fprintf(stderr,") <");
	PrintVector(stderr,PPDiff(c->cp[3],c->cp[2]));
	if ( Dim(SpaceOf(c->cp[0])) == 2 ) {
		fprintf(stderr,"> k=%g\n",Curvature(c,1.0));
	} else
	  fprintf(stderr,">\n");
}


main(argc,argv)
int argc;
char* argv[];
{
	Kdata v1,v2;
	BezierCurve c;
	int curveNumber;
	
	if ( argc == 2 ){
		w = SCreate("world",2);
	} else {
		w = SCreate("world",3);
	}
	WF = StdFrame(w);
	
	if ( atoi(argv[1]) == 0  ||  atoi(argv[1]) == 1 ) {
		ReadData(&v1);
		ReadData(&v2);
	}
	
	fprintf(stderr,"argc = %d\n",argc);
	if ( argc != 2 ) {
		SFF sff1,sff2;
		
		sff1.m[0][0] = sff1.m[1][1] = 
		  sff2.m[0][0] = sff2.m[1][1] = -1.0;
		sff1.m[0][1] = sff1.m[1][0] = 
		  sff2.m[0][1] = sff2.m[1][0] =  0.0;
		sff1.v0 = sff2.v0 = VCreate(StdFrame(SpaceOf(v1.p)),
					    0.0,0.0,1.0);
		sff1.v1 = VNormalize(VVCross(sff1.v0,v1.v));
		sff2.v1 = VNormalize(VVCross(sff2.v0,v2.v));
/* The following is the old calling semantics AND NO LONGER WORKS */
		fprintf(stderr,"NOT IMPLEMENTED.\n");
		exit(1);
		deBoorHolligSabin(v1.p,VDual(v1.v),v1.v,sff1,
				  v2.p,VDual(v2.v),v2.v,sff2,/*PLANE,*/&c);
		PrintCurve(&c);
	} else {
		PrintHeader();
		printf("0.001 setlinewidth\n\n");
		if ( atoi(argv[1]) == 0  ||  atoi(argv[1]) == 2  ||
		     atoi(argv[1]) == 3  ||  atoi(argv[1]) == 4 ) {
			SetType(0);
		} else {
			SetType(1);
		}
		if ( atoi(argv[1]) == 0 ) {
			while( CreateCurve(&v1,&v2,&c,curveNumber) > 
			      					curveNumber ){
				PrintCurve(&c);
				PlotControlPolygon(&c);
				PlotCurve(&c);
			
				printf("gsave\n");
				printf("2 %d translate\n",2*curveNumber);
#if 0
				PlotControlPolygon(&c);
				PlotCurve(&c);
#else
				PlotCurvatureAxis();
				PlotCurvature(&c);
#endif
				printf("grestore\n");
			
				curveNumber++;
			}
		} else if ( atoi(argv[1]) == 1  ||  atoi(argv[1]) == 2 ) {
			BezierCurve HPBezierCreate();
			char* C;

			if ( atoi(argv[1]) == 1 ) {
				C = (char*)HPCreate(v1.p,v1.v,v1.k,
						    v2.p,v2.v,v2.k);
				
				PlotCurve(C);
				
				printf("gsave\n");
				printf("2 %d translate\n",2*curveNumber);
				
				PlotCurvatureAxis();
				PlotCurvature(C);
			} else {
				c = HPBezierCreate(v1.p,v1.v,v1.k,
						   v2.p,v2.v,v2.k);
				PlotCurve(&c);
				PlotControlPolygon(&c);
				printf("gsave\n");
				printf("2 %d translate\n",2*curveNumber);
				
				PlotCurvatureAxis();
				PlotCurvature(&c);
			}
			printf("grestore\n");
		} else if ( atoi(argv[1]) == 3 ) {
			ReadBezierCurve(&c);
			PlotControlPolygon(&c);
			PlotCurve(&c);

			printf("2 %d translate\n",2*curveNumber);
			PlotCurvatureAxis();
			PlotCurvature(&c);
		} else if ( atoi(argv[1]) == 4 ) {
			Read3dBezierCurve(&c);
			PlotControlPolygon(&c);
			PlotCurve(&c);

			printf("2 %d translate\n",2*curveNumber);
			PlotCurvatureAxis();
			PlotCurvature(&c);
		} else {
			fprintf(stderr,"Unknown type %s\n",argv[1]);
			exit(1);
		}
		PrintTrailer();
		    
		if ( atoi(argv[1]) == 0 ) {
			CreateCurve(&v1,&v2,&c,-1);
			PrintCurve(&c);
		}
	}
}
