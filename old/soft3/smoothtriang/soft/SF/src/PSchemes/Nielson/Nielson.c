/*
 *----------------------------------------------------------------------
 *  Function:  Nielson.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>
#include <sys/resource.h>
#include "all.h"

static int verbose=0;
static int samples=1;
static Space World;
static int gk;
static int nielson=0;
static int scaledNielson=0;

void ProcessFace();
void Boundary();

main(argc, argv)
int argc;
char **argv;
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
		fprintf(stderr,"Nielson: normals not given.  Exiting.\n");
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
		fprintf(stderr,"Nielson: finished.  ");
		fprintf(stderr,"%d neighborhoods processed in %d cpu seconds.\n",
			num, u.ru_utime);
	}
	exit(0);
}

double ipow(x,n)
double x;
int n;
{
	double result=1.;
	int i;

	if ( n == 0 ) return 1.;

	for(i=0;i<n;i++){
		result *= x;
	}
	return x;
}

int maxn(w, n)
double w[];
int n;
{
	double m= -1;
	int mn;
	int i;

	for (i=0; i<n; i++) {
		if ( w[i] > m ) {
			m = w[i]; mn = i;
		}
	}
	return mn;
}

Point gp(p0, p1, n0, n1, t)
Point p0;
Point p1;
Normal n0;
Normal n1;
Scalar t;
{
	Point t0, t1;
	Point pa[4];
	Scalar w[4];
	int maxi;

	Boundary(p0, n0, p1, n1, &t0, &t1);
	pa[0] = p0; pa[1] = t0; pa[2] = t1; pa[3] = p1;
	w[0] = (1-t)*(1-t)*(1-t);
	w[1] = 3*t*(1-t)*(1-t);
	w[2] = 3*t*t*(1-t);
	w[3] = t*t*t;
	maxi = maxn(w, 4);
	w[maxi] += (1.-w[0]-w[1]-w[2]-w[3]);
	return PPacN(4, pa, w);
}

Vector gpd(p0, p1, n0, n1, t)
Point p0;
Point p1;
Normal n0;
Normal n1;
Scalar t;
{
	Point t0, t1;
	Vector va[3];
	Scalar w[3];
	int maxi;

	Boundary(p0, n0, p1, n1, &t0, &t1);
	va[0] = PPDiff(t0, p0); va[1] = PPDiff(t1, t0), va[2] = PPDiff(p1, t1);
	w[0] = (1-t)*(1-t);
	w[1] = 2*t*(1-t);
	w[2] = t*t;
	maxi = maxn(w, 3);
	w[maxi] += (1.-w[0]-w[1]-w[2]);
	return VVlcN(3, va, w);
}

Normal gn(p0, p1, n0, n1, t)
Point p0;
Point p1;
Normal n0;
Normal n1;
Scalar t;
{
	Vector vn0, vn1;
	Vector t0, t1;
	Vector vn;

	vn0 = NDual(n0); vn1 = NDual(n1);
	t0 = gpd(p0, p1, n0, n1, 0.);
	t1 = gpd(p0, p1, n0, n1, 1.);

	vn = VVCross(gpd(p0, p1, n0, n1, t),
		     VVAdd(SVMult((1-t), VVCross(vn0, t0)),
			   SVMult(   t , VVCross(vn1, t1))));
	vn = VNormalize(vn);
	return VDual(vn);
}

#define J (i+1)%3
#define K (i+2)%3

Point EvalNielson(vr, b)
Vertex* vr[3];
Scalar b[3];
{
	Point p[3];
	Point pb[3];
	Normal n[3];
	Scalar beta[3];
	int i;

	if ( b[0] == 1. ) {
		return ReturnUDPoint(vr[0]);
	} else if ( b[1] == 1. ) {
		return ReturnUDPoint(vr[1]);
	} else if ( b[2] == 1. ) {
		return ReturnUDPoint(vr[2]);
	}
	for (i=0; i<3; i++) {
		p[i] = ReturnUDPoint(vr[i]);
		n[i] = ReturnUDNormal(vr[i]);
		beta[i] = b[J]*b[K]/(b[i]*b[J] + b[J]*b[K] + b[K]*b[i]);
	}
	for (i=0; i<3; i++) {
		pb[i] = gp(p[i], gp(p[J],p[K],n[J],n[K],b[K]/(1-b[i])), 
			   n[i], gn(p[J],p[K],n[J],n[K],b[K]/(1-b[i])), 
			   1-b[i]);
	}
	return PPacN(3, pb, beta);
}


void ProcessFace( face )
Face* face;
{
	Vertex *v, *vr[3];
	int i;
	
	i = 0;
	ForeachFaceVertex(face,v){
		vr[i++] = v;
	} EndForeach;
	
	if ( gk ) {
		gkTessPatch(vr,EvalNielson,samples,StdFrame(World));
	} else {
		TessPatch(vr,EvalNielson,samples,StdFrame(World));
	}
}

void Boundary(p0, n0, p1, n1, r0, r1)
Point	p0, p1;
Normal	n0, n1;
Point *r0, *r1;
{
	if ( nielson ) {
		NielsonBoundary(p0, n0, p1, n1, r0, r1, scaledNielson);
		return;
	}
	PlanarBoundary(p0, n0, p1, n1, r0, r1);
}

char* Banner = "Nielson";
char* UsageString = "Nielson [option] < mesh ";


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

void SetGaussian()
{
  gk = 1;
}

void SetNielson()
{
	nielson = 1;
	SetTessEps(1e-5);
}

void SetScaledNielson()
{
	nielson = 1;
	scaledNielson = 1;
}

void SetEpsilon(arg)
char* arg[];
{
	SetTessEps(atof(arg[0]));
}


Option Options[] = {
/* Name, 	Routine,	#args,	Help String */
   "h",		Usage,		0,	": 	Print available options.",
   "v",		Verbose,	0,	":	Verbose mode.",
   "n",		SetNielson,	0,	":	Use unscaled Nielson curves.",
   "ns",	SetScaledNielson,0,	":	Use scaled Nielson curves.",
   "s",		SetSamples,	1,	"s:	Set the tesselation level.",
   "gk",	SetGaussian,	0,	":	Sample for Gaussian Curvature.",
   "e",		SetEpsilon,	1,	"eps:	Set epsilon.",

   NULL,	NULL,		0,	NULL
   };
