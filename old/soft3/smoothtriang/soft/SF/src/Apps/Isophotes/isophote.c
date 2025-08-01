/*
 * Copyright (c) 1997, Computer Graphics Laboratory, University of Waterloo
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  isophote.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"

struct triangle {
	Point p[3];
	Normal n[3];
};

#define SQ(A) ((A)*(A))

/* STATIC VARIABLES */
static Vector dir;
static Scalar k[20]=
       /*   5      15     25     35     45     55     65     75     85 */
	{ .996,  .966,  .906,  .819,  .707,  .574,  .423,  .259,  .087,
	 -.996, -.966, -.906, -.819, -.707, -.574, -.423, -.259, -.087};

static int kn=18;
static Scalar offset=0.01;
static Space w;
static Frame wf;
static int color=0;

enum {S3D, PS};
static mode = S3D;
static Scalar psxmin =  1e15;
static Scalar psxmax = -1e15;
static Scalar psymin =  1e15;
static Scalar psymax = -1e15;
static Scalar PSScale = 1.0;


static void fPsFormat(FILE* fp, Scalar bx, Scalar by, Scalar bz, 
		       Scalar ex, Scalar ey, Scalar ez)
{
	fprintf(fp, "%lg %lg moveto\n", bx, by);
	fprintf(fp, "%lg %lg lineto\n", ex, ey);
	fprintf(fp, "stroke\n");
}

#define MIN(X,Y) ((X)<(Y)?(X):(Y))
#define MAX(X,Y) ((X)>(Y)?(X):(Y))

static void fWriteSeg(FILE* fp, Scalar bx, Scalar by, Scalar bz, 
		       Scalar ex, Scalar ey, Scalar ez)
{
	psxmin = MIN(psxmin,bx);
	psxmin = MIN(psxmin,ex);
	psymin = MIN(psymin,by);
	psymin = MIN(psymin,ey);

	psxmax = MAX(psxmax,bx);
	psxmax = MAX(psxmax,ex);
	psymax = MAX(psymax,by);
	psymax = MAX(psymax,ey);

	fPsFormat(fp, bx,by,bz, ex,ey,ez);
}

static void PSOutEdge(Point p, Point q)
{
	Scalar x1,y1,z1;
	Scalar x2,y2,z2;
	PCoords(p, wf, &x1, &y1, &z1);
	PCoords(q, wf, &x2, &y2, &z2);
	fWriteSeg(stdout, x1,y1,z1,  x2,y2,z2);
}

static void fHeader(FILE* fp)
{
	if ( mode == S3D ) {
		printf("d 1 1 1\n");
	} else if (mode == PS) {
		fprintf(fp, "%%!PS-Adobe-2.0 EPSF-2.0\n");
		fprintf(fp, "%%%%Orientation: Portrait\n");
		fprintf(fp, "%%%%BoundingBox: (atend)\n");
		fprintf(fp, "%%%%EndComments\n");
		fprintf(fp, "%%%%EndProlog\n");
		fprintf(fp, "%%%%Page: 1 1\n");
		fprintf(fp, "%g %g scale\n",PSScale,PSScale);
		fprintf(fp, "%g setlinewidth\n",1/PSScale);
	}
}

static void fTrailer(FILE* fp)
{
	if (mode == S3D) {
	} else if (mode == PS) {
		fprintf(fp, "showpage\n");
		fprintf(fp, "%%%%Trailer\n");
		fprintf(fp, "%%%%BoundingBox: %g %g %g %g\n",
			psxmin*PSScale,psymin*PSScale,
			psxmax*PSScale,psymax*PSScale);
	}
}

A3dColor(double k)
{
	static Material prev;
	static int first=1;
	Material cur;

	if ( !color ) return;
	cur = KMat(k, -1., 1., 0.625);
	if ( !first ) {
		if ( prev.diffuse.r == cur.diffuse.r  &&
		     prev.diffuse.g == cur.diffuse.g  &&
		     prev.diffuse.b == cur.diffuse.b ) {
			return;
		}
	}
	prev = cur;
	printf("d %g %g %g\n",cur.diffuse.r, cur.diffuse.g, cur.diffuse.b);
}


A3dOutEdge(Point p1, Point p2)
{
	Scalar x,y,z;

	fprintf(stdout, "L 2 0 0\n");
	PCoords(p1, wf, &x, &y, &z);
	fprintf(stdout,"v %g %g %g \n",x,y,z);
	PCoords(p2, wf, &x, &y, &z);
	fprintf(stdout,"v %g %g %g \n",x,y,z);
	fprintf(stdout, "E 0 0 0\n");
}


struct triangle MakeInteralTri()
{
	struct triangle t;

	if ( ! GetVertexPosition("Triangle.vertex1", wf, &(t.p[0]))  ||
	     ! GetVertexPosition("Triangle.vertex2", wf, &(t.p[1]))  ||
	     ! GetVertexPosition("Triangle.vertex3", wf, &(t.p[2]))  ||
	     ! GetVertexNormal("Triangle.vertex1", wf, &(t.n[0])) ||
	     ! GetVertexNormal("Triangle.vertex2", wf, &(t.n[1])) ||
	     ! GetVertexNormal("Triangle.vertex3", wf, &(t.n[2]))
	    ) {
		fprintf(stderr,"Bad triangle.  exiting.\n");	
		exit(1);
	}
	return t;
}

OutputIsophoteVertex(struct triangle t, Vector dir, Scalar k, Scalar s[3])
{
	int i;
	Scalar sc[2];
	Vector v[2];
	Scalar w;
	Point p;

	for ( i=0; i<3; i++ ) {
		if ( s[i] == k ) {
			break;
		}
	}
	if ( i == 3 ) {
		fprintf(stderr,"OutputIsophoteVertex: not an isophote vertex.\n");
		return;
	}
	w = fabs((s[(i+2)%3]-k)/(s[(i+2)%3]-s[(i+1)%3]));
	p = PPac(t.p[(i+1)%3], t.p[(i+2)%3], w);
	v[0] = NDual(t.n[(i+1)%3]);	v[1] = NDual(t.n[(i+2)%3]);
	sc[0] = w;  sc[1] = 1.-w;
	if (mode == S3D) {
		A3dColor(k);
		A3dOutEdge(PVAdd(t.p[i], SVMult(offset, NDual(t.n[i]))),
			   PVAdd(p, SVMult(offset,
					   VNormalize(VVlcN(2, v, sc)))));
	} else if ( mode == PS ) {
		PSOutEdge(t.p[i], p);
	}
}


void OutputIsophote(struct triangle t, Vector dir, Scalar k)
{
	int b[3];
	Vector v[4];
	Scalar s[3];
	Scalar sc[2];
	Point p[3];
	int i;

	v[0] = VNormalize(NDual(t.n[0]));
	v[1] = VNormalize(NDual(t.n[1]));
	v[2] = VNormalize(NDual(t.n[2]));
	v[3] = v[0];

	s[0] = VVDot(v[0], dir); b[0] = s[0] > k;
	s[1] = VVDot(v[1], dir); b[1] = s[1] > k;
	s[2] = VVDot(v[2], dir); b[2] = s[2] > k;

	if ( s[0] == k  ||  s[1] == k  ||  s[2] == k ) {
		OutputIsophoteVertex(t, dir, k, s);
		return;
	}

	if ( (b[0] + b[1] + b[2])%3 == 0 ) {
		return;
	}

	i = 0;
	if (  (s[0]-k)*(s[1]-k) < 0. ) {
		sc[0] = fabs((s[1]-k)/(s[0]-s[1]));
		sc[1] = 1. - sc[0];
		p[i] = PPacN(2, t.p, sc);
		p[i] = PVAdd(p[i],SVMult(offset,
					 VNormalize(VVlcN(2, v, sc))));
		i++;
	}
	if (  (s[1]-k)*(s[2]-k) < 0. ) {
		sc[0] = fabs((s[2]-k)/(s[1]-s[2]));
		sc[1] = 1. - sc[0];
		p[i] = PPacN(2, t.p+1, sc);
		p[i] = PVAdd(p[i],SVMult(offset,
					 VNormalize(VVlcN(2, v+1, sc))));
		i++;
	}
	if (  (s[2]-k)*(s[0]-k) < 0. ) {
		Point p2[2];
		sc[0] = fabs((s[0]-k)/(s[2]-s[0]));
		sc[1] = 1. - sc[0];
		p2[0] = t.p[2]; p2[1] = t.p[0];
		p[i] = PPacN(2, p2, sc);
		p[i] = PVAdd(p[i],SVMult(offset,
					 VNormalize(VVlcN(2, v+2, sc))));
		i++;
	}
	if ( i != 2 ) {
		fprintf(stderr,"OutputIsophote: error\n");
		exit(1);
	}
	if ( mode == S3D ) {
		A3dColor(k);
		A3dOutEdge(p[0],p[1]);
	} else if (mode == PS) {
		PSOutEdge(p[0], p[1]);
	}
}

main(int argc, char* argv[])
{
	struct triangle t;
	int i;
	
	w = SCreate("world", 3);
	wf = StdFrame(w);

	dir = VCreate(wf, 0., 0., 1.);

	ParseCommandLine(argc, argv);

	fHeader(stdout);

	while( ReadDstruct() != EOF ){
		if ( !QueryDstructPath( "Triangle" ) ) {
			fprintf(stderr, "\
%s: dstruct not Triangle.  Exiting.\n", argv[0]);
			exit(1);
		}
		t = MakeInteralTri();
		for ( i=0; i<kn; i++ ) {
			OutputIsophote(t, dir, k[i]);
		}
	}
	fTrailer(stdout);
	exit(0);
}

void SetDir(char* a[])
{
	dir = VNormalize(VCreate(wf, atof(a[0]), atof(a[1]), atof(a[2])));
}

#define PI 3.141593

void SetK(char* a[])
{
	k[0] = cos( PI*atof(a[0])/180. );
	kn = 1;
}


void Set5K(char* a[])
{
	int i;

	for ( i=0; i<5; i++ ) {
		k[i] = cos( PI*atof(a[i])/180. );
	}
	kn = 5;
}

void Set10K(char* a[])
{
	int i;

	for ( i=0; i<10; i++ ) {
		k[i] = cos( PI*atof(a[i])/180. );
	}
	kn = 10;
}


void Set20K(char* a[])
{
	int i;
	
	for ( i=0; i<20; i++ ) {
		k[i] = cos( PI*atof(a[i])/180. );
	}
	kn = 20;
}


void SetO(char* a[])
{
	offset = atof(a[0]);
}

void SetC(char* a[])
{
	color = 1;
}

static void SetPS(char* a[])
{
	mode = PS;
}

static void SetScale(char* a[])
{
	PSScale = atof(a[0]);
}

char* Banner="Isophote";
char* UsageString = "Isophote [options] < triangle-stream";
Option Options[]={
	"h", Usage, 0, ": print available options.",
	"dir", SetDir, 3, "x y z: set the isophote direction [0 0 1].",
	"k", SetK, 1, "k: set the isophote k.",
	"k5", Set5K, 5, "k1 ... k5: Output 5 isophotes (ki = degrees).",
	"k10", Set10K, 10, "k1...k10: Output 10 isophote (ki = degrees).",
	"k20", Set20K, 20, "k1...k20: Output 20 isophote (ki = degrees).",
	"o", SetO, 1, "offset: set the offset of the isophote from the surface [.01].",
	"c", SetC, 0, ": output colored isolines.",
	"ps", SetPS, 0, ": Output PostScript instead of s3d.",
	"scale",SetScale, 1, "s: Scale the output by s (PostScript only)",


	NULL,NULL,     0, NULL
  };
