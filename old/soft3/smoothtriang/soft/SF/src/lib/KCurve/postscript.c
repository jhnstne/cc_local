/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include <stdio.h>
#include "all.h"
#include "curve.h"
#include "math.h"


void PrintHeader()
{
  printf("%%!\n");
  printf("30 20 translate\n");
  printf("100 100 scale\n");
  printf("0.01 setlinewidth\n");
  printf("/Helvetica findfont .1 scalefont setfont\n");
}

void PrintTrailer()
{
  printf("showpage\n");
}

void PlotCurvature(c)
Curve* c;
{
	int i;
	double min,max;
	double k;
	
	min = 1000.0; max = -1;
	
	printf("%g %g moveto\n",0.0,Curvature(c,0.0));
	for(i=1;i<=50;i++){
		k = Curvature(c,(float)i/50.0);
		printf("%g %g lineto\n",(float)i/50.,k);
	}
	printf("stroke\n");
}

void PlotControlPolygon(c)
Curve* c;
{
	int i;
	Scalar x,y;
	
	PCoords(c->cp[0],StdFrame(SpaceOf(c->cp[0])),&x,&y);
	printf("%g %g moveto\n",x,y);
	for(i=1; i<=c->degree; i++){
		PCoords(c->cp[i],StdFrame(SpaceOf(c->cp[i])),&x,&y);
		printf("%g %g lineto\n",x,y);
	}
	printf("stroke\n\n");
	for(i=0; i<=c->degree;i++){
		PCoords(c->cp[i],StdFrame(SpaceOf(c->cp[i])),&x,&y);
		printf("newpath\n");
		printf("%g %g .01 0 360 arc fill\n",x,y);
	}
}


void PlotCurve(c)
Curve *c;
{
	int i;
	Point p;
	Scalar x,y;
	double max;
	double f=0.0;
	Point EvalCurve();
	
	for(i=0;i<=25;i++){
		p = EvalCurve(c, (float)i/25.0);
		PCoords(p,StdFrame(SpaceOf(p)),&x,&y);
		printf("%g %g ",x,y);
		if ( i == 0 ){
			printf("moveto\n");
		} else {
			printf("lineto\n");
		}
	}
	printf("stroke\n\n");
}


void PlotCurvatureAxis()
{
	printf("0 1 moveto\n");
	printf("0 0 lineto\n");
	printf("1 0 lineto\n");
	printf("0 1 moveto\n");
	printf("1 1 lineto\n");
	printf("stroke\n");
}


void PlotCircle()
{
	printf("gsave\n");
	printf("0.001 setlinewidth\n");
	printf("newpath\n");
	printf("1 -1 1.41421356 45 135 arc\n");
	printf("stroke\n\n");
	printf("grestore\n");
}

void LabelCurve(s)
char* s;
{
	printf(".3 -.1 moveto\n");
	printf("(%s) show\n",s);
}


void Plot3PlanarSpaceCurve(c,n)
Curve* c;
Normal n;
{
	Curve nc;

	Planar3DCurveTo2D(c, n, &nc);
	PlotCurve(&nc);
}


Planar3DCurveTo2D(c, n, cr)
Curve* c;
Normal n;
Curve* cr;
{
	static Space twoSpace;
	static int first = 1;
	Vector v1;
	Frame f;
	Frame nf;
	AffineMap am;
	int i;

	if ( first ) {
		twoSpace = SCreate("2", 2);
		first = 0;
	}

	nf = StdFrame(twoSpace);
	n = VDual(VNormalize(NDual(n)));
	v1 = PPDiff(c->cp[c->degree], c->cp[0]);

	f = FCreate(     c->cp[0], v1,        VVCross(v1,NDual(n)), NDual(n));
	am = ACreate( f, FOrg(nf), FV(nf, 0), FV(nf, 1),            VZero(nf));

	for ( i=0; i<=c->degree; i++ ) {
		cr->cp[i] = PAxform( c->cp[i], am );
	}
	cr->degree = c->degree;
}
