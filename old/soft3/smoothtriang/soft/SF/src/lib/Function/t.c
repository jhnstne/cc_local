#include <stdio.h>
#include "geometry.h"
#include "sff.h"

Space s;

double f(x,y)
double x;
double y;
{
	return x*y;
}

Normal fn(x,y)
double x;
double y;
{
	Vector v1;
	Vector v2;

	v1 = VCreate(StdFrame(s), 1., 0., y);
	v2 = VCreate(StdFrame(s), 0., 1., x);
	return VDual(VNormalize(VVCross(v1, v2)));
}

SFF fs(x,y)
double x;
double y;
{
	SFF sff;
	Vector vn;

	sff.v0 = VCreate(StdFrame(s), 1., 0., y);
	sff.v1 = VCreate(StdFrame(s), 0., 1., x);
	vn = VNormalize(VVCross(sff.v0, sff.v1));
	sff.m[0][0] = VVDot(vn, VCreate(StdFrame(s), 0., 0., 0.));
	sff.m[0][1] = VVDot(vn, VCreate(StdFrame(s), 0., 0., 1.));
	sff.m[1][0] = sff.m[0][1];
	sff.m[1][1] = VVDot(vn, VCreate(StdFrame(s), 0., 0., 0.));
	return sff;
}

main(argc,argv)
int argc;
char* argv[];
{
	s = SCreate("world",3);
	mfunction(argc, argv, s, f, fn, fs);
}
