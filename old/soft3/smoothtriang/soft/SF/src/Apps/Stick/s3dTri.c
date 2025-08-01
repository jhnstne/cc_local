/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  a3dTri.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include "stick.h"

void PrintS3dMaterial(Material m)
{
	static Material prevm;

	if ( prevm.diffuse.r == m.diffuse.r  &&
	     prevm.diffuse.g == m.diffuse.g  &&
	     prevm.diffuse.b == m.diffuse.b  &&
	     prevm.specular.r == m.specular.r  &&
	     prevm.specular.g == m.specular.g  &&
	     prevm.specular.b == m.specular.b  &&
	     prevm.specularity == m.specularity ) {
		return;
	}
	printf("d %g %g %g\n",m.diffuse.r, m.diffuse.g, m.diffuse.b);
	printf("s %g %g %g\n",m.specular.r, m.specular.g, m.specular.b);
	printf("g %d 0 0\n",m.specularity);
	prevm = m;
}

void PrintS3dTriangle(Triangle t)
{
	int i;
	Scalar x,y,z;

	printf("P 3 0 0\n");
	for(i=0;i<3;i++){
		PrintS3dMaterial(t.m);
		NCoords(t.n[i], worldF, &x, &y, &z);
		printf("n %g %g %g\n", x, y, z);
		PCoords(t.p[i], worldF, &x, &y, &z);
		printf("v %g %g %g\n", x, y, z);
	}
	printf("E 0 0 0\n");
}


#if 0
void PrintS3DLine(Stick s)
{
	Scalar x,y,z;

	PCoords(s.p1,worldF,&x,&y,&z);
	printf("%g %g %g 0\n",x,y,z);
	printf("%g %g %g d\n",s.m.diffuse.r, s.m.diffuse.g, s.m.diffuse.b);
	printf("%g %g %g s\n",s.m.specular.r, s.m.specular.g, s.m.specular.b);
	printf("%g 0 0 p\n",s.m.specularity);

	PCoords(s.p2,worldF,&x,&y,&z);
	printf("%g %g %g 1\n",x,y,z);
	printf("%g %g %g d\n",s.m.diffuse.r, s.m.diffuse.g, s.m.diffuse.b);
	printf("%g %g %g s\n",s.m.specular.r, s.m.specular.g, s.m.specular.b);
	printf("%g 0 0 p\n",s.m.specularity);
}
#endif

void S3dVertex(Point p, Normal n)
{
	Scalar x,y,z;

	NCoords(n, worldF, &x, &y, &z);
	printf("n %g %g %g\n",x,y,z);
	PCoords(p, worldF, &x, &y, &z);
	printf("v %g %g %g\n",x,y,z);
}

void PrintS3dOcta(Vector t, Vector b, Vector c[6], Point sc, Material m)
{
	PrintS3dMaterial(m);
	printf("M 6 8 0\n");
	S3dVertex(PVAdd(sc,t),VDual(t));
	S3dVertex(PVAdd(sc,b),VDual(b));
	S3dVertex(PVAdd(sc,c[0]),VDual(c[0]));
	S3dVertex(PVAdd(sc,c[1]),VDual(c[1]));
	S3dVertex(PVAdd(sc,c[2]),VDual(c[2]));
	S3dVertex(PVAdd(sc,c[3]),VDual(c[3]));
	printf("f 0 2 3\n");
	printf("f 0 3 4\n");
	printf("f 0 4 5\n");
	printf("f 0 5 2\n");
	printf("f 1 3 2\n");
	printf("f 1 4 3\n");
	printf("f 1 5 4\n");
	printf("f 1 2 5\n");
	printf("E 0 0 0\n");
}


void PrintS3dStick( Point p1, Point p2, Vector el[9], int delta, Material m )
{
	int i;

	PrintS3dMaterial(m);
	printf("M %d %d 0\n",16-8*delta, 16-8*delta);
	for (i=0; i<8; i+=1+delta) {
		S3dVertex(PVAdd(p1,el[i]),VDual(el[i]));
		S3dVertex(PVAdd(p2,el[i]),VDual(el[i]));
	}
	for (i=0; i<8; i+=1+delta) {
		printf("f %d %d %d\n",2*i/(delta+1),
		       (2*i/(delta+1)+1)%8,((2*i+2*delta)/(delta+1)+2)%8);
		printf("f %d %d %d\n",((2*i+2*delta)/(delta+1)+2)%8,
		       (2*i/(delta+1)+1)%8,
		       ((2*i+2*delta)/(delta+1)+3)%8);
	}
	printf("E 0 0 0\n");
}
