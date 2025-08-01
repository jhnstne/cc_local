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

#if 0
int ReadA3DTriangle(Triangle* t)
{
	float x,y,z;
	char c;
	int i;
	
	for(i=0; i<3; i++){
		if ( scanf("%f %f %f %c",&x,&y,&z,&c) == EOF ){
			return EOF;
		}
		if ( c != 'm' ) {
			fprintf(stderr
				t->p[i] = PCreate(worldF, x,y,z);
			}
		}
		
		for(i=0; i<3; i++){
			if ( scanf(" [ %f %f %f ]",&x,&y,&z) == EOF ) {
				return EOF;
			}
			t->n[i] = NCreate(worldF, x,y,z);
		}
		
		for(i=0;i<3;i++){
			int s;
			scanf(" [ %f %f %f ]",&x,&y,&z);
			scanf(" [ %f %f %f ]",&x,&y,&z);
			scanf(" %d ",&s);
		}
		if ( scanf(" sgpTriangle") == EOF ){
			return EOF;
		}
	}
}
#endif

void PrintA3DTriangle(Triangle t)
{
	int i;
	Scalar x,y,z;
	
	for(i=0;i<3;i++){
		PCoords(t.p[i], worldF, &x, &y, &z);
		printf("%g %g %g %c\n", x, y, z, i==0?'m':'l');
		NCoords(t.n[i], worldF, &x, &y, &z);
		printf("%g %g %g n\n", x, y, z);
		printf("%g %g %g d\n",t.m.diffuse.r, t.m.diffuse.g, t.m.diffuse.b);
		printf("%g %g %g s\n",t.m.specular.r, t.m.specular.g, t.m.specular.b);
		printf("%d 0 0 p\n",t.m.specularity);
	}
}


void PrintA3DLine(Stick s)
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
