/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  main.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"

MyUDItoEV(Vertex* v)
{
	Point p;
	Normal n;
	
	GetUDPoint( v, &p);
	dPutVertexPosition(v->externalData, "Point", p);
	GetUDNormal( v, &n );
	dPutVertexNormal(v->externalData, "Point", n);
}

main()
{
	Mesh *m;
	extern void AddNormals();
	extern normalFunction();
	Space world;
	
	world = SCreate("world",3);
	m = MeshParse(stdin);
	AddGeometry(world,m);
	AddNormals(m,normalFunction);
	WriteItoEMesh2(m, stdout, MyUDItoEV, UDItoEE, UDItoEF);
}
