/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  meshGeometry.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"


/*
 *----------------------------------------------------------------------
 *  Function:  AddGeometry
 *	Add the geometry to each vertex of the mesh.  The mesh
 *  geometry flags will be set only if every vertex has geometry.
 *----------------------------------------------------------------------
 */
void AddGeometry(Space world, Mesh *m)
{
	MeshIterator i;
	Vertex* v;
	
	SetUDGeoFlags(m, G_POSITION | G_NORMAL);
	ForeachMeshVertex(m,v){
		AddVertexGeometry(world,v);
		SetUDGeoFlags(m, ReturnUDGeoFlags(v));
	} EndForeach;
}



/*
 *----------------------------------------------------------------------
 *  Function:  AddVertexGeometry
 *----------------------------------------------------------------------
 */
void AddVertexGeometry(Space world, Vertex* v)
{
	char* s;
	char *t;
	double x,y,z;
	char* index();
	
	if ( v->tag != POINT ) {
		fprintf(stderr,"AddVertexGeometry() passed non-vertex.  ");
		fprintf(stderr,
			"Name = %s, tag = %d.  Exiting.\n",v->name,v->tag);
		exit(1);
	}
	
	SetUDGeoFlags(v, NO_GEOMETRY);
	
	if ( v->externalData == NULL )
	  return;
	
	if ( dGetScalar( v->externalData, "Point.pos[0]", &x) &&
	    dGetScalar( v->externalData, "Point.pos[1]", &y) &&
	    dGetScalar( v->externalData, "Point.pos[2]", &z) ){
		SetUDPoint(v, PCreate(StdFrame(world),x,y,z));
		SetUDGeoFlags(v, ReturnUDGeoFlags(v) | G_POSITION);
	}
	if ( dGetScalar( v->externalData, "Point.norm[0]", &x) &&
	    dGetScalar( v->externalData, "Point.norm[1]", &y) &&
	    dGetScalar( v->externalData, "Point.norm[2]", &z) ){
		SetUDNormal(v, NCreate(StdFrame(world),x,y,z));
		SetUDGeoFlags(v, ReturnUDGeoFlags(v) | G_NORMAL);
	}
}


void UDItoEV(Vertex* v)
{
	Scalar x,y,z;
	Scalar dx,dy,dz;
	Point p;
	Normal n;
	
	switch( ReturnUDGeoFlags(v) ){
	      case NO_GEOMETRY:
		v->externalData = NULL;
		break;
	      case G_POSITION:
		GetUDPoint(v, &p);
		PCoords(p, StdFrame(SpaceOf(p)), &x,&y,&z);
		dPutScalar(v->externalData,"Point.pos[0]",x);
		dPutScalar(v->externalData,"Point.pos[1]",y);
		dPutScalar(v->externalData,"Point.pos[2]",z);
		break;
	      case G_NORMAL:
		GetUDNormal( v, &n );
		NCoords(n,StdFrame(SpaceOf(n)), &dx,&dy,&dz);
		dPutScalar(v->externalData,"Point.norm[0]",dx);
		dPutScalar(v->externalData,"Point.norm[1]",dy);
		dPutScalar(v->externalData,"Point.norm[2]",dz);
		break;
	      case G_POSITION | G_NORMAL:
		GetUDPoint( v, &p);
		GetUDNormal( v, &n );
		PCoords(p, StdFrame(SpaceOf(p)), &x,&y,&z);
		NCoords(n, StdFrame(SpaceOf(n)), &dx,&dy,&dz);
		dPutScalar(v->externalData,"Point.pos[0]",x);
		dPutScalar(v->externalData,"Point.pos[1]",y);
		dPutScalar(v->externalData,"Point.pos[2]",z);
		
		dPutScalar(v->externalData,"Point.norm[0]",dx);
		dPutScalar(v->externalData,"Point.norm[1]",dy);
		dPutScalar(v->externalData,"Point.norm[2]",dz);
		break;
	}
}

void UDItoEE(Edge* e)
{
}

void UDItoEF(Face* f)
{
}

void ConvertGeometryToExternal(Mesh* m)
{
	Vertex* v;
	
	ForeachMeshVertex(m,v){
		dDeleteDstruct(v->externalData);
		UDItoEV(v);
	} EndForeach;
}


static Space etoiS=NULL;

void UDEtoI(Vertex* v)
{
	AddVertexGeometry(etoiS, v);
}

Mesh* MeshUDParse(FILE* fp, Space world)
{
	etoiS = world;

	return (meshEtoIparse(fp, UDEtoI, NULL, NULL));
}
