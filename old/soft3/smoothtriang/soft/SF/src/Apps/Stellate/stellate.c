/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  stellate.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "libgeo.h"
#include "material.h"
#include "libmat.h"
#include "mesh.h"
#include "userData.h"
#include "vertex.h"
#include "patch.h"
#include "util.h"
#include "commandline.h"
#include "usage.h"

Space world;
int normalize=0;

void WriteVertex(v)
Vertex* v;
{
	printf("Nv%s = ",v->name);
	fdWriteDstructOneLine(stdout,v->externalData);
	printf("\n");
}


#define MAX_NEIGH 40


Point FaceCentroid(f)
Face* f;
{
	int count=0;
	int i;
	Vertex* v;
	Point p[MAX_NEIGH];
	double s[MAX_NEIGH];
	
	ForeachFaceVertex(f,v){
		if (count >= MAX_NEIGH) {
			fprintf(stderr,"FaceCentroid(): face has too many neighbors.\n");
			exit(1);
		}
		p[count] = ReturnUDPoint(v);
		count++;
	} EndForeach;
	for(i=0;i<count;i++){
		s[i] = 1.0/count;
	}
	return PPacN(count,p,s);
}

#define GIVEN 1
#define COMPUTE 2

double scaleFactor=1.0;
int normalType = GIVEN;

Vertex* ComputeFaceVertex(f)
Face* f;
{
	Vertex* v;
	Vertex* r;
	Vector n;
	static int current=0;
	char* malloc();
	double x,y,z; double dx,dy,dz;
	Vertex* p[MAX_NEIGH];
	int count,i;
	
	n = VCreate(StdFrame(world),0.0,0.0,0.0);
	
	if ( normalType == GIVEN ) {
		i=0;
		ForeachFaceVertex(f,v){
			i++;
			n = VVAdd( n,  NDual(ReturnUDNormal(v)) );
		} EndForeach;
		n = SVMult( scaleFactor/(float)i, n );
	} else if ( normalType == COMPUTE ) {
		count=0;
		ForeachFaceVertex(f,v){
			p[count++] = v;
		} EndForeach;
		for(i=0;i<count;i++){
			n = VVAdd( n, VVCross( PPDiff(ReturnUDPoint(p[i]), 
						      ReturnUDPoint(p[(i+1)%count])),
					      PPDiff(ReturnUDPoint(p[(i+1)%count]), 
						     ReturnUDPoint(p[(i+2)%count]))));
		} 
		n = SVMult( scaleFactor/(float)count, n );
	} else {
		fprintf(stderr,"Unknown normalType\n");
		exit(1);
	}
	r = NewNode();
	r->tag = POINT;
	if ( normalize ) {
		SetUDNormal(r, VDual(VNormalize(n)));
	} else {
		SetUDNormal(r, VDual(n));
	}
	SetUDPoint(r, PVAdd( FaceCentroid(f), n ));
	r->name = malloc(10);
	sprintf(r->name,"%d\0",current++);
	
	PCoords(ReturnUDPoint(r),StdFrame(world),&x,&y,&z);
	NCoords(ReturnUDNormal(r),StdFrame(world),&dx,&dy,&dz);
	
	dPutScalar(r->externalData,"Point.pos[0]",x);
	dPutScalar(r->externalData,"Point.pos[1]",y);
	dPutScalar(r->externalData,"Point.pos[2]",z);
	
	dPutScalar(r->externalData,"Point.norm[0]",dx);
	dPutScalar(r->externalData,"Point.norm[1]",dy);
	dPutScalar(r->externalData,"Point.norm[2]",dz);
	
	return r;
}

void WriteExtendedFace(f)
Face* f;
{
	Vertex* n[MAX_NEIGH];
	Vertex* v;
	int count=0;
	static int fn=0;
	int i;
	
	ForeachFaceVertex(f,v){
		if (count >= MAX_NEIGH) {
			fprintf(stderr,"WriteExtendedFace(): face has too many neighbors.\n");
			exit(1);
		}
		n[count] = v;
		count++;
	} EndForeach;
	
	if ( ReturnName(f) == NULL ) {
		char buf[1000];

		sprintf(buf, "f%d", fn++);
		SetName(f, buf);
	}

	for(i=0;i<count;i++){
		printf("%sF%d = [ Nv%s, Nv%s, Nv%s ];\n",f->name,i,
		       ((Vertex*)(f->internalData->v))->name, 
		       n[i]->name, n[(i+1)%3]->name);
	}
}


void WriteMesh(m)
Mesh* m;
{
	Face* f;
	Vertex* v;
	int i;
	int first=1;
	
	printf("mesh = {\n\t");
	ForeachMeshFace(m,f){
		i = 0;
		ForeachFaceVertex(f,v){
			if ( first ){
				first = 0;
			} else {
				printf(",\n\t");
			}
			printf("%sF%d",ReturnName(f),i++);
		} EndForeach;
	} EndForeach;
	printf("\n};\n");
}


main(argc,argv)
int argc;
char* argv[];
{
	Mesh* m;
	Face* f;
	Vertex* v;
	
	ParseCommandLine(argc,argv);
	
	world = SCreate("world",3);
	m = MeshParse(stdin);
	AddGeometry(world,m);
	
	if ( !(ReturnUDGeoFlags(m) & G_POSITION) ) {
		normalType = COMPUTE;
	}
	
	ForeachMeshVertex(m,v){
		WriteVertex(v);
	} EndForeach;
	
	ForeachMeshFace(m,f){
		v = ComputeFaceVertex(f);
		SetUDPoint(f, *v);
		f->internalData->v = v;
		WriteVertex( v );
	}EndForeach;
	
	ForeachMeshFace(m,f){
		WriteExtendedFace(f);
	}EndForeach;
	
	WriteMesh(m);
}

void ScaleF(s)
char** s;
{
	double atof();
	
	scaleFactor = atof(*s);
}

void CrossNorm()
{
	normalType = COMPUTE;
}

void Norm()
{
	normalize = 1;
}

char* Banner = "stellate";
char* UsageString = "stellate [options] < mesh";

Option Options[]={
  "h",Usage,0,": \tprint help message.",
  "s",ScaleF,1,"scale: \tadjust the lengths of new faces.",
  "c",CrossNorm,0,":\t use cross product normals.",
  "n",Norm,0,":\tNormalize the new normals.",
  NULL,NULL,0,NULL
  };
