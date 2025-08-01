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
#include "libmat.h"
#include "mesh.h"
#include "operators.h"
#include "userData.h"
#include "material.h"
#include "vertex.h"
#include "patch.h"
#include "util.h"
#include "commandline.h"
#include "usage.h"

Space world;
int normalize=0;

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
			fprintf(stderr,
				"FaceCentroid(): face has too many neighbors.\n");
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

Point OffsetFacePoint(f, pt, nr)
Face* f;
Point pt;
Normal* nr;
{
	Vertex* v;
	Vector vn;
	static int current=0;
	Vertex* p[MAX_NEIGH];
	int count,i;
	
	vn = VCreate(StdFrame(world),0.0,0.0,0.0);
	
	if ( normalType == GIVEN ) {
		i=0;
		ForeachFaceVertex(f,v){
			i++;
			vn = VVAdd( vn,  NDual(ReturnUDNormal(v)) );
		} EndForeach;
		vn = SVMult( scaleFactor/(float)i, vn );
	} else if ( normalType == COMPUTE ) {
		count=0;
		ForeachFaceVertex(f,v){
			p[count++] = v;
		} EndForeach;
		for(i=0;i<count;i++){
			vn = VVAdd( vn, 
				   VVCross( PPDiff(ReturnUDPoint(p[i]), 
						   ReturnUDPoint(p[(i+1)%count])),
					   PPDiff(ReturnUDPoint(p[(i+1)%count]), 
						  ReturnUDPoint(p[(i+2)%count]))));
		} 
		vn = SVMult( scaleFactor/(float)count, vn );
	} else {
		fprintf(stderr,"Unknown normalType\n");
		exit(1);
	}

	pt = PVAdd( FaceCentroid(f), vn );
	if ( normalize ) {
		vn = VNormalize(vn);
	}
	*nr = VDual(vn);
	return pt;
}

main(argc,argv)
int argc;
char* argv[];
{
	Mesh* m;
	Face* f;
	
	ParseCommandLine(argc,argv);
	
	world = SCreate("world",3);
	m = MeshParse(stdin);
	AddGeometry(world,m);
	
	if ( !(ReturnUDGeoFlags(m) & G_POSITION) ) {
		normalType = COMPUTE;
	}
	
	ForeachMeshFace(m,f){
		Point p;
		Normal n;
		Vertex* v;

		p = FaceCentroid(f);
		p = OffsetFacePoint(f, p, &n);
		v = CenterSplitFace(m, f);
		SetUDPoint(v, p);
		SetUDNormal(v, n);
		SetName(v, MeshUniqueName(m, "c"));
	} EndForeach;

	ConvertGeometryToExternal(m);
	WriteMesh(m, stdout);
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
