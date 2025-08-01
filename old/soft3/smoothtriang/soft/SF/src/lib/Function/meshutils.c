/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  meshutils.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <strings.h>
#include <ctype.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "mesh.h"
#include "userData.h"
#include "meshsym.h"


WriteFace(Face* f, FILE* fp)
{
	Vertex* v;
	int first;

	fprintf(fp,"[");
	first = TRUE;
	ForeachFaceVertex(f,v){
		if ( !first ){
			fprintf(fp,", ");
		} else {
			first = FALSE;
		}
		if ( v->name != NULL ) {
			fprintf(fp,"%s",v->name);
		} else {
			fdWriteDstructOneLine(fp,v->externalData);
		}
	} EndForeach;
	fprintf(fp,"]");
	if ( f->externalData != NULL ){
		fprintf(fp," ");
		fdWriteDstructOneLine(fp,f->externalData);
		fprintf(fp,"\n");
	} else {
		fprintf(fp,";\n");
	}
}



WriteItoEMesh(Mesh* m, FILE* fp, void (*itoeV)(), 
	      void (*itoeE)(), void (*itoeF)())
{
	Vertex* v,vp;
	Face* f;
	int first;

	ForeachMeshVertex(m,v){
		if ( v->name != NULL ) {
			fprintf(fp,"%s",v->name);
			if ( v->externalData != NULL ) {
				dDeleteLnode(v->externalData);
				v->externalData = NULL;
			}
			if ( v->internalData != NULL  &&  itoeV != NULL ) {
				itoeV(v);
			}
			if ( v->externalData != NULL ) {
				fprintf(fp," = "); 
				fdWriteDstructOneLine(fp,v->externalData);
				fprintf(fp,"\n");
				dDeleteLnode(v->externalData);
				v->externalData = NULL;
			} else {
				fprintf(fp,";\n");
			}
		} else {
			;
		}
	} EndForeach;

	fprintf(fp,"\n");

	ForeachMeshFace(m,f){
		if ( f->name != NULL ) {
			fprintf(fp, "%s = ", f->name);
			WriteFace(f, fp);
		} else {
			;
		}
	} EndForeach;

	fprintf(fp,"\n");

	fprintf(fp,"%s = {\n",m->name);
	first = TRUE;
	ForeachMeshFace(m,f){
		if ( !first ){
			fprintf(fp,",\n");
		} else
		  first = FALSE;
		if ( f->name != NULL ) {
			fprintf(fp,"	%s",f->name);
		} else {
			fprintf(fp,"	");
			WriteFace(f,fp);
		}
	} EndForeach;
	if ( m->externalData != NULL ) {
		fprintf(fp," }\n\t  ");
		fdWriteDstructOneLine(fp,m->externalData);
		fprintf(fp," \n");
	} else {
		fprintf(fp,"};\n");
	}
}

WriteItoEMesh2(Mesh* m, FILE* fp, void (*itoeV)(), 
	       void (*itoeE)(), void (*itoeF)())
{
	Vertex* v,vp;
	Face* f;
	int first;

	ForeachMeshVertex(m,v){
		if ( v->name != NULL ) {
			fprintf(fp,"%s",v->name);
			if ( v->internalData != NULL  &&  itoeV != NULL ) {
				itoeV(v);
			}
			if ( v->externalData != NULL ) {
				fprintf(fp," = "); 
				fdWriteDstructOneLine(fp,v->externalData);
				fprintf(fp,"\n");
				dDeleteLnode(v->externalData);
				v->externalData = NULL;
			} else {
				fprintf(fp,";\n");
			}
		} else {
			;
		}
	} EndForeach;

	fprintf(fp,"\n");

	ForeachMeshFace(m,f){
		if ( f->name != NULL ) {
			fprintf(fp, "%s = ", f->name);
			WriteFace(f, fp);
		} else {
			;
		}
	} EndForeach;

	fprintf(fp,"\n");

	fprintf(fp,"%s = {\n",m->name);
	first = TRUE;
	ForeachMeshFace(m,f){
		if ( !first ){
			fprintf(fp,",\n");
		} else
		  first = FALSE;
		if ( f->name != NULL ) {
			fprintf(fp,"	%s",f->name);
		} else {
			fprintf(fp,"	");
			WriteFace(f,fp);
		}
	} EndForeach;
	if ( m->externalData != NULL ) {
		fprintf(fp," }\n\t  ");
		fdWriteDstructOneLine(fp,m->externalData);
		fprintf(fp," \n");
	} else {
		fprintf(fp,"};\n");
	}
}


WriteMesh(Mesh* m, FILE* fp)
{
	Vertex* v,vp;
	Face* f;
	int first;
	int freeV=0;

	ForeachMeshVertex(m,v){
		if ( v->rep == NULL ) {
			freeV = 1;
		}
		if ( v->name != NULL ) {
			fprintf(fp,"%s",v->name);
			if ( v->externalData != NULL ) {
				fprintf(fp," = "); 
				fdWriteDstructOneLine(fp,v->externalData);
				fprintf(fp,"\n");
			} else {
				fprintf(fp,";\n");
			}
		} else {
			;
		}
	} EndForeach;

	fprintf(fp,"\n");

	ForeachMeshFace(m,f){
		if ( f->name != NULL ) {
			fprintf(fp, "%s = ", f->name);
			WriteFace(f, fp);
		} else {
			;
		}
	} EndForeach;

	fprintf(fp,"\n");

	fprintf(fp,"%s = {\n",m->name);
	first = TRUE;
	ForeachMeshFace(m,f){
		if ( !first ){
			fprintf(fp,",\n");
		} else
		  first = FALSE;
		if ( f->name != NULL ) {
			fprintf(fp,"	%s",f->name);
		} else {
			fprintf(fp,"	");
			WriteFace(f,fp);
		}
	} EndForeach;
	if ( freeV ) {
		ForeachMeshVertex(m, v) {
			if ( v->rep == NULL  &&  ReturnName(v) != NULL ) {
				if ( !first ) {
					fprintf(fp,",\n");
				} else {
					first = FALSE;
				}
				fprintf(fp,"	%s",ReturnName(v));
			}
		} EndForeach;
	}
	if ( m->externalData != NULL ) {
		fprintf(fp," }\n\t  ");
		fdWriteDstructOneLine(fp,m->externalData);
		fprintf(fp," \n");
	} else {
		fprintf(fp,"};\n");
	}
}


void ClearMeshExternalData(Mesh* m)
{
	Vertex* v;
	Face* f;
	Edge* e;

	ForeachMeshVertex(m,v){
		dDeleteDstruct(v->externalData);
	} EndForeach;

	ForeachMeshFace(m,f){
		dDeleteDstruct(f->externalData);
	} EndForeach;

	ForeachMeshEdge(m,e){
		dDeleteDstruct(e->externalData);
	} EndForeach;
}
