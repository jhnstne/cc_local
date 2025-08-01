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
#include "all.h"


#define UNPROCESSED 0
#define OK 1
#define FLIP 2

#define MAX 10

WriteFace(f,fp)
Face* f;
FILE* fp;
{
  Vertex* v;
  int first;
  int count;
  Vertex* vlist[MAX];
  int i;

  if ( !ReturnUDFlags(f, OK|FLIP) ) {
    fprintf(stderr,"WriteFace: face %s unprocessed.  Exiting.\n",f->name);
    exit(1);
  }

  count = 0;
  ForeachFaceVertex(f,v){
    if ( count >= MAX ) {
      fprintf(stderr,"WriteFace: too many vertices around face %s.  Exiting.\n"
	      , f->name);
      exit(1);
    }
    vlist[count++] = v;
  } EndForeach;

  fprintf(fp,"[");
  first = TRUE;
  for(i=0;i<count;i++){
    if ( !first ){
      fprintf(fp,", ");
    } else {
      first = FALSE;
    }
    if ( ReturnUDFlags(f, FLIP) ) {
      if ( vlist[count-1 -i]->name != NULL ) {
	fprintf(fp,"%s",vlist[count-1 -i]->name);
      } else {
	fdWriteDstructOneLine(fp,v->externalData);
      }
    } else {
      if ( vlist[i]->name != NULL ) {
	fprintf(fp,"%s",vlist[i]->name);
      } else {
	fdWriteDstructOneLine(fp,v->externalData);
      }
    }
  } 
  fprintf(fp,"]");
  if ( f->externalData != NULL ){
    printf(" ");
    fdWriteDstructOneLine(fp,f->externalData);
    printf("\n");
  } else {
    printf(";\n");
  }
}



WriteMesh(m,fp)
Mesh* m;
FILE* fp;
{
  Vertex* v,vp;
  Face* f;
  int first;

  ForeachMeshVertex(m,v){
    if ( v->name != NULL ) {
      fprintf(fp,"%s",v->name);
      if ( v->externalData != NULL ) {
	fprintf(fp," = "); fdWriteDstructOneLine(fp,v->externalData);
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
      fprintf(fp,"%s = ",f->name);
      WriteFace(f,fp);
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
