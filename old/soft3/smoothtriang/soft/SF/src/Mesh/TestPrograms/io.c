/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  io.c
 *	A simple test program that reads and writes a sequence of
 *  meshes, testing each mesh for validity, and fixing bad vertices
 *  if possible.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"

main(argc, argv)
int argc;
char* argv[];
{
  Mesh* m;
  FILE* fp=stdin;

  if ( argc == 2 ) {
	fp = fopen(argv[1], "r");
	if ( fp == NULL ) {
		fprintf(stderr,"Can't open file %s\n",argv[1]);
		exit(1);
	}
  }

  while ( m = MeshParse(fp) ) {
      fprintf(stderr,"%d vertices fixed.\n",FixVertices(m,NULL));
      if ( !ValidMesh(m) ) {
	    fprintf(stderr,"Invalid mesh\n");
      }
      WriteMesh(m,stdout);
  }
}
