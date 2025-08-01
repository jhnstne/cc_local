/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  flip.c
 *	A program to change the orientation of all the faces of a
 *  mesh.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"

main()
{
  Mesh* m;

  m = MeshParse(stdin);
  WriteMesh(m,stdout);
}
