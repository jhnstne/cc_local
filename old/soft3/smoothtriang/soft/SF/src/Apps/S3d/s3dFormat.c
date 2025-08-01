/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include	<stdio.h>
#include	<math.h>
#include 	"geometry.h"
#include	"dstruct.h"
#include	"getset.h"
#include	"material.h"
#include	"s3dFormat.h"

#define SQ(A) ((A)*(A))

extern int fnames;

/*
** Given an Lnode l pointing to a triangle structure, this writes out the
** triangle info in s3d format
*/

void	s3dFormat(FILE *fp, Lnode *l)
{
static	Lnode		*Cur, *ThisVertex;
static	int		i;
static	Scalar		x[3], y[3], z[3], nx[3], ny[3], nz[3];
static	Material	m[3];
static	Material	tm;
static	char		*Field, FieldMem[81];

    Field = FieldMem;

    if ((l == NULL) || 
	(l->car == NULL) || 
	(strcmp(l->car->name, "Triangle"))) {
	fprintf(stderr, "ERROR in s3dFormat: Lnode isn't a valid triangle\n");
      	exit(1);
    }

    dGetRGBMaterial(l,"Triangle",&tm);

    for (i = 0; i <= 2; i++) {
      sprintf(Field, "Triangle.vertex%d", i + 1);

      if (!dLookUpLnode(l, Field, Cur)) {
	 fprintf(stderr, "ERROR in s3dFormat: vertex not found\n");
	 exit(1);
      }

      ThisVertex = Cur->cdr;

      if ( (!dGetScalar(ThisVertex, "pos[0]", &(x[i])))
			||
	   (!dGetScalar(ThisVertex, "pos[1]", &(y[i])))
			||
	   (!dGetScalar(ThisVertex, "pos[2]", &(z[i])))
			||
	   (!dGetScalar(ThisVertex, "norm[0]", &(nx[i])))
			||
	   (!dGetScalar(ThisVertex, "norm[1]", &(ny[i])))
			||
	   (!dGetScalar(ThisVertex, "norm[2]", &(nz[i]))) ) {
	 fprintf(stderr, "ERROR in s3dFormat: bad value in position or normal\n");
	 exit(1);
      }

      /* get mat for this vertex */
      if ( !dGetRGBMaterial(ThisVertex, "", &(m[i])) ){
	m[i] = tm;
      }
    }

    if (i <= 2) {
      fprintf(stderr, "ERROR in s3dFormat: bad vertex count\n");
      exit(1);
    }

    if ( fnames  &&  dQueryDstructPath(l, "Triangle.name") ) {
	   char* name;

	   dGetString(l, "Triangle.name", &name);
	   fprintf( fp, " ## fn %s", name);
    }
    fprintf(fp, "\n");
    fprintf( fp, "P 3 0 0\n");
    for ( i = 0; i <= 2; i++) {
       static double dr,dg,db;
       static double sr,sg,sb;
       static double p;

       /* Output "materials" */
       if ( dr != m[i].diffuse.r  ||  dg != m[i].diffuse.g  ||  
	    db != m[i].diffuse.b ) {
	  fprintf( fp, "d %lg %lg %lg\n",
		   m[i].diffuse.r, m[i].diffuse.g, m[i].diffuse.b);
	  dr = m[i].diffuse.r;
	  dg = m[i].diffuse.g;
	  db = m[i].diffuse.b;
       }

       if ( sr != m[i].specular.r  ||  sg != m[i].specular.g  ||  
	    sb != m[i].specular.b ) {
	  fprintf( fp, "s %lg %lg %lg \n",
		   m[i].specular.r, m[i].specular.g, m[i].specular.b);
	  sr = m[i].specular.r;
	  sg = m[i].specular.g;
	  sb = m[i].specular.b;
       }

       if ( p != m[i].specularity ) {
	  fprintf( fp, "g %d 0 0 \n", m[i].specularity);
	  p = m[i].specularity;
       }


       /* Output vertex normals */
       if ( SQ(nx[i]) + SQ(ny[i]) + SQ(nz[i]) > 1e-6 ) {
	  Scalar num;

	  num = sqrt(SQ(nx[i]) + SQ(ny[i]) + SQ(nz[i]));
	  nx[i] /= num;
	  ny[i] /= num;
	  nz[i] /= num;
	  fprintf( fp, "n %lg %lg %lg \n", nx[i], ny[i], nz[i]);
       } 

       /* Output vertex coordinates */
       fprintf( fp, "v %lg %lg %lg\n", x[i], y[i], z[i]);
    }
    fprintf( fp, "E 0 0 0\n\n");
} 
