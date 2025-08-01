/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include	<stdio.h>
#include 	"all.h"
#include	"sgpFormat.h"

/*
** Given an Lnode l pointing to a triangle structure, this writes out the
** triangle info in sgp format
*/

void	sgpFormat(fp, l)
FILE	*fp;
Lnode	*l;
   {
   static	Lnode		*Cur, *ThisVertex;
   static	int		i;
   static	Scalar		x[3], y[3], z[3], nx[3], ny[3], nz[3];
   static	Material	m[3];
   static	Material	tm;
   static	char		*Field, FieldMem[81];

   Field = FieldMem;

   if ((l == NULL) || (l->car == NULL) || (strcmp(l->car->name, "Triangle")))
      {
      fprintf(stderr, "ERROR in sgpFormat: Lnode isn't a valid triangle\n");
      exit(1);
      }

   dGetRGBMaterial(l,"Triangle",&tm);

   for (i = 0; i <= 2; i++)
      {
      sprintf(Field, "Triangle.vertex%d", i + 1);

      if (!dLookUpLnode(l, Field, Cur))
         {
         fprintf(stderr, "ERROR in sgpFormat: vertex not found\n");
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
	   (!dGetScalar(ThisVertex, "norm[2]", &(nz[i]))) )
	 {
	 fprintf(stderr, "ERROR in sgpFormat: bad value in position or normal\n");
	 exit(1);
	 }
    
      /* get mat for this vertex */
      if ( !dGetRGBMaterial(ThisVertex, "", &(m[i])) ){
	m[i] = tm;
      }
    }

   if (i <= 2)
      {
      fprintf(stderr, "ERROR in sgpFormat: bad vertex count\n");
      exit(1);
      }

   /* Output vertex coordinates */
   fprintf( fp, "[ %lg %lg %lg ] [ %lg %lg %lg ] [ %lg %lg %lg ]\n",
           x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);

   /* Output vertex normals */
   fprintf( fp, "[ %lg %lg %lg ] [ %lg %lg %lg ] [ %lg %lg %lg ]\n",
           nx[0], ny[0], nz[0], nx[1], ny[1], nz[1], nx[2], ny[2], nz[2]);

   /* Output "materials" */
   fprintf( fp, "[ %lg %lg %lg ] [ %lg %lg %lg ] %d\n",
            m[0].diffuse.r, m[0].diffuse.g, m[0].diffuse.b,
            m[0].specular.r, m[0].specular.g, m[0].specular.b,
            m[0].specularity);
   fprintf( fp, "[ %lg %lg %lg ] [ %lg %lg %lg ] %d\n",
            m[1].diffuse.r, m[1].diffuse.g, m[1].diffuse.b,
            m[1].specular.r, m[1].specular.g, m[1].specular.b,
            m[1].specularity);
   fprintf( fp, "[ %lg %lg %lg ] [ %lg %lg %lg ] %d\n",
            m[2].diffuse.r, m[2].diffuse.g, m[2].diffuse.b,
            m[2].specular.r, m[2].specular.g, m[2].specular.b,
            m[2].specularity);

   fprintf( fp, "sgpTriangle\n");
   }

