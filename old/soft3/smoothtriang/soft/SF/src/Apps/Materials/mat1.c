/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** mat1.c
**
** Author: Michael Lounsbery
** Created: Wed Aug 09, 1989 at 11:25:41 AM
** Last Modified: Wed Aug 09, 1989 at 11:25:41 AM
**
** Purpose: Interpolates between 2 different materials
*/


#include <math.h>
#include <stdio.h>
#include "all.h"


void	Mat1(), InsertMat();

Material	MatCombo();


char* matName1="RedPlastic";
char* matName2="RedPlastic";
Material m1, m2;

#define UNDEF	0
#define MAT_NAME 1
#define MAT_RGB 2
#define MAT_HSV 3

int matFlag=UNDEF;



main(int argc, char* argv[])
{
  char outputString[1000];

  ParseCommandLine(argc,argv);

  while( ReadDstruct() != EOF ){
    CopyDstructFields("Triangle","Triangle");

    switch(matFlag){
    case MAT_NAME:
      if ((!MaterialLookup(matName1, &m1)) || (!MaterialLookup(matName2, &m2)))
         {
         fprintf(stderr, "Error: Reference to unknown material name\n");
	 exit(1);
         }
      Mat1(dout, m1, m2);
      break;
    case MAT_RGB:
      Mat1(dout, m1, m2);
      break;
    case MAT_HSV:
      fprintf(stderr,"%s: MAT_HSV: not implemented yet.  Exiting.\n",argv[0]);
      exit(1);
    default:
      fprintf(stderr,"%s: case %d unknown.  Exiting.\n",argv[0],matFlag);
      exit(1);
    }
    FlushDstruct();
  }
}




/*
** Given an Lnode l pointing to a triangle structure, and 2 materials, this
** looks for the li tags on the vertices, and linearly interpolates between
** the values.
*/

void	Mat1(Lnode	*l, Material	mat1, Material mat2)
   {
   Lnode	*Cur, *ThisVertex;
   int		i;
   Scalar	li;
   Material	mat3;
   char		*Field, FieldMem[81];

   Field = FieldMem;

   if ((l == NULL) || (l->car == NULL) || (strcmp(l->car->name, "Triangle")))
      {
      fprintf(stderr, "ERROR in Mat1: Lnode isn't a valid triangle\n");
      exit(1);
      }

   for (i = 0; i <= 2; i++)
      {
      sprintf(Field, "Triangle.vertex%d", i + 1);

      if (!dLookUpLnode(l, Field, Cur))
         {
         fprintf(stderr, "ERROR in Mat1: vertex not found\n");
         exit(1);
         }

      ThisVertex = Cur->cdr;

      if (!dGetScalar(ThisVertex, "material.li", &li))
	 {
	 fprintf(stderr, "ERROR in Mat1: non-existent li value\n");
	 exit(1);
	 }

      mat3 = MatCombo(mat1, mat2, li);
      InsertMat(mat3, ThisVertex);
      dDeleteField(ThisVertex, "material.li");
      }
   }


Material MatCombo(Material mat1, Material mat2, Scalar li)
   {
   Material	mat3;

   mat3.diffuse.r = li * mat1.diffuse.r + (1.0 - li) * mat2.diffuse.r;
   mat3.diffuse.g = li * mat1.diffuse.g + (1.0 - li) * mat2.diffuse.g;
   mat3.diffuse.b = li * mat1.diffuse.b + (1.0 - li) * mat2.diffuse.b;

   mat3.specular.r = li * mat1.specular.r + (1.0 - li) * mat2.specular.r;
   mat3.specular.g = li * mat1.specular.g + (1.0 - li) * mat2.specular.g;
   mat3.specular.b = li * mat1.specular.b + (1.0 - li) * mat2.specular.b;

   mat3.specularity = li * mat1.specularity + (1.0 - li) * mat2.specularity;

   return mat3;
   }


void InsertMat(Material	mat, Lnode *Vertex)
   {
   dPutScalar(Vertex, "material.mat.diffuse[0]", mat.diffuse.r);
   dPutScalar(Vertex, "material.mat.diffuse[1]", mat.diffuse.g);
   dPutScalar(Vertex, "material.mat.diffuse[2]", mat.diffuse.b);

   dPutScalar(Vertex, "material.mat.specular[0]", mat.specular.r);
   dPutScalar(Vertex, "material.mat.specular[1]", mat.specular.g);
   dPutScalar(Vertex, "material.mat.specular[2]", mat.specular.b);

   dPutScalar(Vertex, "material.mat.specularity", (Scalar) mat.specularity);
   }




void SetMat1(char **a)
{
  if (matFlag != UNDEF)
     {
     fprintf(stderr, "You must set material 1 first\n");
     exit(1);
     }

  matName1 = (char *)malloc( strlen(*a) +1 );
  strcpy( matName1, *a);
  matFlag = MAT_NAME;
}


void SetMat2(char **a)
{
  if (matFlag == UNDEF)
     {
     fprintf(stderr, "You must set material 1 first\n");
     exit(1);
     }

  if (matFlag != MAT_NAME)
     {
     fprintf(stderr, "Mixing of material types not allowed\n");
     exit(1);
     }

  matName2 = (char *)malloc( strlen(*a) +1 );
  strcpy( matName2, *a);
  matFlag = MAT_NAME;
}


void SetColor1(char **a)
{
  if (matFlag != UNDEF)
     {
     fprintf(stderr, "You must set material 1 first\n");
     exit(1);
     }

  matFlag = MAT_RGB;

  m1.diffuse.r   = atof(a[0]); printf("a[0]=%s\n",a[0]);
  m1.diffuse.g   = atof(a[1]); printf("a[1]=%s\n",a[0]);
  m1.diffuse.b   = atof(a[2]); printf("a[2]=%s\n",a[0]);
  m1.specular.r  = atof(a[3]); printf("a[3]=%s\n",a[0]);
  m1.specular.g  = atof(a[4]); printf("a[4]=%s\n",a[0]);
  m1.specular.b  = atof(a[5]); printf("a[5]=%s\n",a[5]);
  m1.specularity = atof(a[6]);  /* ugh!  specularity is type in :-( */
}


void SetColor2(char **a)
{
  if (matFlag == UNDEF)
     {
     fprintf(stderr, "You must set material 1 first\n");
     exit(1);
     }

  if (matFlag != MAT_RGB)
     {
     fprintf(stderr, "Mixing of material types not allowed\n");
     exit(1);
     }

  matFlag = MAT_RGB;

  m2.diffuse.r   = atof(a[0]); printf("a[0]=%s\n",a[0]);
  m2.diffuse.g   = atof(a[1]); printf("a[1]=%s\n",a[0]);
  m2.diffuse.b   = atof(a[2]); printf("a[2]=%s\n",a[0]);
  m2.specular.r  = atof(a[3]); printf("a[3]=%s\n",a[0]);
  m2.specular.g  = atof(a[4]); printf("a[4]=%s\n",a[0]);
  m2.specular.b  = atof(a[5]); printf("a[5]=%s\n",a[5]);
  m2.specularity = atof(a[6]);  /* ugh!  specularity is type in :-( */
}


char* Banner="Mat1";
char* UsageString = "Mat1 [options] < triangle-string";
Option Options[]={
  "h", Usage, 0, ": print available options.",
  "m1", SetMat1, 1, "material: get a material from material library.",
  "m2", SetMat2, 1, "material: get a material from material library.",
  "rgb1", SetColor1, 7, "rd gd bd  rs gs bs p: set color.",
  "rgb2", SetColor2, 7, "rd gd bd  rs gs bs p: set color.",
  NULL,NULL,     0, NULL
  };
