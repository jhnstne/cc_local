/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: 03/23/89 at 16:01:14
** Purpose: Interface module for materials.
*/

#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#define MAX_MAT_NAME 30

typedef struct {
    double r, g, b;
} Color;

typedef struct {
    Color diffuse;
    Color specular;
    int specularity;
    char* name;		/* used internally; should only be read by user */
} Material;

/*
  Material MaterialCreate( dr, dg, db, sr, sg, sb, spec)
  double dr, dg, db, sr, sg, sb;
  int spec;

  Create and return a new material.
  The diffuse reflection in the red, green, and blue channels
  is given by dr, dg, and db; the specular reflection is given
  by sr, sg, and sb; the Phong coefficient is given by spec.
*/
extern Material MaterialCreate(double dr, double dg, double db, 
			double sr, double sg, double sb, int spec);



/*
  void MaterialWrite( fp, m)
  FILE *fp;
  Material m;

  Write a material on the given stream.
*/
extern void MaterialWrite(FILE *fp, Material m);


/*
  int MaterialRead( fp, mp)
  FILE *fp;
  Material *mp;

  Read in a material descriptor from the given stream.  One is returned
  on success, zero on end of file.
*/
extern int MaterialRead(FILE *fp, Material *mp);

/*
  int MaterialLookup( name, mp)
  char *name;
  Material *mp;

  Set the material pointed to by mp to the named material.  Returned is
  one on success, zero otherwise.
*/
extern int MaterialLookup(char *name, Material *mp);


/*
  int IsNullMaterial( m)
  Material m;

  Return one if the given material is equivalent to the null material.
  Return zero otherwise.
*/
extern int IsNullMaterial(Material m);

/*
  Material KMat(k, kmin, kmax, gamma)
  double k, kmin, kmax, gamma;

  Return a material based on the value k.  k will be clamped to be in
  the range kmin to kmax.  If kmax < kmin, the null material is returned.
  The color assigned is an HLS color, where H is k (mapped to -1,1),
  L is .5, and S is .9.  The hue range is 240.  A suggested gamma value
  is 0.625.
*/
extern Material KMat(double k, double kmin, double kmax, double Gamma);


/*
  These are pre-defined materials.
*/
extern Material
  NullMaterial, 
  RedPlastic,
  NewBlue,
  NewGreen,
  NewPurple,
  RedMatte,
  GreenPlastic,
  DarkGreenPlastic,
  GreenMatte,
  BluePlastic,
  BlueMatte,
  YellowPlastic,
  YellowMatte,
  PurplePlastic,
  PurpleMatte,
  Steel,
  White,
  Black
  ;

#endif /* _MATERIAL_H_ */
