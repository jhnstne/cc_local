/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: 04/07/89 at 15:19:34
** Purpose: Implementation module for materials.
*/
#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "util.h"
#include "material.h"

Material NullMaterial = {
    { 0.0, 0.0, 0.0},
    { 0.0, 0.0, 0.0},
    0, "NullMaterial" };

Material RedPlastic = {
    { 0.9, 0.4, 0.4 },
    { 0.5, 0.5, 0.5 },
    6, "RedPlastic" };

Material NewGreen = {
    { 0.05, 0.8, 0.05},
    { 0.6, 0.3, 0.6},
    6, "NewGreen" };
 
Material NewBlue = {
    { 0.5, 0.5, 0.9},
    { 0.4, 0.4, 0.4},
    6, "NewBlue" };

Material NewPurple = {
    { 0.8, 0.37, 0.8},
    { 0.5, 0.5, 0.5},
    6, "NewPurple" };

Material RedMatte = {
    { 0.9, 0.4, 0.4 },
    { 0.0, 0.0, 0.0 },
    0, "RedMatte" };

Material GreenPlastic = {
    { 0.4, 0.9, 0.4 },
    { 0.5, 0.5, 0.5 },
    6, "GreenPlastic" };

Material DarkGreenPlastic = {
    { 0.1, 0.6, 0.1 },
    { 0.2, 0.2, 0.2 },
    6, "DarkGreenPlastic" };

Material GreenMatte = {
    { 0.4, 0.9, 0.4 },
    { 0.0, 0.0, 0.0 },
    0, "GreenMatte" };

Material BluePlastic = {
    { 0.4, 0.4, 0.9 },
    { 0.5, 0.5, 0.5 },
    6, "BluePlastic" };

Material BlueMatte = {
    { 0.4, 0.4, 0.9 },
    { 0.0, 0.0, 0.0 },
    0, "BlueMatte" };

Material YellowPlastic = {
    { 0.9, 0.9, 0.5 },
    { 0.5, 0.5, 0.5 },
    6, "YellowPlastic" };

Material YellowMatte = {
    { 0.9, 0.9, 0.4 },
    { 0.0, 0.0, 0.0 },
    0, "YellowMatte" };

Material PurplePlastic = {
    { 0.9, 0.2, 0.9 },
    { 0.5, 0.5, 0.5 },
    6, "PurplePlastic" };

Material PurpleMatte = {
    { 0.9, 0.2, 0.9 },
    { 0.0, 0.0, 0.0 },
    0, "PurpleMatte" };

Material Steel = {
    { 0.3, 0.3, 0.5 },
    { 0.2, 0.6, 0.2 },
    6, "Steel" };

Material White = {
	{ 1., 1., 1. },
	{ 0., 0., 0. },
	1, "White" };

Material Black = {
	{ 0., 0., 0. },
	{ 0., 0., 0. },
	1, "Black" };



/* Symbol table of external and internal names of predefined materials */
static Material* MaterialTable[] = {
     &NullMaterial,
     &RedPlastic,
     &NewBlue,
     &NewPurple,
     &NewGreen,
     &RedMatte,
     &GreenPlastic,
     &DarkGreenPlastic,
     &GreenMatte,
     &BluePlastic,
     &BlueMatte,
     &YellowPlastic,
     &YellowMatte,
     &PurplePlastic,
     &PurpleMatte,
     &Steel,
     &White,
     &Black,
     NULL
};

    
/*
** Create and return a new material.
** The diffuse reflection in the red, green, and blue channels
** is given by dr, dg, and db; the specular reflection is given
** by sr, sg, and sb; the Phong coefficient is given by spec.
*/
Material MaterialCreate(double dr, double dg, double db, 
			double sr, double sg, double sb, int spec)
{
    Material m;

    m.diffuse.r  = dr;   m.diffuse.g  = dg;   m.diffuse.b  = db;
    m.specular.r = sr;   m.specular.g = sg;   m.specular.b = sb;
    m.specularity = spec;
    m.name = NULL;

    return m;
}


/*
** Write a material on the given stream.
*/
void MaterialWrite(FILE *fp, Material m)
{
    fprintf(fp, "%% Material\n");
    fprintf(fp, "%lg %lg %lg %lg %lg %lg %d\n",
	    m.diffuse.r,  m.diffuse.g,  m.diffuse.b,
	    m.specular.r, m.specular.g, m.specular.b,
	    m.specularity);
}

/*
** Read in a material descriptor.  One is returned on success,
** zero on failure.
*/
int MaterialRead(FILE *fp, Material *mp)
{
    char buf[BUFSIZE];
    
    if (!ReadLine( fp, buf)) {
	return 0;
    }
    if (sscanf( buf, "%lf %lf %lf %lf %lf %lf %d",
		&(mp->diffuse.r),  &(mp->diffuse.g),  &(mp->diffuse.b),
		&(mp->specular.r), &(mp->specular.g), &(mp->specular.b),
		&(mp->specularity)) != 7) {
	return 0;
    }
    mp->name = NULL;
    return 1;
}


/*
** Return one if the given material is equivalent to the null material.
** Return zero otherwise.
*/
int IsNullMaterial(Material m)
{
    return
     (m.diffuse.r  == 0.0) && (m.diffuse.g  == 0.0) && (m.diffuse.b  == 0.0) &&
     (m.specular.r == 0.0) && (m.specular.g == 0.0) && (m.specular.b == 0.0) &&
     (m.specularity == 0);
}

/*
** Set the material pointed to by mp to the named material.  If the
** material isn't found, returned zero; else return one.
*/
int MaterialLookup(char *name, Material *mp)
{
    int i;

    
    for (i=0; MaterialTable[i] != NULL; i++) {
	if (strcmp(MaterialTable[i]->name, name) == 0) {
	    *mp = *(MaterialTable[i]);
	    return 1;
	}
    }
    return 0;
}


#define kdiff		(kmax - kmin)
#define	HalfHueRange	(HueRange / 2.0)
static Scalar	HueRange = 240.0;	/* probably won't change */

/*
 ** Routines from Charles for manipulating HLS
 */

static double VALUE(double n1, double n2, double hue)
{
    while (hue>360.0) hue = hue-360.0;
    while (hue<  0.0) hue = hue+360.0;
    if (hue< 60.0) return (n1 + (n2-n1)*hue/60.0);
    if (hue<180.0) return (n2);
    if (hue<240.0) return (n1 + (n2-n1)*(240.0-hue)/60.0);
    return (n1);
}

static void HLStoRGB(double h, double l, double s, 
		     double *r, double *g, double *b)
{
    double m1,m2;
    if (l<=0.5) m2 = l*(1.0+s); else m2 = l+s-(l*s);  
    m1 = 2*l - m2;
    if (s==0.0) {
	*r = l; *g = l; *b = l;
    } else {
	*r = VALUE(m1,m2,h+120.0);
	*g = VALUE(m1,m2,h);
	*b = VALUE(m1,m2,h-120.0);
    }
}

/*
 ** Given a curvature value k, this maps it into HLS space, and from there
 ** into RGB space
 **
 ** Stolen heavily from Charles Loop
 */

Material KMat(double k, double kmin, double kmax, double Gamma)
{
    double	Oldk = k;
    Material	mat;
    Scalar	r, g, b;
    
    if ( kmax < kmin ) {
	return NullMaterial;
    }

    if ( k > kmax ) k = kmax;
    if ( k < kmin ) k = kmin;

    /* map k to hue */
    
    k = 2.0*(kmax - k)/kdiff - 1.0;	/* maps to [-1:1] range */
    
    if (k < -1.0) {
	k = 0.0;
    } else if (1.0 < k)      {
	k = HueRange;
    } else if (k < 0.0) {
	k = HalfHueRange*(1.0 - pow(fabs(k), Gamma));
    }    else {
	k = HalfHueRange*(1.0 + pow(k, Gamma));
    }
    
    HLStoRGB(k, 0.5, 0.9, &r, &g, &b);
    
    mat.diffuse.r  = r;
    mat.diffuse.g  = g;
    mat.diffuse.b  = b;
    
    mat.specular.r  = 0.0;
    mat.specular.g  = 0.0;
    mat.specular.b  = 0.0;
    
    mat.specularity = 1;
    
    return	mat;
}

