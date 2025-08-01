/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  libmat.h
 *----------------------------------------------------------------------
 */

BOOLEAN dGetRGBMaterial(Lnode *d, char* p, Material* m);
BOOLEAN pSetMaterialName(Lnode **d, char *p, char *name);
BOOLEAN pSetRGBMaterial(Lnode **d, char* p, Material* m);

#define GetRGBMaterial(p,m) dGetRGBMaterial(din,(p),(m))

#define dSetMaterialName(d,p,n) pSetMaterialName( &(d), (p), (n) )
#define SetMaterialName(p,n) pSetMaterialName( &dout, (p), (n) )

#define dSetMaterial(d,p,m) pSetMaterial(&(d),(p),(m))
#define SetRGBMaterial(p,m) pSetRGBMaterial(&dout, (p), (m))
