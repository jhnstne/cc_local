/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  userMalloc.c
 *	Contains the routine to malloc the default user data field.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "sff.h"
#include "dstruct.h"
#include "bezier.h"
#include "getset.h"
#include "libgeo.h"
#include "material.h"
#include "libmat.h"
#include "mesh.h"
#include "userData.h"
#include "operators.h"
#include "vertex.h"
#include "patch.h"
#include "tpbezier.h"
#include "util.h"
#include "commandline.h"
#include "usage.h"
#include "intersection.h"
#include "geom.h"



/*
 *----------------------------------------------------------------------
 *  Function:  UDMalloc
 *----------------------------------------------------------------------
 */
Data* UDMalloc()
{
  return (Data*)malloc( sizeof(Data) );
}


/*
 *----------------------------------------------------------------------
 *  Function:  UVDMalloc
 *----------------------------------------------------------------------
 */
VData* UVDMalloc()
{
  return (VData*)malloc( sizeof(VData) );
}


/*
 *----------------------------------------------------------------------
 *  Function:  UEDMalloc
 *----------------------------------------------------------------------
 */
EData* UEDMalloc()
{
  return (EData*)malloc( sizeof(EData) );
}


/*
 *----------------------------------------------------------------------
 *  Function:  UFDMalloc
 *----------------------------------------------------------------------
 */
FData* UFDMalloc()
{
  return (FData*)malloc( sizeof(FData) );
}
