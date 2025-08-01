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
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "sff.h"
#include "userData.h"


/*
 *----------------------------------------------------------------------
 *  Function:  UDMalloc
 *----------------------------------------------------------------------
 */
Data* UDMalloc(void)
{
  return (Data*)malloc( sizeof(Data) );
}


/*
 *----------------------------------------------------------------------
 *  Function:  UVDMalloc
 *----------------------------------------------------------------------
 */
VData* UVDMalloc(void)
{
  return (VData*)malloc( sizeof(VData) );
}


/*
 *----------------------------------------------------------------------
 *  Function:  UEDMalloc
 *----------------------------------------------------------------------
 */
EData* UEDMalloc(void)
{
  return (EData*)malloc( sizeof(EData) );
}


/*
 *----------------------------------------------------------------------
 *  Function:  UFDMalloc
 *----------------------------------------------------------------------
 */
FData* UFDMalloc(void)
{
  return (FData*)malloc( sizeof(FData) );
}
