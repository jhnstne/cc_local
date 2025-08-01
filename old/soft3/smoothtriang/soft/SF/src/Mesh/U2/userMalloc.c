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
#include "all.h"


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
