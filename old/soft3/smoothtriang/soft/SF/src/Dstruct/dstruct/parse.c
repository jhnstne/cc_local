/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File: parse.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "dstruct.h"
#include "dstructMem.h"
#include "getset.h"

main(int argc, char *argv[])
{
    while(ReadDstruct() != EOF){
      dDstructPrint(din);
    }
}
