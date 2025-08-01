/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include	<stdio.h>
#include 	"all.h"
#include 	"sgpFormat.h"

main()
   {
     while( ReadDstruct() != EOF ){
       sgpFormat(stdout, din);
       dDeleteDstruct(din);
     }
     exit(0);
   }
