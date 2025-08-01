/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include	<stdio.h>
#include 	"geometry.h"
#include	"dstruct.h"
#include	"getset.h"
#include	"material.h"
#include	"s3dFormat.h"
#include	"commandline.h"
#include	"usage.h"


int fnames=0;

main(int argc, char* argv[])
{
	ParseCommandLine(argc, argv);

	while( ReadDstruct() != EOF ){
		s3dFormat(stdout, din);
		dDeleteDstruct(din);
	}
	exit(0);
}

void UseFNames()
{
	fnames = 1;
}




char* Banner="S3d";
char* UsageString = "S3d [options] < ds-triangle-stream > s3d";
Option Options[]={
	"h", Usage, 0, ": print available options.",
	"fn",UseFNames,0,": output face names.",

	NULL,NULL,     0, NULL
  };
