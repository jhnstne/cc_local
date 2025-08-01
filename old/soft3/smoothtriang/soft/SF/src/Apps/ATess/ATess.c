/*****************************************************************/
/* Module: ATess.c						 */
/* $Date: 1999/10/25 22:33:06 $                                  */
/* $Revision: 1.5 $                                              */
/* $Source: /p/SurfaceFitting/SF/src/Apps/ATess/RCS/ATess.c,v $  */
/*****************************************************************/
/*
 * Copyright (c) 1998, Graphics Laboratory, University of Waterloo
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  ATess.c
 *	A simple reader/writer of A-patches
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "all.h"
#include "apatch.h"
#include "aptessellate.h"

Space World;
Frame WorldF;
int EdgeSamples = 0;
int GetRootFindingInfo = 0;
int DoPatchColoring = 0;

main(int argc, char* argv[])
{
   ParseCommandLine(argc, argv);
   World = SCreate("World", 3);
   WorldF = StdFrame(World);

   while (ReadDstruct() != EOF) {
      if (QueryDstructPath("Apatch")) {
         Apatch anApatch;
         ReadSingleApatch(&anApatch, World);
         TessellateApatch(anApatch);
         FreeApatch(anApatch);
      }
      else if (QueryDstructPath("ApatchPair")) {
         ApatchPair apPair;
         ReadApatchPair(&apPair, World);
         TessellateApatchPair(apPair);
         FreeApatch(apPair.ap1);
         FreeApatch(apPair.ap2);
      }
      else {
         fprintf(stderr, "%s: Unknown patch type - Exiting.\n", argv[0]);
         exit(1);
      }
   }
  exit(0);
}

/*------------------------------------------------------------*/
/*                   Command Line Handlers                    */
/*------------------------------------------------------------*/
void SetSamples(char *num[])
{
   EdgeSamples = atoi(num[0]);
   if (EdgeSamples < 0) {
      fprintf(stderr, "EdgeSamples must be greater than or equal to zero; defaulting to zero (the control scalar net will be output).\n");
      EdgeSamples = 0;
   }
}

void SetGetRootInfo()
{
   GetRootFindingInfo = 1;
}

void SetPatchColoring()
{
   DoPatchColoring = 1;
}

/* Table of available options -- register new options here */
Option Options[] = {
/* Name   Handler        #Args  HelpString */
   "h",   Usage,            0,  ": Print available options.",
   "s",   SetSamples,       1,  "<EdgeSamples> : Specify the number of samples along edge of the sample net; default of zero outputs the control scalar net.",
   "i",   SetGetRootInfo,   0,   ": Output mean and std. dev. for the # of steps taken to find the root on the surface.",
   "c",   SetPatchColoring, 0,   ": Output the A-patches using a coloring scheme to distinguish face and edge patches.",

/*  DO not delete the next line */
    NULL,    NULL,       0,      NULL,      NULL
  };

/* Global variables go here */
char *Banner  = "";
char *CommandName;
char* UsageString = "ATess < apatch-stream";

