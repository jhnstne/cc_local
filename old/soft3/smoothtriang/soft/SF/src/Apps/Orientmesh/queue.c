/*
 * Copyright (c) 1991, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  queue.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"



#define MAXQ 10000
static Face* farray[MAXQ];
static int first;
static int last;

void AddToQueue(f)
Face* f;
{
  if ( (first - last + MAXQ )%MAXQ == 1 ) {
    fprintf(stderr,"AddToQueue: too many faces.  Exiting.\n");
    exit(1);
  }
  farray[last] = f;
  last = (last+1)%MAXQ;
}

Face* FirstInQueue()
{
  if ( last == first ) {
    return NULL;
  } else {
    Face* rf;

    rf = farray[first];
    first = (first+1)%MAXQ;
    return rf;
  }
}
