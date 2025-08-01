/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: Wed Jul 05, 1989 at 10:18:09 AM
** Purpose: Interface module for control vertices used by neighborhood
**   fitters.
*/

#ifndef _VERTEX_H_
#define _VERTEX_H_

#include "geometry.h"

typedef struct {
    Point position;
    Normal normal;
} VERTEX;

#endif /* _VERTEX_H_ */
