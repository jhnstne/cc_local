/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  userData.h
 *----------------------------------------------------------------------
 */


#define NO_GEOMETRY 0
#define G_POSITION 1
#define G_NORMAL 2

typedef struct userData {
  int geometryFlags;
  Point p;
  Normal n;
  Vertex* v;
} Data;

#define ReturnUDGeoFlags(d) ( (d)->internalData == NULL ? \
			     0 : (d)->internalData->geometryFlags )

Point ReturnUDPoint();
Normal ReturnUDNormal();
