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
#define G_RESERVED1 4
#define G_RESERVED2 8

typedef struct  userData {
  int geometryFlags;
  Space s;
  double x,y,z;
  double nx,ny,nz;
  SFF sff;
} Data;

typedef struct userVData {
  void* ptr;
  Point p;
  Normal n;
  int geometryFlags;
} VData;

typedef struct userEData {
  void* ptr;
} EData;

typedef struct userFData {
  void* ptr;
} FData;

#define ReturnUDGeoFlags(d) ( (d)->internalData == NULL ? \
			     0 : (d)->internalData->geometryFlags )

Point ReturnUDPoint();
Normal ReturnUDNormal();
Data* UDMalloc();
VData* UVDMalloc();
EData* UEMalloc();
FData* UFMalloc();
