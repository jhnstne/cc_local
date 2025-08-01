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
#define USER_FLAGS 0xFFFF0

typedef struct  userData {
  int geometryFlags;
  Point p;
  Normal n;
} Data;

typedef struct userVData {
  void* ptr;
  int geometryFlags;
  Point p;
  Normal n;
} VData;

typedef struct userEData {
  void* ptr;
} EData;

typedef struct userFData {
  void* ptr;
} FData;

#define ReturnUDGeoFlags(d) ( (d)->internalData == NULL ? \
			     0 : (d)->internalData->geometryFlags )

BOOLEAN GetUDPoint(void* n, Point* p);
BOOLEAN SetUDPoint(void* n, Point p);
BOOLEAN GetUDNormal(void* node, Normal* n);
BOOLEAN SetUDNormal(void* node, Normal n);

void UDItoEE();
void UDItoEF();
#define WriteUDMesh(M, FP) WriteItoEMesh((M), (FP), UDItoEV, UDItoEE, UDItoEF)

Point ReturnUDPoint(void* n);
Normal ReturnUDNormal(void* node);
Data* UDMalloc(void);
VData* UVDMalloc(void);
EData* UEMalloc(void);
FData* UFMalloc(void);
Mesh* MeshUDParse(FILE* fp, Space world);

int SetUDGeoFlags(void* ptr, int f);	/* normally ptr is Node*, but may
					   want to have Mesh* */
int SetUDFlags(Node* n, int f);
int ClearUDFlags(Node* n, int f);
int ReturnUDFlags(Node* n, int f);

void AddGeometry(Space world, Mesh *m);
void AddVertexGeometry(Space world, Vertex* v);
void UDItoEV(Vertex* v);
void ConvertGeometryToExternal(Mesh* m);
void UDEtoI(Vertex* v);
