
/* PFoley.h */

#ifndef PFOLEY_H
#define PFOLEY_H

typedef enum eRetVal {
    RV_OK = 0,
    RV_BAD_FRAME,
    RV_BAD_PROJECTION,
    RV_POINTS_COPLANAR
} RetVal;

#define EPSILON_SMALL_VECTOR    1e-04
#define EPSILON_DOT_PRODUCT     1e-06
#define M_EPSILON       1.0e-05
#define OUTPUT_PATCH    0

#define BORDER_THRESHOLD	3

extern int      patch_count;
extern int      count_no_frame;
extern int      count_bad_normals;
extern int      count_coplanar;

extern Space World;    /* Space where faces and patches reside */


RetVal GetFrame (Vertex* v0, Vertex* v1, Vertex* v2, Vertex* v, Frame* f);
BOOLEAN ConstructBoundary (Vertex* vr[3], Patch* patch, Frame fr);
RetVal GetBisector (Point* p0, Point* p1,  
		    Point* p2, Point* p, Vector* result);
BOOLEAN BadCenter (Patch* patch);
BOOLEAN BadBoundaries (Patch* patch);

#endif

