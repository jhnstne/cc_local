/*
 *----------------------------------------------------------------------
 *  File:  boundary.h
 *----------------------------------------------------------------------
 */

void Jensen(Point S0, Point S1, Point T0, Point T3, Point S2, Point S3,
	    Point* T1, Point* T2, Scalar c);
void ChiyokuraKimura(Point S0, Point S1, Point T0, Point T3, Point S2, Point S3,
		     Point *T1, Point *T2,
		     Point a0, Point a3, int new);
void MinMaxBoundary(Vertex* s0, 
		    Point S1, Point T0, Point T3, Point S2,
		    Vertex* s3,
		    Point *T1, Point *T2,
		    Point a0, Point a3,
		    Point R0, Point R3, int new);

void ComputeCrossBoundary(Vertex* s0,
			  Point S1, Point T0, Point T3, Point S2,
			  Vertex* s3,
			  Point *T1, Point *T2,
			  Point a0, Point a3,
			  Vertex* A,	/* The third vertex, on the farside of the edge */
			  Scalar jensen);

int qBoundary(Vertex* v1,Vertex* v2, Point* p1, Point* p2);

void PlanarBoundary(Point p0, Normal n0, 
		    Point p1, Normal n1,
		    Point* r0, Point* r1);

void NielsonBoundary(Point p0, Normal n0, 
		     Point p1, Normal n1,
		     Point* r0, Point* r1,
		     int scaledNielson);

void FurthBoundary(Point p0, Normal n0, Point p1, Normal n1, 
		   Point* r0, Point* r1);

