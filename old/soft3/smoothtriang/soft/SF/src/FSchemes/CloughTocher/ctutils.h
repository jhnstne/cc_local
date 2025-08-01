/*
 *----------------------------------------------------------------------
 *  File:  ctutils.h
 *----------------------------------------------------------------------
 */

extern Point EdgeTangent(Vertex* vertex, Point point);
void GetBaryCoords (Point p0, Point p1, Point p2, Point p, Scalar b[3]);
void ConstructBoundary (Vertex* vr[3], Patch* patch);
