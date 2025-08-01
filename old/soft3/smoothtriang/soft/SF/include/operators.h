/*
 *----------------------------------------------------------------------
 *  File:  operators.h
 *----------------------------------------------------------------------
 */

#ifndef _OPERATORS_H_
#define _OPERATORS_H_

#include "mesh.h"

BOOLEAN AddMeshVertex( Mesh *m, Vertex *v, int fast );
BOOLEAN AddMeshEdge( Mesh *m, Edge *e, int fast );
BOOLEAN AddMeshFace( Mesh *m, Face *f, int fast );
BOOLEAN RemoveMeshVertex( Mesh *m, Vertex *v );
BOOLEAN RemoveMeshEdge( Mesh *m, Edge *e );
BOOLEAN RemoveMeshFace( Mesh *m, Face *f );
void DeleteMeshVertex( Mesh* m, Vertex* v );
void DeleteMeshFace( Mesh* m, Face* f );
void DeleteMeshEdge( Mesh* m, Edge* e );
Vertex* AddVertexEdge(Mesh *m, Edge *e);
Vertex* CenterSplitFace(Mesh* m, Face* f );
Face* CreateFace( Mesh *m );
Face* CreateNFace( Mesh *m, Vertex* v[], int n );
Face* CreateOpposingTriFace(Mesh* m, Edge* e1, Edge* e2);
Vertex* CreateVertex( Mesh *m );
BOOLEAN DestroyFace(Mesh *m, Face *f);
BOOLEAN DestroyVertex(Mesh *m, Face *v);
int FixVertices(Mesh* m, void (*f)(Mesh*, Vertex*, Vertex*));
BOOLEAN JoinHalfEdge(Mesh *m, Edge *he1, Edge *he2);
BOOLEAN JoinFaces(Mesh *m, Face *f1,Face *f2, int flg );
BOOLEAN JoinVertices(Mesh* m, Vertex* v1,Vertex* v2, int flg);
Edge* RemoveVertexEdge(Mesh *m, Vertex *v );
Edge* SplitEdge(Mesh *m, Edge *e1);
Face* SplitFace(Mesh *m, Face *f1, Vertex *v1, Vertex *v2);
Vertex* SplitTriFace3to1(Mesh *m, Face *f);

Face *AddNFace( Mesh *m, Vertex* v[], int n );
int ConnectHalfEdges( Mesh* m );


#endif _OPERATORS_H_
