/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  mesh.h
 *  Last Modified: Sun Jun 25, 1989 at 09:28:42 AM
 *----------------------------------------------------------------------
 */

/* 
** Author: Steve Mann (originally Charles Loop)
** Purpose: Defines data structures and constants needed to make a mesh.
** Last Modified: Fri Jun 23, 1989 at 01:29:11 PM
*/


#ifndef _MESH_H_
#define _MESH_H_

/* types of entities */
#define FACE 1
#define POINT 2
#define EDGE 3
#define MESH 4

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif


/* structure for keeping lists of vertices and faces of mesh */
typedef struct LinkType {
  struct LinkType* next;
  char* data;
} Link;



/* Mesh data structures */

/*
  Hacker's note:
  The first few fields of a Node and an Edge match so that they
  can share some of the same routines.
*/

typedef struct NodeType {
  Lnode* externalData;
  struct userData* internalData;
  char* name;
  int iflags;
  int tag;		/* tag with type: face or vertex */
  struct EdgeType* rep;	/* one of the edges adjacent to the node */
} Node;


typedef Node Vertex;
typedef Node Face;


/* This is really a half edge */
typedef struct EdgeType {
  Lnode* externalData;
  struct userData* internalData;
  char* name;
  int iflags;
  Node* p;/* primal -- vertex */
  Node* d;/* dual -- face */
  struct EdgeType* sym;
  struct EdgeType* next;
  struct EdgeType* prev;
} Edge;


typedef struct MeshType {
  Lnode* externalData;
  struct userData* internalData;
  char* name;
  Link* vertexList;
  Link* faceList;
  Link* edgeList;
} Mesh;

/*
 *----------------------------------------------------------------------
 *  Flags:
 *	I_REP	- used as representative in the mesh.  All vertices
 *		  and faces should have this set.  Only one of a
 *		  half edge pair should set this.
 *	I_DEL	- Marked for deletion.
 *	I_REM	- Marked for removal as rep of mesh.  I_REP should be
 *		  set.
 *----------------------------------------------------------------------
 */
#define I_REP 1
#define I_DEL 2
#define I_REM 4



typedef struct {
  int type;
  Mesh* mesh;
  Link** list;
  Node* face;
  Node* vertex;
  Edge* edge;
  Node* currentNode;
  Edge* currentEdge;
} MeshIterator;


/* Function definitions */


Mesh* meshparse();
#define MeshParse(fp) meshparse((fp))
Mesh* meshEtoIparse();
#define MeshEtoIParse(fp,eV,eE,eF) meshEtoIparse((fp),(eV),(eE),(eF))


/* Creation Routines */
Node* NewNode(void);
Edge* NewEdge(void);
Mesh* NewMesh(void);
Face* NewFace(void);
Vertex* NewVertex(void);
char* MeshUniqueName(Mesh* m, char* s);

/* */
void SetInternalData(Vertex* v, void* d);
void* ReturnInternalData(Vertex* v);
void SetName(void* v, char* n);
char* ReturnName(void* v);


/* Predicates */
BOOLEAN BoundaryVertex(Vertex* v);
BOOLEAN BoundaryEdge(Edge* e);
BOOLEAN BoundaryFace(Face* f);



/* Iterators */
void InitMeshFaceIterator(Mesh* mesh, MeshIterator* iterator);
Face* NextMeshFace(MeshIterator* iterator);
void InitMeshVertexIterator(Mesh* mesh, MeshIterator* iterator);
Vertex* NextMeshVertex(MeshIterator* iterator);

void InitFaceVertexIterator(Face* face, MeshIterator* iterator);
Vertex* NextFaceVertex(MeshIterator* iterator);
void InitFaceEdgeIterator(Face* face, MeshIterator* iterator);
Edge* NextFaceEdge(MeshIterator* iterator);
void InitFaceFaceIterator(Face* face, MeshIterator* iterator);
Face* NextFaceFace(MeshIterator* iterator);

void InitVertexFaceIterator(Vertex* vertex, MeshIterator* iterator);
Face* NextVertexFace(MeshIterator* iterator);
void InitVertexEdgeIterator(Vertex* vertex, MeshIterator* iterator);
Edge* NextVertexEdge(MeshIterator* iterator);
void InitVertexVertexIterator(Vertex* vertex, MeshIterator* iterator);
Vertex* NextVertexVertex(MeshIterator* iterator);

void InitMeshEdgeIterator(Mesh* mesh, MeshIterator* iterator);
Edge* NextMeshEdge(MeshIterator* iterator);

Vertex* VertexPath(Vertex* v1, Vertex* v2, char* path);

/* Iterator macros */

#define ForeachMeshFace(m,f) \
	{MeshIterator iter; for(InitMeshFaceIterator((m),&iter); \
			     (f)=NextMeshFace(&iter); ) {


#define ForeachMeshVertex(m,v) \
	{MeshIterator iter; for(InitMeshVertexIterator((m),&iter); \
			     (v)=NextMeshVertex(&iter); ) {

#define ForeachMeshEdge(m,e) \
	{MeshIterator iter; for(InitMeshEdgeIterator((m),&iter); \
			     (e)=NextMeshEdge(&iter); ) {


#define ForeachFaceFace(f1,f2) \
	{MeshIterator iter; for(InitFaceFaceIterator((f1),&iter); \
			     (f2)=NextFaceFace(&iter); ) {

#define ForeachFaceVertex(f,v) \
	{MeshIterator iter; for(InitFaceVertexIterator((f),&iter); \
			     (v)=NextFaceVertex(&iter); ) {

#define ForeachFaceEdge(f,e) \
	{MeshIterator iter; for(InitFaceEdgeIterator((f),&iter); \
			     (e)=NextFaceEdge(&iter); ) {


#define ForeachVertexFace(v,f) \
	{MeshIterator iter; for(InitVertexFaceIterator((v),&iter); \
			     (f)=NextVertexFace(&iter); ) {

#define ForeachVertexEdge(v,e) \
	{MeshIterator iter; for(InitVertexEdgeIterator((v),&iter); \
			     (e)=NextVertexEdge(&iter); ) {

#define ForeachVertexVertex(v1,v2) \
	{MeshIterator iter; for(InitVertexVertexIterator((v1),&iter); \
			     (v2)=NextVertexVertex(&iter); ) {


#define EndForeach }}

#endif /* _MESH_H_ */
