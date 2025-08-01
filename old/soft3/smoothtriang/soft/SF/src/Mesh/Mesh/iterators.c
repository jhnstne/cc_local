/*
 *----------------------------------------------------------------------
 *  File:  iterators.c
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include <strings.h>
#include <ctype.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"


#define MESH_FACE 1
#define MESH_VERTEX 2
#define MESH_EDGE 3
#define FACE_VERTEX 4
#define VERTEX_FACE 5
#define VERTEX_EDGE 6
#define VERTEX_VERTEX 7
#define FACE_EDGE 8
#define FACE_FACE 9

/*
 *----------------------------------------------------------------------
 *  Function:  InitMeshFaceIterator
 *	Initialize the mesh face iterator.  This iterator will
 *  give all the faces in a mesh.
 *----------------------------------------------------------------------
 */
void InitMeshFaceIterator(Mesh* mesh, MeshIterator* iterator)
{
  iterator->mesh = mesh;
  iterator->list = &(mesh->faceList);
  iterator->type = MESH_FACE;
}


/*
 *----------------------------------------------------------------------
 *  Function:  NextMeshFace
 *	Give the next face in the mesh.
 *----------------------------------------------------------------------
 */
Face* NextMeshFace(MeshIterator* iterator)
{
  Face* returnFace;

  if ( iterator->type != MESH_FACE ) {
    fprintf(stderr,"NextMeshFace(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( *(iterator->list) == NULL  ) {
    return NULL;
  }
  do {
    returnFace = (Face*)((*(iterator->list))->data);
    if ( returnFace  &&  QIFlags(returnFace, I_DEL | I_REM) ) {
      Link* l;

      l = *(iterator->list);
      *(iterator->list) = l->next;
      if ( *(iterator->list) == NULL  ) {
        return NULL;
      }
      /* get rid of l */
      FreeFace((Face*)(l->data));
      free(l);
      returnFace = NULL;
    } else {
      iterator->list = &((*(iterator->list))->next);
    } 
  } while ( returnFace == NULL );
  return returnFace;
}






/*
 *----------------------------------------------------------------------
 *  Function:  InitMeshVertexIterator
 *	Initialize the mesh vertex iterator.  This iterator will
 *  give all the vertices in a mesh.
 *----------------------------------------------------------------------
 */
void InitMeshVertexIterator(Mesh* mesh, MeshIterator* iterator)
{
  iterator->mesh = mesh;
  iterator->list = &(mesh->vertexList);
  iterator->type = MESH_VERTEX;
}


/*
 *----------------------------------------------------------------------
 *  Function:  NextMeshVertex
 *	Give the next vertex in the mesh.
 *----------------------------------------------------------------------
 */
Vertex* NextMeshVertex(MeshIterator* iterator)
{
  Vertex* returnVertex=NULL;

  if ( iterator->type != MESH_VERTEX ) {
    fprintf(stderr,"NextMeshVertex(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( *(iterator->list) == NULL  ) {
    return NULL;
  }
  do {
    returnVertex = (Vertex*)((*(iterator->list))->data);
    if ( returnVertex  &&  QIFlags(returnVertex, I_DEL | I_REM) ) {
      Link* l;

      l = *(iterator->list);
      *(iterator->list) = l->next;
      if ( *(iterator->list) == NULL  ) {
        return NULL;
      }
      /* get rid of l */
      FreeVertex((Vertex*)(l->data));
      free(l);
      returnVertex = NULL;
    } else {
      iterator->list = &((*(iterator->list))->next);
    }
  } while ( returnVertex == NULL );
  return returnVertex;
}


/*
 *----------------------------------------------------------------------
 *  Function:  InitMeshEdgeIterator
 *	Initialize the mesh edge iterator.  This iterator will
 *  give all the vertices in a mesh.
 *----------------------------------------------------------------------
 */
void InitMeshEdgeIterator(Mesh* mesh, MeshIterator* iterator)
{
  iterator->mesh = mesh;
  iterator->list = &(mesh->edgeList);
  iterator->type = MESH_EDGE;
}


/*
 *----------------------------------------------------------------------
 *  Function:  NextMeshEdge
 *	Give the next edge in the mesh.
 *----------------------------------------------------------------------
 */
Edge* NextMeshEdge(MeshIterator* iterator)
{
  Edge* returnEdge;

  if ( iterator->type != MESH_EDGE ) {
    fprintf(stderr,"NextMeshEdge(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( *(iterator->list) == NULL  ) {
    return NULL;
  }
  do {
    returnEdge = (Edge*)((*(iterator->list))->data);
    if ( returnEdge  &&  QIFlags(returnEdge, I_DEL | I_REM) ) {
      Link* l;

      if ( debugLevel(1) ) {
	      fprintf(stderr,"Remove edge %d (%d) (%d)\n",returnEdge,returnEdge->sym,returnEdge->sym!=NULL?returnEdge->sym->sym:0);
	      fprintf(stderr,"\t%d\n",QIFlags(returnEdge,I_DEL|I_REM));
	      if ( returnEdge->sym ) 
		fprintf(stderr,"\t%d %d\n",returnEdge->sym,
			QIFlags(returnEdge->sym,7));
      }
      ClearIFlags(returnEdge, I_REP);
      l = *(iterator->list);
      *(iterator->list) = l->next;
      if ( *(iterator->list) == NULL  ) {
        return NULL;
      }
      /* get rid of l */
      if ( QIFlags(returnEdge, I_DEL) ) {
	      FreeEdge(returnEdge);
      }
      free(l);
      returnEdge = NULL;
    } else {
      iterator->list = &((*(iterator->list))->next);
    }
  } while ( returnEdge == NULL );
  return returnEdge;
}






/*
 *----------------------------------------------------------------------
 *  Function:  InitFaceVertexIterator
 *	Initiaize the face vertex iterator.  This iterator will
 *  give all the vertices surrounding a face.  Faces are assumed
 *  to be complete.  That is, the vertices neighboring a face are
 *  assumed to form a ring.
 *----------------------------------------------------------------------
 */
void InitFaceVertexIterator(Face* face, MeshIterator* iterator)
{
  iterator->face = face;
  iterator->type = FACE_VERTEX;
  iterator->currentEdge = iterator->edge = face->rep;
  iterator->currentNode = iterator->edge->p;
}





/*
 *----------------------------------------------------------------------
 *  Function:  NextFaceVertex
 *	Give the next vertex neighboring a face.
 *----------------------------------------------------------------------
 */
Vertex* NextFaceVertex(MeshIterator* iterator)
{
  Vertex* returnVertex;

  if ( iterator->type != FACE_VERTEX ) {
    fprintf(stderr,"NextFaceVertex(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( iterator->currentNode == NULL ) {
    return NULL;
  }

  returnVertex = iterator->currentNode;
  if ( iterator->currentEdge->next == iterator->edge ) {
    iterator->currentNode = NULL;
  } else {
    iterator->currentEdge = iterator->currentEdge->next;
    iterator->currentNode = iterator->currentEdge->p;
  }
  return returnVertex;
}




/*
 *----------------------------------------------------------------------
 *  Function:  InitFaceFaceIterator
 *	Initiaize the face face iterator.  This iterator will
 *  give all the faces surrounding a face.  Faces are assumed
 *  to be complete.  That is, the vertices neighboring a face are
 *  assumed to form a ring.
 *----------------------------------------------------------------------
 */
void InitFaceFaceIterator(Face* face, MeshIterator* iterator)
{
  int count;

  iterator->face = face;
  iterator->type = FACE_FACE;
  iterator->currentEdge = (iterator->edge = face->rep);

  /* find the first non-NULL face */
  count = 0;
  while (iterator->currentEdge->sym == NULL  &&
	 iterator->currentEdge->sym != iterator->edge){
    iterator->currentEdge = iterator->currentEdge->next;
    count++;
  }
  if ( count != 0  &&  iterator->currentEdge == iterator->edge ){
    iterator->currentNode = NULL;
  } else {
    iterator->currentNode = iterator->currentEdge->sym->d;
  }
}




/*
 *----------------------------------------------------------------------
 *  Function:  NextFaceFace
 *	Give the next face neighboring a face.
 *----------------------------------------------------------------------
 */
Face* NextFaceFace(MeshIterator* iterator)
{
  Face* returnFace;

  if ( iterator->type != FACE_FACE ) {
    fprintf(stderr,"NextFaceFace(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( iterator->currentNode == NULL ) {
    return NULL;
  }

  returnFace = iterator->currentNode;
  if ( iterator->currentEdge->next == iterator->edge ) {
    iterator->currentNode = NULL;
  } else {
    /* find the first non-NULL face */
    iterator->currentEdge = iterator->currentEdge->next;
    while (iterator->currentEdge->sym == NULL  &&
	   iterator->currentEdge->sym != iterator->edge){
      iterator->currentEdge = iterator->currentEdge->next;
    }
    if ( iterator->currentEdge == iterator->edge ){
      iterator->currentNode = NULL;
    } else {
      iterator->currentNode = iterator->currentEdge->sym->d;
    }
  }
  return returnFace;
}






/*
 *----------------------------------------------------------------------
 *  Function:  InitFaceEdgeIterator
 *	Initiaize the face edge iterator.  This iterator will
 *  give all the vertices surrounding a face.  Faces are assumed
 *  to be complete.  That is, the vertices neighboring a face are
 *  assumed to form a ring.
 *----------------------------------------------------------------------
 */
void InitFaceEdgeIterator(Face* face, MeshIterator* iterator)
{
  iterator->face = face;
  iterator->type = FACE_EDGE;
  iterator->currentEdge = iterator->edge = face->rep;
}



/*
 *----------------------------------------------------------------------
 *  Function:  NextFaceEdge
 *	Give the next edge neighboring a face.
 *----------------------------------------------------------------------
 */
Edge* NextFaceEdge(MeshIterator* iterator)
{
  Edge* returnEdge;

  if ( iterator->type != FACE_EDGE ) {
    fprintf(stderr,"NextFaceEdge(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( iterator->currentEdge == NULL ){
    return NULL;
  }

  returnEdge = iterator->currentEdge;
  if ( iterator->currentEdge->next == iterator->edge ) {
    iterator->currentEdge = NULL;
  } else {
    iterator->currentEdge = iterator->currentEdge->next;
  }
  return returnEdge;
}


/*
 *----------------------------------------------------------------------
 *  Function:  InitVertexFaceIterator
 *	Initialize the vertex face iterator.  This iterator will give
 *  all the faces surrounding a vertex.  If the vertex is a boundary
 *  vertex, its ring of neighboring faces will not be complete.  In
 *  this case, we will find the "clockwise" most face, and start our
 *  iterator with it.
 *----------------------------------------------------------------------
 */
void InitVertexFaceIterator(Vertex* vertex, MeshIterator* iterator)
{
  Edge* e;

  iterator->vertex = vertex;
  iterator->type = VERTEX_FACE;

  if ( vertex->rep == NULL ) {
    iterator->currentNode = NULL;
    return;
  }
  if ( BoundaryVertex(vertex) ){
    e = vertex->rep;
    while( e->next->sym != NULL ) {
      e = e->next->sym;
    }
    iterator->currentEdge = iterator->edge = e;
    iterator->currentNode = iterator->edge->d;
  } else /* not a boundary vertex */ {
    iterator->currentEdge = iterator->edge = vertex->rep;
    iterator->currentNode = iterator->edge->d;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  NextVertexFace
 *	Give the next face surrounding a vertex.
 *----------------------------------------------------------------------
 */
Face* NextVertexFace(MeshIterator* iterator)
{
  Face* returnFace;

  if ( iterator->type != VERTEX_FACE ) {
    fprintf(stderr,"NextVertexFace(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( iterator->currentNode == NULL ) {
    return NULL;
  }

  returnFace = iterator->currentNode;
  if ( iterator->currentEdge->sym == NULL  ||
       iterator->currentEdge->sym->prev == iterator->edge ) {
    iterator->currentNode = NULL;
  } else {
    iterator->currentEdge = iterator->currentEdge->sym->prev;
    iterator->currentNode = iterator->currentEdge->d;
  }
  return returnFace;
}






/*
 *----------------------------------------------------------------------
 *  Function:  InitVertexEdgeIterator
 *	Initialize the vertex edge iterator.  This iterator will give
 *  all the edges adjacent to a vertex.  If the vertex is a boundary
 *  vertex, we will iterate around the vertex in "clockwise" order.
 *----------------------------------------------------------------------
 */
void InitVertexEdgeIterator(Vertex* vertex, MeshIterator* iterator)
{
  Edge* e;

  iterator->vertex = vertex;
  iterator->type = VERTEX_EDGE;

  if ( vertex->rep == NULL ) {
    iterator->currentEdge = NULL;
    return;
  }
  if ( BoundaryVertex(vertex) ){
    e = vertex->rep;
    while( e->next->sym != NULL ) {
      e = e->next->sym;
    }
    iterator->currentEdge =  e;
    iterator->edge = NULL;
  } else /* not a boundary vertex */ {
    iterator->edge = vertex->rep;
    iterator->currentEdge = iterator->edge->sym->prev;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  NextVertexEdge
 *	Give the next edge adjacent to a vertex.
 *----------------------------------------------------------------------
 */
Edge* NextVertexEdge(MeshIterator* iterator)
{
  Edge* returnEdge;

  if ( iterator->type != VERTEX_EDGE ) {
    fprintf(stderr,"NextVertexEdge(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( iterator->currentEdge == NULL ) {
    return NULL;
  }

  returnEdge = iterator->currentEdge;
  if ( iterator->currentEdge == NULL  ||  /* these two conditions are 
					     the same */
       iterator->currentEdge == iterator->edge ){
    iterator->currentEdge = NULL;
  } else {
    if ( iterator->currentEdge->sym == NULL ) {
      iterator->currentEdge = NULL;
    } else {
      iterator->currentEdge = iterator->currentEdge->sym->prev;
    }
  }
  return returnEdge;
}



/*
 *----------------------------------------------------------------------
 *  Function:  InitVertexVertexIterator
 *	Initialize the vertex vertex iterator.  This iterator will give
 *  all the verticess neighboringing a vertex.  If the vertex is a boundary
 *  vertex, its ring of neighboring vertices will not be complete.  In
 *  this case, we will find the "clockwise" most vertex, and start our
 *  iterator with it.
 *----------------------------------------------------------------------
 */
void InitVertexVertexIterator(Vertex* vertex, MeshIterator* iterator)
{
  Edge* e;

  iterator->vertex = vertex;
  iterator->type = VERTEX_VERTEX;

  if ( vertex->rep == NULL ) {
    iterator->currentNode = NULL;
    return;
  }
  if ( BoundaryVertex(vertex) ){
    e = vertex->rep;
    while( e->next->sym != NULL ) {
      e = e->next->sym;
    }
    iterator->currentEdge =  e;
    iterator->edge = NULL;
    iterator->currentNode = e->next->p;
  } else /* not a boundary vertex */ {
    iterator->edge = vertex->rep;
    iterator->currentNode = iterator->edge->prev->p;
    iterator->currentEdge = iterator->edge->sym->prev;
  }
}


/*
 *----------------------------------------------------------------------
 *  Function:  NextVertexVertex
 *	Give the next vertex surrounding a vertex.
 *----------------------------------------------------------------------
 */
Vertex* NextVertexVertex(MeshIterator* iterator)
{
  Vertex* returnVertex;

  if ( iterator->type != VERTEX_VERTEX ) {
    fprintf(stderr,"NextVertexVertex(): wrong type of iterator.  Exiting.\n");
    exit(1);
  }

  if ( iterator->currentNode == NULL ) {
    return NULL;
  }

  returnVertex = iterator->currentNode;
  if ( iterator->currentEdge == NULL  ||  /* these two conditions are 
					     the same */
       iterator->currentEdge == iterator->edge ){
    iterator->currentNode = NULL;
  } else {
    iterator->currentNode = iterator->currentEdge->prev->p;
    if ( iterator->currentEdge->sym == NULL ) {
      iterator->currentEdge = NULL;
    } else {
      iterator->currentEdge = iterator->currentEdge->sym->prev;
    }
  }
  return returnVertex;
}
