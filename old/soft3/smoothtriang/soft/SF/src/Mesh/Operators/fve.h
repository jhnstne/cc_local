/* in mesh.h:
*/
void InitVertexEdgeIterator();
Edge *NextVertexEdge();

/*
    NOTE: the VertexEdge iterator returns all edges pointing to the vertex;
          if the vertex is on a boundary, then it will not return the edge
 	  on the boundary pointing away from the vertex.
*/
/*
#define ForeachVertexEdge( v, e )					\
        { MeshIterator iter; for ( InitMeshVertexEdge( (v), &iter );	\
				    (e) = NextVertexEdge( &iter ); ) {

*/




Vertex *newVertex();
Face *newFace();
Edge *newEdge();

