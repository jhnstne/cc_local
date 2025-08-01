/*
 *----------------------------------------------------------------------
 *  File:  CreateVertex
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"


/*
    Creates a new vertex.
    externalData, name pointers to NULL.
    Returns the face created.
*/
Vertex *CreateVertex( Mesh *m )
{
	Vertex *v;

	v = NewVertex();
	AddMeshVertex(m, v, 1);
	return v;
}
