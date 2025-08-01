/*
    Removes a vertex from existence.  The vertex must be disconnected from all
    faces and edges.
    Returns TRUE if vertex remove, FALSE if vertex not removed.
*/

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"

BOOLEAN DestroyVertex(Mesh *m, Face *v )
{
    if ( v->rep != NULL ) {
	return FALSE;
    }

    DeleteMeshVertex(m, v);

    if ( getenv("SDEBUG")  &&  !ValidMesh(m) ) {
	    fprintf(stderr,"Invalid leaving DestroyVertex\n");
    }
    return( TRUE );
}    
