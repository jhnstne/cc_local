/*
 *----------------------------------------------------------------------
 *  File:  addremove.c
 *
 *	Routines to add and remove vertices, faces, and edges to a mesh.  
 *	The item is added to the mesh iff it is non-Null and if it is not 
 *	on the mesh already.  It is removed from the mesh if it is on the
 *	mesh lists.
 *	Return TRUE if added (removed), FALSE if not added.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "grailTypes.h"
#include "dstruct.h"
#include "mesh.h"
#include "operators.h"



BOOLEAN AddMeshVertex( Mesh *m, Vertex *v, int fast )
    /* flag: non-zero if we're *sure* that edge isn't in list */
{
    Link *tra;     /* traverses mesh mesh vertex list, prevents duplicates */

    if ( QIFlags(v, I_REP) ) {
	    if ( QIFlags(v, I_DEL) ) {
		    fprintf(stderr,
			   "AddMeshVertex: trying to add a deleted vertex\n");
		    exit(1);
	    }
	    if ( QIFlags(v, I_REM) ) {
		    ClearIFlags(v, I_REM);
	    }
	    return FALSE;
    }
    SetIFlags(v, I_REP);

    AddToList(&(m->vertexList), v);

    return( TRUE );
}


/*
    Also checks for edge's sym pointer in mesh edge list; if in list, then
    won't add edge
*/
BOOLEAN AddMeshEdge( Mesh *m, Edge *e, int fast )
	/* flag: non-zero if we're *sure* that edge isn't in list */
{
    Link *tra;     /* traverses mesh mesh edge list, prevents duplicates */

    if ( QIFlags(e, I_REP) ) {
	    if ( QIFlags(e, I_DEL) ) {
		fprintf(stderr,"AddMeshEdge: trying to add deleted edge\n");
		exit(1);
	    }
	    if ( QIFlags(e, I_REM) ) {
		    ClearIFlags(e, I_REM);
	    }
	    return FALSE;
    }
    ClearIFlags(e, I_REM);
    SetIFlags(e, I_REP);

    AddToList(&(m->edgeList), e);

    return( TRUE );
}


BOOLEAN AddMeshFace( Mesh *m, Face *f, int fast )
	/* flag: non-zero if we're *sure* that edge isn't in list */
{
    Link *tra;     /* traverses mesh mesh face list, prevents duplicates */

    if ( QIFlags(f, I_REP) ) {
	    if ( QIFlags(f, I_DEL|I_REM) ) {
		    ClearIFlags(f, I_DEL|I_REM);
	    }
	    return FALSE;
    }
    SetIFlags(f, I_REP);

    AddToList(&(m->faceList), f);

    return( TRUE );
}


BOOLEAN RemoveMeshVertex( Mesh *m, Vertex *v )
{
    if ( v == NULL ) {
	    return 0;
    }
    SetIFlags(v, I_REM);
    return 1;
}


BOOLEAN RemoveMeshEdge( Mesh *m, Edge *e )
{
    if ( e == NULL ) {
	    return 0;
    }
    if ( QIFlags(e, I_REP) ) {
	    if ( !QIFlags(e, I_REM|I_DEL) ) {
		    SetIFlags(e, I_REM);
	    }
	    return;
    }
    /* a bit unclear what we should do if not REP; for now, do
       nothing */
    return 1;
}


BOOLEAN RemoveMeshFace( Mesh *m, Face *f )
{
    if ( f == NULL ) {
	    return 0;
    }
    SetIFlags(f, I_REM);
    return 1;
}

void DeleteMeshVertex( Mesh* m, Vertex* v )
{
	if ( v != NULL ) {
		SetIFlags(v, I_DEL);
		v->tag = 0;
		v->rep = 0;
	}
}

void DeleteMeshFace( Mesh* m, Face* f )
{
	if ( f != NULL ) {
		SetIFlags(f, I_DEL);
		f->tag = 0;
		f->rep = 0;
	}
}

void DeleteMeshEdge( Mesh* m, Edge* e )
{
	if (debugLevel(3))fprintf(stderr,"DME(%d)\n",e);
	if ( QIFlags(e, I_REP) ) {
		SetIFlags(e, I_DEL);
	} else {
		if ( e->sym != NULL  &&  e->sym->sym == e ) {
			e->sym->sym = NULL;
		}
		FreeEdge(e);
	}
}
