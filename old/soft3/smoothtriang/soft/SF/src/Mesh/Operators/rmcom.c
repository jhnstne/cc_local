#include <stdio.h>
#include "all.h"

#define assertx(A) if ( !(A) ) fprintf(stderr,"assert failed %d\n",__LINE__);

/* Remove connected component with representative vertex v from mesh. */
static removecomponent(m, v)
Mesh* m;
Vertex *v;
{
	Vertex *v1,*v2,*va[3];
	Edge *e;
	Face *f,*fa[2];
	int i,nf,nv,failure;
	int count=0;
	for (;;) {
		v1=0;
		ForeachVertexEdge(v,e) {
			assertx(e);
			GetEdgeVertices(e,&v1,&v2);
			assertx(v1 && v2);
			failure=0;
			if (JoinVertices(m,v1,v2,1)) {
				fprintf(stderr,"JoinV %d\n",count++);
				break;
			}
			failure=1;
		} EndForeach;
		if ( debugLevel(2) ) {
			ConvertGeometryToExternal(m);
			WriteMesh(m,stderr);
			fprintf(stderr,"v1=%d\n",v1);
		}
#if 1
		if ( count == 1 ) {
			FILE* fp;
			ConvertGeometryToExternal(m);
			fp = fopen("redCat.m","w+");
			WriteMesh(m,fp);
			fclose(fp);
		}
#else
		ConvertGeometryToExternal(m);
		WriteMesh(m, stdout);
#endif
		if (!v1) {
			/* only one vertex remains (component must have had
			   zero faces) */
			DestroyVertex(m,v);
			return;
		}
		if (failure)
			break;
		v=v1;
	}
	/* vertex v has no adjacent edge that can be collapsed */
	i=0;
	ForeachVertexFace(v,f) {
		assertx(i<2);
		fa[i++]=f;
	} EndForeach;
	nf=i;
	assertx(nf>0);
	i=0;
	ForeachFaceVertex(fa[0],v) {
		assertx(i<3);
		va[i++]=v;
	} EndForeach;
	nv=i;
	assertx(nv==3);

	for (i=0;i<nf;i++) {
		assertx(DestroyFace(m,fa[i]));
	}
	for (i=0;i<nv;i++)
		assertx(DestroyVertex(m,va[i]));
}

main()
{
	Mesh* m;
	Vertex* v;
	Edge* e;
	Space world;
	Frame wf;

	m = MeshParse(stdin);
	world = SCreate("hi",3);
	AddGeometry(world, m);
	ForeachMeshVertex(m, v) {
		fprintf(stderr,"rmcom(%d)\n",v);
		removecomponent(m, v);
		fprintf(stderr,"component removed\n");
		WriteMesh(m, stderr);
		if (1) break;
	} EndForeach;
	
}

DumpMesh(m)
Mesh* m;
{
	Vertex* v;
	Edge* e;
	Face* f;

	ForeachMeshVertex(m, v) {
		fprintf(stderr,"V %s %d rep = %d\n",v->name,v,v->rep);
	} EndForeach;
	ForeachMeshFace(m, f) {
		fprintf(stderr,"F %d rep = %d\n",f,f->rep);
		ForeachFaceEdge(f, e) {
			fprintf(stderr,"\t%d ->p %s %d %d\n",
				e, e->p->name,e->p,e->d);
		} EndForeach;
	} EndForeach;
	ForeachMeshEdge(m, e) {
		fprintf(stderr,"%d \n",e);
		fprintf(stderr,"\t%s %d\n",e->p->name,e->p);
		fprintf(stderr,"\tsym %d\n",e->sym);
		fprintf(stderr,"\tnext %d\n",e->next);
		fprintf(stderr,"\tprev %d\n",e->prev);
		if ( e->sym ) {
			fprintf(stderr,"%d \n",e->sym);
			fprintf(stderr,"\t%s %d\n",e->sym->p->name,e->sym->p);
			fprintf(stderr,"\tsym %d\n",e->sym->sym);
			fprintf(stderr,"\tnext %d\n",e->sym->next);
			fprintf(stderr,"\tprev %d\n",e->sym->prev);
		}
	} EndForeach;
}
