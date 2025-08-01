#include <stdio.h>
#include "all.h"

main()
{
	Mesh* m;
	Space world;
	Face* f;
	Point pa[10];
	int i;
	Vertex* v;
	SFF sff;

	world = SCreate("world",3);

	m = MeshParse(stdin);
	AddGeometry(world, m);

	ForeachMeshFace(m, f) {
		i = 0;
		ForeachFaceVertex(f, v) {
			pa[i] = ReturnUDPoint(v);
			i++;
		} EndForeach;
		if ( i != 3 ) {
			fprintf(stderr,"Face has %d verts\n",i);
			exit(1);
		}
		fprintf(stderr,"%g\n",VMag(VVCross(PPDiff(pa[0],pa[1]),
						   PPDiff(pa[2],pa[1]))));
	} EndForeach;
	ForeachMeshVertex(m, v) {
		dGetSFF(v->externalData, "Point.sff", StdFrame(world), &sff);
		fprintf(stderr, "k = %g\n",EvalSFF(&sff, sff.v0));
	} EndForeach;
}
