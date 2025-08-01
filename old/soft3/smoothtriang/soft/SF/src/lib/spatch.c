/*
 *----------------------------------------------------------------------
 *  File:  spatch.c
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "getset.h"
#include "mesh.h"
#include "userData.h"
#include "operators.h"
#include "material.h"
#include "vertex.h"
#include "patch.h"
#include "util.h"
#include "commandline.h"
#include "usage.h"
#include "sff.h"
#include "spatch.h"
#include "mindex.h"

typedef struct SPD {
	Lnode* ed;
	double d[2];
	struct SPD* next;
	SFF sff;
} SPData;

SetVertexDomain(Vertex* v, Face* f, double d[2])
{
	SPData* ptr;

	ptr = (SPData*)malloc(sizeof(SPData));
	if ( v->internalData == NULL ) {
		v->internalData = UDMalloc();
	}
	ptr->next = (SPData*)(v->internalData->ptr);
	v->internalData->ptr = (void*)ptr;
	ptr->ed = f->externalData;
	ptr->d[0] = d[0];	ptr->d[1] = d[1]; 
}

GetVertexDomain(Vertex* v, Face* f, double d[2])
{
	SPData* ptr;

	for (ptr=(SPData*)(v->internalData->ptr); 
	     ptr != NULL; 
	     ptr = ptr->next) {
		if ( ptr->ed == f->externalData ) {
			d[0] = ptr->d[0];
			d[1] = ptr->d[1];
			return;
		}
	}
	fprintf(stderr, "GetVertexDomain: vertex %s has no domain\n",
		ReturnName(v));
	fprintf(stderr, "                 associated with face %s\n", 
		ReturnName(f));
	exit(1);
}

SetVertexSFF(Vertex* v, Face* f, SFF sff)
{
	SPData* ptr;
/*fprintf(stderr,"SVSFF(%s, %s = %d)\n",ReturnName(v),ReturnName(f),
f->externalData);*/
	for (ptr=(SPData*)(v->internalData->ptr); 
	     ptr != NULL; 
	     ptr = ptr->next) {
		if ( ptr->ed == f->externalData ) {
			ptr->sff = sff;
			return;
		}
	}
	fprintf(stderr, "SetVertexSFF: vertex %s is not associated\n",
		ReturnName(v));
	fprintf(stderr, "              with face %s\n", 
		ReturnName(f));
	exit(1);
}

GetVertexSFF(Vertex* v, Face* f, SFF* sff)
{
	SPData* ptr;
/*fprintf(stderr,"GVSFF(%s, %s = %d)\n",ReturnName(v),ReturnName(f),
f->externalData);*/
	for (ptr=(SPData*)(v->internalData->ptr); 
	     ptr != NULL; 
	     ptr = ptr->next) {
		if ( ptr->ed == f->externalData ) {
			*sff = ptr->sff;
			return;
		}
	}
	fprintf(stderr, "GetVertexSFF: vertex %s is not associated\n",
		ReturnName(v));
	fprintf(stderr, "              with face %s\n", 
		ReturnName(f));
	exit(1);
}

UseVertexSFF(Vertex* v, Face* f)
{
	SPData* ptr;
	SFF sff;

	if ( !SpatchSurface() ) {
		return;
	}
	for (ptr=(SPData*)(v->internalData->ptr); 
	     ptr != NULL; 
	     ptr = ptr->next) {
		if ( ptr->ed == f->externalData ) {
			Normal n;

			sff = ptr->sff;
			pPutSFF(&(v->externalData), "Point.sff", &sff);
			if ( GetUDNormal(v, &n) ) {
				if ( VMag(VVCross(VNormalize(VVCross(sff.v0,sff.v1)),
					  NDual(n))) > 1e-4 ) {
					fprintf(stderr,"Warning:UVS: adjusting normal for vertex %s.\n",
						ReturnName(v));
					SetUDNormal(v, 
						    VDual(VNormalize(VVCross(sff.v0, sff.v1))));
				}
			}
			return;
		}
	}
	fprintf(stderr, "UseVertexSFF: vertex %s is not associated\n",
		ReturnName(v));
	fprintf(stderr, "              with face %s=%d\n", 
		ReturnName(f),f->externalData);
	exit(1);
}


static Vertex* va[100];
static Point pa[100];

/* We need to determine the corresponence between the Spatch corner
   vertices and the vertices of the face.  The integer that we return
   is the vertex of the face that corresponds to the "first" corner
   of the Spatch */
/* Let us hope that both have the same clockwise orientation */
static int spatchFaceCorrespondance(Spatch* sp, Face* f)
{
	MI mi;
	Vertex* v;
	int i,v0;

	mi = InitMIndex();
	ForeachFaceVertex(f, v) {
		va[i++] = v;
	} EndForeach;

	AllocIndex(&mi, sp->s-1);
	for (i=0; i<sp->s; i++) {
		mi.i[i] = sp->d;
		pa[i] = sp->net[MtoI(mi)];
		mi.i[i] = 0;
	}
	for (i=0; i<sp->s; i++) {
		if ( PPDist(pa[0], ReturnUDPoint(va[i])) < 1e-4 ) {
			break;
		}
	}
	if ( i==sp->s ) {
		fprintf(stderr, "spatchFaceCorrespondance: face %s not attached to spatch.\n");
		exit(1);
	}
	v0 = i;
	FreeMIndex(&mi);
	return v0;
}

static void addVertexData(Vertex* v, Face* f, Spatch* sp, Scalar d[2])
{
	Point p;
	Normal n;
	SFF sff;
	Point p1;
	Normal n1;

	SetVertexDomain(v, f, d);
	EvalSpatchWNormalSFF(*sp, d, &p, &n, &sff, 1);
	SetVertexSFF(v, f, sff);
	if ( GetUDPoint(v, &p1) ) {
		if ( PPDist(p, p1) > 1e-4 ) {
			fprintf(stderr, "aVD: positions differ.\n");
			exit(1);
		}
	} else {
		SetUDPoint(v, p);
		dPutVertexPosition(v->externalData, 
				   "Point",
				   ReturnUDPoint(v));
	}

	if ( GetUDNormal(v, &n1) ) {
		if ( VMag(VVCross(NDual(n), NDual(n1))) > 1.1e-4 ) {
			fprintf(stderr, "aVD: normals differ by %g.\n",
				VMag(VVCross(NDual(n), NDual(n1))));
			exit(1);
		}
	} else {
		SetUDNormal(v, n);
		dPutVertexNormal(v->externalData, 
				   "Point",
				   ReturnUDNormal(v));
	}
}


/* corr give the correspondance between the Spatch vertices and the Face
   vertices */
static void addCornerData(Face* f, Spatch* sp, int corr)
{
	Vertex* v;
	int i;

	for (i=0; i<sp->s; i++) {
		Scalar d[2];

		d[0] = cos(2*M_PI*i/sp->s);
		d[1] = sin(2*M_PI*i/sp->s);
		addVertexData(va[(i+corr)%sp->s], f, sp, d);
	}

	ForeachFaceVertex(f, v) {
		SetUDGeoFlags(v, G_SPATCH_BOUNDARY);
	} EndForeach;
}

/* take a degree d < 3 curve and make it degree 3 */
static degreeRaiseSpatchBoundary(Point b[4], int d)
{
	switch(d) {
	      case 1:
		b[3] = b[1];
		b[2] = PPac(b[1], b[0], 2./3.);
		b[1] = PPac(b[0], b[1], 2./3.);
		return;
	      case 2:
		b[3] = b[2];
		b[2] = PPac(b[2], b[1], 1./3.);
		b[1] = PPac(b[0], b[1], 1./3.);
		return;
	      default:
		return;
	}
}


setSpatchBezierBoundary(Point b[], Edge* e, Spatch sp)
{
	int i;

	if ( e->internalData == NULL ) {
		e->internalData = UDEMalloc();
	}
	if ( ((EData*)(e->internalData))->bc == NULL ) {
		((EData*)(e->internalData))->bc = 
		    (Point *) malloc (sizeof(Point) * (sp.d+1));
		for (i=0; i<=sp.d; i++) {
			((EData*)(e->internalData))->bc[i] = b[i];
		}
		((EData*)(e->internalData))->deg = sp.d;
	} 
	if ( e->sym != NULL  &&  ((EData*)(e->sym->internalData))->bc != NULL){
		for (i=0; i<=sp.d; i++) {
			if ( PPDist(((EData*)(e->sym->internalData))->bc[i], 
				    b[sp.d-i]) > 1e-4 ) {
				fprintf(stderr, "sSBB: boundaries don't match.\n");
				break;
			}
		}
	}
}

static void setSpatchBoundaryData(Spatch sp, Face* f, int corr)
{
	int i,j;
	MI mi;
	Point bp[20];
	Edge* e;

	ForeachFaceEdge(f, e) {
		if ( sp.d <= 3 ) {
			if ( ReturnEdgeType(e) == ET_BEZIER ) {
				free(((EData*)(e->sym->internalData))->bc);
				((EData*)(e->sym->internalData))->bc = NULL;
				ForceEdgeType(e, ET_CUBIC);
			} else {
				SetEdgeType(e, ET_CUBIC);
			}
		} else {
			if ( ReturnEdgeType(e) != ET_CUBIC ) {
				SetEdgeType(e, ET_BEZIER);
			}
		}
	} EndForeach;

	if ( sp.s > 19 ) {
		fprintf(stderr, "setSpatchBoundaryData: can't handle > 19 sides.\n");
		exit(1);
	}
	mi = InitMIndex();
	AllocIndex(&mi, sp.s-1);
	for (i=0; i<sp.s; i++) {
		mi.i[i] = sp.d;
		for (j=0; j<=sp.d; j++) {
			mi.i[i] -= j;
			mi.i[(i+1)%sp.s] += j;
			bp[j] = sp.net[MtoI(mi)];
			mi.i[i] += j;
			mi.i[(i+1)%sp.s] -= j;
		}
		mi.i[i] = 0;
		GetVertexEdge(va[(i+corr)%sp.s], va[(i+corr+1)%sp.s], &e);
		if ( sp.d < 3 ) {
			degreeRaiseSpatchBoundary(bp, sp.d);
			SetEdgeP1(e, bp[1]);
			SetEdgeP2(e, bp[2]);
		} else if ( sp.d == 3 ) {
			SetEdgeP1(e, bp[1]);
			SetEdgeP2(e, bp[2]);
		} else if ( ReturnEdgeType(e) == ET_BEZIER ) {
			setSpatchBezierBoundary(bp, e, sp);
		}
	}
}

InitSpatchMesh(Mesh* m, Frame frame)
{
	Face* f;
	Face* f2;
	Lnode* ed;
	Vertex* v;
	Spatch sp;
	Scalar d[2];
	int corr;
	Edge* e;

	d[0] = d[1] = 0.;
	ForeachMeshFace(m, f) {
		ed = f->externalData;
		if ( !dQueryDstructPath(ed, "Face.Spatch") ) {
			fprintf(stderr, "InitSpatchMesh: face %s has no spatch.\n",
				ReturnName(f));
			exit(1);
		}
		if ( !dGetSpatch(ed, "Face", &sp, frame) ) {
			fprintf(stderr, "dGetSpatch failed.\n");
			exit(1);
		}
		corr = spatchFaceCorrespondance(&sp, f);
		addCornerData(f, &sp, corr);
		setSpatchBoundaryData(sp, f, corr);
		if ( numSides(f) != 3  ||  BoundaryFace(f) ) {
			v = CenterSplitFace(m, f);
			ForeachVertexEdge(v, e) {
				SetEdgeType(e, ET_SECOND_ORDER);
			} EndForeach;

			ForeachVertexFace(v, f2) {
				f2->externalData = ed;
				if ( ReturnName(f2) == NULL ) {
					SetName(f2, MeshUniqueName(m, "fn"));
				}
			} EndForeach;
			SetName(v, MeshUniqueName(m, "vn"));
			addVertexData(v, f, &sp, d);
		}
		FreeSpatch(sp);
	} EndForeach;
}

static setSpatchVertex(Mesh* m, Vertex* vt, Vertex* v0, Vertex* v1, Face* f)
{
	Edge* e;
	Face* f1;
	Face* f2;
	Spatch sp;
	extern Frame wf;
	double d[3][2];
	Point p;
	Normal n;
	SFF sff;
	Point p2;
	Point n2;

	dGetSpatch(f->externalData, "Face", &sp, wf);
	GetVertexDomain(v0, f, d[0]);
	GetVertexDomain(v1, f, d[1]);
	d[2][0] = (d[0][0]+d[1][0])/2;
	d[2][1] = (d[0][1]+d[1][1])/2;
	EvalSpatchWNormalSFF(sp, d[2], &p, &n, &sff, 1);
	if ( GetUDPoint(vt, &p2) ) {
		if ( PPDist(p2, p) > 1e-4 ) {
 fprintf(stderr,"WARNING: setSpatchVertex: positions differ by %g for %s\n",
	PPDist(p2, p), ReturnName(vt));
		}
	}
	if ( GetUDNormal(vt, &n2 ) ) {
		if ( VMag(VVCross(NDual(n), NDual(n2))) > 1e-4 ) {
 fprintf(stderr,"WARNING: setSpatchVertex: normals differ by %g for %s\n",
	VMag(VVCross(NDual(n), NDual(n2))),ReturnName(vt));
		}
	}
	SetUDPoint(vt, p);
	dPutVertexPosition(vt->externalData, "Point", p);
	SetUDNormal(vt, n);
	dPutVertexNormal(vt->externalData, "Point", n);
	pPutSFF(&(vt->externalData), "Point.sff", &sff);
	SetVertexDomain(vt, f, d[2]);
	SetVertexSFF(vt, f, sff);

	FreeSpatch(sp);
}

static copyBezier(Edge* e1, Edge* e2)
{
	EData* ed1;
	EData* ed2;
	int i;

	ed1 = ((EData*)(e1->internalData));
	ed2 = ((EData*)(e2->internalData));
	if ( ed2->bc != NULL ) {
		free(ed2->bc);
	}
	ed2->bc = (Point*)malloc(sizeof(Point)*(ed1->deg+1));

	ed2->deg = ed1->deg;
	for (i=0; i<=ed2->deg; i++) {
		ed2->bc[i] = ed1->bc[i];
	}
}

static evalInPlace(EData* ed, int side)
{
	int i,j;

	if ( side == 0 ) {
		for (i=0; i<ed->deg; i++) {
			for (j=0; j<ed->deg-i; j++) {
				ed->bc[j] = PPac(ed->bc[j], ed->bc[j+1], 0.5);
			}
		}
	} else {
		for (i=0; i<ed->deg; i++) {
			for (j=ed->deg; j>i; j--) {
				ed->bc[j] = PPac(ed->bc[j], ed->bc[j-1], 0.5);
			}
		}
	}
}


static splitBezierData(Edge* e1, Edge* e2)
{
	copyBezier(e1, e2);
	evalInPlace(((EData*)(e1->internalData)), 1);
	evalInPlace(((EData*)(e2->internalData)), 0);

	if (e1->sym != NULL) {
		copyBezier(e1->sym, e2->sym);
		evalInPlace(((EData*)(e1->sym->internalData)), 0);
		evalInPlace(((EData*)(e2->sym->internalData)), 1);
	}
}


SplitSpatchBoundary(Mesh* m, Edge* e, Vertex* vt, Vertex* v1, Vertex* v2, 
		    Face* f1, Face* f2, Point* curv)
{
	Edge* NextEdge();

	setSpatchVertex(m, vt, v1, v2, f1);
	if ( f2 != NULL ) {
		setSpatchVertex(m, vt, v1, v2, f2);
	}
	switch ( ReturnEdgeType(e) ) {
	      case ET_CUBIC:
		SetUDGeoFlags(vt, G_SPATCH_BOUNDARY);
		SplitCubicBoundary(vt, v1, curv);
		SetEdgeType(NextEdge(e), ET_CUBIC);
		break;
	      case ET_BEZIER:
		SetUDGeoFlags(vt, G_SPATCH_BOUNDARY);
		SetEdgeType(NextEdge(e), ET_BEZIER);
		splitBezierData(e, NextEdge(e));
		break;
	      case ET_SECOND_ORDER:
		SetEdgeType(NextEdge(e), ET_SECOND_ORDER);
		break;
	      default:
		fprintf(stderr,"SplitSpatchBoundary: unknown type.\n");
		break;
	}
}
