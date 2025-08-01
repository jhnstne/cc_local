/* Do a 3 to 1 split on the face f of mesh h
   by introducing a new vertex at the centroid of the face */
Vertex *SplitFace3to1(Mesh *m, Face *f )
{
	Edge *e,*ea[3],*enew1,*enew12;
	Vertex *v0,*v1,*v2,*vnew1,*vnew2;
	Face *fnew,*f1,*f2;
	Point p,p0,p1,p2;
	int i=0,debug=1;
	assertx(facenumsides(f)==3);
	ForeachFaceEdge(f,e) {
		ea[i++]=e;
	} EndForeach;
	GetEdgeVertices(ea[1],&v0,&v1);
	GetEdgeVertices(ea[2],&v1,&v2);
	assertx(v0 && v1 && v2);
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	vnew1=AddVertexEdge(m,ea[1]); /* v0 to v1 */
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	assertx(fnew=SplitFace(m,f,v2,vnew1));
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	/* fnew is next to v1, f is next to v0 */
	GetVertexEdge(v2,vnew1,&enew1);
	assertx(enew1);
	vnew2=AddVertexEdge(m,enew1);
	assertx(vnew2);
	assertx(GetUDPoint(v0,&p0));
	assertx(GetUDPoint(v1,&p1));
	assertx(GetUDPoint(v2,&p2));
	p=PPac3(p0,p1,p2,1./3.,1./3.,1./3.);
	vnew2->name=uniquename();
	assertx(SetUDPoint(vnew2,p));
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	assertx(SplitFace(m,f,v0,vnew2));
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	assertx(SplitFace(m,fnew,vnew2,v1));
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	GetVertexEdge(vnew1,vnew2,&enew12);
	assertx(enew12);
	GetEdgeFaces(enew12,&f1,&f2);
	assertx(f1==f || f1==fnew);
	assertx(f2==f || f2==fnew);
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	assertx(JoinFace(m,f,enew12,fnew));
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	assertx(CollapseEdge(m,v0,vnew1)==vnew1);
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	assertx(DestroyVertex(m,vnew1));
	if (debug) assertx(ValidMesh(m));	/* temporary ?? */
	return (vnew2);
}
