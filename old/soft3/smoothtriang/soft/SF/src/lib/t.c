#include <stdio.h>
#include <math.h>
#include <strings.h>
#include "all.h"

  Frame f;

main(int argc, char* argv[])
{
  Point p1,p2,p3;
  Vector v1,v2,v3;
  Normal n1,n2,n3;
  Space s;
  int num=10;
  Point ip;
  Point c1,c2;
  int i;
  Point m;
  Scalar x,y,z;

  s = SCreate("duh",3);
  f = StdFrame(s);

  if ( argc == 2 ) {
    num = atoi(argv[1]);
  }

#if 0
  p1 = PCreate(f, 0., 1., 0.);
  p2 = PCreate(f, 1., 0., 0.);
  c1 = PCreate(f, 1., 1., 0.);
  c2 = PCreate(f, 0., 0., 0.);

  for(i=0; i<=num; i++){
    m = PPac(c1, c2, (float)i/(float)num);
    v1 = PPDiff(m,p1);
    v2 = PPDiff(m,p2);
    fprintf(stderr,"%g\n",VVDot(VNormalize(v1),VNormalize(v2)));
    if ( LineIntLine(p1,v1,p2,v2,&ip) ) {
      fprintf(stderr," %d of %d: lines intersect\n",i,num);
      PCoords(m, f, &x, &y, &z);
      fprintf(stderr," (%g %g) vs ",x,y);
      PCoords(ip, f, &x, &y, &z);
      fprintf(stderr,"(%g %g)  %g\n",x,y,PPDist(m,ip));
    } else {
      fprintf(stderr," %d of %d: lines do NOT intersect\n",i,num);
      if ( LineIntersectLine(p1,v1,p2,v2,&ip) ) {
	Scalar x,y,z;
	fprintf(stderr," BUT: lines intersect\n",i,num);
	PCoords(m, f, &x, &y, &z);
	fprintf(stderr," (%g %g) vs ",x,y);
	PCoords(ip, f, &x, &y, &z);
	fprintf(stderr,"(%g %g)  %g\n",x,y,PPDist(m,ip));
      }
    }
  }
#if 0
  z = 1.0;
  p1 = PCreate(f, 0.0, 0.0, 0.0);
  p2 = PCreate(f, -1.0, 2.0, z);
  p3 = PCreate(f, 1.0, 0.0, z);

  n1 = NCreate(f, 0.0, 0.0, 1.0);
  c1 = PCreate(f, 1., 1., z);
  c2 = PCreate(f, 0., 0., z);

  for(i=0; i<=num; i++){
    m = PPac(c1, c2, (float)i/(float)num);
    v2 = PPDiff(m,p2);
    v3 = PPDiff(m,p3);
    n2 = VDual(VNormalize(VVCross(NDual(n1),v2)));
    n3 = VDual(VNormalize(VVCross(NDual(n1),v3)));
    fprintf(stderr,"Dot of normals is %g\n",VVDot(NDual(n2),NDual(n3)));
    fprintf(stderr," Dist of mid to ans = %g\n",PPDist(PVAdd(m,SVMult(-z,NDual(n1))),PPac3(p1,p2,p3,1./3.,1./3.,1./3.)));
    if ( Int3Planes(p1,n1,p2,n2,p3,n3,&ip ) ) {
      fprintf(stderr," planes intersect.\n");
      fprintf(stderr," Dist from true intersection = %g\n",
	      PPDist( ip,PVAdd(m,SVMult(-z,NDual(n1))) ));
    } else {
      fprintf(stderr," planes DON'T intersect.\n");
      fprintf(stderr," Dist of guess from true intersection = %g\n",
	      PPDist( ip,PVAdd(m,SVMult(-z,NDual(n1))) ));
    }
  }
#endif
#else
  CircArcLen(FOrg(f), FV(f,1), PVAdd(FOrg(f), FV(f,0)));
  CircArcLen(FOrg(f), VVAdd(FV(f,1),FV(f,0)), PVAdd(FOrg(f), FV(f,0)));
  for (i=0; i<10; i++) {
	  fprintf(stderr,"%d %g\n",10000000*i,1-CircArcLen(FOrg(f), VVAdd(SVMult(10000000.*i,FV(f,0)), FV(f,1)), PVAdd(FOrg(f), FV(f,0))));
  }
#endif
#if 0
  {  
  SFF s;
  Space World;
  Frame Wf;
  Vertex* v;
  Mesh* m;
  SFF sff;

  World = SCreate("World", 3);
  Wf = StdFrame(World);

  m = MeshParse(stdin);
  ForeachMeshVertex(m, v){
        if ( dGetSFF(v->externalData,"Point.sff",Wf,&sff) ) {
                printf("%g\n",SFFGaussianCurvature(&sff));
        }
  } EndForeach;
  }
#endif
}
