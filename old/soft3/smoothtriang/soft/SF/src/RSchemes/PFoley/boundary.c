/*
 *----------------------------------------------------------------------
 *  File:  boundary.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"


/*
 *----------------------------------------------------------------------
 *  Function:  UnitNormPerpTo_InPlaneOf
 *----------------------------------------------------------------------
 */
Vector UnitNormPerpTo_InPlaneOf(V1, V2)
Vector V1, V2;
{
  Vector R;

  R = VVCross(V1,V2);
  R = VVCross(R,V1);
  R = SVMult(1.0/VMag(R), R);
  return R;
}


/*
 *----------------------------------------------------------------------
 *  Function:  ChiyokuraKimura
 *----------------------------------------------------------------------
 */
void ChiyokuraKimura(S0,S1,T0, T3,S2,S3, T1,T2)
Point S0, S1, T0, T3, S2, S3;
Point *T1, *T2;
{
  Vector A0, A1, A2, A3;
  Vector B0, B1, B2, B3;
  Vector C0, C1, C2;
  Scalar h0,h1,k0,k1,z;
  Frame f;
  extern int new;

  B0 = PPDiff(T0, S0);
  B3 = PPDiff(T3, S3);
  C0 = PPDiff(S1, S0);
  C1 = PPDiff(S2, S1);
  C2 = PPDiff(S3, S2);

  A0 = UnitNormPerpTo_InPlaneOf(C0, B0);
  A3 = UnitNormPerpTo_InPlaneOf(C2, B3);
  A1 = VVAdd( SVMult( 2.0/3.0, A0 ),  SVMult( 1.0/3.0, A3 ) );
  A2 = VVAdd( SVMult( 1.0/3.0, A0 ),  SVMult( 2.0/3.0, A3 ) );

  /* extract h0,h1,k0,k1 */
  f = FCreate("f1",S0,A0,C0,VVCross(A0,C0));
  VCoords(B0,f,&k0,&h0,&z);
  f = FCreate("f1",S3,A3,C2,VVCross(A3,C2));
  VCoords(B3,f,&k1,&h1,&z);

  /* calc B1, B2 */
  B1 = VVAdd( VVAdd( SVMult((k1-k0)/3.0,A0), SVMult(k0, A1)),
	      VVAdd( SVMult(2.0*h0/3.0, C1), SVMult(h1/3.0,C0)));
  B2 = VVAdd( VVAdd( SVMult(k1,A2), SVMult((k0-k1)/3.0,A3)),
	      VVAdd( SVMult(h0/3.0,C2), SVMult(2.0*h1/3.0, C1)));


  *T1 = PVAdd(S1, B1);
  *T2 = PVAdd(S2, B2);

}


/*
 *----------------------------------------------------------------------
 *  Function:  Jensen
 *----------------------------------------------------------------------
 */
void Jensen(S0,S1,T0, T3,S2,S3, T1,T2, c)
Point S0, S1, T0, T3, S2, S3;
Point *T1, *T2;
Scalar c;
{
  Vector A0, A1, A2, A3;
  Vector B0, B1, B2, B3;
  Vector C0, C1, C2;
  Scalar h0,h1,k0,k1,z;
  Frame f;
  extern int new;

  B0 = PPDiff(T0, S0);
  B3 = PPDiff(T3, S3);
  C0 = PPDiff(S1, S0);
  C1 = PPDiff(S2, S1);
  C2 = PPDiff(S3, S2);

  A0 = UnitNormPerpTo_InPlaneOf(C0, B0);
  A3 = UnitNormPerpTo_InPlaneOf(C2, B3);
  A1 = VVAdd( SVMult( 2.0/3.0, A0 ),  SVMult( 1.0/3.0, A3 ) );
  A2 = VVAdd( SVMult( 1.0/3.0, A0 ),  SVMult( 2.0/3.0, A3 ) );

  /* extract h0,h1,k0,k1 */
  f = FCreate("f1",S0,A0,C0,VVCross(A0,C0));
  VCoords(B0,f,&k0,&h0,&z);
  f = FCreate("f1",S3,A3,C2,VVCross(A3,C2));
  VCoords(B3,f,&k1,&h1,&z);

  /* calc B1, B2 */
  c = c - .5;
  B1 = VVAdd(VVAdd( SVMult( 1./3. * k0, A3 ), SVMult( 2./3.* (c*k0+c*k1), A0)),
	     VVAdd( SVMult( 2./3. * h0, C1 ), SVMult( 1./3. * h1, C0 ) ));

  B2 = VVAdd(VVAdd( SVMult( 1./3. * k1, A3 ), SVMult( 2./3.* (c*k0+c*k1), A0)),
	     VVAdd( SVMult( 1./3. * h0, C2 ), SVMult( 2./3.*h1, C1 ) ));

  *T1 = PVAdd(S1, B1);
  *T2 = PVAdd(S2, B2);

}

