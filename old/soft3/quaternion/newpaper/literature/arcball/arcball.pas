unit arcball;

interface

uses
  GL;

type
  //these first few types are identical to Mike Lischke's geometry.pas
  //you can remove these definitions if you include has files
  //start geometry.pas definitions
  PAffineFltVector = ^TAffineFltVector;
  TAffineFltVector = array[0..2] of Single;
  PAffineVector = ^TAffineVector;
  TAffineVector = TAffineFltVector;
  THomogeneousFltVector = array[0..3] of Single;
  THomogeneousFltMatrix = array[0..3] of THomogeneousFltVector;
  TMatrix = THomogeneousFltMatrix;
  TVector4f = THomogeneousFltVector;

  TQuaternion = record
    case Integer of
      0: (ImagPart: TAffineVector;
        RealPart: Single);
      1: (Vector: TVector4f);
  end;
  //end of geometry.pas definitions

  Point3D = record
    x, y, z: Integer;
  end;

  TArcBall = class
  private
    FCenter: Point3D;
    FRadius: Single;
    FqNow, FqDown, FqDrag: TQuaternion;
    FvNow, FvDown, FvFrom, FvTo, FvrFrom, FvrTo: TQuaternion;
    FmNow, FmDown: TMatrix;
    FDragging: Boolean;
    procedure DrawArc(vFrom, vTo: TVector4f);
  public
    procedure Init(radius: Single = 1);
    procedure PointRotateArcBall(var lPoint3D: Point3D);
    procedure BeginDrag;
    procedure EndDrag;
    procedure Update;
    procedure MouseMove(x, y: Integer);
    procedure Render;
    property Dragging: Boolean read FDragging;
    property Matrix: TMatrix read FmNow;
  end;

function MatrixMultiply(M1, M2: TMatrix): TMatrix; register;

const
  X = 0;
  Y = 1;
  Z = 2;
  W = 3;
  IdentityMatrix: TMatrix = ((1, 0, 0, 0),
                             (0, 1, 0, 0),
                             (0, 0, 1, 0),
                             (0, 0, 0, 1));

implementation

//this function from (c) Copyright 1999, Dipl. Ing. Mike Lischke (public@lischke-online.de)
//also in geometry.pas
function QuaternionConjugate(Q: TQuaternion): TQuaternion; assembler;
  // returns the conjugate of a quaternion
  // EAX contains address of Q
  // EDX contains address of result
asm
              FLD DWORD PTR [EAX]
              FCHS
              WAIT
              FSTP DWORD PTR [EDX]
              FLD DWORD PTR [EAX + 4]
              FCHS
              WAIT
              FSTP DWORD PTR [EDX + 4]
              FLD DWORD PTR [EAX + 8]
              FCHS
              WAIT
              FSTP DWORD PTR [EDX + 8]
              MOV EAX, [EAX + 12]
              MOV [EDX + 12], EAX
end;

//Defines a unit Quaternion
procedure QuaternionOne(var Q: TQuaternion);
begin

  Q.ImagPart[X] := 0;
  Q.ImagPart[Y] := 0;
  Q.ImagPart[Z] := 0;
  Q.RealPart := 1;

end;

//Compute the position of the mouse on sphere
//The mouse click defines X and Y, so
//this routine calculates the Z [Depth] of the click
function MouseOnSphere(lPos: TQuaternion): TQuaternion;
var
  lmag, lScale: Single;
  lBallmouse: TQuaternion;
begin

  lBallmouse.ImagPart[X] := lPos.ImagPart[X];
  lBallmouse.ImagPart[Y] := lPos.ImagPart[Y];
  lMag := sqr(lBallmouse.ImagPart[X]) + sqr(lBallmouse.ImagPart[Y]);
  if lMag > 1 then
  begin
    lScale := 1.0 / sqrt(lMag);
    lBallmouse.ImagPart[X] := lScale * lBallmouse.ImagPart[X];
    lBallmouse.ImagPart[Y] := lScale * lBallmouse.ImagPart[Y];
    lBallmouse.ImagPart[Z] := 0;
  end
  else lBallmouse.ImagPart[Z] := sqrt(1 - lMag);
  Result := lBallmouse;

end;

function Qt_FromBallPoints(var lFrom, lTo: TQuaternion): TQuaternion;
var
  lQ: TQuaternion;
begin

  lQ.ImagPart[X] := lFrom.ImagPart[Y] * lTo.ImagPart[Z] - lFrom.ImagPart[Z] *
    lTo.ImagPart[Y];
  lQ.ImagPart[Y] := lFrom.ImagPart[Z] * lTo.ImagPart[X] - lFrom.ImagPart[X] *
    lTo.ImagPart[Z];
  lQ.ImagPart[Z] := lFrom.ImagPart[X] * lTo.ImagPart[Y] - lFrom.ImagPart[Y] *
    lTo.ImagPart[X];
  lQ.RealPart := lFrom.ImagPart[X] * lTo.ImagPart[X] + lFrom.ImagPart[Y] *
    lTo.ImagPart[Y] + lFrom.ImagPart[Z] * lTo.ImagPart[Z];

  Result := lQ;

end;

function V3_(lx, ly, lz: Single): TQuaternion;
begin

  Result.ImagPart[X] := lX;
  Result.ImagPart[Y] := lY;
  Result.ImagPart[Z] := lZ;
  Result.RealPart := 0;

end;

procedure Qt_ToBallPoints(var lQ, arcFrom, arcTo: TQuaternion);
var
  s: Single;
begin

  s := sqrt(sqr(lQ.ImagPart[X]) + sqr(lQ.ImagPart[Y]));

  if s = 0 then arcFrom := v3_(0, 1.0, 0)
  else arcFrom := v3_(-lQ.ImagPart[Y] / S, lQ.ImagPart[X] / S, 0.0);

  arcTo.ImagPart[X] := lQ.RealPart * arcFrom.ImagPart[X] -
    lQ.ImagPart[Z] * arcFrom.ImagPart[Y];
  arcTo.ImagPart[Y] := lQ.RealPart * arcFrom.ImagPart[Y] +
    lQ.ImagPart[Z] * arcFrom.ImagPart[X];
  arcTo.ImagPart[Z] := lQ.ImagPart[X] * arcFrom.ImagPart[Y] -
    lQ.ImagPart[Y] * arcFrom.ImagPart[X];

  if lQ.RealPart < 0.0 then arcFrom := V3_(-arcFrom.ImagPart[X], - arcFrom.ImagPart[Y], 0.0);

end;

function Q_Mul(Ql, Qr: TQuaternion): TQuaternion;
begin

  Result.RealPart := qL.RealPart * qR.RealPart - qL.ImagPart[X] *
    qR.ImagPart[X] - qL.ImagPart[Y] * qR.ImagPart[Y] - qL.ImagPart[Z] * qR.ImagPart[Z];
  Result.ImagPart[X] := qL.RealPart * qR.ImagPart[X] + qL.ImagPart[X] *
    qR.RealPart + qL.ImagPart[Y] * qR.ImagPart[Z] - qL.ImagPart[Z] * qR.ImagPart[Y];
  Result.ImagPart[Y] := qL.RealPart * qR.ImagPart[Y] + qL.ImagPart[Y] *
    qR.RealPart + qL.ImagPart[Z] * qR.ImagPart[X] - qL.ImagPart[X] * qR.ImagPart[Z];
  Result.ImagPart[Z] := qL.RealPart * qR.ImagPart[Z] + qL.ImagPart[Z] *
    qR.RealPart + qL.ImagPart[X] * qR.ImagPart[Y] - qL.ImagPart[Y] * qR.ImagPart[X];

end;

function Qt_ToMatrix(Q: TQuaternion): TMatrix;
var
  Nq, s, xs, wx, xx, yy, ys, wy, xy, yz, zs, wz, xz, zz: Single;
begin

  Nq := sqr(q.ImagPart[X]) + sqr(q.ImagPart[y]) + sqr(q.ImagPart[z]) + sqr(q.RealPart);

  if Nq > 0 then s := 2 / Nq
  else s := 0;
  xs := q.ImagPart[X] * s;
  ys := q.ImagPart[Y] * s;
  zs := q.ImagPart[Z] * s;
  wx := q.RealPart * xs;
  wy := q.RealPart * ys;
  wz := q.RealPart * zs;
  xx := q.ImagPart[X] * xs;
  xy := q.ImagPart[X] * ys;
  xz := q.ImagPart[X] * zs;
  yy := q.ImagPart[Y] * ys;
  yz := q.ImagPart[Y] * zs;
  zz := q.ImagPart[Z] * zs;
  Result[X, X] := 1 - (yy + zz);
  Result[X, Y] := xy + wz;
  Result[X, Z] := xz - wy;
  Result[Y, X] := xy - wz;
  Result[Y, Y] := 1.0 - (xx + zz);
  Result[Y, Z] := yz + wx;
  Result[Z, X] := xz + wy;
  Result[Z, Y] := yz - wx;
  Result[Z, Z] := 1.0 - (xx + yy);
  Result[W, X] := 0;
  Result[W, Y] := 0;
  Result[W, Z] := 0;
  Result[X, W] := 0;
  Result[Y, W] := 0;
  Result[Z, W] := 0.0;
  Result[W, W] := 1.0;

end;

function MatrixMultiply(M1, M2: TMatrix): TMatrix; register;
  // multiplies two 4x4 matrices
var 
  I, J: Integer;
  TM: TMatrix;
begin

  for I := 0 to 3 do
    for J := 0 to 3 do
      TM[I, J] := M1[I, X] * M2[X, J] +
        M1[I, Y] * M2[Y, J] +
        M1[I, Z] * M2[Z, J] +
        M1[I, W] * M2[W, J];
        
  Result := TM;

end;

(* Halve arc between unit vectors v0 and v1. *)
function V3_Bisect(v0, v1: TVector4f): TVector4f;
var
  v: TVector4f;
  Nv: Single;
  i: Integer;
begin

  for i := 0 to 2 do v[i] := v0[i] + v1[i];

  Nv := sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (Nv < 1.0e-5) then
  begin
    v[0] := 0;
    v[1] := 0;
    v[2] := 0;
  end
  else for i := 0 to 2 do v[i] := v[i]/Nv;
  Result := v;

end;

{******************************************************************************}

//Call at startup: sensible values
procedure TArcBall.Init(radius: Single);
begin

  FCenter.x := 0;
  FCenter.y := 0;
  FCenter.z := 0;
  FRadius := radius;
  QuaternionOne(FvDown);
  QuaternionOne(FvNow);
  QuaternionOne(FqDown);
  QuaternionOne(FqNow);
  FmNow := IdentityMatrix;
  FmDown := IdentityMatrix;

end;

//Defines the X,Y,Z rotation for a point
procedure TArcBall.PointRotateArcBall(var lPoint3D: Point3D);
var
  V: Point3D;
begin

  V := lPoint3D;

  lPoint3D.X := round(V.X * FmNow[X, X] + V.Y * FmNow[Y,X] + V.Z * FmNow[Z, X] {+ V.W * M[W, X]});
  lPoint3D.Y := round(V.X * FmNow[X, Y] + V.Y * FmNow[Y,Y] + V.Z * FmNow[Z, Y] {+ V.W * gBall.mNow[W, Y]});
  lPoint3D.Z := round(V.X * FmNow[X, Z] + V.Y * FmNow[Y,Z] + V.Z * FmNow[Z, Z] {+ V.W * gBall.mNow[W, Z]});
  //TV.W := V.X * gBall.mNow[X, W] + V.Y * M[Y, W] + V.Z * M[Z, W] + V.W * M[W, W];

end;

//sets the boolean 'dragging' to true -
// mouse movements will now be automatically updated
procedure TArcBall.BeginDrag;
begin

  FDragging := TRUE;
  FvDown := FvNow;

end;

//updates the matrix mDown with the current position
procedure TArcBall.EndDrag;
begin

  //Note: I update mDown
  FvDown := FvNow;
  FmDown := FmNow;
  FDragging := FALSE;

end;

procedure TArcBall.Update;
begin

  FvFrom := MouseOnSphere(FvDown);
  FvTo := MouseOnSphere(FvNow);

  if FDragging then
  begin
    FqDrag := Qt_FromBallPoints(FvFrom, FvTo);
    FqNow := Q_Mul(FqDrag, FqDown);
  end;

  Qt_ToBallPoints(FqDown, FvrFrom, FvrTo);
  FmNow := Qt_ToMatrix(QuaternionConjugate(FqNow));

  //Note: new position is composite of starting position and
  //new rotation, so we multiply matrixes.
  FmNow := MatrixMultiply(FmDown, FmNow);

end;

procedure TArcBall.MouseMove(x, y: Integer);
begin

  FvNow.ImagPart[0] := (X-FRadius)/FRadius;
  FvNow.ImagPart[1] := -(Y-FRadius)/FRadius;
  FvNow.ImagPart[2] := 0;

  if not FDragging then Exit;

  Update;

end;

procedure TArcBall.DrawArc(vFrom, vTo: TVector4f);
const
  LG_NSEGS = 4;
  NSEGS = 1 shl LG_NSEGS;
var
  i, j: Integer;
  pts: array [0..NSEGS] of TVector4f;
  dot: Single;
begin

  pts[0] := vFrom;
  pts[NSEGS] := vTo;
  pts[1] := vTo;

  for i := 0 to LG_NSEGS - 1 do pts[1] := V3_Bisect(pts[0], pts[1]);
  dot := 2*(pts[0][0]*pts[1][0] + pts[0][1]*pts[1][1] + pts[0][2]*pts[1][2]);
  for i := 2 to NSEGS - 1 do
  begin
    for j := 0 to 2 do
      pts[i][j] := pts[i-1][j]*dot - pts[i-2][j];
  end;
  glBegin(GL_LINE_STRIP);
    for i := 0 to NSEGS do glVertex3fv(@pts[i]);
  glEnd;

end;

procedure TArcBall.Render;
var
  i: Integer;
const
  S = 64;
begin

  if Dragging then
  begin
    glPushMatrix;
    glLoadIdentity;
    glMatrixMode(GL_PROJECTION);
    glPushMatrix;
    glLoadIdentity;
    glMatrixMode(GL_MODELVIEW);
      glBegin(GL_LINE_LOOP);
        for i := 0 to S - 1 do
        begin
          glVertex3f(cos(2*PI*i/S), sin(2*PI*i/S), 0);
        end;
      glEnd;

      glBegin(GL_POINTS);
        glVertex3fv(@FvFrom);
        glVertex3fv(@FvTo);
      glEnd;
      DrawArc(FvFrom.Vector, FvTo.Vector);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix;
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix;
  end;

end;

end.
