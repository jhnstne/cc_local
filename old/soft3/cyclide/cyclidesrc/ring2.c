#include <math.h>

#define  EPSILON     0.00005
#define  EQUAL(x, y) (fabs((x)-(y)) < EPSILON)

#define  RING_NUMC   40            /* polygons around V-circles     */
#define  RING_NUMT   60            /* polygons around H-circles     */

/* ---------------------------------------------------------------- */
/*     The following two macros compute the x- and y-coordinate     */
/* of the intersection point of two lines in specific positions.    */
/* The first line is determined by (u,v) and (b,0), while the       */
/* second one is determined by (p,q) and (a,0).  Note that in the   */
/* program these two lines are never parallel to each other and     */
/* therefore their intersection points are always well-defined.     */
/* ---------------------------------------------------------------- */

#define  INTERx(u,v,b,p,q,a) ((v)*((p)-(a))*(b)-(q)*((u)-(b))*(a))/ \
                             ((v)*((p)-(a))    -(q)*((u)-(b)))
#define  INTERy(u,v,b,p,q,a) ((v)*(q)*((b)-(a)))/                   \
                             ((v)*((p)-(a))    -(q)*((u)-(b)))

static float  ring_point[RING_NUMC][RING_NUMT][3];
static float  ring_norm[RING_NUMC][RING_NUMT][3];

static float  cone1[RING_NUMC][2][3];
static float  cone1_norm[RING_NUMC][2][3];
static float  cone2[RING_NUMC][2][3];
static float  cone2_norm[RING_NUMC][2][3];

static int    ring_initialized = 0;

/* ---------------------------------------------------------------- */
/* FUNCTION  initring :                                             */
/*    initialize data for ring cyclide                              */
/* ---------------------------------------------------------------- */

static void initring(float r1, float r2, float d, float L1, float L2,
                     float start, float end)
{
     int    i, j, k;
     float  fi, fj;
     int    numc = RING_NUMC;
     int    numt = RING_NUMT;
     float  pi   = acos(-1.0);
     float  twopi = pi+pi;
     float  Ax, Ay;                /* <Ax,Ay> = outer startint pt   */
     float  Bx, By;                /* <Bx,By> = inner starting pt   */
     float  Ox, Oy;                /* <Ox,Oy> = center of V-circle  */
     float  Px, Py;                /* <Px,Py> = cone VTX            */
     float  Vx, Vy;                /* horizontal vector (diameter)  */
     float  r;                     /* radius of V-circle            */
     float  cos, sin, cosi, sini;
     float  rr1, rr2;
     float  h1, h2;
     float  t1, t2;
     float  dx, dy, dz, n;
     float  angle, delta;
     float  AA1, BB1, CC1;
     float  AA2, BB2, CC2;
     float  Qx, Qy, DDD;
     float  Sign;
     float  h[3];

     delta = (end - start)/(numt-1);

     for (j = 0, angle = start; j < numt; j++, angle += delta) {
          if (EQUAL(angle, 0.0)) {
               Ax = r1-r1/(r1+r2)*d;
               Ay = 0.0;
               Bx = r2+r2/(r1+r2)*d;
               By = 0.0;
               Ox = (Ax + Bx)/2.0;
               Oy = (Ay + By)/2.0;
               Px = Ox;
               Py = Oy;
          }
          else if (EQUAL(angle, pi)) {
               Ax = -(r1+r1/(r1+r2)*d);
               Ay = 0.0;
               Bx = -r2+r2/(r1+r2)*d;
               By = 0.0;
               Ox = (Ax + Bx)/2.0;
               Oy = (Ay + By)/2.0;
               Px = Ox;
               Py = Oy;
          }
          else {                   /* V-circles in between          */
               cos = fcos(angle);
               sin = fsin(angle);
               rr1 = r1/(r1+r2)*d;
               h1  = rr1*rr1*cos*cos+(r1*r1-rr1*rr1);
               t1  = -rr1*cos + fsqrt(h1);
               if (t1 < 0.0) t1 = -rr1*cos - fsqrt(h1);
               Ax = t1*cos;
               Ay = t1*sin;

               rr2 = r2/(r1+r2)*d;
               h2  = rr2*rr2*cos*cos+(r2*r2-rr2*rr2);
               t2  = rr2*cos + fsqrt(h2);
               if (t2 < 0.0) t2 = rr2*cos - fsqrt(h2);
               Bx  = t2*cos;
               By  = t2*sin;

               Ox = (Ax + Bx)/2.0;
               Oy = (Ay + By)/2.0;
               Px = INTERx(Ax,Ay,-rr1,Bx,By,rr2);
               Py = INTERy(Ax,Ay,-rr1,Bx,By,rr2);
          }
          Vx = Ax - Ox;
          Vy = Ay - Oy;
          r  = fsqrt(Vx*Vx + Vy*Vy);

          for (i = 0; i < numc; i++) {
               fi   = (float) i;
               cosi = fcos(twopi*fi/numc);
               sini = fsin(twopi*fi/numc);
               ring_point[i][j][0] = Vx*cosi + Ox;
               ring_point[i][j][1] = Vy*cosi + Oy;
               ring_point[i][j][2] = r*sini;

               dx = ring_point[i][j][0] - Px;
               dy = ring_point[i][j][1] - Py;
               dz = ring_point[i][j][2];

               n = fsqrt(dx*dx + dy*dy + dz*dz);
               ring_norm[i][j][0] = dx/n;
               ring_norm[i][j][1] = dy/n;
               ring_norm[i][j][2] = dz/n;
          }
     
          if (j == 0 || j == numt-1) {
               AA1 = Ax+r1*d/(r1+r2);
               BB1 = Ay;
               CC1 = r1*d*Ax/(r1+r2)+r1*r1*d*d/((r1+r2)*(r1+r2))-r1*r1;

               AA2 = Bx-r2*d/(r1+r2);
               BB2 = By;
               CC2 =-r2*d*Bx/(r1+r2)+r2*r2*d*d/((r1+r2)*(r1+r2))-r2*r2;

               DDD = AA1*BB2 - AA2*BB1;
               Qx  = -(CC1*BB2-CC2*BB1)/DDD;
               Qy  = -(AA1*CC2-AA2*CC1)/DDD;

               Sign = (Ay < 0.0) ? -1.0 : 1.0;

               if (j == 0) {
                    for (i = 0; i < numc; i++) {
                         for (k = 0; k < 3; k++) {
                              dx  = ring_point[i][j][0] - Qx;
                              dy  = ring_point[i][j][1] - Qy;
                              dz  = ring_point[i][j][2];
                              n   = fsqrt(dx*dx + dy*dy + dz*dz);

                              h[0] = dx/n;
                              h[1] = dy/n;
                              h[2] = dz/n;

                              cone1[i][0][k] = ring_point[i][j][k];
                              cone1[i][1][k] = ring_point[i][j][k]-Sign*L1*h[k];
                              cone1_norm[i][0][k] = cone1_norm[i][1][k] 
                                             = ring_norm[i][j][k];
                         }
                    }
               }
               else if (j == numt-1) {
                    for (i = 0; i < numc; i++) {
                         for (k = 0; k < 3; k++) {
                              dx  = ring_point[i][j][0] - Qx;
                              dy  = ring_point[i][j][1] - Qy;
                              dz  = ring_point[i][j][2];
                              n   = fsqrt(dx*dx + dy*dy + dz*dz);

                              h[0] = dx/n;
                              h[1] = dy/n;
                              h[2] = dz/n;

                              cone2[i][0][k] = ring_point[i][j][k];
                              cone2[i][1][k] = ring_point[i][j][k]+Sign*L2*h[k];
                              cone2_norm[i][0][k] = cone2_norm[i][1][k] 
                                             = ring_norm[i][j][k];
                         }
                    }
               }
          }
     }
}

/* ---------------------------------------------------------------- */
/* FUNCTION  setring :                                              */
/*    build up the point/normal structure of a ring cyclide         */
/* ---------------------------------------------------------------- */

void  setring(float r1, float r2, float d, float L1, float L2,
                     float start, float end)
{
     initring(r1, r2, d, L1, L2, start, end);
     ring_initialized = 1;
}

/* ---------------------------------------------------------------- */
/* FUNCTION  fring :                                                */
/*    filled ring cyclide                                           */
/* ---------------------------------------------------------------- */

void  fring(float r1, float r2, float d, float L1, float L2,
                     float start, float end)
{
     int  i, j;
     int  numc = RING_NUMC;
     int  numt = RING_NUMT;

     if (!ring_initialized)
          setring(r1, r2, d, L1, L2, start, end);

     for (i = 0; i < numc; i++) {
          bgntmesh();
               n3f(ring_norm[(i+1)%numc][0]);
               v3f(ring_point[(i+1)%numc][0]);
               for (j = 0; j < numt-1; j++) {
                    n3f(ring_norm[i][j]);
                    v3f(ring_point[i][j]);
                    n3f(ring_norm[(i+1)%numc][(j+1)%numt]);
                    v3f(ring_point[(i+1)%numc][(j+1)%numt]);
               }
               n3f(ring_norm[i][numt-1]);
               v3f(ring_point[i][numt-1]);
          endtmesh();
     }
}


void  fcone1(float r1, float r2, float d, float L1, float L2,
                     float start, float end)
{
     int  i, j;
     int  numc = RING_NUMC;
     int  numt = RING_NUMT;

     if (!ring_initialized)
          setring(r1, r2, d, L1, L2, start, end);

     for (i = 0; i < numc; i++) {
          bgnpolygon();
               n3f(cone1_norm[i%numc][0]);
               v3f(cone1[i%numc][0]);
               n3f(cone1_norm[(i+1)%numc][0]);
               v3f(cone1[(i+1)%numc][0]);
               n3f(cone1_norm[(i+1)%numc][1]);
               v3f(cone1[(i+1)%numc][1]);
               n3f(cone1_norm[i%numc][1]);
               v3f(cone1[i%numc][1]);
          endpolygon();
     }
}

void  fcone2(float r1, float r2, float d, float L1, float L2,
                     float start, float end)
{
     int  i, j;
     int  numc = RING_NUMC;
     int  numt = RING_NUMT;

     if (!ring_initialized)
          setring(r1, r2, d, L1, L2, start, end);

     for (i = 0; i < numc; i++) {
          bgnpolygon();
               n3f(cone2_norm[i%numc][0]);
               v3f(cone2[i%numc][0]);
               n3f(cone2_norm[(i+1)%numc][0]);
               v3f(cone2[(i+1)%numc][0]);
               n3f(cone2_norm[(i+1)%numc][1]);
               v3f(cone2[(i+1)%numc][1]);
               n3f(cone2_norm[i%numc][1]);
               v3f(cone2[i%numc][1]);
          endpolygon();
     }
}

