/* ================================================================ */
/* SHAPE : Ring Cyclide                                             */
/*     This set of functions accepts two radii and the distance     */
/* between the centers of two principle horizontal circles and      */
/* generates a ring type cyclide.  Let r1, r2 and d be the radii &  */
/* the distance between centers.  Then, we have                     */
/*         1) r1 > r2 + d : the generated surface is of ring type   */
/*         2) r1 = r2 + d : the surface is a single horned          */
/*         3) r1 < r2 + d : a double horned cyclide                 */
/*                                                                  */
/*     The only one thing you have to do is by calling fring() for  */
/* filled surface, or wring() for wireframe.                        */
/*                                                                  */
/*     You can increase RING_NUMC and RING_NUMT for a smoother      */
/* surface, or decrease it to have higher efficiency but a not so   */
/* smooth surface.                                                  */
/*                                                                  */
/*     Another place you may change is the origin.  In this routine */
/* the origin is placed at the midpoint of the leftmost and the     */
/* rightmost points on the x-axis.  Therefore, in function setring, */
/* remove +mid AND -mid, you will make the inner center of          */
/* similitude to the origin.                                        */
/*                                                                  */
/* Copyright Ching-Kuang Shene                          Mar/31/1992 */
/* ================================================================ */

#include <math.h>

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

static int    ring_initialized = 0;

/* ---------------------------------------------------------------- */
/* FUNCTION  initring :                                             */
/*    initialize data for ring cyclide                              */
/* ---------------------------------------------------------------- */

static void initring(float r1, float r2, float d)
{
     int    i, j;
     float  fi, fj;
     int    numc = RING_NUMC;
     int    numt = RING_NUMT;
     float  twopi = 2*acos(-1.0);
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
     float  left, right, mid;

     left = -(r1+r1/(r1+r2)*d);
     if (left >= (-r2+r2/(r1+r2)*d)) left = -r2+r2/(r1+r2)*d;
     right = r1 - r1/(r1+r2)*d;
     if (right < r2+r2/(r1+r2)*d)  right = r2+r2/(r1+r2)*d;
     mid = (left+right)/2.0;


     for (j = 0; j < numt; j++) {  /* for each V-circle around H-cir*/
          fj = (float) j;          
          if (j == 0) {            /* the left V-circle             */
               Ax = r1-r1/(r1+r2)*d;
               Ay = 0.0;
               Bx = r2+r2/(r1+r2)*d;
               By = 0.0;
               Ox = (Ax + Bx)/2.0;
               Oy = (Ay + By)/2.0;
               Px = Ox;
               Py = Oy;
          }
          else if (j == (RING_NUMT/2)) { /* the right V-circle      */
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
               cos = fcos(twopi*j/numt);
               sin = fsin(twopi*j/numt);
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
               sini = fsin(twopi*fi/numc);  /* remove mid if you want*/
               ring_point[i][j][0] = Vx*cosi + Ox - mid;
               ring_point[i][j][1] = Vy*cosi + Oy;
               ring_point[i][j][2] = r*sini;

               dx = ring_point[i][j][0] - Px + mid; /* another mid   */
               dy = ring_point[i][j][1] - Py;
               dz = ring_point[i][j][2];

               n = fsqrt(dx*dx + dy*dy + dz*dz);
               ring_norm[i][j][0] = dx/n;
               ring_norm[i][j][1] = dy/n;
               ring_norm[i][j][2] = dz/n;
          }
     }
}

/* ---------------------------------------------------------------- */
/* FUNCTION  setring :                                              */
/*    build up the point/normal structure of a ring cyclide         */
/* ---------------------------------------------------------------- */

void  setring(float r1, float r2, float d)
{
     initring(r1, r2, d);
     ring_initialized = 1;
}

/* ---------------------------------------------------------------- */
/* FUNCTION  wring :                                                */
/*    wireframe ring cyclide                                        */
/* ---------------------------------------------------------------- */

void  wring(float r1, float r2, float d)
{
     int  i, j;
     int  numc = RING_NUMC;
     int  numt = RING_NUMT;

     if (!ring_initialized)
          setring(r1, r2, d);

     for (i = 0; i < numc; i++) {
          for (j = 0; j < numt; j++) {
               bgnclosedline();
                    v3f(ring_point[(i+1)%numc][j]);
                    v3f(ring_point[i][j]);
                    v3f(ring_point[(i+1)%numc][(j+1)%numt]);
               endclosedline();

               bgnclosedline();
                    v3f(ring_point[i][j]);   
                    v3f(ring_point[(i+1)%numc][(j+1)%numt]);
                    v3f(ring_point[i][(j+1)%numt]);
               endclosedline();
          }
     }
}

/* ---------------------------------------------------------------- */
/* FUNCTION  fring :                                                */
/*    filled ring cyclide                                           */
/* ---------------------------------------------------------------- */

void  fring(float r1, float r2, float d)
{
     int  i, j;
     int  numc = RING_NUMC;
     int  numt = RING_NUMT;

     if (!ring_initialized)
          setring(r1, r2, d);

     for (i = 0; i < numc; i++) {
          bgntmesh();
               n3f(ring_norm[(i+1)%numc][0]);
               v3f(ring_point[(i+1)%numc][0]);
               for (j = 0; j < numt; j++) {
                    n3f(ring_norm[i][j]);
                    v3f(ring_point[i][j]);
                    n3f(ring_norm[(i+1)%numc][(j+1)%numt]);
                    v3f(ring_point[(i+1)%numc][(j+1)%numt]);
               }
               n3f(ring_norm[i][0]);
               v3f(ring_point[i][0]);
          endtmesh();
     }
}
