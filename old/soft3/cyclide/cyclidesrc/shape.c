#include   <gl.h>
#include   <math.h>

typedef   float  vector[3];        /* three dimensional vector             */

static vector front  = {  0.0,  0.0,  1.0}; /* useful vectors for the cube */
static vector back   = {  0.0,  0.0, -1.0}; /* beam, letter f and axes     */
static vector top    = {  0.0,  1.0,  0.0};
static vector bottom = {  0.0, -1.0,  0.0};
static vector right  = {  1.0,  0.0,  0.0};
static vector left   = { -1.0,  0.0,  0.0};
static vector center = {  0.0,  0.0,  0.0};
 

/* ======================================================================= */
/* SHAPE : axes                                                            */
/*    draws a wireframe set of right-handed axes.                          */
/* ======================================================================= */

void  axes(void)
{
     bgnline();
          v3f(center);
          v3f(right);
     endline();
     cmov(right[0], right[1], right[2]);
     charstr("X");

     bgnline();
          v3f(center);
          v3f(top);
     endline();
     cmov(top[0], top[1], top[2]);
     charstr("Y");

     bgnline();
          v3f(center);
          v3f(front);
     endline();
     cmov(front[0], front[1], front[2]);
     charstr("Z");
}


/* ======================================================================= */
/* SHAPE : cube                                                            */
/* ======================================================================= */

#define    CUBE_X    1.0           /* default cube size                    */
#define    CUBE_Y    1.0
#define    CUBE_Z    1.0

static float  cube_point[8][3];    /* cube points                          */

static Boolean  cube_initialized = FALSE;


/* ----------------------------------------------------------------------- */
/* FUNCTION setcube :                                                      */
/* ----------------------------------------------------------------------- */

void  setcube(float x, float y, float z)
{
     cube_point[1][0] = cube_point[2][0] =
     cube_point[5][0] = cube_point[6][0] = x/2.0;

     cube_point[0][0] = cube_point[3][0] =
     cube_point[4][0] = cube_point[7][0] = -x/2.0;

     cube_point[2][1] = cube_point[3][1] =
     cube_point[6][1] = cube_point[7][1] = y/2.0;

     cube_point[0][1] = cube_point[1][1] =
     cube_point[4][1] = cube_point[5][1] = -y/2.0;

     cube_point[0][2] = cube_point[1][2] =
     cube_point[2][2] = cube_point[3][2] = z/2.0;

     cube_point[4][2] = cube_point[5][2] =
     cube_point[6][2] = cube_point[7][2] = -z/2.0;

     cube_initialized = TRUE;
}


/* ----------------------------------------------------------------------- */
/* FUNCTION wcube :                                                        */
/*    wireframe cube.                                                      */
/* ----------------------------------------------------------------------- */

void  wcube(void)
{
     if (!cube_initialized)
          setcube(CUBE_X, CUBE_Y, CUBE_Z);

     bgnclosedline();              /* draw bottom facing closedline        */
          v3f(cube_point[4]);
          v3f(cube_point[5]);
          v3f(cube_point[1]);
          v3f(cube_point[0]);
     endclosedline();

     bgnclosedline();              /* draw left facing closedline          */
          v3f(cube_point[3]);
          v3f(cube_point[7]);
          v3f(cube_point[4]);
          v3f(cube_point[0]);
     endclosedline();

     bgnclosedline();              /* draw right facing closedline         */
          v3f(cube_point[5]);
          v3f(cube_point[6]);
          v3f(cube_point[2]);
          v3f(cube_point[1]);
     endclosedline();

     bgnclosedline();              /* draw top facing closedline           */
          v3f(cube_point[6]);
          v3f(cube_point[7]);
          v3f(cube_point[3]);
          v3f(cube_point[2]);
     endclosedline();

     bgnclosedline();              /* draw front facing closedline         */
          v3f(cube_point[1]);
          v3f(cube_point[2]);
          v3f(cube_point[3]);
          v3f(cube_point[0]);
     endclosedline();

     bgnclosedline();              /* draw back facing closedline          */ 
          v3f(cube_point[6]);
          v3f(cube_point[5]);
          v3f(cube_point[4]);
          v3f(cube_point[7]);
     endclosedline();
}

/* ----------------------------------------------------------------------- */
/* FUNCTION fcube :                                                        */
/*    filled cube.                                                         */
/* ----------------------------------------------------------------------- */

void  fcube(void)
{
     if (!cube_initialized)
          setcube(CUBE_X, CUBE_Y, CUBE_Z);

     bgnpolygon();                 /* draw bottom facing polygon           */
          n3f(bottom);
          v3f(cube_point[4]);
          v3f(cube_point[5]);
          v3f(cube_point[1]);
          v3f(cube_point[0]);
     endpolygon();

     bgnpolygon();                 /* draw left facing polygon             */
          n3f(left);
          v3f(cube_point[3]);
          v3f(cube_point[7]);
          v3f(cube_point[4]);
          v3f(cube_point[0]);
     endpolygon();

     bgnpolygon();                 /* draw right facing polygon            */
          n3f(right);
          v3f(cube_point[5]);
          v3f(cube_point[6]);
          v3f(cube_point[2]);
          v3f(cube_point[1]);
     endpolygon();

     bgnpolygon();                 /* draw top facing polygon              */
          n3f(top);
          v3f(cube_point[6]);
          v3f(cube_point[7]);
          v3f(cube_point[3]);
          v3f(cube_point[2]);
     endpolygon();

     bgnpolygon();                 /* draw front facing polygon            */
          n3f(front);
          v3f(cube_point[1]);
          v3f(cube_point[2]);
          v3f(cube_point[3]);
          v3f(cube_point[0]);
     endpolygon();

     bgnpolygon();                 /* draw back facing polygon             */ 
          n3f(back);
          v3f(cube_point[6]);
          v3f(cube_point[5]);
          v3f(cube_point[4]);
          v3f(cube_point[7]);
     endpolygon();
}

#define    CONE_FACETS    50       /* number of facets around the cone      */


static Boolean  cone_initialized   = FALSE;

static float cone_point[CONE_FACETS][3];    /* cone vertices                */
static float cone_norm[CONE_FACETS][3];     /* cone normal                  */
static float cone_top[3];                   /* the vettex of the cone       */

static void initcone(float point[][3], float norm[][3])
{
     int   i;
     int   facets = CONE_FACETS;
     float theta;                  /* angle around the cone base            */
     float nx, ny, nz;             /* normal vector <nx,ny,nz>              */
     float l;                      /* length of normal vextor <nx,ny,nz>    */

     for (i = 0; i < facets; i++) {
          theta = 2.0*M_PI*(float) i/(float) facets;
          point[i][0] = fcos(theta);
          point[i][1] = 0.0;
          point[i][2] = fsin(theta);

          nx = fcos(theta);
          ny = fcos(M_PI/4.0)*fsin(M_PI/4.0);
          nz = fsin(theta);
          l  = fsqrt(nx*nx + ny*ny + nz*nz);

          norm[i][0] = nx/l;
          norm[i][1] = ny/l;
          norm[i][2] = nz/l;
     }
     cone_top[0] = 0.0;
     cone_top[1] = 1.0;
     cone_top[2] = 0.0;
}

/* ------------------------------------------------------------------------ */
/* FUNCTION wcone :                                                         */
/*    wireframe cone of radius 1.0 and height 1.0 with the origin at the    */
/* center of the base.                                                      */
/* ------------------------------------------------------------------------ */

void  wcone(void)
{
     int  i;
     int  facets = CONE_FACETS;

     if (!cone_initialized) {
          initcone(cone_point, cone_norm);
          cone_initialized = TRUE;
     }

     for (i = 0; i < facets-1; i++) {
          bgnclosedline();
               v3f(cone_top);
               v3f(cone_point[i]);
               v3f(cone_point[(i+1)%facets]);
          endclosedline();
     }
     bgnclosedline();
          v3f(cone_top);
          v3f(cone_point[facets-1]);
          v3f(cone_point[0]);
     endclosedline();
}

/* ------------------------------------------------------------------------ */
/* FUNCTION fcone :                                                         */
/*    filled cone of radius 1.0 and height 1.0 with the origin at the center*/
/* of the base.                                                             */
/* ------------------------------------------------------------------------ */

void  fcone(void)
{
     int   i, facets = CONE_FACETS;

     if (!cone_initialized) {
          initcone(cone_point, cone_norm);
          cone_initialized = TRUE;
     }
     
     bgntmesh();
          n3f(cone_norm[0]);
          v3f(cone_point[0]);

          for (i = 1; i <= facets; i++) {
               n3f(cone_norm[i%facets]);
               v3f(cone_top);
               n3f(cone_norm[i%facets]);
               v3f(cone_point[i%facets]);
          }
     endtmesh();
}

/* ======================================================================= */
/* SHAPE : torus                                                           */
/* ======================================================================= */

#define   TORUS_NUMC    20        /* polygons around the cross section     */
#define   TORUS_NUMT    40        /* polygons around the outside of torus  */
#define   TORUS_RAD_XC  0.3       /* cross section radius                  */
#define   TORUS_RAD     1.0       /* torus radius                          */

static float torus_point[TORUS_NUMC][TORUS_NUMT][3];
static float torus_norm[TORUS_NUMC][TORUS_NUMT][3];

static Boolean torus_initialized = FALSE;

/* ----------------------------------------------------------------------- */
/* FUNCTION inittorus :                                                    */
/*    initialize data for torus.                                           */
/* ----------------------------------------------------------------------- */

static void inittorus(float rc,   /* radius of the cross section           */
                      float rt,   /* radius of the torus                   */
                      float point[TORUS_NUMC][TORUS_NUMT][3],
                      float norm[TORUS_NUMC][TORUS_NUMT][3])  /* normals   */
{
     int    i, j;
     int    numc = TORUS_NUMC;
     int    numt = TORUS_NUMT;
     float  twopi, fi, fj, xc, yc, nx, ny, nz, n;

     twopi = 2.0 * M_PI;

     for (i = 0; i < numc; i++) { /* go around cross section               */
          fi = (float) i;
          for (j = 0; j < numt; j++) { /* go around top view               */
               fj = (float) j;
               point[i][j][0] = (rt+rc*fcos(twopi*fi/numc))*fcos(twopi*fj/numt);
               point[i][j][1] = (rt+rc*fcos(twopi*fi/numc))*fsin(twopi*fj/numt);
               point[i][j][2] = rc*fsin(twopi*fi/numc);

               xc = rt*fcos(twopi*fj/numt);
               yc = rt*fsin(twopi*fj/numt);

               nx = point[i][j][0] - xc;
               ny = point[i][j][1] - yc;
               nz = point[i][j][2];

               n = fsqrt(nx*nx + ny*ny + nz*nz);

               norm[i][j][0] = nx/n;
               norm[i][j][1] = ny/n;
               norm[i][j][2] = nz/n;
          }
     }
}


/* ----------------------------------------------------------------------- */
/* FUNCTION settorus :                                                     */
/*    set the cross sectional and torus radius.                            */
/* ----------------------------------------------------------------------- */

void  settorus(float rc, float rt)
{
     inittorus(rc, rt, torus_point, torus_norm);
     torus_initialized = TRUE;
}

/* ----------------------------------------------------------------------- */
/* FUNCTION  wtorus :                                                      */
/*    wireframe torus.                                                     */
/* ----------------------------------------------------------------------- */

void  wtorus(void)
{
     int  i, j;
     int  numc = TORUS_NUMC;
     int  numt = TORUS_NUMT;

     if (!torus_initialized)
          settorus(TORUS_RAD_XC, TORUS_RAD);

     for (i = 0; i < numc; i++) {
          for (j = 0; j < numt; j++) {
               bgnclosedline();
                    v3f(torus_point[(i+1)%numc][j]);
                    v3f(torus_point[i][j]);
                    v3f(torus_point[(i+1)%numc][(j+1)%numt]);
               endclosedline();

               bgnclosedline();
                    v3f(torus_point[i][j]);
                    v3f(torus_point[(i+1)%numc][(j+1)%numt]);
                    v3f(torus_point[i][(j+1)%numt]);
               endclosedline();
           }
     }
}


/* ----------------------------------------------------------------------- */
/* FUNCTION ftorus :                                                       */
/*    filled torus.                                                        */
/* ----------------------------------------------------------------------- */

void  ftorus(void)
{
     int  i, j;
     int  numc = TORUS_NUMC;
     int  numt = TORUS_NUMT;

     if (!torus_initialized)
          settorus(TORUS_RAD_XC, TORUS_RAD);

     for (i = 0; i < numc; i++) {
          bgntmesh();
               n3f(torus_norm[(i+1)%numc][0]);
               v3f(torus_point[(i+1)%numc][0]);
               for (j = 0; j < numt; j++) {
                    n3f(torus_norm[i][j]);
                    v3f(torus_point[i][j]);
                    n3f(torus_norm[(i+1)%numc][(j+1)%numt]);
                    v3f(torus_point[(i+1)%numc][(j+1)%numt]);
               }
               n3f(torus_norm[i][0]);
               v3f(torus_point[i][0]);
          endtmesh();
     }
}


