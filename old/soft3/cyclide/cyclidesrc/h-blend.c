#include  <gl.h>
#include  <device.h>
#include  <stdlib.h>
#include  <stdio.h>
#include  <math.h>
#include  "ring4.c"

void  initialize(void);
void  drawscene(float, float, float, float, float, float, float);

int   xmax, ymax;             /* size of the window                        */

float light_model[] = {       /* define the lighting model properties array*/
     AMBIENT, 0.1, 0.1, 0.1,
     ATTENUATION, 1.0, 0.0,
     LOCALVIEWER, 1.0,
     LMNULL
};

float white_light[] = {       /* define the light source properties array  */
     AMBIENT, 0.2, 0.2, 0.2,  /* This is a white, infinite light source,   */
     POSITION, 0.0, 1.0, 1.0, 0.0, /* shining from the top.                */
     LCOLOR, 1.0, 1.0, 1.0,
     LMNULL
};

float other_light[] = {       /* define the light source properties array  */
     AMBIENT, 0.2, 0.2, 0.2,  /* This is a white, infinite light source,   */
     POSITION, 0.0, -1.0, -1.0, 0.0, /* shining from the bottom            */
     LCOLOR, 1.0, 1.0, 1.0,
     LMNULL
};

float material1_plastic[] = {  /* define a material properties array        */
     AMBIENT, 0.2, 0.1, 0.1, 
     DIFFUSE, 1.0, 1.0, 0.0,
     SPECULAR, 0.0, 0.0, 0.0,
     SHININESS, 0.0,
     LMNULL
};

float material2_plastic[] = {  /* define a material properties array        */
     AMBIENT, 0.2, 0.1, 0.1, 
     DIFFUSE, 0.1, 0.8, 0.5,
     SPECULAR, 0.0, 0.0, 0.0,
     SHININESS, 0.0,
     LMNULL
};

float material3_plastic[] = {  /* define a material properties array        */
     AMBIENT, 0.2, 0.1, 0.1, 
     DIFFUSE, 0.5, 0.1, 0.8,
     SPECULAR, 0.0, 0.0, 0.0,
     SHININESS, 0.0,
     LMNULL
};

int angle;

void  main(int argc, char *argv[])
{
     Boolean exitflag = FALSE;
     short   attached = 0;    /* attached to window                        */
     short   value;
     int     dev;
     float   r1, r2, d, L1, L2, START, END, start, end;
     float   pi = acos(-1.0);

     if (argc < 8) {
          info();
          exit(0);
     }
     r1 = atof(argv[1]);
     r2 = atof(argv[2]);
     d  = atof(argv[3]);
     L1 = atof(argv[4]);
     L2 = atof(argv[5]);
     START = atof(argv[6]);
     END   = atof(argv[7]);
     start = START*pi/180.0;
     end   = END*pi/180.0;
     angle = atof(argv[8]);

     initialize();
     while (exitflag == FALSE) {
          drawscene(r1, r2, d, L1, L2, start, end);
          while ((exitflag == FALSE) && (qtest() || !attached)) {
               dev = qread(&value);
               if ((dev == ESCKEY) && (value == 0))
                    exitflag = TRUE;
               else if (dev == WINFREEZE || dev == REDRAWICONIC) {
                    frontbuffer(TRUE);
                    pushmatrix();
                    reshapeviewport();
                    drawscene(r1, r2, d, L1, L2, start, end);
                    popmatrix();
                    frontbuffer(FALSE);
               }
               else if (dev == REDRAW)
                    reshapeviewport();
               else if (dev == INPUTCHANGE)
                    attached = value;
          }
     }
     exit(0);
}


/* ----------------------------------------------------------------------- */
/* FUNCTION  initialize :                                                  */
/* ----------------------------------------------------------------------- */

void  initialize(void)
{
     int  gid;

     xmax = getgdesc(GD_XPMAX);
     ymax = getgdesc(GD_YPMAX);
     prefposition(xmax/4, xmax*3/4, ymax/4, ymax*3/4);
     gid = winopen("Blending with Horizontal Circles");
     minsize(xmax/10, ymax/10);
     keepaspect(7, 5);
     winconstraints();

     RGBmode();
     doublebuffer();
     gconfig();

     zbuffer(TRUE);
     qdevice(ESCKEY);
     
     qenter(REDRAW, gid);

     mmode(MVIEWING);           /* double matrix mode since using lighting */

     lmdef(DEFLMODEL, 1, 10, light_model);
     lmdef(DEFLIGHT,  1, 14, white_light);
     lmdef(DEFMATERIAL, 1, 11, material1_plastic);

     lmdef(DEFLIGHT, 2, 14, other_light);

     lmdef(DEFMATERIAL, 2, 11, material2_plastic);
     lmdef(DEFMATERIAL, 3, 11, material3_plastic);

     lmbind(LMODEL, 1);
}

/* ----------------------------------------------------------------------- */
/* FUNCTION  drawscene :                                                   */
/* ----------------------------------------------------------------------- */

void  drawscene(float r1, float r2, float d, float L1, float L2,
                float start, float end)
{
     float  left, right, mid;

     left = -(r1+r1/(r1+r2)*d);
     if (left >= (-r2+r2/(r1+r2)*d)) left = -r2+r2/(r1+r2)*d;
     right = r1 - r1/(r1+r2)*d;
     if (right < r2+r2/(r1+r2)*d)  right = r2+r2/(r1+r2)*d;
     mid = (left+right)/2.0;

     czclear(0xFFFF, getgdesc(GD_ZMAX));

     perspective(450, (float)xmax/(float)ymax, 0.1, 100.0);

     pushmatrix();
          polarview(8.0, 300, 0, 0);
          translate(-2.0*mid, 0.0, 0.0);
          lmbind(LIGHT0, 1);
          lmbind(LIGHT1, 2);
          pushmatrix();
               rotate(angle, 'x');
               lmbind(MATERIAL, 1);
               fring(r1, r2, d, L1, L2, start, end);
               lmbind(MATERIAL, 2);
               fcone1(r1, r2, d, L1, L2, start, end);
               lmbind(MATERIAL, 3);
               fcone2(r1, r2, d, L1, L2, start, end);
               lmbind(MATERIAL, 1);
          popmatrix();
     popmatrix();

     swapbuffers();
}

void  info(void)
{
     printf("\n");
     printf("\n************************************************************");
     printf("\n*  Usage:  h-blend  r1 r2 d l1 l2 start_angle end_angle    *");
     printf("\n*                                                          *");
     printf("\n*  `h-blend' is a program showing the blending of two      *");
     printf("\n*  cones along two h-circles on a Dupin cyclide.           *");
     printf("\n*                                                          *");
     printf("\n*  command line arguments:                                 *");
     printf("\n*     r1            -- the radius of the larger principal  *");
     printf("\n*                      vertical circle                     *");
     printf("\n*     r2            -- the radius of the smaller principal *");
     printf("\n*                      vertical circle                     *");
     printf("\n*     d             -- the distance between the two circles*");
     printf("\n*     l1            -- the length of the tube from the 1st *");
     printf("\n*                      cone.  Positive (resp., negative)   *");
     printf("\n*                      means that the tube lies between    *");
     printf("\n*                      (resp., outside) the cone's vertex  *");
     printf("\n*                      and the tangent circle on the cone. *");
     printf("\n*     l2            -- the length of the tube of the 2nd   *");
     printf("\n*                      cone.                               *");
     printf("\n*     start_angle   -- the angle indicates the starting    *");
     printf("\n*                      position of the boundary H-circle   *");
     printf("\n*                      that is the  tangent circle on the  *");
     printf("\n*                      first cone.                         *");
     printf("\n*     end_angle     -- the angle indicates the ending      *");
     printf("\n*                      position of the boundary H-circle   *");
     printf("\n*                      that is the tangent circle on the   *");
     printf("\n*                      second cone.  start<end has to be   *");
     printf("\n*                      satisfied.  Thus, in some case,     *");
     printf("\n*                      negative values are required.       *");
     printf("\n*                                                          *");
     printf("\n*  NOTES:                                                  *");
     printf("\n*     (1) The boundary H-circles on the cyclide are        *");
     printf("\n          approximations of the true H-circles.  Thus, it  *");
     printf("\n*         is possible that the tubes DO NOT look like parts*");
     printf("\n*         of right cones.  This will be corrected in a new *");
     printf("\n*         version of this program.                         *");
     printf("\n*                                                          *");
     printf("\n*  Ching-Kuang Shene                          July/30/1992 *");
     printf("\n************************************************************");
     printf("\n");
}
 
