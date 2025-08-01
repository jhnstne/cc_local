#include  <gl.h>
#include  <device.h>
#include  <stdlib.h>
#include  <stdio.h>
#include  <math.h>
#include  "ring2.c"

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
     angle = atoi(argv[8]);

     initialize();
     while (exitflag == FALSE) {
          drawscene(r1, r2, d, L1, L2, start, end);
          while ((exitflag == FALSE) && (qtest() || !attached)) {
               dev = qread(&value);
               if ((dev == ESCKEY) && (value == 0))
                    exitflag = TRUE;
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
     gid = winopen("Blending with Vertical Circles #1");
     minsize(xmax/10, ymax/10);
     keepaspect(xmax, ymax);
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

     czclear(0xFFFF,getgdesc(GD_ZMAX));

     perspective(450, (float)xmax/(float)ymax, 1.0, 10.0);

     pushmatrix();
          polarview(6.0, 300, 0, 0);
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
     printf("\n*  Usage: v-blend  r1 r2 dist l1 l2 start_angle end_angle  *");
     printf("\n*                                                          *");
     printf("\n*  Parameter Descriptions:                                 *");
     printf("\n*     r1                -- the radius of the larger        *");
     printf("\n*                          principal horizontal circle.    *");
     printf("\n*     r2                -- the radius of the smaller       *");
     printf("\n*                          principal horizontal circle.    *");
     printf("\n*     dist              -- the distance between the centers*");
     printf("\n*     l1                -- the length of the tube attached *");
     printf("\n*                          to the start_angle part.        *");
     printf("\n*     l2                -- the length of the tube attached *");
     printf("\n*                          to the end_angle part.          *");
     printf("\n*     start_angle       -- the starting angle of the piece *");
     printf("\n*                          of the blending cyclide.        *");
     printf("\n*     end_angle         -- the ending angle of the piece   *");
     printf("\n*                          of the blending cyclide.        *");
     printf("\n*                                                          *");
     printf("\n*  Ching-Kuang Shene                        August/03/1992 *");
     printf("\n************************************************************");
     printf("\n");
}

