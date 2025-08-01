/* ======================================================================= */
/* PROGRAM  showring.c                                                     */
/*    This program demonstrates the use of the GL lighting model.          */
/* A torus is drawn using a green plastic material characteristic.         */
/* A single light source illuminates the object.                           */
/* ======================================================================= */

#include  <gl.h>
#include  <device.h>
#include  "ring.c"

void  initialize(void);
void  drawscene(float, float, float);

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
     POSITION, 0.0, -1.0, -1.0, 0.0, /* shining from the top.                */
     LCOLOR, 1.0, 1.0, 1.0,
     LMNULL
};


float green_plastic[] = {     /* define a material properties array        */
     AMBIENT, 0.2, 0.1, 0.1, 
     DIFFUSE, 0.8, 0.5, 1.0,
     SPECULAR, 0.0, 0.0, 0.0,
     SHININESS, 30.0,
     LMNULL
};

int  angle;

void  main(int argc, char *argv[])
{
     Boolean exitflag = FALSE;
     short   attached = 0;    /* attached to window                        */
     short   value;
     int     dev;
     float   r1, r2, d;

     if (argc < 4) {
          printf("Usage: showring outer_radius inner_radius\n");
          exit(0);
     }
     r1 = atof(argv[1]);
     r2 = atof(argv[2]);
     d  = atof(argv[3]);
     if ((r1 <= 0) || (r2 <= 0)) {
          printf("radii must be positive\n");
          exit(0);
     }
     angle = atoi(argv[4]);

     initialize();
     while (exitflag == FALSE) {
          drawscene(r1, r2, d);
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
     gid = winopen("Ring Cyclide");
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
     lmdef(DEFMATERIAL, 1, 11, green_plastic);

     lmdef(DEFLIGHT, 2, 14, other_light);

     lmbind(LMODEL, 1);
}

/* ----------------------------------------------------------------------- */
/* FUNCTION  drawscene :                                                   */
/* ----------------------------------------------------------------------- */

void  drawscene(float r1, float r2, float d)
{

     czclear(0xffffff, getgdesc(GD_ZMAX));

     perspective(450, (float)xmax/(float)ymax, 1.0, 10.0);

     pushmatrix();
          polarview(5.0, 0, 0, 0);
          lmbind(LIGHT0, 1);
          lmbind(LIGHT1, 2);
          pushmatrix();
               rotate(angle, 'x');
               lmbind(MATERIAL, 1);
               fring(r1, r2, d);
               lmbind(MATERIAL, 1);
          popmatrix();
     popmatrix();

     swapbuffers();
}

