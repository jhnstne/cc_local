/*
        File: GLUTgc.c
        Author: J.K. Johnstone
        Created: 22 October 1996
	Last modified: 30 October 1996
        Purpose: Given a rational spine r(u),
		 construct a rational generalized cylinder with spine r(u)
		 using a moving coordinate frame for the cross-section
		 that follows the tangent r'(u)
		 (i.e., for all u, the x-axis of the frame is r'(u)).

		 For simplicity, we presently input a set of points
		 and define r(u) as the interpolating B-spline.
	Reference: John K. Johnstone and James T. Williams (1996)
		   `Rational tangent-following frames for rational generalized
		   cylinders', UAB Technical report, November 1996.
*/

#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <REAL.h>
#include <Bez.h>
#include <Bez4d.h>
#include <Vec4d.h>
#include <Fit4d.h>
#include <Matrix.h>
#include <Misc.h>		/* PI */
#include <Vec.h>
#include <Quaternion.h>
#include <Fit3sphere.h>
#include <Inversion4d.h>
#include <GLUTDraw.h>

#define PTSPERBEZSEGMENT 5	/* # of points to draw on each Bezier segment */
#define NTANGENTFRAMECURVEPTS 100
#define NSAMPLES  100		/* NSAMPLES>1 dictates the granularity at which */
				/* the curve is sampled; */
		/* NSAMPLES points on the curve will have their tangents sampled */
#define NFRAMES 500		/* # of displayed frames */
#define TANGDELTA 0.1		/* deviation, in radians, from previous stored */
		/* tangent sample that forces new tangent sample to be stored */
		/* .017 is about one degree */

typedef struct {	/* stored sample */
  REAL u;	/* parameter value on spine */
  V3r  pt;	/* point at that parameter value */
  V3r  tang;	/* tangent at that parameter value */
  V4r  q;	/* quaternion representing some tangent-following frame */
		/* (the one chosen by our method) at that parameter value */
} Sample;

static char *RoutineName;
static void usage()
{
  printf("Usage is %s", RoutineName);
  printf("\n\t[-a] - animation\n");
  printf("\n\t < InputData\n");
}

static GLfloat   transx, transy, transz, rotx, roty, rotz, zoom;
static GLfloat   initrotx, initroty, initrotz;
static GLboolean rotate=0;	/* start rotating? */
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean rightMouseDown=0;
static int	 firstx=1;	/* first MOUSEX reading? */
static int 	 firsty=1;
static int	 oldx,oldy;	/* previous value of MOUSEX and MOUSEY */
static int	 DRAWSPINE=1;
static int  	 DRAWALLFRAMES=0;
static int	 DRAWPOINTSAMPLES=0;
static int	 DRAWTANGENTSAMPLES=0;
static int	 DRAWFRAMESAMPLES=1;
static int	 ANIMATION=0;
static int 	 DEBUG = 0;	/* print debug statements? */
V3r		 white =  {1.0, 1.0, 1.0};
V3r		 black =  {0.0, 0.0, 0.0};
V3r		 red =    {1.0, 0.0, 0.0};
V3r 		 green =  {0.0, 1.0, 0.0};
V3r		 blue =   {0.0, 0.0, 1.0};
V3r     	 yellow = {1.0, 1.0, 0.0};
V3r		 magenta= {1.0, 0.0, 1.0};
V3r		 cyan   = {0.0, 1.0, 1.0};

V3r     spineDisplay[NFRAMES+5];
V4r     frameDisplay[NFRAMES+5];
Sample *sample;   	/* stored tangent samples */
int     nSamples;
int	frame=0;	/* frame presently being drawn in animation */

void gfxinit(void)
{
/*GLfloat mat_ambient[]    = {0.8, 0.1, 0.1, 1.0}; */
  GLfloat mat_ambient[]    = {0.1745, 0.01175, 0.01175}; 
  GLfloat mat_specular[]   = {0.1, 0.1, 0.1, 1.0};
/*GLfloat mat_diffuse[]    = {0.8, 0.5, 1.0, 1.0}; */  		/* purple */
  GLfloat mat_diffuse[]    = {0.61424, 0.04136, 0.04136, 1.0}; 	/* red */
  GLfloat mat_emission[]   = {0.1, 0.1, 0.1, 1.0};
/*GLfloat high_shininess[] = { 30.0 }; */
  GLfloat high_shininess[] = { 0.6 * 128.0 };
  GLfloat light0_position[] = {-2.0, 0.0, 0.0, 0.0};		/* from left */
  GLfloat light1_position[] = {2.0, 0.0, 0.0, 0.0};		/* right */
  GLfloat light2_position[] = {0.0, -2.0, 0.0, 0.0};		/* back */
  GLfloat light3_position[] = {0.0, 2.0, 0.0, 0.0};		/* front */
  GLfloat light4_position[] = {0.0, 0.0, 2.0, 0.0};		/* top */
  GLfloat light5_position[] = {0.0, 0.0, -2.0, 0.0};		/* bottom */
  GLfloat light_ambient[]   = {0.2, 0.2, 0.2, 1.0};
  GLfloat whitelight[]      = {1.0, 1.0, 1.0, 1.0};

  glClearColor (1.0, 1.0, 1.0, 1.0);
/*glEnable(GL_DEPTH_TEST); */
  glShadeModel (GL_FLAT);

  glMaterialfv (GL_FRONT, GL_AMBIENT,   mat_ambient);
  glMaterialfv (GL_FRONT, GL_DIFFUSE,   mat_diffuse);
  glMaterialfv (GL_FRONT, GL_SPECULAR,  mat_specular);
  glMaterialfv (GL_FRONT, GL_SHININESS, high_shininess);
  glMaterialfv (GL_FRONT, GL_EMISSION,  mat_emission); 
/*glColorMaterial (GL_FRONT, GL_DIFFUSE); */        /* set material using color */ 
/*glEnable (GL_COLOR_MATERIAL); */

/*glLightModeli (GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);  */ /* more expensive */
  glLightfv (GL_LIGHT0, GL_POSITION, light0_position);  
  glLightfv (GL_LIGHT1, GL_POSITION, light1_position); 
  glLightfv (GL_LIGHT2, GL_POSITION, light2_position); 
  glLightfv (GL_LIGHT3, GL_POSITION, light3_position); 
  glLightfv (GL_LIGHT4, GL_POSITION, light4_position); 
  glLightfv (GL_LIGHT5, GL_POSITION, light5_position);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, whitelight);
  glLightfv (GL_LIGHT1, GL_DIFFUSE, whitelight);
  glLightfv (GL_LIGHT2, GL_DIFFUSE, whitelight);
  glLightfv (GL_LIGHT3, GL_DIFFUSE, whitelight);
  glLightfv (GL_LIGHT4, GL_DIFFUSE, whitelight);
  glLightfv (GL_LIGHT5, GL_DIFFUSE, whitelight);

/*glEnable (GL_LIGHTING); */
  glEnable (GL_LIGHT0);
  glEnable (GL_LIGHT1);
  glEnable (GL_LIGHT2);
  glEnable (GL_LIGHT3);
  glEnable (GL_LIGHT4);
  glEnable (GL_LIGHT5);

  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_POINT_SMOOTH); 
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST);
  glEnable (GL_LINE_SMOOTH);
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glPointSize (10.0);

  transx = 0.0;  transy = 0.0;  transz = 0.0;
  rotx = initrotx = 0.0;	/* 90 */
  roty = initroty = 0.0;
  rotz = initrotz = 0.0;	/* 40 */
  zoom = .75;
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -2.0, 2.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void Rotate (void)
{
  rotz += 2.0; 
  if (rotz > 360.0) 
    rotz -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void AnimateFrame (void)
{
  frame++;
  if (frame >= NFRAMES)
   {
    frame = 0; 
    sleep(2);
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) {
    if (rotate || ANIMATION)
      glutIdleFunc (NULL);
  }
  else if (rotate)
    glutIdleFunc (Rotate);
  else if (ANIMATION)
    glutIdleFunc (AnimateFrame);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  if (key == 27)	/* ESCAPE */
    exit(1);
  else		/* toggle rotation */
    if (rotate) {
      rotate=0;
      if (ANIMATION)
        glutIdleFunc (AnimateFrame);
      else
	glutIdleFunc (NULL);
    }
    else {
      rotate=1;
      glutIdleFunc (Rotate);
    }
}

/******************************************************************************/
/******************************************************************************/

void mouse (int button, int state, int x, int y)
{
  switch (button) {
  case GLUT_LEFT_BUTTON:
	switch (state) {
	case GLUT_DOWN: 
 	  leftMouseDown = firstx = firsty = 1;
	  glutSetCursor (GLUT_CURSOR_UP_DOWN);
	  break;
	case GLUT_UP: 
	  leftMouseDown = 0;
	  glutSetCursor (GLUT_CURSOR_INHERIT);
	  break;
	default: break;
	}
	break;
  case GLUT_MIDDLE_BUTTON:
	switch (state) {
	case GLUT_DOWN:
	  middleMouseDown = firstx = firsty = 1; 
	  glutSetCursor (GLUT_CURSOR_CYCLE);
	  break;
	case GLUT_UP: 		
	  middleMouseDown = 0; 
	  glutSetCursor (GLUT_CURSOR_INHERIT);
	  break;
	default: break;
	}
	break;
  default:
	break;
  }
}

/******************************************************************************/
/******************************************************************************/

void motion (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoom -= (float).02*(x-oldx);
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transx += .01*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transy += .01*(y-oldy); /* TRANSLATION: Y */
   }
  else if (middleMouseDown) 
   {
    if (firstx)  firstx=0;
    else { roty += .5*(x-oldx); if (roty > 360.0) roty -= 360.0; } /* ORI: Y */

    if (firsty)  firsty=0;
    else { rotx += .5*(y-oldy); if (rotx > 360.0) rotx -= 360.0; } /* ORI: X */
   }
  oldx = x;  
  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menu (int value)
{
  switch (value) {
  case 2:	DRAWSPINE = !DRAWSPINE;    	break;
  case 3: 	DRAWALLFRAMES = !DRAWALLFRAMES; break;
  default: 	break;
  }
  glutPostRedisplay();
}

void sampleMenu (int value)
{
 switch (value) {
 case 1:	DRAWPOINTSAMPLES   = !DRAWPOINTSAMPLES;		break;
 case 2:	DRAWTANGENTSAMPLES = !DRAWTANGENTSAMPLES;	break;
 case 3:	DRAWFRAMESAMPLES   = !DRAWFRAMESAMPLES;		break;
 default:	break;
 }
 glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void display (void)
{
  int i,j;
  V3r tangEndpt;

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transx, transy, transz);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  glScalef  (zoom, zoom, zoom);

  if (DRAWSPINE)
   {
    glColor3fv (blue);
    DrawCurve (spineDisplay, NFRAMES);
   }

  if (DRAWALLFRAMES && !ANIMATION)
   {
    for (i=0; i<NFRAMES; i++)
      DrawFrame (spineDisplay[i], frameDisplay[i]); 
   }

  if (DRAWPOINTSAMPLES)
   {
    glColor3fv (black);
    glPointSize (10.0);
    glBegin (GL_POINTS);
    for (i=0; i<nSamples; i++)
      glVertex3fv (sample[i].pt);
    glEnd();
   }

  if (DRAWTANGENTSAMPLES)
   {
    glColor3fv (black);
    glBegin (GL_LINES);
    for (i=0; i<nSamples; i++)
     {
      glVertex3fv (sample[i].pt);
      for (j=0; j<3; j++)
        tangEndpt[j] = sample[i].pt[j] + sample[i].tang[j];
      glVertex3fv (tangEndpt);
     }
    glEnd();
   }

  if (DRAWFRAMESAMPLES)		/* red along x-axis */
   {
    for (i=0; i<nSamples; i++)
      DrawFrame (sample[i].pt, sample[i].q);
   }

  if (ANIMATION)
   {
    DrawFrame (spineDisplay[frame], frameDisplay[frame]);
   }

/*  glColor3f (0.5, 0.5, 0.5);
  glLineWidth (1.0); 
  glutWireSphere (1.0, 20, 20); */

  /* draw tangent frame curve */
/*  glColor3f (1.0, 0.0, 0.0);
  glLineWidth (4.0);
  glBegin (GL_LINE_STRIP);
  for (i=0; i<=NTANGENTFRAMECURVEPTS; i++)
    glVertex3f (tfcurveDisplay[i][0],
	    	tfcurveDisplay[i][1],
		tfcurveDisplay[i][3]);
  glEnd(); */

  glPopMatrix();
  glutSwapBuffers ();
}

/******************************************************************************
	Build array of points on tangent frame curve associated with `tangent'.
******************************************************************************/

void PrepareTangentFrameCurve (V3r tangent, V4r **tfcurveDisplay)
{
  int i;
  REAL theta;
  V4r q, qtheta;	/* qtheta is quaternion for rotation by theta about x-axis */

  *tfcurveDisplay = (V4r *) malloc ((NTANGENTFRAMECURVEPTS+1) * sizeof(V4r));
/*  QionForX2V (tangent, q); */
  q[2] = q[3] = 0;   		/* for quaternion with $x_3=0$ */
  q[0] = q[1] = .7071;	/* 1/sqrt(2) */
  for (i=0; i<=NTANGENTFRAMECURVEPTS; i++)
   {
    theta = (float) i/(float) NTANGENTFRAMECURVEPTS * 2.0 * PI;
    qtheta[0] = cos(theta/2.0);
    qtheta[1] = sin(theta/2.0);
    qtheta[2] = 0;
    qtheta[3] = 0;
    QionMultiply (q, qtheta, (*tfcurveDisplay)[i]);
   }
}

/******************************************************************************
	Step along the spine in constant increments
	and record the sample and its tangent if the tangent change is
	greater than TANGDELTA.
******************************************************************************/

static void ComputeTangentSamples (Bez *spine, Sample **sample, int *nSamples)
{
  REAL  u;
  Bez   spineTang; /* hodograph of spine curve */
  REAL  angle, delta;
  V3r   uTang;	   
  int   lastSample; /* index of last sample in tangent sample array */

  Hodograph (spine, &spineTang);
  *sample = (Sample *) malloc ((NSAMPLES+5) * sizeof(Sample));  /* NSAMPLES is an upperbound */
  (*sample)[0].u = spine->knots[0];	  /* 1st stored tangent = tangent at 1st endpt */
  Tangent (&spineTang, spine->knots[0], (*sample)[0].tang);
  lastSample = 0;
  delta = (spine->knots[spine->L] - spine->knots[0]) / (NSAMPLES-1);
  /* IMPROVEMENT: SHOULD SAMPLE MORE DENSELY (CHANGE DELTA) WHEN CURVATURE IS HIGH */
  for (u=spine->knots[0];		/* sample the spine and store tangent */
       u<=spine->knots[spine->L];	/* whenever tangent change > TANGDELTA */
       u += delta)
   {
    Tangent (&spineTang, u, uTang);
    angle = Angle (uTang, (*sample)[lastSample].tang);
    if (angle > TANGDELTA)
     {
      lastSample++;
      (*sample)[lastSample].u = u;
      Copy3r ((*sample)[lastSample].tang, uTang);
printf("Tangent sample %i = (%f,%f,%f)\n", lastSample, uTang[0], uTang[1], uTang[2]);
     }
   }
  if ((*sample)[lastSample].u != spine->knots[spine->L])  
   {					/* place sample at end of spine */
    lastSample++;
    (*sample)[lastSample].u = spine->knots[spine->L];
    Tangent (&spineTang, spine->knots[spine->L], (*sample)[lastSample].tang);
   }
  *nSamples = lastSample+1;
}

/******************************************************************************
	Compute quaternion samples associated with tangent samples,
	by computing great circles of tangents on S^3 and 
	repeatedly finding closest point to previous quaternion sample.
******************************************************************************/

static void ComputeQuaternionSamples (Sample *sample, int nSamples)
{
  int   i,j;
  V4r   q2;	    /* quaternion that rotates x-axis into tangent */
  V4r   q1; 	    /* quaternion that rotates around x-axis */
  V4r   greatCircle1, greatCircle2; 	/* 2 nonantipodal pts on great circle */
  V4r   line1, line2; 			/* 2 pts on line (inverse of great circle) */
  V4r   invLastSample;			/* inverse of last sample */
  V4r   invClosestPt;			/* closest point on line */
					/* = inverse of closest pt on circle */

  QionFromX2V (sample[0].tang, sample[0].q);  /* set 1st quaternion sample */
					     /* to q2q1(0)=q2 */
printf("quaternion sample 0 = (%f,%f,%f,%f)", sample[0].q[0], sample[0].q[1],
			sample[0].q[2], sample[0].q[3]);
  for (i=1; i<nSamples; i++)		/* compute next quaternion */
   {
    QionFromX2V (sample[i].tang, q2);
/* printf("q2=(%f,%f,%f,%f)\n", q2[0], q2[1], q2[2], q2[3]); */
    q1[0] = cos(PI/4); q1[1] = sin(PI/4); q1[2] = 0; q1[3] = 0;	/* q1(PI/2) */
    QionMultiply (q2, q1, greatCircle1);	/* 1st pt on great circle */
    q1[0] = 0;         q1[1] = 1;  	  q1[2] = 0; q1[3] = 0;	/* q1(PI) */
    QionMultiply (q2, q1, greatCircle2);	/* 2nd pt on great circle */
printf("\tgreatCircle2 = (%f,%f,%f,%f)\n", greatCircle2[0], greatCircle2[1], greatCircle2[2], greatCircle2[3]);
	/* BIZARRELY, THIS PRINT STATEMENT IS NECESSARY FOR CORRECT COMPILATION */

    Invert4d (greatCircle1, line1, q2, 1.0);	/* invert w.r.t q2 */
    Invert4d (greatCircle2, line2, q2, 1.0);
    Invert4d (sample[i-1].q, invLastSample, q2, 1.0);

    ClosestPtOnLine4r (invLastSample, invClosestPt, line1, line2);
    Invert4d (invClosestPt, sample[i].q, q2, 1.0);  
		/* inverse of closest pt is closest pt on 3-sphere, */
		/* since inversion preserves angles */

/* if (Dot4r (sample[i-1].q, sample[i].q) < 0) */	/* angle(ith, i+1st samples) > PI/2 */
			/* so flip i+1st to equivalent antipodal quaternion */
			/* which will be closer */
/*    {
      for (j=0; j<4; j++)  
				
	sample[i].q[j] = -sample[i].q[j];
      printf("Flipping sample %i...\n", i);
     } */

printf("quaternion sample %i = (%f,%f,%f,%f)", i, sample[i].q[0], sample[i].q[1],
			sample[i].q[2], sample[i].q[3]);
   }
}

/******************************************************************************/
/******************************************************************************/

int main(int argc, char **argv)
{
  int   ArgsParsed=0;
  int   sMenu;
  int   i;
  REAL  u;	/* parameter on spine */
  int   n;	/* # of data points */
  V3r   *pt;	/* array of data points */
  Bez   spine;  /* spine curve, defined as interpolating $C^2$ Bezier curve */
		/* through the data points */
  RatBez4d frameCurve;		/* quaternion curve */
  V4r   *qSample;	/* quaternion samples extracted from samples */
  REAL  *uSample;
  REAL  delta;
/*V3r   tangent; */

  RoutineName = argv[ArgsParsed++];
  while (ArgsParsed < argc)
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
       {
	case 'a':
		ANIMATION=1;  
		DRAWFRAMESAMPLES=0;
		break;
        case 'h': 
	default: 
		usage(); exit(-1);
       }
    else {usage(); exit(-1);}
 
  /************************************************************/

  scanf("%i",&n);					/* input */
  pt = (V3r *) malloc ((n+5) * sizeof(V3r));
  for (i=0; i<n; i++)
    scanf ("%f %f %f", &(pt[i][0]), &(pt[i][1]), &(pt[i][2]));
  FitCubicBez (n, pt, &spine);

  ComputeTangentSamples    (&spine, &sample, &nSamples);
  for (i=0; i<nSamples; i++)
    PointOnBez (&spine, sample[i].u, sample[i].pt);

  ComputeQuaternionSamples (sample, nSamples);
/* for (i=0; i<nSamples; i++)
  printf("sample[%i].q = (%f,%f,%f,%f)\n", i, sample[i].q[0], sample[i].q[1], 
		sample[i].q[2], sample[i].q[3]); */

  /* gather quaternion samples into single array to pass to FitSexticBezOn3sphere */
  qSample = (V4r *) malloc (nSamples * sizeof(V4r));
  for (i=0; i<nSamples; i++)
    Copy4r (qSample[i], sample[i].q);

  /* gather u samples into single array to pass to FitSexticBezOn3sphere */
  uSample = (REAL *) malloc (nSamples * sizeof(REAL));
  for (i=0; i<nSamples; i++)
    uSample[i] = sample[i].u;

  FitSexticBezOn3sphere    (nSamples, qSample, 1, uSample, &frameCurve);
	/* SHAPE OF FRAMECURVE WILL BE AFFECTED BY ASSIGNING KNOTS OF SPINE */
	/* CHECK IN GLUTS3CURVE THAT FRAMECURVE STILL LOOKS OK BY SPITTING OUT */
	/* QUATERNIONS AND KNOT VALUES TO DATA FILE */
	/* PROBLEM IS IN VISUALIZING: QUATERNIONS WON'T BE IN X_4=0 PLANE */
  free (qSample);

/*  PrintRatBez_4d (&frameCurve); */

  /* test that frame curve does what we expect at knot values */
/*  for (i=0; i<nSamples; i++)
    PointOnRatBez_4d (&frameCurve, spine.knots[i], frameDisplay[i]);  */

  /* precompute a collection of frames on spine */
  delta = (spine.knots[spine.L] - spine.knots[0]) / (NFRAMES-1);
  for (i=0, u=spine.knots[0]; i<NFRAMES; i++, u += delta)
   {
    PointOnBez (&spine, u, spineDisplay[i]);
    PointOnRatBez_4d (&frameCurve, u, frameDisplay[i]);
   } 

/*  measure deviation from true tangent at each point */

  /* VISUALIZE STEPPING ALONG SPINE AND CHOOSING SAMPLES */
  /* WITH TANGENT ALWAYS DRAWN AND LEAVING PEBBLES BEHIND AS IT SAVES A SAMPLE */

	/* stub */

/* tangent[0] = tangent[1] = tangent[2] = 1;
*  PrepareTangentFrameCurve (tangent, &tfcurveDisplay);
*/

/*  for (i=0; i<=NTANGENTFRAMECURVEPTS; i++)
    printf("%f %f %f %f\n", tfcurveDisplay[i][0], tfcurveDisplay[i][1],
		tfcurveDisplay[i][2], tfcurveDisplay[i][3]); */

  /************************************************************/

  glutInitWindowPosition (0,0);
  glutInitWindowSize (500,500);
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow ("Rational generalized cylinder");
  glutDisplayFunc (display);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motion);
  glutReshapeFunc (reshape);
  glutVisibilityFunc (visibility);
  gfxinit();
  sMenu = glutCreateMenu (sampleMenu);
  glutAddMenuEntry ("Points", 1);
  glutAddMenuEntry ("Tangents", 2);
  glutAddMenuEntry ("Frames", 3);
  glutCreateMenu (menu);
  glutAddSubMenu   ("Samples", sMenu);
  glutAddMenuEntry ("Spine curve", 2);
  glutAddMenuEntry ("Frames", 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);
  glutMainLoop();
  if (ANIMATION)
    glutIdleFunc (AnimateFrame);
  return 0;             /* ANSI C requires main to return int. */
}

