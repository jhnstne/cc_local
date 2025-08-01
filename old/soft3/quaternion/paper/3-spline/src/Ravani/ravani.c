/*
  File:    	 ravani.c
  Author:  	 J.K. Johnstone
  Created: 	 18 December 1998
  Last modified: 7 January 1999
  Purpose: 	 Input: n unit quaternions (as 4-vectors) and n knots.
  	   	 Output: rational interpolating quaternion spline 
		 	 (natural cubic spline, with zero initial 
			  angular velocity and acceleration).
  Reference: 	 F.C. Park and B. Ravani, Smooth Invariant Interpolation of Rotations,
  	     	 ACM Transactions on Graphics, 16(3), July 1997, 277-295.
  History:	 1/7/99: change float to double for no overflow in large examples
  		 1/8/99: back to float (not the problem)
*/

#include <GL/glut.h>
#include <GL/glu.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "Bez4d.h"
#include "AllColor.h"
#include "Fit4d.h"			/* yes, need it */
#include "Fit3sphere.h"
#include "Geom.h"
#include "Misc.h"
#include "Quality.h"			/* yes, need it */
#include "REAL.h"
#include "Vec.h"
#include "Vec4d.h"

#define NQUALITYPROBES 	 5000	/* # of points to discretely sample in TangAccel_Spherical */
#define ESSENTIALLYZERO .0001

static char *RoutineName;
static void usage()
{
  printf("Usage is %s\n", RoutineName);
  printf("\t[-d]   - (d)on't visualize (model in true 4D)\n");
  printf("\t[-P #] - # of probes for covariant acceleration computation (default=50)\n");
  printf("\t< InputData\n");
}

static GLfloat   transx, transy, transz, rotx, roty, rotz, zoom;
static GLfloat   initrotx, initroty, initrotz;
static GLboolean rotate		    =0;	/* start rotating? */
static GLboolean PAN		    =0;	/* rotate object back and forth for 3d effect? */
static int 	 panLeft	    =0; /* control panning for 3d effect */
static int 	 panRight	    =1;
static GLboolean leftMouseDown	    =0;
static GLboolean middleMouseDown    =0;
static int	 firstx		    =1;	/* first MOUSEX reading? */
static int 	 firsty		    =1;
static int	 oldx,oldy;		/* previous value of MOUSEX and MOUSEY */

static GLboolean DEBUG		    =0;	/* print debug statements? */
static GLboolean VISUALIZE	    =0;	/* display? */

static GLboolean DRAWDATAPTS	    =1;
static GLboolean DRAWORDEROFPTS     =0; /* annotate data point by its number? */
static GLboolean DRAWS3CURVE	    =1;

typedef float Matrix3[3][3];	/* 3x3 matrix type */

int       n;			/* # of data points */
V4r      *pt;			/* data points as unit quaternions */
Matrix3  *R;			/* data points as rotation matrices */
float	 *knot;			/* knot values at data points (part of input) */
int	  nSample;
V4r      *ptSample;		/* point samples from Lie curve on S3 */
Bez4d     Liecurve;		/* Bezier curve approx. of Lie curve */
RatBez4d  RatLiecurve;		/* rational Bezier version, for compatibility with accel software */
int       nProbes = NQUALITYPROBES; /* # probes in covariant accel computation */
float	  covAccel=0;

void gfxinit()
{
  glClearColor (1.0, 1.0, 1.0, 1.0);
  glShadeModel (GL_FLAT);
  glEnable    (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable    (GL_POINT_SMOOTH); 
  glHint      (GL_POINT_SMOOTH_HINT, GL_FASTEST);
  glEnable    (GL_LINE_SMOOTH);
  glHint      (GL_LINE_SMOOTH_HINT, GL_NICEST);
  glPointSize (10.0);
  glLineStipple (1, 0xAAAA);
  transx = 0.0;  transy = 0.0;  transz = 0.0;
  rotx = initrotx = 90.0;
  roty = initroty = 0.0;
  rotz = initrotz = 0.0;	/* 73 for #2 visible control polygon; previously 40 */
  zoom = 1.5;
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -6.0, 6.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void Rotate ()
{
  rotz += 2.0; 
  if (rotz > 360.0) rotz -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void Pan ()
{
  if (panLeft)
   {
    rotz += 0.1;
    if (rotz > 360.0) rotz -= 360.0;
    panLeft++;
    if (panLeft==200) { panLeft=0; panRight=1; }
   }
  else if (panRight)
   {
    rotz -= 0.1;
    if (rotz < 0.0) rotz += 360.0;
    panRight++;
    if (panRight==200) { panRight=0; panLeft=1; }
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) {
    if (rotate || PAN) glutIdleFunc (NULL);
  }
  else if (rotate)     glutIdleFunc (Rotate);
  else if (PAN)	       glutIdleFunc (Pan);
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

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1);	break;		/* ESCAPE */
  case 'r':	if (rotate) { rotate=0; glutIdleFunc (NULL); }  /* toggle rotation */
		else 	    { rotate=1; PAN=0; glutIdleFunc (Rotate); } break;
  case 'p':	if (PAN) { PAN=0; glutIdleFunc (NULL); }	/* toggle pan */
  		else 	 { PAN=1; rotate=0; glutIdleFunc (Pan); } break;
  default:	break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menu (int value)
{
  switch (value) {
  case 1:	DRAWDATAPTS = !DRAWDATAPTS;    			break;
  case 9:	DRAWS3CURVE = !DRAWS3CURVE; 			break;
  case 18: 	DRAWORDEROFPTS = !DRAWORDEROFPTS;		break;
  default: 	break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void display ()
{
  int i,j;

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transx, transy, transz);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  glScalef  (zoom, zoom, zoom);

  glLineWidth (1.0);
  glColor3fv (Grey);
  glutWireSphere (1.0, 40, 40);
  
  if (DRAWDATAPTS)
   {
    glColor3fv (Blue);
    glBegin (GL_POINTS);	/* project to x3=0 in original space */
    for (i=0; i<n; i++)  glVertex3f (pt[i][0], pt[i][1], pt[i][3]);
    glEnd();
   }
  if (DRAWS3CURVE) 
   {
    glColor3fv (Blue);
    glLineWidth (4.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<2*nProbes; i++)
      glVertex3f (ptSample[i][0], ptSample[i][1], ptSample[i][3]);
    glEnd();
   }
  	/***************************/
  if (DRAWORDEROFPTS)
   {
    glColor3fv (Black);
    for (i=0; i<n; i++)
     {
      glRasterPos3f (pt[i][0], pt[i][1], pt[i][3]+.1);
      if (i<9)
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 49+i);   /* `1' = 49 in ASCII */
      else
       {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 48+(i+1)/10);   /* `0' = 48 in ASCII */
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 48+((i+1)%10)); 
       }
     }
   }

  glPopMatrix();
  glutSwapBuffers ();
}

/******************************************************************************
	Translate from skew-symmetric matrix (in so(3)) to 3-vector.
	element of so(3) is represented as a 3-vector as follows (p. 281):

		0   -r3   r2
		r3   0   -r1
	       -r2   r1   0
	       
******************************************************************************/

static void skew2vec (Matrix3 mat, V3r vec)
{
  vec[0] = mat[2][1];
  vec[1] = mat[0][2];
  vec[2] = mat[1][0];
}

/******************************************************************************
	Inverse of skew2vec: from 3-vector to assoc. 3x3 skew-symmetric matrix.	       
******************************************************************************/

static void vec2skew (const V3r vec, Matrix3 mat)
{
  mat[0][0] = mat[1][1] = mat[2][2] = 0.;
  mat[2][1] = vec[0];  mat[1][2] = -vec[0];
  mat[0][2] = vec[1];  mat[2][0] = -vec[1];
  mat[1][0] = vec[2];  mat[0][1] = -vec[2];
}

/******************************************************************************
	Matrix multiplication: compute AB.
******************************************************************************/

static void matmatmult (Matrix3 a, Matrix3 b, Matrix3 ab)
{
  int i,j,k;
  
  for (i=0; i<3; i++)
    for (j=0; j<3; j++)
     {
      ab[i][j]=0.;
      for (k=0; k<3; k++)  ab[i][j] += a[i][k] * b[k][j];
     }
}

/******************************************************************************
	Matrix transpose.
******************************************************************************/

static void transpose (Matrix3 a, Matrix3 at)
{
  int i,j;
  for (i=0; i<3; i++) for (j=0; j<3; j++) at[i][j] = a[j][i];
}

/**************************************************************
	Compute cross product of U and V, placing result in W.
***************************************************************/

static void cross (V3r u, V3r v, V3r w)
{
 w[0] = u[1]*v[2] - u[2]*v[1];
 w[1] = u[2]*v[0] - u[0]*v[2];
 w[2] = u[0]*v[1] - u[1]*v[0];
}

/******************************************************************************
	Exponential mapping of Lie algebra.
	Given an element of the Lie algebra so(3) (a skew-symmetric 3x3 matrix)
	produce an element of the Lie group SO(3).
	In general, exponential map is e(A) = I + A + A^2/2! + ...
	but this is simplified for so(3).
	Reference: p. 282, Park and Ravani, formula (5)
******************************************************************************/

void Lieexp (const V3r skew, Matrix3 rot)
{
  int i,j;
  float norm, foo, bar, m[3][3], mm[3][3];
  
  norm = sqrt(skew[0]*skew[0] + skew[1]*skew[1] + skew[2]*skew[2]);
  vec2skew (skew, m);
  matmatmult (m, m, mm);
  foo = sin(norm) / norm;
  bar = (1-cos(norm)) / (norm*norm);
  for (i=0; i<3; i++)
    for (j=0; j<3; j++)
      if (i==j) rot[i][j] = 1 + foo * m[i][j] + bar * mm[i][j];
      else      rot[i][j] =     foo * m[i][j] + bar * mm[i][j];
}

/******************************************************************************
	Inverse of the exponential map of Lie algebra.
	Given element of SO(3) (a rotation matrix, a member of the Lie group)
	produce an element of so(3) (a skew-symmetric matrix, a member of
	the Lie algebra), the element that maps to the rotation matrix
	under the exponential map.
	element of so(3) is represented as a 3-vector as follows (p. 281):

		0   -r3   r2
		r3   0   -r1
	       -r2   r1   0

	Reference: p. 282, Park and Ravani, formula (6).
******************************************************************************/

void Lielog (const Matrix3 rot, V3r skew)
{
  int   i,j;
  float phi, trace, c;
  float mat[3][3];
  
  trace = rot[0][0] + rot[1][1] + rot[2][2];
  if (trace != -1)
   {
    phi = acos((trace-1.)/2.);	/* yes, this is in [0,PI] */
    c = phi / (2*sin(phi));
    for (i=0;i<3;i++) for (j=0;j<3;j++) mat[i][j] = c * (rot[i][j] - rot[j][i]);
    skew2vec (mat, skew);
   }
  else
   {
    printf("Error in Lielog: trace is -1.\n");
    exit(-1);
   }
}

/******************************************************************************
	Translate a unit quaternion to the equivalent 3x3 rotation matrix.
	Reference: Shoemake, p. 253, but using correct offdiagonal entries
	(m_{ij} -> -m_{ij} if i \neq j in Shoemake's formula).
******************************************************************************/

void Qion2RotMatrix (float q[4], Matrix3 M)
{
  float w,x,y,z;

  w = q[0]; x = q[1]; y = q[2]; z = q[3];
  
  M[0][0] = 1 - 2*y*y - 2*z*z;			/* 1st column */
  M[1][0] = 2*x*y + 2*w*z;
  M[2][0] = 2*x*z - 2*w*y;

  M[0][1] = 2*x*y - 2*w*z;			/* 2nd column */
  M[1][1] = 1 - 2*x*x - 2*z*z;
  M[2][1] = 2*y*z + 2*w*x;

  M[0][2] = 2*x*z + 2*w*y;			/* 3rd column */
  M[1][2] = 2*y*z - 2*w*x;
  M[2][2] = 1 - 2*x*x - 2*y*y;
}

/******************************************************************************
	Translate a rotation matrix to the equivalent unit quaternion.
	Reference: Shoemake, p. 253, but using correct offdiagonal entries
	(m_{ij} -> -m_{ij} if i \neq j in Shoemake's formula).
******************************************************************************/

void RotMatrix2Qion (Matrix3 m, V4r q)
{
  float w2,x2,y2;

  w2 = (1 + m[0][0] + m[1][1] + m[2][2]) / 4;
  if (fabs(w2) > ESSENTIALLYZERO)		/* not `zero' */
   {
    q[0] = sqrt(w2);
    q[1] = (m[2][1] - m[1][2]) / (4*q[0]);
    q[2] = (m[0][2] - m[2][0]) / (4*q[0]);
    q[3] = (m[1][0] - m[0][1]) / (4*q[0]);
   }
  else
   {
    q[0] = 0;  x2 = (-m[1][1] - m[2][2]) / 2;
    if (fabs (x2) > ESSENTIALLYZERO)
     {
      q[1] = sqrt(x2);
      q[2] = m[0][1] / (2*q[1]);
      q[3] = m[0][2] / (2*q[1]);
     }
    else
     {
      q[1] = 0;  y2 = (1 - m[2][2]) / 2;
      if (fabs(y2) > ESSENTIALLYZERO)
       {
	q[2] = sqrt(y2);  q[3] = m[1][2] / (2*q[2]);
       }
      else
       {
	q[2] = 0;  q[3] = 1;
       }
     }
   }
}

/******************************************************************************/
/******************************************************************************/

int main(int argc, char **argv)
{
  int      i,j,k;
  int      ArgsParsed=0;
  Matrix3  Rt, RR;
  V3r     *r;		/* skew-symmetric matrices assoc with data pts */
  Matrix3 *A;
  float   norm, foo, bar;
  Matrix3  m,mm;
  V3r      s,t,u;
  V3r     *a,*b,*c;
  float   cosnorm, sinnorm, dot, norm2, foo1, foo2, foo3, foo4;
  V3r      st, su, sst, tst, ssu;
  float    delta, param, tau, tau2, tau3;
  Matrix3  exptau, Rtau;
  V3r 	   tauvec;

  RoutineName = argv[ArgsParsed++];
  while (ArgsParsed < argc)
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
       {
	case 'd':	VISUALIZE = 0; 		break;
	case 'P':	nProbes = atoi(argv[ArgsParsed++]); break;
        case 'h': 
	default: 	usage(); exit(-1);
       }
    else { usage(); exit(-1); }
 
  /************************************************************/
  
  scanf ("%i", &n);
  if (n < 2) FatalError ("Need at least two input points\n");
  pt   = (V4r *) malloc (n * sizeof(V4r));
  knot = (float *) malloc (n * sizeof(float));
  R    = (Matrix3 *) malloc (n * sizeof(Matrix3));
  r    = (V3r *) malloc (n * sizeof(V3r));
  A    = (Matrix3 *) malloc (n * sizeof(Matrix3));
  a    = (V3r *) malloc (n * sizeof(V3r));
  b    = (V3r *) malloc (n * sizeof(V3r));
  c    = (V3r *) malloc (n * sizeof(V3r));
  nSample = 2*nProbes;
  ptSample = (V4r *) malloc (nSample * sizeof(V4r));
  for (i=0; i<n; i++)  scanf ("%f %f %f %f", pt[i], pt[i]+1, pt[i]+2, pt[i]+3);
  for (i=0; i<n; i++)  scanf ("%f", knot+i);
  
  if (VISUALIZE)
   {
    for (i=0; i<n; i++)  { pt[i][2] = 0; Unit4r (pt[i], pt[i]); }
    printf("Input projected to x3=0 hyperplane for visualization...\n");
   } 
  for (i=0; i<n; i++)  Qion2RotMatrix (pt[i], R[i]);
  
  /* preprocessing */
  for (i=1; i<n; i++)
   {
    transpose  (R[i-1], Rt);
    matmatmult (Rt, R[i], RR);
    Lielog     (RR, r[i]);
/* printf("r[%i]: (%f,%f,%f)\n", i, r[i][0], r[i][1], r[i][2]); */
		  
    /* compute A[i] */
    norm = sqrt(r[i][0]*r[i][0] + r[i][1]*r[i][1] + r[i][2]*r[i][2]);
    vec2skew (r[i], m);
    matmatmult (m, m, mm);
    foo = (1-cos(norm)) / (norm*norm);
    bar = (norm - sin(norm)) / (norm*norm*norm);
    for (j=0; j<3; j++)
      for (k=0; k<3; k++)
        if (j==k) A[i][j][k] = 1 - foo * m[j][k] + bar * m[j][k];
        else      A[i][j][k] =   - foo * m[j][k] + bar * m[j][k];
   }
   
  /* initialization */
  for (i=0; i<3; i++) { b[1][i] = c[1][i] = 0.; a[1][i] = r[1][i]; }
  
  /* iteration */
  for (i=2; i<n; i++)
   {
    for (j=0; j<3; j++)
     {
      s[j] = r[i][j];
      t[j] = 3*a[i-1][j] + 2*b[i-1][j] + c[i-1][j];
      u[j] = 6*a[i-1][j] + 2*b[i-1][j];
     }
    printf("s[%i]: (%f,%f,%f)\n", i, s[0],s[1],s[2]);
    printf("t[%i]: (%f,%f,%f)\n", i, t[0],t[1],t[2]);
    printf("u[%i]: (%f,%f,%f)\n", i, u[0],u[1],u[2]);
    for (j=0; j<3; j++)
     {
      c[i][j] = 0.;
      for (k=0; k<3; k++)  c[i][j] += A[i-1][j][k] * c[i-1][k];
     }
     /* doublechecked to here */

    norm = sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
/* printf("norm: %f\n", norm); */
    norm2 = norm*norm;
    sinnorm = sin(norm);
    cosnorm = cos(norm);
    dot = s[0]*t[0] + s[1]*t[1] + s[2]*t[2];
    foo1 = (dot * (2*cosnorm + norm*sinnorm  - 2)) / (norm2*norm2);
/* printf("foo1: %f\n", foo1); */
    foo2 = (1 - cosnorm) / norm2;
/* printf("foo2: %f\n", foo2); */
    foo3 = (dot * (3*sinnorm - norm*cosnorm - 2*norm)) / (norm2*norm2*norm);
/* printf("foo3: %f\n", foo3); */
    foo4 = (norm - sinnorm) / (norm2*norm);
/* printf("foo4: %f\n", foo3); */
    cross (s,t,st);
/* printf("st: (%f,%f,%f)\n", st[0],st[1],st[2]); */
    cross (s,u,su);
    cross (s,st,sst);
    cross (t,st,tst);
    cross (s,su,ssu);
    for (j=0; j<3; j++)
      b[i][j] = .5 * (u[j] - foo1 * st[j] - foo2 * su[j]
      		      + foo3 * sst[j] + foo4 * (tst[j] + ssu[j]));
    for (j=0; j<3; j++)
      a[i][j] = s[j] - b[i][j] - c[i][j]; 
printf("a[%i]: (%f,%f,%f)\n", i, a[i][0], a[i][1], a[i][2]);
printf("b[%i]: (%f,%f,%f)\n", i, b[i][0], b[i][1], b[i][2]);
printf("c[%i]: (%f,%f,%f)\n", i, c[i][0], c[i][1], c[i][2]);
   }
   
  /* result: get point samples */
  delta = (knot[n-1] - knot[0]) / (nSample-1);	/* step size between samples */
  for (k=0;k<4;k++) ptSample[0][k] = pt[0][k];
  param = knot[0] + delta;
  for (j=1; j<nSample-1; j++)
   {
    i = 1;  		/* find knot interval [t_i-1, t_i] containing param */
    while (!(param >= knot[i-1] && param <= knot[i]))  i++;
    	/* translate global param to local tau */
    tau = (param - knot[i-1]) / (knot[i] - knot[i-1]);  
/* printf("tau: %f\n", tau); */
    tau2 = tau*tau;  tau3 = tau*tau2;
    	/* compute rotation matrix R(t) */
    for (k=0; k<3; k++) tauvec[k] = a[i][k]*tau3 + b[i][k]*tau2 + c[i][k]*tau;
/* printf("tauvec: (%f,%f,%f)\n", tauvec[0], tauvec[1], tauvec[2]); */
    Lieexp (tauvec, exptau);
    matmatmult (R[i-1], exptau, Rtau);
    RotMatrix2Qion (Rtau, ptSample[j]);
    param += delta;
   }
  for (k=0;k<4;k++) ptSample[nSample-1][k] = pt[n-1][k];
  
  printf("Point samples from Lie curve:\n");
  for (i=0; i<nSample; i++)
    printf("(%f,%f,%f,%f)\n", ptSample[i][0], ptSample[i][1], ptSample[i][2], ptSample[i][3]);
  
  /* interpolate a 4d Bezier curve through the samples */
  FitCubicBez_4d (nSample, ptSample, &Liecurve);
  
  /* translate Liecurve into rational Bezier so that we can use CovariantAccel_Spherical */
  RatLiecurve.d = Liecurve.d;
  RatLiecurve.L = Liecurve.L;
  RatLiecurve.knots = (float *) malloc ((Liecurve.L+1)*sizeof(float));
  for (i=0; i<=Liecurve.L; i++)  RatLiecurve.knots[i] = Liecurve.knots[i];
  RatLiecurve.x1 = (float *) malloc ((Liecurve.d * Liecurve.L + 1)*sizeof(float));
  RatLiecurve.x2 = (float *) malloc ((Liecurve.d * Liecurve.L + 1)*sizeof(float));
  RatLiecurve.x3 = (float *) malloc ((Liecurve.d * Liecurve.L + 1)*sizeof(float));
  RatLiecurve.x4 = (float *) malloc ((Liecurve.d * Liecurve.L + 1)*sizeof(float));
  RatLiecurve.weights = (float *) malloc ((Liecurve.d * Liecurve.L + 1)*sizeof(float));
  for (i=0; i<=Liecurve.d * Liecurve.L; i++)
   {
    RatLiecurve.x1[i] = Liecurve.x1[i];
    RatLiecurve.x2[i] = Liecurve.x2[i];
    RatLiecurve.x3[i] = Liecurve.x3[i];
    RatLiecurve.x4[i] = Liecurve.x4[i];
    RatLiecurve.weights[i] = 1.;
   }
  covAccel = CovariantAccel_Spherical (&RatLiecurve, nProbes);
  printf("Covariant acceleration of Lie curve: %f\n", covAccel);

  /************************************************************/

  if (VISUALIZE)
   {
    glutInitWindowPosition (0,0);
    glutInitWindowSize (500,500);
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow ("Rational quaternion spline");
    glutDisplayFunc    (display);
    glutKeyboardFunc   (keyboard);
    glutMouseFunc      (mouse);
    glutMotionFunc     (motion);
    glutReshapeFunc    (reshape);
    glutVisibilityFunc (visibility);
    gfxinit();
    glutCreateMenu   (menu);
    glutAddMenuEntry ("Data points", 1);
    glutAddMenuEntry ("S3 curve", 9); 
    glutAddMenuEntry ("Order of data points", 18);
    glutAttachMenu (GLUT_RIGHT_BUTTON);
    glutMainLoop();
   }
  return 0;
}

