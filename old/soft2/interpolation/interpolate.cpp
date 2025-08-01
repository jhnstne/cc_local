/*
  File:          interpolate.cpp
  Author:        J.K. Johnstone 
  Created:	 10 August 2001
  Last Modified: 1 June 2012
  Purpose:       Interpolate a collection of 2d point sets by 
                 cubic Bezier splines (by default, closed).
  Input data: 	 ~/Data/curve; ~/Data/bezierSurf/extrude;
                 ~/Data/bezierSurf/revo; ~/Research/tangcurve/umbra/data;
  History: 	 8/27/02: Added display of control polygon.
  	 	 9/16/02: Added open curves.
		 6/29/03: Added display of sample points.
		          Incorporates new sampling technique in 
			  BezierCurve::prepareDisplay.
		 6/30/03: Changed default PTSPERBEZSEGMENT from 30 to 10.
		          Finished cleaning.
                 7/3/03:  Added PRINTOUT option (for Postscript).
		 10/15/05: Different colour for different curves.
		           Number the curves (both to handle large number 
			   of curves in Dacheux's grant.rawctr data).
  Clean status:  6/1/12
*/

//       1         2         3         4         5         6         7         8
// 45678901234567890123456789012345678901234567890123456789012345678901234567890

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif
#include <iostream>
#include <fstream>
using namespace std;
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>

#include "basic/AllColor.h"
#include "basic/Miscellany.h"		                         // itoa, drawPt
#include "basic/Vector.h"		
#include "shape/Reader.h"		       // ReadPtCloud, ScaleToUnitSquare
#include "curve/BezierCurve.h" // fit,fitClosed,prepareDisplay,draw,drawCtrlPoly

#define PTSPERBEZSEGMENT 10              // # pts to draw on each Bezier segment

static char *RoutineName;
static void usage()
 {
   cout << "Cubic B-spline interpolation. " 
	<< endl << endl
	<< "   -d [display density of Bezier segment (default: 10)]" << endl
	<< "   -o: open curves" << endl
	<< "   -p: Postscript printing mode" << endl
	<< "   -h: this help message" << endl
	<< endl
	<< "Input: ptCloudsBraced.pts (each pt cloud surrounded by braces)" 
	<< endl;
 }

static GLfloat   transxob, transyob, zoomob;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean CLOSED=1;		// closed input curves?
static GLboolean DRAWPT=1;		// draw data points?
static GLboolean DRAWCURVE=1;		// draw curved obstacles?
static GLboolean LABELPT=0;		// number the data points and curves?
static GLboolean DRAWCTRL=0;		// draw control polygon?
static GLboolean DRAWSAMPLE=0;          // draw sample points?
static GLboolean MULTICOLOUR=0;         // draw curves in different colours?
static GLboolean LABELCURVE=0;          // label the curves?

V2fArrArr	Pt;		// data points, organized into curves
BezCurve2fArr 	obstacle;	// interpolating cubic Bezier curves
V2fArr		sampleAvg;	// sample mean of each curve's data points
int		obstacleWin;	// window identifier 
int       	nPtsPerSegment = PTSPERBEZSEGMENT;
int 		PRINTOUT=0;	// printing mode?

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  glClearColor (1.0, 1.0, 1.0, 1.0);
  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glEnable (GL_POINT_SMOOTH);
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);
  transxob = transyob = 0.0;
  zoomob = 1.8;
}

/******************************************************************************/
/******************************************************************************/

void reshape(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, 
	  -1000,1000);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) glutIdleFunc (NULL);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 'p': 	DRAWPT     = !DRAWPT;	   break;
  case 'c':     DRAWCURVE  = !DRAWCURVE;   break;
  case 'C':  	DRAWCTRL   = !DRAWCTRL;	   break;
  case 'n':	LABELPT    = !LABELPT; 	   break;
  case 's':     DRAWSAMPLE = !DRAWSAMPLE;  break;
  case 'm':     MULTICOLOUR= !MULTICOLOUR; break;
  case 'l':     LABELCURVE = !LABELCURVE;  break;
  case 27:	exit(1); 		   break;		       // ESCAPE
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menu (int value)
{
  switch (value) {
  case 1:  DRAWPT     = !DRAWPT;	break;
  case 2:  DRAWCURVE  = !DRAWCURVE;	break;
  case 3:  DRAWCTRL   = !DRAWCTRL;	break;
  case 4:  LABELPT    = !LABELPT;	break;
  case 5:  DRAWSAMPLE = !DRAWSAMPLE;    break;
  case 6:  MULTICOLOUR = !MULTICOLOUR;  break;
  case 7:  LABELCURVE = !LABELCURVE;    break;
  default:   				break;
  }
  glutPostRedisplay();
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
	  glutSetCursor (GLUT_CURSOR_UP_DOWN); break;
	case GLUT_UP: 
	  leftMouseDown = 0;
	  glutSetCursor (GLUT_CURSOR_INHERIT); break;
	default: break;
	}
	break;
  case GLUT_MIDDLE_BUTTON:
	switch (state) {
	case GLUT_DOWN:
	  middleMouseDown = firstx = firsty = 1; 
	  glutSetCursor (GLUT_CURSOR_CYCLE); break;
	case GLUT_UP: 		
	  middleMouseDown = 0; 
	  glutSetCursor (GLUT_CURSOR_INHERIT); break;
	default: break;
	}
	break;
  default: break;
  }
}

/******************************************************************************/
/******************************************************************************/

void motion (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomob -= (float).02*(x-oldx);           // zoom
    if (zoomob < 0.0) zoomob = 0.0;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transxob += .02*(x-oldx);    // translation in x
    if (firsty)  firsty=0; else transyob += .02*(y-oldy);    // translation in y
   }
  oldx = x;  
  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void display ()
{
  int i,j,k;   char str[10];
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  if (DRAWPT)
   {
    glColor3fv (Black);
    glBegin(GL_POINTS);
    for (i=0; i<Pt.getn(); i++)
      for (j=0; j<Pt[i].getn(); j++)
	glVertex2f (Pt[i][j][0], Pt[i][j][1]);
    glEnd();
   }
  if (DRAWCTRL)
   {
    glColor3fv (Black);
    for (i=0; i<obstacle.getn(); i++)
      obstacle[i].drawCtrlPoly();
   }
  if (DRAWCURVE)
   {
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    for (i=0; i<obstacle.getn(); i++)	
     {
      if (MULTICOLOUR) 
	glColor3f (material[i%20][4], material[i%20][5], material[i%20][6]);
      obstacle[i].draw(); 
     }
   }
  if (LABELPT)		                                     // label the points
   {
    glColor3fv (Black);
    for (i=0; i<Pt.getn(); i++)
      for (j=0; j<Pt[i].getn(); j++)	
       {
        glRasterPos2f (Pt[i][j][0]+.01, Pt[i][j][1]+.01);
	itoa (j, str);
	for (k=0; k<strlen(str); k++)
	  glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
       }
   }
  if (LABELCURVE)                                             // label the curve
   {
    glColor3fv (Black);
    for (i=0; i<Pt.getn(); i++)
     {
      glRasterPos2f (sampleAvg[i][0], sampleAvg[i][1]);
      glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'C');
      itoa (i, str);
      for (k=0; k<strlen(str); k++)
	glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
     }
   }
  if (DRAWSAMPLE)
   {
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    for (i=0; i<obstacle.getn(); i++) obstacle[i].draw(1); 
   }
  glPopMatrix();
  glutSwapBuffers ();
}

/*******************************************************************************
*******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'o': CLOSED = 0;					break;
      case 'p': PRINTOUT = 1;                                   break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }
  
  ifstream infile;  infile.open(argv[argc-1]);
  ReadComment (infile);
  ReadPtCloudBraced (infile, Pt);
  ScaleToUnitSquare (Pt);
  sampleAvg.allocate(Pt.getn());  obstacle.allocate(Pt.getn());
  for (int i=0; i<Pt.getn(); i++)
   { 
    if (CLOSED)	obstacle[i].fitClosed (Pt[i]);
    else        obstacle[i].fit	      (Pt[i]);
    if (CLOSED) obstacle[i].prepareDisplay (nPtsPerSegment,0);
    else        obstacle[i].prepareDisplay (nPtsPerSegment,1);
    Pt[i].mean (sampleAvg[i]);
   }

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);

  int xsize,ysize;
  if (PRINTOUT) xsize = ysize = 350; else xsize = ysize = 600;
  glutInitWindowSize (xsize,ysize);

  int xleft=0;		// x-coord of lefthand side (164 for small windows)
  int titleht=20;	// top titlebar height
  glutInitWindowPosition (xleft,titleht);

  char titlebar[100]; 
  strcpy (titlebar, "Cubic B-spline interpolation");  
  if (!PRINTOUT)
   {
    strcat (titlebar, " (");
    strcat (titlebar, argv[argc-1]); // dataset
    strcat (titlebar, ")");
   }
  obstacleWin = glutCreateWindow (titlebar);

  glutReshapeFunc (reshape);
  glutVisibilityFunc (visibility);
  glutKeyboardFunc (keyboard);
  glutCreateMenu (menu);
  glutAddMenuEntry ("Data points [p]", 			     1);
  glutAddMenuEntry ("Interpolating cubic Bezier curve [c]",  2);
  glutAddMenuEntry ("Control polygon [C]",		     3);
  glutAddMenuEntry ("Number data points [n]", 		     4);
  glutAddMenuEntry ("Sample points [s]",                     5);
  glutAddMenuEntry ("Multicolour curves [m]",                6);
  glutAddMenuEntry ("Label curves [l]",                      7);
  glutAttachMenu (GLUT_RIGHT_BUTTON);
  glutMouseFunc (mouse);
  glutMotionFunc (motion);  
  glutDisplayFunc (display);
  gfxinit();
  glutMainLoop();
  return 0;
}

