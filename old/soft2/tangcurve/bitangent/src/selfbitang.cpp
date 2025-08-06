/*
  File:          selfbitang.cpp
  Author:        J.K. Johnstone 
  Created:	 12 August 2001
  Last Modified: 13 August 2001
  Purpose:       Compute the self-bitangents of the given curves.
		 Builds on tangCurve.c++, which computes tangential curves.
		 Very similar to bitang.c++ (only changes in main and display).
  Sequence: 	 3rd in a sequence (interpolate, tangCurve, selfbitang)
  Input: 	 k 2d polygons, implicitly defining k interpolating cubic 
  		 Bezier curves
  History: 	 
*/

#include <GL/glut.h>
#include <GL/glu.h>
#include <fstream.h>
#include <iostream.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>

#include "AllColor.h"
#include "Miscellany.h"		
#include "Vector.h"		
#include "MiscVector.h"		
#include "BezierCurve.h"	
#include "TangCurve.h"		// CommonTangent, selfIntersect, draw, visible	

#define PTSPERBEZSEGMENT 30      // # pts to draw on each Bezier segment

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 30)" << endl;
  cout << "\t[-e eps] (accuracy at which intersections are made: default .0001)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWHIT=1;		// draw dual self-intersections?
static GLboolean DRAWBITANGA=1;		// draw self-bitangents from a-space?
static GLboolean DRAWBITANGB=1;		// draw self-bitangents from b-space?
static GLboolean DRAWVISTANG=0;		// draw visible self-bitangents?

Array<BezierCurve2f> 	obstacle;	// interpolating cubic Bezier curves
Array<TangentialCurve>  obduala;	// associated tangential a-curves
Array<TangentialCurve>  obdualb;	// associated tangential b-curves
Array<CommonTangentArr> bitangA;	// self-bitangents from a-space for each curve
Array<CommonTangentArr> bitangB;	// self-bitangents from b-space for each curve
Array<IntArr>		vistangA;	// is this a-bitangent visible?
Array<IntArr>		vistangB;	// is this b-bitangent visible?
float colour[7][3] = {{1,0,0}, {0,0,1}, {0,1,0}, {0,0,0}, {1,0,1}, {0,1,1}, {1,1,0}};
int			obstacleWin;	// primal window identifier 
int			dualWin;	// a-dual window identifier
int			dualWin2;	// b-dual window identifier
int       		nPtsPerSegment = PTSPERBEZSEGMENT;

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
  zoomdual = 1;
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -1000.,1000.);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE)
    glutIdleFunc (NULL);
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

void motionob (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomob -= (float).01*(x-oldx);
    if (zoomob < 0.0) zoomob = 0.0;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transxob += .01*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transyob += .01*(y-oldy); /* TRANSLATION: Y */
   }
  oldx = x;  
  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void motiondual (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdual -= (float).001*(x-oldx);
    if (zoomdual < 0.0) zoomdual = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 				break;	// ESCAPE
  case '1':	DRAWBITANGA = DRAWBITANGB = 1;
  		DRAWVISTANG = 0;			break;
  case '2':	DRAWBITANGA = 1; DRAWBITANGB=0;
  		DRAWVISTANG = 0;			break;
  case '3':	DRAWBITANGB = 1; DRAWBITANGA=0;	
  		DRAWVISTANG = 0;			break;
  case '4':	DRAWVISTANG = 1; 
  		DRAWBITANGA=DRAWBITANGB=0;		break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1:	DRAWBITANGA = DRAWBITANGB = 1;		
  		DRAWVISTANG = 0;			break;
  case 2:	DRAWBITANGA = 1; DRAWBITANGB=0;
  		DRAWVISTANG = 0;			break;
  case 3:	DRAWBITANGB = 1; DRAWBITANGA=0;
  		DRAWVISTANG = 0;			break;
  case 4:	DRAWVISTANG = 1; 
  		DRAWBITANGA=DRAWBITANGB=0;		break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1:	DRAWHIT = !DRAWHIT;		break;
  default:   					break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  int nOb = obstacle.getn(); 
  for (i=0; i<nOb; i++)
   {
    glColor3fv (colour[i%7]);
    obstacle[i].draw();
   }
  if (DRAWBITANGA)
   {
    glColor3fv (Black);
    for (i=0; i<nOb; i++)
      for (j=0; j<bitangA[i].getn(); j++)
	bitangA[i][j].draw (obstacle);
   }
  if (DRAWBITANGB)
   {
    glColor3fv (Magenta);
    for (i=0; i<nOb; i++)
      for (j=0; j<bitangB[i].getn(); j++)
	bitangB[i][j].draw (obstacle);
   }
  if (DRAWVISTANG)
   {
    for (i=0; i<nOb; i++)
     {
      glColor3fv (Black);
      for (j=0; j<bitangA[i].getn(); j++)
	if (vistangA[i][j]) bitangA[i][j].draw (obstacle);
      glColor3fv (Magenta);
      for (j=0; j<bitangB[i].getn(); j++)
	if (vistangB[i][j]) bitangB[i][j].draw (obstacle);
     }
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayDual ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  for (i=0; i<obstacle.getn(); i++)    // active segments of tangential a-curves
   {
    glColor3fv (colour[i%7]);
    obduala[i].drawT (nPtsPerSegment*5);
   }
  if (DRAWHIT)				// intersections in a-space
   {
    glColor3fv (Black);
    glBegin(GL_POINTS);
    int nOb = obstacle.getn(); 
    for (i=0; i<nOb; i++)
      for (j=0; j<bitangA[i].getn(); j++)
        obduala[i].drawPt (bitangA[i][j].param1);
    glEnd();
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayDual2 ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
   
  for (i=0; i<obstacle.getn(); i++)    // active segments of tangential b-curves
   {
    glColor3fv (colour[i%7]);
    obdualb[i].drawT (nPtsPerSegment*5);
   }
  if (DRAWHIT)				// intersections in b-space
   {
    glColor3fv (Magenta);
    glBegin(GL_POINTS);
    int nOb = obstacle.getn(); 
    for (i=0; i<nOb; i++)
      for (j=0; j<bitangB[i].getn(); j++)
        obdualb[i].drawPt (bitangB[i][j].param1);
    glEnd();
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

void inputCurves (char *file, Array<BezierCurve2f> &obstacle)
{
  ifstream infile;  infile.open(file);  
  V2fArrArr Pt;		// data points, organized into polygons
  read (infile, Pt);
  scaleToUnitSquare (Pt);
  obstacle.allocate(Pt.getn());
  for (int i=0; i<Pt.getn(); i++)
   { 
    obstacle[i].fitClosed (Pt[i]);
    obstacle[i].prepareDisplay (nPtsPerSegment);
   }
}

/******************************************************************************
	Compute clipped tangential a/b-curves.
******************************************************************************/

void buildTangentialCurves (Array<BezierCurve2f> &obstacle)
{
  obduala.allocate(obstacle.getn());  obdualb.allocate(obstacle.getn());
  for (int i=0; i<obstacle.getn(); i++)	
   {
    obduala[i].createA(obstacle[i], i);
    obduala[i].prepareDisplay (nPtsPerSegment*5);
    obdualb[i].createB(obstacle[i], i);
    obdualb[i].prepareDisplay (nPtsPerSegment*5);
   }
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  float     eps = .0001;	// accuracy of intersection computation
  int 	    i,j;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  inputCurves(argv[argc-1], obstacle);
  buildTangentialCurves (obstacle);
  int nOb = obstacle.getn();   // self-intersect tangential curves to find bitangents
  bitangA.allocate(nOb);  bitangB.allocate(nOb);
  for (i=0; i<nOb; i++)
   {
    obduala[i].selfIntersect (bitangA[i], eps);
    obdualb[i].selfIntersect (bitangB[i], eps);
   }
  vistangA.allocate(nOb);  vistangB.allocate(nOb);
  for (i=0; i<nOb; i++)
   {
    vistangA[i].allocate (bitangA[i].getn());
    for (j=0; j<bitangA[i].getn(); j++)
      if (bitangA[i][j].visible(obstacle))	vistangA[i][j] = 1;
      else 					vistangA[i][j] = 0;
    vistangB[i].allocate (bitangB[i].getn());
    for (j=0; j<bitangB[i].getn(); j++)
      if (bitangB[i][j].visible(obstacle))	vistangB[i][j] = 1;
      else 					vistangB[i][j] = 0;
   }
  
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int titleht = 20; 	// top titlebar is 20 units high
  //  int xleft    = 164;		// x-coord of lefthand side for small windows
  //  int xsize = 400, ysize = 400;	// small windows
  int xleft = 0;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  int barmargin = 8; 	// width of side bar surrounding picture
  int halfysize = (ysize - 2*titleht)/2;
  int dualxleft = xleft+xsize+2*barmargin-1;
  int adualy    = titleht+10;
  int bdualy    = titleht+10+halfysize+2*titleht+1;

  glutInitWindowPosition (xleft,titleht);		// primal window
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Curves (");  
  strcat (titlebar, argv[argc-1]);  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Self-bitangents", 		    1);
  glutAddMenuEntry ("Self-bitangents from a-space", 2);
  glutAddMenuEntry ("Self-bitangents from b-space", 3);
  glutAddMenuEntry ("Visible self-bitangents", 	    4);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft,adualy);		// a-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential a-curves: more vertical tangents");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Dual self-intersections", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft, bdualy);		// b-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential b-curves: more horizontal tangents");
  dualWin2 = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual2);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Dual self-intersections", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

