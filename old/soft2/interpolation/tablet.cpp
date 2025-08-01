/*
  File:          tablet.cpp
  Author:        J.K. Johnstone 
  Created:	 23 December 2003
  Last Modified: 23 December 2003
  Purpose:       Build a raw point set using the tablet.
  Sequence:	 Precursor to interpolation.
  Dependence:    Vector.
  Input: 	 None.
  Status:        Creating.
  Copyright 2003 by John K. Johnstone.
  History: 	 
*/

#include <GL/glut.h>
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
#include "Miscellany.h"		// itoa
#include "Vector.h"		
// #include "MiscVector.h"		// read, scaleToUnitSquare

#define WINDOWS 0		 // running under Windows?

static char *RoutineName;
static void usage()
 {
  cout << "Input data points from a tablet and direct to a file." << endl;
  cout << "At most 20 curves and at most 500 points per curve." << endl;
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d] (demo mode: first two clicks probe extreme window coordinates)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <outfile>.pts" << endl;
 }

static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWPT=1;		// draw data points?
static GLboolean LABELPT=0;		// number the data points?

int xsize,ysize;                        // window size (in window coordinates)
int xleft=0, xright=1500,               // window corners (in window coordinates)
    ybot=1000, ytop=0;
float xcenter=750,ycenter=500;          // center of window (in window coordinates)
int firstClick=0,secondClick=0;         // to monitor first 2 left mouse clicks
int firstPtOfCurve=1;                   // is this data point the first point of the curve?
ofstream                outfile;        // output file in which points are dumped
V2fArrArr		Pt;		// data points, organized into curves
int                     nCurve = 0;     // present curve
IntArr                  nPt;            // # of data points on each curve
int			obstacleWin;	// window identifier 
int 			PRINTOUT=0;	// 0 for displaying on screen, 1 for printing out image

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
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  //  gluOrtho2D(0, (double) xsize, 0, (double) ysize);
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
	  if (firstClick)
	    {
	      cout << "First click" << endl;
	      cout << x << " " << y << endl;
	      xleft = x; ybot = y; firstClick=0; secondClick=1;
	    }
	  else if (secondClick)
	    {
	      cout << "Second click" << endl;
	      cout << x << " " << y << endl;
	      xright = x; ytop = y; secondClick=0;
	      xcenter = (xright - xleft)/2.;  ycenter = (ybot - ytop)/2.;
	      // x-window coords increase left to right, while
	      // y-window coords increase top to bottom
	      }
	  else
	    {
	      Pt[nCurve][nPt[nCurve]][0] = (x-xcenter)/250.;  // (x-xcenter) / (xsize/6.);
	      Pt[nCurve][nPt[nCurve]][1] = (ycenter-y)/250.;  // (ycenter-y) / (ysize/4.);
	      if (firstPtOfCurve)
		{
		  outfile << "{" << endl;
		  firstPtOfCurve = 0;
		}
	      outfile << Pt[nCurve][nPt[nCurve]][0] << " "
		      << Pt[nCurve][nPt[nCurve]][1] << endl;
	      nPt[nCurve]++;
	    }
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

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 		  break;		// ESCAPE
  case 'p': 	DRAWPT     = !DRAWPT;	  break;
  case 'n':	LABELPT    = !LABELPT; 	  break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1:  DRAWPT     = !DRAWPT;	break;
  case 2:  outfile << "}" << endl;
	   firstPtOfCurve = 1;
	   nCurve++;                    break;
  case 4:  LABELPT    = !LABELPT;	break;
  default:   				break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  if (DRAWPT)
   {
    glColor3fv (Black);
    if (WINDOWS)
      for (i=0; i<=nCurve; i++)
        for (j=0; j<nPt[i]; j++)
          drawPt (Pt[i][j][0], Pt[i][j][1]);
    else
     {
      glBegin(GL_POINTS);
      for (i=0; i<=nCurve; i++)
        for (j=0; j<nPt[i]; j++)
          glVertex2f (Pt[i][j][0], Pt[i][j][1]);
      glEnd();
     }
   }
  glColor3fv (Red);                            // draw the curves, last one open and others closed
  for (i=0; i<nCurve; i++)
    {
      glBegin (GL_LINE_LOOP);
      for (j=0; j<nPt[i]; j++)
	glVertex2f (Pt[i][j][0], Pt[i][j][1]);
      glEnd();
    }
  glBegin(GL_LINE_STRIP);
  for (j=0; j<nPt[nCurve]; j++)
    glVertex2f (Pt[nCurve][j][0], Pt[nCurve][j][1]);
  glEnd();
  if (LABELPT)
   {
    char str[10];
    glColor3fv (Black);
    for (i=0; i<nCurve; i++)
      for (j=0; j<nPt[i]; j++)			  // label the points
       {
        glRasterPos2f (Pt[i][j][0]+.01, Pt[i][j][1]+.01);
	itoa (j, str);
	for (k=0; k<strlen(str); k++)
	  glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
       }
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

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
      case 'd': firstClick = 1;                                 break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }
  
  outfile.open(argv[argc-1]);
  outfile << "[ Data file generated by tablet ]" << endl;
  Pt.allocate(20);           // at most 20 curves
  nPt.allocate(20);
  for (int i=0; i<20; i++) 
    {
      Pt[i].allocate(500);  // at most 500 points per curve
      nPt[i] = 0;
    }
  

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);

  int xleft;		// x-coord of lefthand side
  int titleht; 		// top titlebar height
  xsize	= 1500; ysize = 1000;
  if (WINDOWS)
   {
    xleft 	= 0;
    titleht 	= 12;
   }
  else
   {
    xleft 	= 0;			// 164 for small windows
    if (PRINTOUT) xsize = ysize = 350;  // 350 for small windows (less reduction required ==> better image clarity)
    titleht 	= 20;
   }

  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Data points");  
  if (!PRINTOUT)
   {
    strcat (titlebar, " (");
    strcat (titlebar, argv[argc-1]);  
    strcat (titlebar, ")");
   }
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Add points using left mouse", -1);
  glutAddMenuEntry ("Close a curve and move to next curve using middle mouse", 2);
  glutAddMenuEntry ("Data points [p]", 			     1);
  glutAddMenuEntry ("Number data points [n]", 		     4);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

