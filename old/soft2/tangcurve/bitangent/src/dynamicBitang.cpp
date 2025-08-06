/*
  File:          dynamicBitang.cpp
  Author:        J.K. Johnstone 
  Created:	 27 May 2004
  Last Modified: 27 May 2004
  Purpose:       Dynamically compute bitangents,
                 to analyze flaws in the computation.
  Sequence:	 3rd in a sequence (interpolate, tangentialCurve, bitang)
  Input: 	 1 closed curve and 1 dynamic open curve,
                 the latter defining the spine of a skeletal curve.
  History: 	 5/27/04: Created from bitang.cpp.
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
#include "Vector.h"		
#include "BezierCurve.h"	// inputCurves
#include "TangCurve.h"		// buildTangentialCurves, CommonTangent, intersect, draw, visible
#include "Scene2d.h"

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// 0 for running under Unix, 1 for Windows

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-e eps] (accuracy at which intersections are made: default .0001)" << endl;
  cout << "\t[-S] (scene input)" << endl;
  cout << "\t[-p] (set to Postscript printing mode; default is screen display)" << endl;
  cout << "\t[-l] (laptop)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob, zoomduala, zoomdualb;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean SCENEINPUT=0;          // scene input?
static GLboolean DRAWHIT=1;		// draw dual intersections?
static GLboolean DRAWBITANGA=1;		// draw bitangents from a-space?
static GLboolean DRAWBITANGB=1;		// draw bitangents from b-space?
static GLboolean DRAWVISTANG=0;		// draw visible bitangents?

Array<BezierCurve2f> 	obstacle;	// interpolating cubic Bezier curves
float                   eps = .0001;	// accuracy of intersection computation
float                   tGrow;          // present end parameter value of growing envelope
float                   R=.1;           // radius of envelope
float                   delta=.1;       // sampling step size
BezierCurve2f           spine;          // spine curve, defining the dynamic envelope if DYNAMICSPINE
Array<TangentialCurve>  obduala;	// associated tangential a-curves
Array<TangentialCurve>  obdualb;	// associated tangential b-curves
Array<CommonTangentArr> bitangA;	// bitangents from a-space for each pair of curves
Array<CommonTangentArr> bitangB;	// bitangents from b-space for each pair of curves
					// curve i/j bitangents are stored in 
					// index i*obstacle.getn() + j
Array<IntArr>		vistangA;	// is this a-bitangent visible?
Array<IntArr>		vistangB;	// is this b-bitangent visible?
float colour[7][3] = {{1,0,0}, {0,0,1}, {0,1,0}, {0,0,0}, {1,0,1}, {0,1,1}, {1,1,0}};
float greyscale[7][3] = {{0,0,0}, {.1,.1,.1}, {.2,.2,.2}, {.3,.3,.3}, {.4,.4,.4}, {.5,.5,.5}, {.6,.6,.6}};
int			obstacleWin;	// primal window identifier 
int			dualWin;	// a-dual window identifier
int			dualWin2;	// b-dual window identifier
int       		nPtsPerSegment = PTSPERBEZSEGMENT;
int 			PRINTOUT=0;	// 0 for displaying on screen, 1 for printing out image
int                     LAPTOP=0;       // display environment for laptop?

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
  zoomob = 1.5;
  zoomduala = 1.5; zoomdualb = .75;
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

void motionduala (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomduala -= (float).001*(x-oldx);
    if (zoomduala < 0.0) zoomduala = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void motiondualb (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdualb -= (float).001*(x-oldx);
    if (zoomdualb < 0.0) zoomdualb = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 			    break;	// ESCAPE
  case '1':	DRAWBITANGA = DRAWBITANGB = 1;		
  		DRAWVISTANG = 0;		    break;
  case '2':	DRAWBITANGA = 1; DRAWBITANGB=0;
  		DRAWVISTANG = 0;		    break;
  case '3':	DRAWBITANGB = 1; DRAWBITANGA=0;
  		DRAWVISTANG = 0;		    break;
  case '4':	DRAWVISTANG = 1; 
  		DRAWBITANGA=DRAWBITANGB=0;	    break;
  case 'g':     tGrow += .1;                           // animate the envelope growth 
                if (tGrow > spine.getLastKnot())
		  tGrow = spine.getLastKnot();  
		cout << "Growing envelope to " << tGrow << endl;
		break;
  case 'G':     if (tGrow > .2) 
                  tGrow -= .1;                        // backwards
		cout << "Shrinking envelope to " << tGrow << endl;
		break;
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
  		DRAWVISTANG = 0;		    break;
  case 2:	DRAWBITANGA = 1; DRAWBITANGB=0;
  		DRAWVISTANG = 0;		    break;
  case 3:	DRAWBITANGB = 1; DRAWBITANGA=0;
  		DRAWVISTANG = 0;		    break;
  case 4:	DRAWVISTANG = 1; 
  		DRAWBITANGA=DRAWBITANGB=0;	    break;
  default:   	break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1:	DRAWHIT = !DRAWHIT;		    break;
  default:   					    break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void buildBitang()
{
  int i,j,k;
  buildTangentialCurves (obstacle, nPtsPerSegment/2, obduala, obdualb);

  int nOb = obstacle.getn();   // intersect tangential curves to find bitangents
  bitangA.allocate(nOb*nOb);  bitangB.allocate(nOb*nOb);
  for (i=0; i<nOb; i++)
    for (j=i+1; j<nOb; j++)		// bitangents between i and j
     {
      obduala[i].intersect (obduala[j], bitangA[i*nOb+j], eps);
      cout << "Bitangents from a-space" << endl;
      for (k=0; k<bitangA[i*nOb+j].getn(); k++)
	cout << bitangA[i*nOb+j][k] << endl;

      obdualb[i].intersect (obdualb[j], bitangB[i*nOb+j], eps);
      cout << "Bitangents from b-space" << endl;
      for (k=0; k<bitangB[i*nOb+j].getn(); k++) 
	cout << bitangB[i*nOb+j][k] << endl;
     }
  /*  
  vistangA.allocate(nOb*nOb);  vistangB.allocate(nOb*nOb);
  for (i=0; i<nOb; i++)
    for (j=i+1; j<nOb; j++)		// visible bitangents between i and j
     {
      int ij = i*nOb+j;
      vistangA[ij].allocate (bitangA[ij].getn());
      for (k=0; k<bitangA[ij].getn(); k++)
        if (bitangA[ij][k].visible(obstacle))	vistangA[ij][k] = 1;
	else 					vistangA[ij][k] = 0;
      vistangB[ij].allocate (bitangB[ij].getn());
      for (k=0; k<bitangB[ij].getn(); k++)
        if (bitangB[ij][k].visible(obstacle))	vistangB[ij][k] = 1;
	else 					vistangB[ij][k] = 0;
	} 
  */
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  // build next envelope of spine
  spine.buildEnvelope (spine.getKnot(0), tGrow, delta, R, obstacle[1]);
  buildBitang();

  int nOb = obstacle.getn(); 
  for (i=0; i<nOb; i++)
   {
    if (PRINTOUT) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
    obstacle[i].draw(); 
   }
  if (DRAWBITANGA)
   {
    glColor3fv (Black);
    for (i=0; i<nOb; i++)
      for (j=1; j<nOb; j++)	// a-bitangents between curves i and j
        for (k=0; k<bitangA[i*nOb+j].getn(); k++)
	  bitangA[i*nOb+j][k].draw ();
   }
  if (DRAWBITANGB)
   {
    if (PRINTOUT) glColor3f (.5,.5,.5); else glColor3fv (Magenta);
    for (i=0; i<nOb; i++)
      for (j=1; j<nOb; j++)	// b-bitangents between curves i and j
        for (k=0; k<bitangB[i*nOb+j].getn(); k++)
	  bitangB[i*nOb+j][k].draw ();
   }
  if (DRAWVISTANG)
   {
    for (i=0; i<nOb; i++)
      for (j=1; j<nOb; j++)
       {
        int ij=i*nOb+j;
        glColor3fv (Black);
        for (k=0; k<bitangA[ij].getn(); k++)
	  if (vistangA[ij][k])	bitangA[ij][k].draw ();
        if (PRINTOUT) glColor3f (.5,.5,.5); else glColor3fv (Magenta);
        for (k=0; k<bitangB[ij].getn(); k++)
	  if (vistangB[ij][k])	bitangB[ij][k].draw ();
       }
   }
     
  glPopMatrix();
  glutSwapBuffers ();
  //  glutPostRedisplay();	
}

/******************************************************************************/
/******************************************************************************/

void displayDual ()
{
  int i,j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomduala, zoomduala, zoomduala);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  int nOb = obstacle.getn(); 
  for (i=0; i<nOb; i++)                 // active segments of tangential a-curves
   {
    if (PRINTOUT) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
    obduala[i].drawT (nPtsPerSegment/2);
   }
  if (DRAWHIT)				// intersections in a-space
   {
    glColor3fv (Black);
    for (i=0; i<nOb; i++)
      for (j=1; j<nOb; j++)		// intersections between i and j
        for (k=0; k<bitangA[i*nOb+j].getn(); k++)
	  if (WINDOWS)
	   {
	    V2f p;  obduala[i].eval (bitangA[i*nOb+j][k].param1, p);
	    drawPt (p[0], p[1]);
	   }
	  else 
	    obduala[i].drawPt (bitangA[i*nOb+j][k].param1);
   }

  glPopMatrix();
  glutSwapBuffers ();
  //  glutPostRedisplay();	
}

/******************************************************************************/
/******************************************************************************/

void displayDual2 ()
{
  int i,j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdualb, zoomdualb, zoomdualb);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
   
  int nOb = obstacle.getn(); 
  for (i=0; i<nOb; i++)                 // active segments of tangential b-curves
   {
    if (PRINTOUT) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
    obdualb[i].drawT (nPtsPerSegment/2);
   }
  if (DRAWHIT)				// intersections in b-space
   {
    if (PRINTOUT) glColor3f (.5,.5,.5); else glColor3fv (Magenta);
    for (i=0; i<nOb; i++)
      for (j=1; j<nOb; j++)		// intersections between i and j
        for (k=0; k<bitangB[i*nOb+j].getn(); k++)
	  if (WINDOWS)
	   {
	    V2f p;  obdualb[i].eval (bitangB[i*nOb+j][k].param1, p);
	    drawPt (p[0], p[1]);
	   }
	  else 
	    obdualb[i].drawPt (bitangB[i*nOb+j][k].param1);
   }

  glPopMatrix();
  glutSwapBuffers ();
  // glutPostRedisplay();	
}

/******************************************************************************
	Input curves and scale to unit cube.
	This scaling allows infinite lines to be reduced to finite segments.
******************************************************************************/

void inputCurves (char *file, Array<BezierCurve2f> &obstacle)
{
  int i,j;
  ifstream infile;  infile.open(file);  
  V2fArrArr Pt;		// data points, organized into polygons
  read (infile, Pt);
  assert (Pt.getn() == 2);
  scaleToUnitSquare (Pt);
  obstacle.allocate(Pt.getn());
  for (i=0; i<Pt.getn(); i++)
   { 
    if (i==1) obstacle[i].fit (Pt[i]);
    else      obstacle[i].fitClosed (Pt[i]);
    if (i==1) obstacle[i].prepareDisplay (nPtsPerSegment, 1);
    else      obstacle[i].prepareDisplay (nPtsPerSegment, 0);
   }
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  int 	    i,j,k;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'S': SCENEINPUT = 1;                                 break;
      case 'p': PRINTOUT = 1;					break;
      case 'l': LAPTOP = 1;                                     break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  if (SCENEINPUT)
    {
      Scene2d scene;
      ifstream infile;  infile.open(argv[argc-1]);  
      scene.read (infile);
      infile.close();
      scene.build (nPtsPerSegment);
      scene.extract (obstacle);
      for (i=0; i<obstacle.getn(); i++) obstacle[i].prepareDisplay (nPtsPerSegment);
    }
  else inputCurves(argv[argc-1], obstacle);
  spine = obstacle[1]; tGrow = spine.getKnot(0) + .2;

  
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int xleft;		// x-coord of lefthand side for large windows
  int xsize, ysize;	// window size
  int barmargin; 	// width of side bar surrounding picture
  int titleht; 		// top titlebar height
  int adualy, bdualy;   // starting y-coordinate of right windows
  int halfysize;        // size of top right window
  int dualxleft;        // left side of right windows
  if (WINDOWS)
    {
      xleft 	= 0;
      xsize 	= 350; ysize = 350;
      barmargin = 6;
      titleht 	= 12;
      halfysize = (ysize - 2*titleht)/2;
      adualy    = titleht+10;
      bdualy    = titleht+10+halfysize+2*titleht+1;
      dualxleft = xleft+xsize+2*barmargin-1;
    }
  else if (LAPTOP)
    {
      xleft     = 0;
      xsize     = ysize = 500;
      barmargin = 12;
      titleht   = 0;
      halfysize = (ysize - 24)/2;
      adualy    = 0;
      bdualy    = titleht+halfysize+24;
      dualxleft = xleft+xsize+barmargin;
    }
  else if (PRINTOUT)
    {
      xleft     = 0;
      xsize     = ysize = 350;              // less reduction required ==> better image clarity)
      barmargin = 8;
      titleht = 20;
      halfysize = (ysize - 2*titleht)/2;
      adualy    = titleht+10;
      bdualy    = titleht+10+halfysize+2*titleht+1;
      dualxleft = xleft+xsize+2*barmargin-1;
    }
  else
    {
      xleft 	= 0;			
      xsize     = ysize = 600;                // 600 for standard windows
      barmargin = 8;
      titleht 	= 20;
      halfysize = (ysize - 2*titleht)/2;
      adualy    = titleht+10;
      bdualy    = titleht+10+halfysize+2*titleht+1;
      dualxleft = xleft+xsize+2*barmargin-1;
    }

  glutInitWindowPosition (xleft,titleht);		// primal window
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Curves");  
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
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Bitangents", 	           1);
  glutAddMenuEntry ("a-bitangents (from a-space)", 2);
  glutAddMenuEntry ("b-bitangents (from b-space)", 3);
  glutAddMenuEntry ("Visible bitangents",          4);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft,adualy);		// a-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential a-curves: steep tangents");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionduala);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Dual intersections", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft, bdualy);		// b-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential b-curves: shallow tangents");
  dualWin2 = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual2);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondualb);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Dual intersections", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

