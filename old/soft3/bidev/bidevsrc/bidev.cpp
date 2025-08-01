/*
  File:          bidev.cpp
  Author:        J.K. Johnstone 
  Created:	 15 August 2001
  Last Modified: 1 November 2002
  Purpose:       Compute the bitangent developables of two surfaces,
  Sequence:	 3rd in a sequence (surfinterpolate, tangentialSurf, bidev)
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
#include "Vector.h"
#include "MiscVector.h"		
#include "BezierSurf.h"
#include "TangSurf.h"	

#define PTSPERBEZSEGMENT 5   	// # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 5)" << endl;
  cout << "\t[-e epsilon accuracy for clipping] (default: .1)" << endl;
  cout << "\t[-E epsilon accuracy for intersection] (default: .0001)" << endl;
  cout << "\t[-m] (direct bicubic Bezier control mesh input)" << endl;
  cout << "\t[-M] (unified bicubic Bezier control mesh input)" << endl;
  cout << "\t[-j] (just tangential a-surface)" << endl;
  cout << "\t[-s] (store)" << endl;
  cout << "\t[-S] (use stored tangential surface systems)" << endl;
  cout << "\t[-D] (don't display tangential surfaces)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts3 or <file>.cpt3" << endl;
 }

static GLfloat   transxob, transyob, rotxob, rotyob, rotzob, zoomob;
static GLfloat   transxdualA, transydualA, rotxdualA, rotydualA, rotzdualA, zoomdualA;
static GLfloat   transxdualB, transydualB, rotxdualB, rotydualB, rotzdualB, zoomdualB;
static GLfloat   transxdualC, transydualC, rotxdualC, rotydualC, rotzdualC, zoomdualC;
static int 	 panLeft=0, panRight=1; // control panning for 3d effect
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean ROTATEOB=0;		// rotate obstacles?
static GLboolean PANOB=0; 		// rotate object back and forth for 3d effect?
static GLboolean WIRE=0;		// draw surfaces in wireframe?
static GLboolean CTRLMESH=0;		// input data is control mesh directly?
static GLboolean UNIFIEDCTRLMESH=0;	// input data is unified control mesh? (not just a collection of separate bicubic patches)
static GLboolean ALLTS=1;		// compute all tangential surfaces?
static GLboolean DRAWSURF=1;		// draw primal surfaces?
static GLboolean DRAWNORM=0;		// draw primal surface normals?
static GLboolean DRAWTANGSPACE=0;	// draw primal tangent spaces?
static GLboolean DRAWTANGSURFA1=0;	// draw first  tangential a-surface?
static GLboolean DRAWTANGSURFA2=0;	// draw second tangential a-surface?
static GLboolean DRAWTANGSURFB1=0;	// draw first tangential b-surface?
static GLboolean DRAWTANGSURFB2=0;	// draw second tangential b-surface?
static GLboolean DRAWTANGSURFC1=0;	// draw first tangential c-surface?
static GLboolean DRAWTANGSURFC2=0;	// draw second tangential c-surface?
static GLboolean DRAWINTERSECTIONA=0;	// draw a-surface intersections?
static GLboolean DRAWINTERSECTIONB=0;	// draw b-surface intersections?
static GLboolean DRAWINTERSECTIONC=0;	// draw c-surface intersections?
static GLboolean DRAWINTERSECTIONTRACEB=0;	
static GLboolean DRAWCTRLNETA=0;	// draw a-surface control net?
static GLboolean DRAWCTRLNETB=0;	// draw b-surface control net?
static GLboolean DRAWCTRLNETC=0;	// draw c-surface control net?
static GLboolean DRAWACTIVEPT=0;	// draw active point?
static GLboolean DRAWABOX=1;		// draw box in a-dual space?
static GLboolean DRAWBBOX=1;		// draw box in b-dual space?
static GLboolean DRAWCBOX=1;		// draw box in c-dual space?
static GLboolean STORE=0;		// store the tangential surface systems?
static GLboolean STORED=0;		// use stored tangential surface systems, rather than computing from scratch
static GLboolean NOTANGDISPLAY=0;	// don't display tangential surfaces?
static GLboolean DRAWDIRECTRIX=0;	// draw directrix curves?
static GLboolean DRAWLOFTING=0;		// draw bitangent developables?

Array<BezierSurf3f> 	obstacle;	// primal surfaces
Array<TangentialSurf>   obdualA; 	// associated tangential a-surfaces
Array<TangentialSurf>	obdualB; 	// associated tangential b-surfaces
Array<TangentialSurf>	obdualC; 	// associated tangential c-surfaces
PatchIntersection 	iCurveAdual;	// intersection curves of a-surfaces in dual space
PatchIntersection 	iCurveBdual;	// intersection curves of b-surfaces in dual space
PatchIntersection 	iCurveCdual;	// intersection curves of c-surfaces in dual space
PatchIntersection 	iCurve;		// meta-intersection curves, combining iCurveA, iCurveB and iCurveC
PatchIntArrArrArr	iCurveATrace; // all levels of intersection curves of a-surfaces
			// per patch combo, per level, per curve
PatchIntArrArrArr	iCurveBTrace; // all levels of intersection curves of b-surfaces
PatchIntArrArrArr	iCurveCTrace; // all levels of intersection curves of c-surfaces
PatchIntersection	iCurveAprimalOnFirst,	// image of intersection curves wrt 0th obstacle
			iCurveBprimalOnFirst, iCurveCprimalOnFirst;
PatchIntersection	iCurveAprimalOnSecond,	// image of intersection curves wrt 1st obstacle
			iCurveBprimalOnSecond, iCurveCprimalOnSecond;
PatchIntersection	directrix0,directrix1;	// directrix curves of developables
int			level=0;	// trace level
float 			uActive,vActive;// parameters of active point
float 			uDelta,vDelta;	// increment of parameter value per step
float			uFirstKnot,uLastKnot,vFirstKnot, vLastKnot;
int			obstacleWin;	// primal window identifier 
int			dualWinA,dualWinB,dualWinC; // dual window identifiers
int       		density = PTSPERBEZSEGMENT;
float 			eps = .01;	// accuracy at which to clip tangential surfaces
float     		epsInt = .0001;	// accuracy at which to intersect tangential surfaces

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  GLfloat ambient[] 	   = {0.0, 0.0, 0.0, 1.0};
  GLfloat diffuse[] 	   = {1.0, 1.0, 1.0, 1.0};
  GLfloat position[]       = {0.0, 3.0, 3.0, 0.0};
  GLfloat position1[]      = {3.0, 0.0, 0.0, 0.0};
  GLfloat position2[]      = {-3.0,0.0, 0.0, 0.0};
  GLfloat position3[]      = {0.0,-3.0, 0.0, 0.0};
  GLfloat lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
  GLfloat local_view[] 	   = {0.0};

  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, position);
  glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, position1);
  glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT2, GL_POSITION, position2);
  glLightfv(GL_LIGHT3, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT3, GL_POSITION, position3);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);

  glFrontFace(GL_CW);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHT2);
  glEnable(GL_LIGHT3);
  glEnable(GL_AUTO_NORMAL);
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  glClearColor (1.0, 1.0, 1.0, 1.0);

/*
  glShadeModel (GL_SMOOTH);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
*/

  glEnable (GL_POINT_SMOOTH);
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);
  transxob = transyob = 0; rotxob = -90; rotyob = rotzob = 0; zoomob = 1;
  transxdualA=transydualA=0; rotxdualA=rotydualA=rotzdualA=0; zoomdualA=.4;
  transxdualB=transydualB=0; rotxdualB=rotydualB=rotzdualB=0; zoomdualB=.4;
  transxdualC=transydualC=0; rotxdualC=rotydualC=rotzdualC=0; zoomdualC=.4;
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

void RotateOb (void)
{
  rotzob += 0.5; 
  if (rotzob > 360.0) rotzob -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void PanOb (void)
{
  if (panLeft)
   {
    rotzob += 0.1;
    if (rotzob > 360.0) rotzob -= 360.0;
    panLeft++;
    if (panLeft==200) { panLeft=0; panRight=1; }
   }
  else if (panRight)
   {
    rotzob -= 0.1;
    if (rotzob < 0.0) rotzob += 360.0;
    panRight++;
    if (panRight==200) { panRight=0; panLeft=1; }
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE)
    glutIdleFunc (NULL);
  else if (ROTATEOB) glutIdleFunc (RotateOb);
  else if (PANOB)    glutIdleFunc (PanOb);
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
  else if (middleMouseDown) 
   {
    if (firstx)  firstx=0;
    else { rotyob += .5*(x-oldx); if (rotyob > 360.0) rotyob -= 360.0; } /* ORI: Y */

    if (firsty)  firsty=0;
    else { rotxob += .5*(y-oldy); if (rotxob > 360.0) rotxob -= 360.0; } /* ORI: X */
   }
  oldx = x;  
  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void motiondualA (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdualA -= (float).01*(x-oldx);
    if (zoomdualA < 0.0) zoomdualA = 0.0;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transxdualA += .01*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transydualA += .01*(y-oldy); /* TRANSLATION: Y */
   }
  else if (middleMouseDown) 
   {
    if (firstx)  firstx=0;
    else { rotydualA += .5*(x-oldx); if (rotydualA > 360.0) rotydualA -= 360.0; } /* ORI: Y */

    if (firsty)  firsty=0;
    else { rotxdualA += .5*(y-oldy); if (rotxdualA > 360.0) rotxdualA -= 360.0; } /* ORI: X */
   }
  oldx = x; oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void motiondualB (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdualB -= (float).01*(x-oldx);
    if (zoomdualB < 0.0) zoomdualB = 0.0;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transxdualB += .01*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transydualB += .01*(y-oldy); /* TRANSLATION: Y */
   }
  else if (middleMouseDown) 
   {
    if (firstx)  firstx=0;
    else { rotydualB += .5*(x-oldx); if (rotydualB > 360.0) rotydualB -= 360.0; } /* ORI: Y */

    if (firsty)  firsty=0;
    else { rotxdualB += .5*(y-oldy); if (rotxdualB > 360.0) rotxdualB -= 360.0; } /* ORI: X */
   }
  oldx = x; oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void motiondualC (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdualC -= (float).01*(x-oldx);
    if (zoomdualC < 0.0) zoomdualC = 0.0;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transxdualC += .01*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transydualC += .01*(y-oldy); /* TRANSLATION: Y */
   }
  else if (middleMouseDown) 
   {
    if (firstx)  firstx=0;
    else { rotydualC += .5*(x-oldx); if (rotydualC > 360.0) rotydualC -= 360.0; } /* ORI: Y */

    if (firsty)  firsty=0;
    else { rotxdualC += .5*(y-oldy); if (rotxdualC > 360.0) rotxdualC -= 360.0; } /* ORI: X */
   }
  oldx = x; oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 			break;	// ESCAPE
  case '1': 	DRAWSURF = !DRAWSURF;		break;
  case 'N':	DRAWNORM = !DRAWNORM;		break;
  case '3': 	DRAWTANGSPACE = !DRAWTANGSPACE;	break;
  case 'r':     ROTATEOB = !ROTATEOB;			// rotate
 	     	if (ROTATEOB) 
		     glutIdleFunc (RotateOb); 
		else glutIdleFunc (NULL); 	break;
  case 'p':	PANOB = !PANOB;				// pan
		if (PANOB) 
		     glutIdleFunc (PanOb); 
		else glutIdleFunc (NULL); 	break;
  case 'w':     WIRE = !WIRE;			break;	// wireframe
  case 'a':	DRAWTANGSURFA1 = 1; DRAWTANGSURFA2 = 0;	
  		DRAWTANGSURFB1 = 1; DRAWTANGSURFB2 = 0;	
  		DRAWTANGSURFC1 = 1; DRAWTANGSURFC2 = 0;	break;
  case 's':	DRAWTANGSURFA1 = 0; DRAWTANGSURFA2 = 1;	
  		DRAWTANGSURFB1 = 0; DRAWTANGSURFB2 = 1;	
  		DRAWTANGSURFC1 = 0; DRAWTANGSURFC2 = 1;	break;
  case 'd':	DRAWTANGSURFA1 = DRAWTANGSURFA2 = 1;	
  		DRAWTANGSURFB1 = DRAWTANGSURFB2 = 1;	
  		DRAWTANGSURFC1 = DRAWTANGSURFC2 = 1;	break;
  case 'f':	DRAWTANGSURFA1 = DRAWTANGSURFA2 = 0;	
  		DRAWTANGSURFB1 = DRAWTANGSURFB2 = 0;	
  		DRAWTANGSURFC1 = DRAWTANGSURFC2 = 0;	break;
  case 'C': 	DRAWINTERSECTIONA = !DRAWINTERSECTIONA; 
  		DRAWINTERSECTIONB = !DRAWINTERSECTIONB;
		DRAWINTERSECTIONC = !DRAWINTERSECTIONC; break;
  case 'A':  	DRAWACTIVEPT = !DRAWACTIVEPT;	break;	
  case 'h':	uActive -= uDelta;			// move left
    		if (uActive < uFirstKnot)
		  uActive = uLastKnot;		break;
  case 'j':	uActive += uDelta;			// move right
    		if (uActive > uLastKnot) 
		  uActive = uFirstKnot;		break;
  case 'u':	vActive += vDelta;			// move up
    		if (vActive > vLastKnot) 
		  vActive = vFirstKnot;		break;
  case 'n':	vActive -= vDelta;			// move down
    		if (vActive < vFirstKnot) 
		  vActive = vLastKnot;		break;
  case 'b':	DRAWABOX = !DRAWABOX; 
  		DRAWBBOX = !DRAWBBOX;
		DRAWCBOX = !DRAWCBOX; 		break;
  case ' ':	level = (level+1)%10;		break;
  case 8: 	level = mod(level-1,10);	break;	// backspace
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  default:   					break;
  case 0:  	WIRE = !WIRE;			break;
  case 1:	DRAWSURF = !DRAWSURF;		break;
  case 2:	DRAWNORM = !DRAWNORM;		break;
  case 3: 	DRAWTANGSPACE = !DRAWTANGSPACE;	break;
  case 4:  	DRAWACTIVEPT = !DRAWACTIVEPT;	break;	
  case 5:	uActive -= uDelta;			// move left
    		if (uActive < uFirstKnot)
		  uActive = uLastKnot;		break;
  case 6:	uActive += uDelta;			// move right
    		if (uActive > uLastKnot) 
		  uActive = uFirstKnot;		break;
  case 7:	vActive += vDelta;			// move up
    		if (vActive > vLastKnot) 
		  vActive = vFirstKnot;		break;
  case 8:	vActive -= vDelta;			// move down
    		if (vActive < vFirstKnot) 
		  vActive = vLastKnot;		break;
  case 9:	DRAWDIRECTRIX = !DRAWDIRECTRIX; break;
  case 10:	DRAWLOFTING = !DRAWLOFTING;     break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDualA (int value)
{
  switch (value) {
  case 1:	DRAWTANGSURFA1 = 1; DRAWTANGSURFA2 = 0;	break;
  case 2:	DRAWTANGSURFA1 = 0; DRAWTANGSURFA2 = 1;	break;
  case 3:	DRAWTANGSURFA1 = 1; DRAWTANGSURFA2 = 1;	break;
  case 4:	DRAWTANGSURFA1 = 0; DRAWTANGSURFA2 = 0;	break;
  case 5: 	DRAWINTERSECTIONA = !DRAWINTERSECTIONA; break;
  case 6:	DRAWABOX = !DRAWABOX;			break;
  case 20:	DRAWCTRLNETA = !DRAWCTRLNETA;		break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDualB (int value)
{
  switch (value) {
  case 1:	DRAWTANGSURFB1 = 1; DRAWTANGSURFB2 = 0;	break;
  case 2:	DRAWTANGSURFB1 = 0; DRAWTANGSURFB2 = 1;	break;
  case 3:	DRAWTANGSURFB1 = 1; DRAWTANGSURFB2 = 1;	break;
  case 4:	DRAWTANGSURFB1 = 0; DRAWTANGSURFB2 = 0;	break;
  case 5: 	DRAWINTERSECTIONB = !DRAWINTERSECTIONB; break;
  case 6:	DRAWBBOX = !DRAWBBOX;			break;
  case 7: 	DRAWINTERSECTIONTRACEB = !DRAWINTERSECTIONTRACEB; break;
  case 20:	DRAWCTRLNETB = !DRAWCTRLNETB;		break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDualC (int value)
{
  switch (value) {
  case 1:	DRAWTANGSURFC1 = 1; DRAWTANGSURFC2 = 0; break;
  case 2:	DRAWTANGSURFC1 = 0; DRAWTANGSURFC2 = 1;	break;
  case 3:	DRAWTANGSURFC1 = 1; DRAWTANGSURFC2 = 1;	break;
  case 4:	DRAWTANGSURFC1 = 0; DRAWTANGSURFC2 = 0;	break;
  case 5: 	DRAWINTERSECTIONC = !DRAWINTERSECTIONC; break;
  case 6:	DRAWCBOX = !DRAWCBOX;			break;
  case 20:	DRAWCTRLNETC = !DRAWCTRLNETC;		break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
// cout << "Entering displayOb" << endl;
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomob, zoomob, zoomob);
  glTranslatef (transxob, transyob, 0);
  glRotatef (rotxob, 1.0, 0.0, 0.0);
  glRotatef (rotyob, 0.0, 1.0, 0.0);
  glRotatef (rotzob, 0.0, 0.0, 1.0);
  if (DRAWSURF)					// primal surfaces
   {
    if (WIRE)
     {
      glDisable (GL_LIGHTING);
      for (i=0; i<obstacle.getn(); i++)
       { glColor3fv (material[i%24]+4); obstacle[i].draw(1); }
      glEnable (GL_LIGHTING);
     }
    else
      for (i=0; i<obstacle.getn(); i++)	
       {
        glMaterialfv(GL_FRONT, GL_AMBIENT,  material[i%24]);
        glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[i%24]+4);
        glMaterialfv(GL_FRONT, GL_SPECULAR, material[i%24]+8);
        glMaterialf(GL_FRONT, GL_SHININESS, material[i%24][12] * 128.0);
        obstacle[i].draw();
       }
   }
  if (DRAWNORM)
   {
    glDisable (GL_LIGHTING);
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[i%24]+4);
      obstacle[i].drawNorm();
     }
    glEnable (GL_LIGHTING);
   }
  if (DRAWTANGSPACE)
   {
    glDisable (GL_LIGHTING);
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[i%24]+4);
      obstacle[i].drawTangSpace();
     }
    glEnable (GL_LIGHTING);
   }
  if (DRAWACTIVEPT)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    obstacle[0].drawTangPlane (uActive, vActive);
    glEnable (GL_LIGHTING);
   }
  if (DRAWDIRECTRIX)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Black);
    directrix0.draw();
    directrix1.draw();
    glEnable (GL_LIGHTING);
   }
  if (DRAWLOFTING)
   {
    glDisable (GL_LIGHTING);
    glColor3fv (Blue);
//    glBegin (GL_TRIANGLE_STRIP);	need to merge curves first
    glBegin(GL_LINES);
    for (i=0; i<directrix0.getnCurve(); i++)
      for (j=0; j<directrix0.getnPt(i); j++)
       {
        V3f pt;
        directrix0.getPt (i,j,pt);
        glVertex3f (pt[0], pt[1], pt[2]);
        directrix1.getPt (i,j,pt);
        glVertex3f (pt[0], pt[1], pt[2]);
       }
    glEnd();
    glEnable (GL_LIGHTING);
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
// cout << "Exiting displayOb" << endl;
}

/******************************************************************************/
/******************************************************************************/

void displayDualA ()
{
// cout << "Entering displayDualA" << endl;
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdualA, zoomdualA, zoomdualA);
  glTranslatef (transxdualA, transydualA, 0);
  glRotatef (rotxdualA, 1.0, 0.0, 0.0);
  glRotatef (rotydualA, 0.0, 1.0, 0.0);
  glRotatef (rotzdualA, 0.0, 0.0, 1.0);
  
  if (DRAWABOX)
   {
    glDisable (GL_LIGHTING);
    glColor3fv (Black);
    glBegin(GL_LINE_LOOP);
    glVertex3f (-10.,-1.,-1.);  glVertex3f (-10., 1.,-1.); glVertex3f (-10., 1., 1.); glVertex3f (-10.,-1., 1.);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f (10.,-1.,-1.);   glVertex3f (10., 1.,-1.);  glVertex3f (10., 1., 1.);  glVertex3f (10.,-1., 1.);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f (-10.,-1.,-1.);	glVertex3f ( 10.,-1.,-1.);
    glVertex3f (-10., 1.,-1.);	glVertex3f ( 10., 1.,-1.);
    glVertex3f (-10., 1., 1.);	glVertex3f ( 10., 1., 1.);
    glVertex3f (-10.,-1., 1.);  glVertex3f ( 10.,-1., 1.);
    glEnd();
    glEnable (GL_LIGHTING);
   }
  if (DRAWTANGSURFA1)		// wireframe of first clipped tangential a-surface
   {
    if (WIRE)
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[0]+4); obdualA[0].drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[0]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[0]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[0]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[0][12] * 128.0);
      obdualA[0].drawT(density,0);
     }
   }
  if (DRAWTANGSURFA2)		// wireframe of second clipped tangential a-surface
   {
    if (WIRE)
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[1]+4); obdualA[1].drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[1]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[1]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[1]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[1][12] * 128.0);
      obdualA[1].drawT(density,0);
     }
   }
  if (DRAWINTERSECTIONA)	// a-surface intersections
   {
    glDisable (GL_LIGHTING);
    glLineWidth (3.0);
    glColor3fv (Black);
    iCurveAdual.draw();
    glLineWidth (1.0);
    glEnable (GL_LIGHTING);
   }
  if (DRAWCTRLNETA)		// control net of clipped tangential a-surface
   {
    glDisable (GL_LIGHTING);
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[i%24]+4);
      obdualA[i].drawCtrlNetT();
     }
    glEnable (GL_LIGHTING);
   }

  if (DRAWACTIVEPT)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    obdualA[0].drawPt (uActive, vActive);
    glEnable (GL_LIGHTING);
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
// cout << "Exiting displayDualA" << endl;
}

/******************************************************************************/
/******************************************************************************/

void displayDualB ()
{
// cout << "Entering displayDualB" << endl;
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdualB, zoomdualB, zoomdualB);
  glTranslatef (transxdualB, transydualB, 0);
  glRotatef (rotxdualB, 1.0, 0.0, 0.0);
  glRotatef (rotydualB, 0.0, 1.0, 0.0);
  glRotatef (rotzdualB, 0.0, 0.0, 1.0);
  
  if (DRAWBBOX)
   {
    glDisable (GL_LIGHTING);
    glColor3fv (Black);
    glBegin(GL_LINE_LOOP);
    glVertex3f (-1.,-10.,-1.);  glVertex3f (1.,-10.,-1.); glVertex3f (1.,-10.,1.); glVertex3f (-1.,-10.,1.);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f (-1,10,-1);   glVertex3f (1,10,-1);  glVertex3f (1,10,1);  glVertex3f (-1,10,1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f (-1,-10,-1);	glVertex3f (-1,10,-1);
    glVertex3f (1,-10,-1);	glVertex3f (1, 10,-1);
    glVertex3f (1,-10, 1);	glVertex3f (1, 10,1);
    glVertex3f (-1,-10, 1);     glVertex3f (-1,10, 1);
    glEnd();
    glEnable (GL_LIGHTING);
   }
  if (ALLTS) {
  if (DRAWTANGSURFB1)
   {
    if (WIRE)
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[0]+4); obdualB[0].drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[0]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[0]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[0]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[0][12] * 128.0);
      obdualB[0].drawT(density,0);
     }
   }
  if (DRAWTANGSURFB2)
   {
    if (WIRE)
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[1]+4); obdualB[1].drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[1]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[1]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[1]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[1][12] * 128.0);
      obdualB[1].drawT(density,0);
     }
   }
  if (DRAWINTERSECTIONB)	// b-surface intersections
   {
    glDisable (GL_LIGHTING);
    glLineWidth (3.0);
    glColor3fv (Black);
    iCurveBdual.draw();
    glLineWidth (1.0);
    glEnable (GL_LIGHTING);
   }
  if (DRAWINTERSECTIONTRACEB)
   {
    glDisable (GL_LIGHTING);
    glLineWidth (3.0);
    glColor3fv (Black);
    for (i=0; i<iCurveBTrace[0][level].getn(); i++)
      iCurveBTrace[0][level][i].draw();
    glLineWidth (1.0);
    glEnable (GL_LIGHTING);
   }
  if (DRAWCTRLNETB)		// control net of clipped tangential b-surface
   {
    glDisable (GL_LIGHTING);
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[i%24]+4);
      obdualB[i].drawCtrlNetT();
     }
    glEnable (GL_LIGHTING);
   }
  if (DRAWACTIVEPT)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    obdualB[0].drawPt (uActive, vActive);
    glEnable (GL_LIGHTING);
   }
  }
  glEnable (GL_LIGHTING);
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
// cout << "Exiting displayDualB" << endl;
}

/******************************************************************************/
/******************************************************************************/

void displayDualC ()
{
// cout << "Entering displayDualC" << endl;
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdualC, zoomdualC, zoomdualC);
  glTranslatef (transxdualC, transydualC, 0);
  glRotatef (rotxdualC, 1.0, 0.0, 0.0);
  glRotatef (rotydualC, 0.0, 1.0, 0.0);
  glRotatef (rotzdualC, 0.0, 0.0, 1.0);
  
  if (DRAWCBOX)
   {
    glDisable (GL_LIGHTING);
    glColor3fv (Black);
    glBegin(GL_LINE_LOOP);
    glVertex3f (-1,-1,-10);  glVertex3f (1,-1,-10); glVertex3f (1, 1,-10); glVertex3f (-1, 1,-10);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f (-1,-1,10);   glVertex3f (1,-1,10);  glVertex3f (1, 1,10);  glVertex3f (-1, 1,10);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f (-1,-1,-10);  glVertex3f (-1,-1,10);
    glVertex3f ( 1,-1,-10);  glVertex3f ( 1,-1,10);
    glVertex3f ( 1, 1,-10);  glVertex3f ( 1, 1,10);
    glVertex3f (-1, 1,-10);  glVertex3f (-1, 1,10);
    glEnd();
    glEnable (GL_LIGHTING);
   }
  if (ALLTS) {
  if (DRAWTANGSURFC1)
   {
    if (WIRE)
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[0]+4); obdualC[0].drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[0]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[0]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[0]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[0][12] * 128.0);
      obdualC[0].drawT(density,0);
     }
   }
  if (DRAWTANGSURFC2)
   {
    if (WIRE)
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[1]+4); obdualC[1].drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[1]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[1]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[1]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[1][12] * 128.0);
      obdualC[1].drawT(density,0);
     }
   }
  if (DRAWINTERSECTIONC)	// c-surface intersections
   {
    glDisable (GL_LIGHTING);
    glLineWidth (3.0);
    glColor3fv (Black);
    iCurveCdual.draw();
    glLineWidth (1.0);
    glEnable (GL_LIGHTING);
   }
  if (DRAWCTRLNETC)		// control net of clipped tangential c-surface
   {
    glDisable (GL_LIGHTING);
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[i%24]+4);
      obdualC[i].drawCtrlNetT();
     }
    glEnable (GL_LIGHTING);
   }
  if (DRAWACTIVEPT)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    obdualC[0].drawPt (uActive, vActive);
    glEnable (GL_LIGHTING);
   }
  }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
// cout << "Exiting displayDualC" << endl;
}

/******************************************************************************
	Read in data points to be fit, or a control mesh.
******************************************************************************/

void inputSurfaces (char *file, Array<BezierSurf3f> &obstacle)
{
  int i,j;
  ifstream infile;  infile.open(file);
  V3fArrArrArr Pt;
  read (infile, Pt);  scaleToUnitCube (Pt);  
  assert (Pt.getn() == 2);
  IntArr numSegu(2), numSegv(2);
  if (UNIFIEDCTRLMESH) 
    for (i=0; i<2; i++) infile >> numSegu[i] >> numSegv[i];
  infile.close();
  obstacle.allocate(Pt.getn());
  for (i=0; i<Pt.getn(); i++)
   { 
    if (CTRLMESH)	// each Pt[i] is a bicubic patch
     {
      FloatArr knot(2);  knot[0] = 0; knot[1] = 1;
      obstacle[i].create (3, 3, 3, 1, 1, Pt[i], knot, knot);  // define one bicubic patch
     }
    else if (UNIFIEDCTRLMESH)  // each Pt[i] is a collection of bicubic patches
     {
      FloatArr knotu(numSegu[i]+1), knotv(numSegv[i]+1);
      for (j=0; j<=numSegu[i]; j++) knotu[j] = j;
      for (j=0; j<=numSegv[i]; j++) knotv[j] = j;
      obstacle[i].create (3, 3, 3, numSegu[i], numSegv[i], Pt[i], knotu, knotv);
     }
    else obstacle[i].fit (Pt[i]);
    if (!STORE) obstacle[i].prepareDisplay (density);
   }
}

/******************************************************************************
	Read Bezier surfaces.
	Either build their tangential surface systems, or read them.
******************************************************************************/

void readInput(char *file)
{
  int i,nOb;
  if (STORED)		// read in tangential surface system from file storage
   {
   		cout << "Using stored tangential surface systems" << endl;
    nOb = 2;
    obstacle.allocate(nOb); obdualA.allocate(nOb); obdualB.allocate(nOb); obdualC.allocate(nOb);
    ifstream infile(file);	string comment;
    readComment (infile, comment);
    for (i=0; i<nOb; i++)
     {
      getLeftBrace (infile);
      obstacle[i].readBezSurf (infile);
      getRightBrace (infile);  getLeftBrace (infile);
      obdualA[i].readTangSurf (infile);	// should call readRatBezSurf
      getRightBrace (infile);  getLeftBrace (infile);
      obdualB[i].readTangSurf (infile);
      getRightBrace (infile);  getLeftBrace (infile);    
      obdualC[i].readTangSurf (infile);
      getRightBrace (infile);
     }
    infile.close();
		cout << "Preparing display of obstacle" << endl;
    for (i=0; i<nOb; i++) obstacle[i].prepareDisplay (density);
   }
  else			// compute tangential surface system from scratch
   {
    inputSurfaces (file, obstacle);
    nOb = obstacle.getn();
    obdualA.allocate(nOb); obdualB.allocate(nOb); obdualC.allocate(nOb);
    for (i=0; i<nOb; i++)	// build tangential surfaces in 3 dual spaces
     {
      BezierSurf1f a,b,c,d;
      obdualA[i].tangSurfComponents (obstacle[i], a, b, c, d);
      obdualA[i].createA (i,a,b,c,d,eps);
      if (ALLTS)
       {
        obdualB[i].createB (i,a,b,c,d,eps);
        obdualC[i].createC (i,a,b,c,d,eps);
       }
     }
    if (STORE)
     {
      string outfileName(file);
      changeSuffix (outfileName, ".tangsurf");
      ofstream outfile(outfileName.c_str());
      outfile << "[ Tangential surface systems generated by bidev.cpp ]" << endl;
      for (i=0; i<nOb; i++)
       {
        outfile << "{" << endl;
        obstacle[i].storeBezSurf (outfile);
        outfile << "}" << endl;
        outfile << "{" << endl;
        obdualA[i].storeTangSurf (outfile);
	outfile << "}" << endl;
	outfile << "{" << endl;
	obdualB[i].storeTangSurf (outfile);
	outfile << "}" << endl;
	outfile << "{" << endl;
	obdualC[i].storeTangSurf (outfile);
	outfile << "}" << endl;
       }
      outfile.close();
      exit(1);
     }
   } 
  if (!NOTANGDISPLAY)
    for (i=0; i<nOb; i++)
     {
		cout << "Preparing display of tangential a-surface for obstacle " << i << "..." << endl; 
      obdualA[i].prepareDisplay (density);
      if (ALLTS)
       {
		cout << "Preparing display of tangential b-surface for obstacle " << i << "..." << endl; 
        obdualB[i].prepareDisplay (density);
		cout << "Preparing display of tangential c-surface for obstacle " << i << "..." << endl; 
        obdualC[i].prepareDisplay (density);
       }
     }
}

/******************************************************************************
	Given the intersection curves in a-, b-, and c-dual spaces,
	collect them together across dual spaces into
 	meta-intersection curves.
		
	We will not keep points in this meta-intersection curve,
	just parameter values, since the points lose meaning
	(and are retrievable in any particular dual space anyway
	by evaluation of the parameter values).
******************************************************************************/

void sewCurveBetweenDualSpaces (PatchIntersection &iCurveA, 
		     	        PatchIntersection &iCurveB, 
		     	        PatchIntersection &iCurveC,
				PatchIntersection &iCurveSewn)
{
  int i;
  // print out endpoints of all intersection curves to see if they are matchable
  cout << "Endpoints of open intersection curves in a-space:" << endl;
  V3f foo;
  for (i=0; i<iCurveA.getnCurve(); i++)
    if (!iCurveA.getClosed(i))
     {
      iCurveA.getPt (i,0,			    foo);  cout << foo << endl;
      iCurveA.getPt (i,iCurveA.getnPt(i) - 1, foo);  cout << foo << endl;
     }
  cout << "Endpoints of open intersection curves in b-space:" << endl;
  for (i=0; i<iCurveB.getnCurve(); i++)
    if (!iCurveB.getClosed(i))
     {
      iCurveB.getPt (i,0,			    foo);  cout << foo << endl;
      iCurveB.getPt (i,iCurveB.getnPt(i) - 1, foo);  cout << foo << endl;
     }
  cout << "Endpoints of open intersection curves in c-space:" << endl;
  for (i=0; i<iCurveC.getnCurve(); i++)
    if (!iCurveC.getClosed(i))
     {
      iCurveC.getPt (i,0,			    foo);  cout << foo << endl;
      iCurveC.getPt (i,iCurveC.getnPt(i) - 1, foo);  cout << foo << endl;
     }

/*
  int i;
  IntArr markA (iCurveA.getn());	// has this a-curve been dealt with?
  IntArr markB (iCurveB.getn());
  IntArr markC (iCurveC.getn());
  markA.clear(); markB.clear(); markC.clear();
  walk around the curves finding partners
  int nPt = 0;
  int maxn;
  for (i=0; i<iCurveA.getn(); i++)  maxn += iCurveA[i].pt.getn();
  for (i=0; i<iCurveB.getn(); i++)  maxn += iCurveB[i].pt.getn();
  for (i=0; i<iCurveC.getn(); i++)  maxn += iCurveC[i].pt.getn();
  V2fArr apar (maxn), bpar (maxn);	// parameters of intersection curve on obstacle[0] and obstacle[1]
  apar[0] = iCurveA[0].apar[0];
  bpar[0] = iCurveA[0].bpar[0];
  buddyA (iCurveA[0].pt[0], nextdualspace, nextCurve, nextPt);
*/
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
      case 'd': density = atoi(argv[ArgsParsed++]);	break;
      case 'e': eps = atof(argv[ArgsParsed++]);		break;
      case 'E': epsInt = atof(argv[ArgsParsed++]);	break;
      case 'm': CTRLMESH = 1;				break;
      case 'M': UNIFIEDCTRLMESH = 1;			break;
      case 'j': ALLTS = 0;				break;
      case 's': STORE  = 1;				break;
      case 'S': STORED = 1;				break;
      case 'D': NOTANGDISPLAY = 1;			break;
      case 'h': 
      default:	usage(); exit(-1);			break;
      }
   else ArgsParsed++;
  }
  
  readInput(argv[argc-1]);
  
	cout << "Intersecting tangential a-surfaces..." << endl;  
  obdualA[0].intersect (obdualA[1], iCurveAdual, iCurveATrace, epsInt);
	cout << "Intersecting tangential b-surfaces..." << endl;  
  obdualB[0].intersect (obdualB[1], iCurveBdual, iCurveBTrace, epsInt); 
	cout << "Intersecting tangential c-surfaces..." << endl;
  obdualC[0].intersect (obdualC[1], iCurveCdual, iCurveCTrace, epsInt); 
cout << "iCurveAdual: " << endl;	iCurveAdual.print(); 
cout << "iCurveBdual: " << endl;	iCurveBdual.print();
cout << "iCurveCdual: " << endl;	iCurveCdual.print();  

  // map intersection curves back to primal space, 
  // where they become directrix curves on obstacles P and Q
  obstacle[0].aMap (iCurveAdual, iCurveAprimalOnFirst);	
  obstacle[0].aMap (iCurveBdual, iCurveBprimalOnFirst);
  obstacle[0].aMap (iCurveCdual, iCurveCprimalOnFirst);
  obstacle[1].bMap (iCurveAdual, iCurveAprimalOnSecond);
  obstacle[1].bMap (iCurveBdual, iCurveBprimalOnSecond);
  obstacle[1].bMap (iCurveCdual, iCurveCprimalOnSecond);

  // stitch together from 3 different sources
  directrix0 =  iCurveAprimalOnFirst;
  directrix0 += iCurveBprimalOnFirst;
  directrix0 += iCurveCprimalOnFirst;
  directrix1 =  iCurveAprimalOnSecond;
  directrix1 += iCurveBprimalOnSecond;
  directrix1 += iCurveCprimalOnSecond;

  // splice directrix0 together, with directrix1 mimicking its splicing
  // ...
  
  // merge directrix0 components with directrix1 components
  // to form loftings
  
  // build line of lofting between each pair of points on directrices
  // LoftRuled lofting(directrix0.getnCurve());
  // for (i=0; i<directrix0.getnCurve(); i++)
  //  lofting[i].create (i, directrix0, directrix1);
    
  // WOULD RATHER HAVE A PARAMETRIC CURVE AS DIRECTRIX, NOT A BUNCH OF POINTS

//	cout << "Sewing intersection curves" << endl;
// sewCurveBetweenDualSpaces (iCurveAdual, iCurveBdual, iCurveCdual, iCurve);
   
  uFirstKnot = obstacle[0].getKnotu(0);	
  vFirstKnot = obstacle[0].getKnotv(0);
  uLastKnot  = obstacle[0].getKnotu (obstacle[0].getnKnotu()-1);
  vLastKnot  = obstacle[0].getKnotv (obstacle[0].getnKnotv()-1);
  uActive    = (uLastKnot + uFirstKnot) / 2.;	// start in middle
  vActive    = (vLastKnot + vFirstKnot) / 2.;	
  if (WINDOWS) 
   {
    uDelta = (uLastKnot - uFirstKnot) / 200.;
    vDelta = (vLastKnot - vFirstKnot) / 200.;
   }
  else	       
   {
    uDelta = (uLastKnot - uFirstKnot) / 800.;
    vDelta = (vLastKnot - vFirstKnot) / 800.;
   }
   
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  int titleht = 20; 	// top titlebar is 20 units high
  int xleft    = 164;	// x-coord of lefthand side
  int xsize = 400, ysize = 400;
  int barmargin = 8; 	// width of side bar surrounding picture
  int halfysize = (ysize - 2*titleht)/2;

  glutInitWindowPosition (xleft,titleht);		// primal window
  glutInitWindowSize (xsize,ysize+219);	// make it a bit bigger for 3 windows
  char titlebar[100]; 
  strcpy (titlebar, "Primal surfaces (");  
  strcat (titlebar, argv[argc-1]);  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  gfxinit();
  glutReshapeFunc (reshape);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Surfaces [1]", 	     1);
  glutAddMenuEntry ("Normals [N]",     	     2);
  glutAddMenuEntry ("Tangent space [3]",     3);
  glutAddMenuEntry ("Shaded/wireframe [w]",  0);	
  glutAddMenuEntry ("Active point [A]",      		      4);
  glutAddMenuEntry ("Move active point left [h]",	      5);
  glutAddMenuEntry ("Move active point right [j]",	      6);
  glutAddMenuEntry ("Move active point up [u]",	      	      7);
  glutAddMenuEntry ("Move active point down [n]",	      8);
  glutAddMenuEntry ("Directrix curves",	      		      9);
  glutAddMenuEntry ("Bitangent developables",	      	      10);
  glutAttachMenu (GLUT_RIGHT_BUTTON);
							// a-dual window
  glutInitWindowPosition (xleft+xsize+2*barmargin-1, titleht+10);  
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential a-surfaces");
  dualWinA = glutCreateWindow (titlebar);
  gfxinit();  
  glutReshapeFunc (reshape);
  glutDisplayFunc (displayDualA);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondualA);
  glutCreateMenu (menuDualA);
  glutAddMenuEntry ("First tangential a-surface [a]", 	         1);
  glutAddMenuEntry ("Second tangential a-surface [s]", 	         2);
  glutAddMenuEntry ("Both [d]", 				 3);
  glutAddMenuEntry ("Neither [f]", 				 4);
  glutAddMenuEntry ("Intersection curves [C]", 			 5);
  glutAddMenuEntry ("Box [b]", 					 6);
  glutAddMenuEntry ("Tangential a-surface control nets [2]", 	 20);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

							// b-dual window
  glutInitWindowPosition (xleft+xsize+2*barmargin-1,
  			  titleht+10+halfysize+2*titleht+1);
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential b-surfaces");
  dualWinB = glutCreateWindow (titlebar);
  gfxinit();  
  glutReshapeFunc (reshape);
  glutDisplayFunc (displayDualB);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondualB);
  glutCreateMenu (menuDualB);
  glutAddMenuEntry ("First tangential b-surface [a]", 	         1);
  glutAddMenuEntry ("Second tangential b-surface [s]", 	         2);
  glutAddMenuEntry ("Both [d]", 				 3);
  glutAddMenuEntry ("Neither [f]",			 	 4);
  glutAddMenuEntry ("Intersection curves [C]", 			 5);
  glutAddMenuEntry ("Box [b]", 					 6);
  glutAddMenuEntry ("Trace the intersection curves", 		 7);
  glutAddMenuEntry ("Tangential b-surface control nets [2]", 	 20);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

							// c-dual window
  glutInitWindowPosition (xleft+xsize+2*barmargin-1,
  			  titleht+10+2*halfysize+4*titleht);
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential c-surfaces");
  dualWinC = glutCreateWindow (titlebar);
  gfxinit();  
  glutReshapeFunc (reshape);
  glutDisplayFunc (displayDualC);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondualC);
  glutCreateMenu (menuDualC);
  glutAddMenuEntry ("First tangential c-surface [a]", 	         1);
  glutAddMenuEntry ("Second tangential c-surface [s]", 	         2);
  glutAddMenuEntry ("Both [d]", 				 3);
  glutAddMenuEntry ("Neither [f]", 			 	 4);
  glutAddMenuEntry ("Intersection curves [C]", 			 5);
  glutAddMenuEntry ("Box [b]", 					 6);
  glutAddMenuEntry ("Tangential c-surface control nets [2]", 	 20);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
