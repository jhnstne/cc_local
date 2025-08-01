/*
  File:          silhouette.cpp
  Author:        J.K. Johnstone 
  Created:	 18 December 2002 (from tangentialSurf.cpp)
  Last Modified: 23 December 2002
  Purpose:       Compute the smooth silhouette of the given surface,
  		 using tangential surfaces.
		 Builds on tangentialSurf.cpp.
  Sequence:	 3rd in a sequence (surfinterpolate, tangentialSurf, silhouette)
  Input: 	 k 3d rectangular point sets
  Output: 	 k Bezier surfaces
  History: 	 12/23/02: Added read-from-storage capability (STORED).
		 1/8/03:   Added perspective projection option from viewpoint.
		 	   Changed back to single colour for tang surfaces
			   and silhouette curve.
			   Add -D option (preparing tangential surfaces for
			   display dominates silhouette computation).
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
#include "Vector.h"		// IntArrArr (active flags)
#include "MiscVector.h"		
#include "BezierSurf.h"		// drawNorm, drawTangSpace (primal surfaces)
				// BezierSurf1f
#include "TangSurf.h"		// tangSurfComponents, createA/B/C, 
				// prepareDisplay, drawT

#define PTSPERBEZSEGMENT 5   	// # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 5)" << endl;
  cout << "\t[-e epsilon accuracy for clipping] (default: .01)" << endl;
  cout << "\t[-E epsilon accuracy for intersection] (default: .0001)" << endl;
  cout << "\t[-m] (direct bicubic Bezier control mesh input)" << endl;
  cout << "\t[-M] (unified bicubic Bezier control mesh input)" << endl;
  cout << "\t[-S] (use stored tangential surface system)" << endl;
  cout << "\t[-D] (don't display tangential surfaces)" << endl;
  cout << "\t[-v] x y z (viewpoint)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts3" << endl;
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
static GLboolean WIREA=0;		// draw a-surfaces in wireframe?
static GLboolean WIREB=0;		// draw b-surfaces in wireframe?
static GLboolean WIREC=0;		// draw c-surfaces in wireframe?
static GLboolean CTRLMESH=0;		// input data is control mesh directly?
static GLboolean UNIFIEDCTRLMESH=0;	// input data is unified control mesh? (not just a collection of separate bicubic patches)
static GLboolean DRAWSURF=1;		// draw primal surfaces?
static GLboolean DRAWNORM=0;		// draw primal surface normals?
static GLboolean DRAWTANGSPACE=0;	// draw primal tangent spaces?
static GLboolean DRAWTANGSURFA=1;	// draw tangential a-surfaces?
static GLboolean DRAWTANGSURFB=1;	// draw tangential b-surfaces?
static GLboolean DRAWTANGSURFC=1;	// draw tangential c-surfaces?
static GLboolean DRAWCTRLNETA=0;	// draw a-surface control net?
static GLboolean DRAWCTRLNETB=0;	// draw b-surface control net?
static GLboolean DRAWCTRLNETC=0;	// draw c-surface control net?
static GLboolean DRAWNORMA=0;		// draw a-surface normals?
static GLboolean DRAWNORMB=0;		// draw b-surface normals?
static GLboolean DRAWNORMC=0;		// draw c-surface normals?
static GLboolean DRAWACTIVEPT=0;	// draw active point?
static GLboolean DRAWABOX=1;		// draw box in a-dual space?
static GLboolean DRAWBBOX=1;		// draw box in b-dual space?
static GLboolean DRAWCBOX=1;		// draw box in c-dual space?
static GLboolean DRAWVIEWPT=0;		// draw viewpoint and its duals?
static GLboolean DRAWSILHOUETTECOMP=0;	// draw silhouette components?
static GLboolean DRAWSILHOUETTE=1;	// draw full silhouette?
static GLboolean STORED=0;		// use stored tangential surface system, rather than computing from scratch
static GLboolean ORTHO=0;		// orthographic projection?
static GLboolean NOTANGDISPLAY=0;	// don't display tangential surfaces?

Array<BezierSurf3f> 	obstacle;	// primal surfaces
Array<TangentialSurf>   obdualA; 	// associated tangential a-surfaces
Array<TangentialSurf>	obdualB; 	// associated tangential b-surfaces
Array<TangentialSurf>	obdualC; 	// associated tangential c-surfaces
V3f			viewpt(2,2,2);	// viewpoint w.r.t silhouette is computed
TangentialSurf 		viewptadual,viewptbdual,viewptcdual;	// duals of viewpoint (each a plane)
PatchIntersection	iCurveadual,iCurvebdual,iCurvecdual;	// intersection of viewpoint dual and tangential surfaces
PatchIntersection 	iCurveaprimal,iCurvebprimal,iCurvecprimal; // silhouette, as primal version of iCurvea/b/cdual (same parameter values, different points)
PatchIntersection	silhouette;	// silhouette curve, spliced from 3 dual spaces
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
  if (ORTHO)
    glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -1000.,1000.);
  else
    gluPerspective(60.0, (GLfloat)w/(GLfloat)h, 1.0, 20.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if (!ORTHO)
    gluLookAt (viewpt[0], viewpt[1], viewpt[2], 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
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
  case 'a':  	DRAWACTIVEPT = !DRAWACTIVEPT;	break;	
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
  case 'v': 	DRAWVIEWPT = !DRAWVIEWPT;	break;
  case 's': 	DRAWSILHOUETTE=!DRAWSILHOUETTE; break;
  case 'c': 	DRAWSILHOUETTECOMP=!DRAWSILHOUETTECOMP; break;
  case 'o':	ORTHO = !ORTHO;			break;
  case 'f':	break;
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
  case 9:	DRAWVIEWPT = !DRAWVIEWPT;	break;	
  case 10:	break;
  case 11: 	DRAWSILHOUETTECOMP=!DRAWSILHOUETTECOMP; break;
  case 12: 	DRAWSILHOUETTE=!DRAWSILHOUETTE; break;
  case 13: 	ORTHO = !ORTHO;			break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDualA (int value)
{
  switch (value) {
  case 1:	DRAWTANGSURFA = !DRAWTANGSURFA;		break;
  case 2:	DRAWCTRLNETA = !DRAWCTRLNETA;		break;
  case 3:	DRAWNORMA = !DRAWNORMA;			break;
  case 4:	WIREA = !WIREA;				break;
  case 6:	DRAWABOX = !DRAWABOX;			break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDualB (int value)
{
  switch (value) {
  case 1:	DRAWTANGSURFB = !DRAWTANGSURFB;		break;
  case 2:	DRAWCTRLNETB = !DRAWCTRLNETB;		break;
  case 3:	DRAWNORMB = !DRAWNORMB;			break;
  case 4:	WIREB = !WIREB;				break;
  case 6:	DRAWBBOX = !DRAWBBOX;			break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDualC (int value)
{
  switch (value) {
  case 1:	DRAWTANGSURFC = !DRAWTANGSURFC;		break;
  case 2:	DRAWCTRLNETC = !DRAWCTRLNETC;		break;
  case 3:	DRAWNORMC = !DRAWNORMC;			break;
  case 4:	WIREC = !WIREC;				break;
  case 6:	DRAWCBOX = !DRAWCBOX;			break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
// cout << "Entering displayOb" << endl;
  int i;
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
       { glColor3fv (material[(i+1)%24]+4); obstacle[i].draw(1); }
      glEnable (GL_LIGHTING);
     }
    else
      for (i=0; i<obstacle.getn(); i++)	
       {
        glMaterialfv(GL_FRONT, GL_AMBIENT,  material[(i+1)%24]);
        glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[(i+1)%24]+4);
        glMaterialfv(GL_FRONT, GL_SPECULAR, material[(i+1)%24]+8);
        glMaterialf(GL_FRONT, GL_SHININESS, material[(i+1)%24][12] * 128.0);
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
  if (DRAWVIEWPT)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    glBegin(GL_POINTS);
    glVertex3f (viewpt[0], viewpt[1], viewpt[2]);
    glEnd();
    glEnable (GL_LIGHTING);    
   }
  if (DRAWSILHOUETTECOMP)
   {
    glDisable (GL_LIGHTING);
    glLineWidth (3.0);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    iCurveaprimal.draw();
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Green);
    iCurvebprimal.draw();
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Blue);
    iCurvecprimal.draw();
    glLineWidth (1.0);
    glEnable (GL_LIGHTING);
   }
  if (DRAWSILHOUETTE)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Black);
    silhouette.draw();
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
  if (DRAWTANGSURFA && !NOTANGDISPLAY)	
   {
    if (WIREA)		// wireframe of clipped tangential a-surface
     { 
      glDisable (GL_LIGHTING);
      for (i=0; i<obstacle.getn(); i++)
       {
        glColor3fv (material[(i+1)%24]+4); obdualA[i].drawT (density,1);
       }
      glEnable (GL_LIGHTING);
     }
    else 
      for (i=0; i<obstacle.getn(); i++)
       { 
        glMaterialfv(GL_FRONT, GL_AMBIENT,  material[(i+1)%24]);
        glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[(i+1)%24]+4);
        glMaterialfv(GL_FRONT, GL_SPECULAR, material[(i+1)%24]+8);
        glMaterialf(GL_FRONT, GL_SHININESS, material[(i+1)%24][12] * 128.0);
	obdualA[i].drawT(density,0);
       }
   }
  if (DRAWCTRLNETA)		// control net of clipped tangential a-surface
   {
    glDisable (GL_LIGHTING);
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[(i+1)%24]+4);
      obdualA[i].drawCtrlNetT();
     }
    glEnable (GL_LIGHTING);
   }

// not computing normals for tangential surfaces anymore (too expensive)
// if you want to compute again, 
// have tangSurf::prepareDisplay call prepareActiveDisplay(active,density)
//  if (DRAWNORMA)		// draw normals at sample points
//    for (i=0; i<obstacle.getn(); i++)
//     {
//      glColor3fv (material[i%24]+4);
//      obdualA[i].drawNormT (density);
//     }
  if (DRAWACTIVEPT)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    obdualA[0].drawPt (uActive, vActive);
    glEnable (GL_LIGHTING);
   }
  if (DRAWVIEWPT)
   {
    if (WIREA)		// draw dual of viewpoint
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[1]+4); viewptadual.drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[1]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[1]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[1]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[1][12] * 128.0);
      viewptadual.drawT(density,0);
     }
   }
  if (DRAWSILHOUETTECOMP)
   {
    glDisable (GL_LIGHTING);	// draw silhouette in dual space
    glLineWidth (3.0);
    glColor3fv (Black);
    iCurveadual.draw();
    glLineWidth (1.0);
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
  if (DRAWTANGSURFB && !NOTANGDISPLAY)	
   {
    if (WIREB)		// wireframe of clipped tangential b-surface
     {
      glDisable (GL_LIGHTING);
      for (i=0; i<obstacle.getn(); i++)
       {
        glColor3fv (material[(i+1)%24]+4); obdualB[i].drawT (density,1);
       }
      glEnable (GL_LIGHTING);
     }
    else
      for (i=0; i<obstacle.getn(); i++)
       { 
        glMaterialfv(GL_FRONT, GL_AMBIENT,  material[(i+1)%24]);
        glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[(i+1)%24]+4);
        glMaterialfv(GL_FRONT, GL_SPECULAR, material[(i+1)%24]+8);
        glMaterialf(GL_FRONT, GL_SHININESS, material[(i+1)%24][12] * 128.0);
	obdualB[i].drawT(density,0);
       }
   }
  if (DRAWCTRLNETB)		// control net of clipped tangential b-surface
   {
    glDisable (GL_LIGHTING);
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[(i+1)%24]+4);
      obdualB[i].drawCtrlNetT();
     }
    glEnable (GL_LIGHTING);
   }
/*  if (DRAWNORMB)		// draw normals at sample points
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[i%24]+4);
      obdualB[i].drawNormT (density);
     }
*/
  if (DRAWACTIVEPT)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    obdualB[0].drawPt (uActive, vActive);
    glEnable (GL_LIGHTING);
   }
  if (DRAWVIEWPT)
   {
    if (WIREB)		// draw dual of viewpoint
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[1]+4); viewptbdual.drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[1]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[1]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[1]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[1][12] * 128.0);
      viewptbdual.drawT(density,0);
     }
   }
  if (DRAWSILHOUETTECOMP)
   {
    glDisable (GL_LIGHTING);	// draw silhouette in dual space
    glLineWidth (3.0);
    glColor3fv (Black);
    iCurvebdual.draw();
    glLineWidth (1.0);
    glEnable (GL_LIGHTING);
   }

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
  if (DRAWTANGSURFC && !NOTANGDISPLAY)		
   {
    if (WIREC)		// wireframe of clipped tangential c-surface
     { 
      glDisable (GL_LIGHTING);
      for (i=0; i<obstacle.getn(); i++)
       {
        glColor3fv (material[(i+1)%24]+4); obdualC[i].drawT (density,1);
       }
      glEnable (GL_LIGHTING);
     }
    else 
      for (i=0; i<obstacle.getn(); i++)
       { 
        glMaterialfv(GL_FRONT, GL_AMBIENT,  material[(i+1)%24]);
        glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[(i+1)%24]+4);
        glMaterialfv(GL_FRONT, GL_SPECULAR, material[(i+1)%24]+8);
        glMaterialf(GL_FRONT, GL_SHININESS, material[(i+1)%24][12] * 128.0);
	obdualC[i].drawT(density,0);
       }
   }
  if (DRAWCTRLNETC)		// control net of clipped tangential c-surface
   {
    glDisable (GL_LIGHTING);
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[(i+1)%24]+4);
      obdualC[i].drawCtrlNetT();
     }
    glEnable (GL_LIGHTING);
   }

/*  if (DRAWNORMC)		// draw normals at sample points
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (material[i%24]+4);
      obdualC[i].drawNormT (density);
     }
*/
  if (DRAWACTIVEPT)
   {
    glDisable (GL_LIGHTING);
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    obdualC[0].drawPt (uActive, vActive);
    glEnable (GL_LIGHTING);
   }
  if (DRAWVIEWPT)
   {
    if (WIREC)		// draw dual of viewpoint
     { 
      glDisable (GL_LIGHTING);
      glColor3fv (material[1]+4); viewptcdual.drawT (density,1);
      glEnable (GL_LIGHTING);
     }
    else 
     { 
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[1]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[1]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[1]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[1][12] * 128.0);
      viewptcdual.drawT(density,0);
     }
   }
  if (DRAWSILHOUETTECOMP)
   {
    glDisable (GL_LIGHTING);	// draw silhouette in dual space
    glLineWidth (3.0);
    glColor3fv (Black);
    iCurvecdual.draw();
    glLineWidth (1.0);
    glEnable (GL_LIGHTING);
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
  ifstream infile;  infile.open(file);
  V3fArrArrArr Pt;
  read (infile, Pt);  scaleToUnitCube (Pt);  
  int numSegu, numSegv;
  if (UNIFIEDCTRLMESH) infile >> numSegu >> numSegv;
  infile.close();
  obstacle.allocate(Pt.getn());
  for (int i=0; i<Pt.getn(); i++)
   { 
    if (CTRLMESH)	// each Pt[i] is a bicubic patch
     {
      FloatArr knot(2);  knot[0] = 0; knot[1] = 1;
      obstacle[i].create (3, 3, 3, 1, 1, Pt[i], knot, knot);  // define one bicubic patch
     }
    else if (UNIFIEDCTRLMESH)  // each Pt[i] is a collection of bicubic patches
     {
      FloatArr knotu(numSegu+1), knotv(numSegv+1);
      int j;
      for (j=0; j<=numSegu; j++) knotu[j] = j;
      for (j=0; j<=numSegv; j++) knotv[j] = j;
      obstacle[i].create (3, 3, 3, numSegu, numSegv, Pt[i], knotu, knotv);
     }
    else obstacle[i].fit (Pt[i]);
    obstacle[i].prepareDisplay (density);
   }
}

/******************************************************************************
******************************************************************************/

void readInput(char *file)
{
  int i,nOb;
  if (STORED)		// read in tangential surface system from file storage
   {
   		cout << "Using stored tangential surface system" << endl;
    nOb = 1;	// use nOb to make this code consistent with bidev.cpp
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
      obdualB[i].createB (i,a,b,c,d,eps);
      obdualC[i].createC (i,a,b,c,d,eps);
     }
   }
  if (!NOTANGDISPLAY)
    for (i=0; i<nOb; i++)
     {
		cout << "Preparing display of tangential a-surface for obstacle " << i << "..." << endl; 
      obdualA[i].prepareDisplay (density);
		cout << "Preparing display of tangential b-surface for obstacle " << i << "..." << endl; 
      obdualB[i].prepareDisplay (density);
		cout << "Preparing display of tangential c-surface for obstacle " << i << "..." << endl; 
      obdualC[i].prepareDisplay (density);
     }
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
      case 'S': STORED = 1;				break;
      case 'v': viewpt[0] = atof(argv[ArgsParsed++]);
      		viewpt[1] = atof(argv[ArgsParsed++]);
		viewpt[2] = atof(argv[ArgsParsed++]);	break;
      case 'D': NOTANGDISPLAY = 1;			break;
      case 'h': 
      default:	usage(); exit(-1);			break;
      }
   else ArgsParsed++;
  }
  
  readInput(argv[argc-1]);
  viewptadual.createA (0,viewpt);
  viewptbdual.createB (0,viewpt);
  viewptcdual.createC (0,viewpt);
  viewptadual.prepareDisplay (density);
  viewptbdual.prepareDisplay (density);
  viewptcdual.prepareDisplay (density);
  PatchIntArrArrArr fooTrace;
  cout << "Intersection in a-space" << endl;
  viewptadual.intersect (obdualA[0], iCurveadual, fooTrace, epsInt);
  cout << "Intersection in b-space" << endl;
  viewptbdual.intersect (obdualB[0], iCurvebdual, fooTrace, epsInt);
  cout << "Intersection in c-space" << endl;
  viewptcdual.intersect (obdualC[0], iCurvecdual, fooTrace, epsInt);
  // cout << "Dual of a-silhouette: " << endl;  iCurveadual.print();
  // cout << "Dual of b-silhouette: " << endl;  iCurvebdual.print();
  // cout << "Dual of c-silhouette: " << endl;  iCurvecdual.print();
  cout << "Computing silhouette from intersection curve" << endl;
  obstacle[0].bMap (iCurveadual, iCurveaprimal);
  obstacle[0].bMap (iCurvebdual, iCurvebprimal);
  obstacle[0].bMap (iCurvecdual, iCurvecprimal);
  // cout << "a-silhouette: " << endl;  iCurveaprimal.print();
  // cout << "b-silhouette: " << endl;  iCurvebprimal.print();
  // cout << "c-silhouette: " << endl;  iCurvecprimal.print();
  // stitch together the silhouette curves from the three dual spaces
  silhouette =  iCurveaprimal;
  silhouette += iCurvebprimal;
  silhouette += iCurvecprimal;
  while (silhouette.spliceClosest (10*epsInt,0,1))	// commented out until recently
    ; 
  // cout << "Full silhouette: " << endl; silhouette.print();

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
  glutAddMenuEntry ("Active point [a]",      		      4);
  glutAddMenuEntry ("Move active point left [h]",	      5);
  glutAddMenuEntry ("Move active point right [j]",	      6);
  glutAddMenuEntry ("Move active point up [u]",	      	      7);
  glutAddMenuEntry ("Move active point down [n]",	      8);
  glutAddMenuEntry ("Viewpoint [v]",			      9);
  glutAddMenuEntry ("Silhouette components [c]",	     11);
  glutAddMenuEntry ("Silhouette [s]",			     12);
  glutAddMenuEntry ("Flip normals [f]",			     10);
  glutAddMenuEntry ("Orthographic projection [o]",	     13);
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
  glutAddMenuEntry ("Tangential a-surfaces [1]", 	         1);
  glutAddMenuEntry ("Tangential a-surface control nets [2]", 	 2);
  glutAddMenuEntry ("Normals (not available, for speed) [3]",	 3);
  glutAddMenuEntry ("Wireframe", 				 4);
  glutAddMenuEntry ("Box [b]", 					 6);
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
  glutAddMenuEntry ("Tangential b-surfaces [1]", 	         1);
  glutAddMenuEntry ("Tangential b-surface control nets [2]", 	 2);
  glutAddMenuEntry ("Normals (not available) [3]", 	         3);
  glutAddMenuEntry ("Wireframe", 				 4);
  glutAddMenuEntry ("Box [b]", 					 6);
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
  glutAddMenuEntry ("Tangential c-surfaces [1]", 	         1);
  glutAddMenuEntry ("Tangential c-surface control nets [2]", 	 2);
  glutAddMenuEntry ("Normals (not available) [3]", 	         3);
  glutAddMenuEntry ("Wireframe", 				 4);
  glutAddMenuEntry ("Box [b]", 					 6);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
