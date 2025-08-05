/*
  File:          umbra.cpp
  Author:        J.K. Johnstone 
  Created:	 15 February 2002
  Last Modified: 28 February 2006
  Purpose:       Compute the umbra cast by a scene.
		 Builds on bitang.c++, which computes bitangents.
  Sequence:	 4th in a sequence (interpolate, tangCurve, bitang, umbra)
  Input: 	 k 2d polygons, implicitly defining k interpolating cubic 
  		 Bezier curves; the first curve is the light, the remaining
		 curves are the objects in the scene
  History: 	 c. 3/8/02: Add refinement of umbral bitangent
  		 3/14/02: Animated refinement.
		 4/16/02: Defined umbral polygons directly.
		 	  Allowed varying size room.
		 1/21/03: Allow bitangents and umbrae to be shown
		 	  for only one obstacle (the first)
		 1/27/03: Added filled local umbra option.
		 1/28/03: Added maximal umbra option.
		 	  Added umbraOb/umbraExtreme to inner
		 3/13/03: Added umbral labels.
		 3/19/03: Added umbral shading.
		 4/16/03: Whited out obstacle interior for prettier shadow display.
		 5/1/03:  Added scene input option.
		 5/19-20/03: Added piercing sweep.
		 10/28/03: changed bitang to Array<UmbralBitangArr> from
		           Array<CommonTangentArr> so that it can call outer and inner
		 10/31/03: Introduced closeEps in inner/outer/pierce calls, 
		           and set to .02/radiusRoom so that intersections in nonstandard 
			   room sizes (standard is 2) are filtered appropriately.
			   Changed sampling rate in defineBackUmbra to .1 (from .01), 
			   which improves performance and robustness 
			   (if sampled points are close to tangent, triangulation may 
			   diagnose nonsimple polygon).
		 11/03/03: Added sameSideEps parameter choice.
		 11/04/03: Added inner bitangents of A and B as candidates 
		           in inner piercing sweep.
		 11/08/03: Split buildPierceBitang into buildInnerPierce and 
		           buildOuterPierce, since outer piercing bitangents only need 
			   to be computed wrt light, saving much computation.
		 12/30/03: Cleaning; adding level control.
		           Changed closeEps default to .1/radiusRoom
			   and sameSideEps default to .3.
		 1/2/04:   Refining inner piercing sweep. 
		           Changed eps to epsIntersect.
		 2/10/04:  Added minimum feature size.
		 2/11/04:  Culled bitangents within featureSize of another
		           (that is, coordinates of both points are within featureSize)
		 ROBUSTNESS PROGRESS:
		 1) cull bitangents whose pts of tangency are within featureSize 
		    of another bitangent
		 2) robustly diagnose inner and outer bitangents by culling intersections
		    within feature-size of the point of tangency
		 3) potential improvement: adjust feature-size locally, based on curvature
		    at a point of tangency, since flatter regions yield false 
		    intersections further from the point of tangency than highly curved
		    regions
		 6/21-28/04: Added direct/indirect bitangents and direct/indirect 
		          visual events.
		 6/28/04:  Light --> distinguished object.
		 2/28/06: updated to modern C++ library
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

#include "basic/AllColor.h"
#include "basic/Miscellany.h"		
#include "basic/Vector.h"		
#include "basic/MiscVector.h"		
#include "curve/BezierCurve.h"	
#include "curve/Scene2d.h"
#include "tangcurve/TangCurve.h"		// CommonTangent, intersect, draw, visible
#include "umbra/UmbralBitang.h"

#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment
#define MAXDIRECT        10      // maximum number of direct bitangents to one obstacle 
                                 // in this scene (there is no inherent limit)
#define MAXINDIRECT      10      // maximum number of indirect bitangents between 
                                 // 2 obstacles in this scene (there is no inherent limit)

static char *RoutineName;
static void usage()
 {
  cout << "Build the umbra of a smooth 2D scene with respect to a distinguished object." 
       << endl;
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-e epsIntersect] (accuracy at which intersections are made: default .001)" 
       << endl;
  cout << "\t[-E epsIntersectSmall] (accuracy at which intersections are made in defineGlobalFrontUmbra: default .000001)" 
       << endl;
  cout << "\t[-c closeEps]    (intersections are ignored within this nhood of " << endl;
  cout << "\t\t\t\t parameter value of pt of bitangency; default: .1/radiusRoom)" << endl;
  cout << "\t[-F featureSize] (points within this distance are indistinguishable) default .05" << endl;
  cout << "\t[-f sameSideEps] (parameter jump to get away from tangent and test side the curve lies on: default .3)" << endl;
  cout << "\t[-s #] (obstacle that has refining umbral bitangent animated: 1<=n<=nOb [default: 1])" << endl;
  cout << "\t[-u #] (which umbral bitangent to animate: 0 or 1 [default: 0])" << endl;
  cout << "\t[-r #] (radius of bounding room; default 2)" << endl;
  cout << "\t[-w]   (weird case: A surrounds L: don't compute everything)" << endl;
  cout << "\t[-S]   (scene input)" << endl;
  cout << "\t[-t]   (only compute tangents, for debugging)" << endl;
  cout << "\t[-P #] (number of levels to compute)" << endl;
  cout << "\t    0: scene display" << endl;
  cout << "\t    1: bitangents"    << endl;
  cout << "\t    2: outer bitangents" << endl;
  cout << "\t    3: inner bitangents" << endl;
  cout << "\t    4: outer piercing bitangents" << endl;
  cout << "\t    5: inner piercing bitangents" << endl;
  cout << "\t    6: local back umbra" << endl;
  cout << "\t    7: local back penumbra" << endl;
  cout << "\t    8: local front umbra" << endl;
  cout << "\t    9: local front penumbra" << endl;
  cout << "\t   10: inner sweep" << endl;
  cout << "\t   11: outer sweep" << endl;
  cout << "\t   12: inner piercing sweep" << endl;
  cout << "\t   13: outer piercing sweep" << endl;
  cout << "\t   14: global back umbra" << endl;
  cout << "\t   15: global front umbra" << endl;
  cout << "\t[-h]   (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWOBSTACLE=1;	// draw obstacles?
static GLboolean DRAWBITANG=0;		// draw bitangents?
static GLboolean DRAWSELFBITANG=0;	// draw self-bitangents?
static GLboolean DRAWDIRECTBITANG=0;    // draw bitangents defining direct visual events?
static GLboolean DRAWINDIRECTBITANG=0;  // draw bitangents defining indirect v. events?
static GLboolean DRAWDIRECTVE=0;        // draw direct visual events?
static GLboolean DRAWINDIRECTVE=0;      // draw indirect visual events?
static GLboolean DRAWONEBITANG=0;	// draw one bitangent at a time?
static GLboolean DRAWONELATE=0;         // draw the late segment of one bitangent?
static GLboolean DRAWONEINTERM=0;       // draw the intermediate segment of one bitangent?
static GLboolean DRAWOTANG=0;		// draw outer bitangents?
static GLboolean DRAWITANG=0;		// draw inner bitangents?
static GLboolean DRAWOPTANG=0;		// draw outer piercing bitangents?
static GLboolean DRAWIPTANG=0;		// draw inner piercing bitangents?
static GLboolean DRAWFRONTPENUMBRA=0;   // draw front penumbra?
static GLboolean DRAWMAXFRONTUMBRA=0;	// draw maximal front umbra?
static GLboolean DRAWLOCALFRONTUMBRA=0;	// draw local front umbrae?
static GLboolean DRAWGLOBALFRONTUMBRA=0;// draw global front umbra?
static GLboolean DRAWBACKPENUMBRA=0;    // draw back penumbra?
static GLboolean DRAWMAXBACKUMBRA=0;	// draw maximal back umbra?
static GLboolean DRAWLOCALBACKUMBRA=0;	// draw local back umbrae?
static GLboolean DRAWGLOBALBACKUMBRA=0;	// draw global back umbra?
static GLboolean UMBRAFILLED=1;		// draw global umbra filled?
static GLboolean LABELFRONTUMBRA=0;	// label front umbra with 'u'?
static GLboolean LABELBACKUMBRA=0;	// label back umbra with 'u'?
static GLboolean SHADEFRONTUMBRA=0;	// shade local front umbra?
static GLboolean LABEL=0;		// label the distinguished object (with 0) and obstacles (with 1,2,...)?
static GLboolean LETTERLABEL=0;         // label the d.-object (with A) and obstacles (B,C,...)?
static GLboolean LABELLIGHT=1;          // label the distinguished object with A?
static GLboolean ANIMATEINNERSWEEP=0;	// animate inner-sweeping umbral bitangent?
static GLboolean ANIMATEOUTERSWEEP=0;	// animate outer-sweeping umbral bitangent?
static GLboolean ANIMATEINPIERCESWEEP=0;// animate innerpiercing-sweeping umbral bitang?
static GLboolean ANIMATEOUTPIERCESWEEP=0;// animate outerpiercing-sweeping umbral bitang?
static GLboolean DRAWINNERUMBTANG=0;	// draw inner umbral bitangents?
static GLboolean DRAWOUTERUMBTANG=0;	// draw outer umbral bitangents?
static GLboolean DRAWPIERCEUMBTANG=0;	// draw piercing umbral bitangents?
static GLboolean ONLYFIRST=0;		// only draw bitangents/umbrae for 1st obstacle?
static GLboolean LABELRESPONSIBLE=0;     // label object responsible for blockage?
static GLboolean SCENEINPUT=0;          // scene input?
static GLboolean DRAWORIGIN=0;          // draw the origin?
static GLboolean SURROUND=0;            // does A surround L?
static GLboolean COMPUTETANGONLY=0;     // only compute bitangents, not umbra (for debugging)?
int              level=15;              // computation level

Array<BezierCurve2f> 	obstacle;	// interpolating cubic Bezier curves
int                     nOb;            // redundant, but useful
V2fArr			obCentroid;	// obstacle centroids
Array<TangentialCurve>  obduala;	// associated tangential a-curves
Array<TangentialCurve>  obdualb;	// associated tangential b-curves
Array<UmbralBitangArr>  bitang;		// bitangents between curves and 'light'
					// curve i/j bitangents are stored in 
					// index i*obstacle.getn() + j
					// (0 is the 'light')
Array<UmbralBitangArr>  selfbitang;     // selfbitangents of every curve
int 			nextbitang=0;	// counter for cycling thru bitangents
int 			nextcurve=1;	// another counter for same purpose
float 			radiusRoom=2;	// radius of bounding room's square (which is centered at origin)
Polygon2f		room;		// bounding room
Array<UmbralBitangArr>  direct;		// bitangents that define direct visual events;
                                        // for each obstacle to 'light'
Array<UmbralBitangArr>  indirect;	// bitangents that define indirect visual events;
                                        // between each pair of non-light obstacles;
                                        // stored like inner
Array<UmbralBitangArr>  outer;		// outer bitangents for each obstacle
					// to light, 2 per obstacle
Array<UmbralBitangArr>  inner;		// inner bitangents for each curve,
					// stored like bitang;
					// both [i*nOb+j] and [j*nOb+i] are stored
					// even though these are equivalent (for elegance in replaceByInner)
Array<UmbralBitangArr>  outerpierce;	// outer piercing bitangents for each curve,
					// arbitrary # per obstacle
Array<UmbralBitangArr>  innerpierce;    // inner piercing bitangents for each obstacle,
                                        // stored like inner but arbitrary # per obstacle
                                        // in [i*nOb+j], obstacle[i] is dominant
                                        // that is, bitangent pierces obstacle[i]
Array<V2fArr>           posteriorSample;// for each obstacle, samples on the posterior wrt light
Array<UmbralBitangArr>  sweptInner;	// global umbral bitangents generated by inner sweep,
					// 2 for each obstacle
Array<UmbralBitangArr>  sweptOuter;	// global umbral bitangents 
					// generated by outer sweep,
					// 2 for each obstacle
Array<UmbralBitangArr>  sweptInPierce;  // global umbral bitangents
                                        // generated by inner piercing sweep (for front umbra),
                                        // 1 per outer piercing bitangent
Array<UmbralBitangArr>  sweptOutPierce; // global umbral bitangents
                                        // generated by outer piercing sweep (for front umbra),
                                        // 1 per outer piercing bitangent (1 per component)
Array<Polygon2f>	globalBackUmbra;		// (global) umbral polygon (bounded by room) for each obstacle
Array<Polygon2f>	localBackUmbra;	// (local) back umbral polygon (bounded by room) for each obstacle
Array<Polygon2f>	backPenumbra;	// maximal umbral polygon (bounded by room) for each obstacle
Array<Polygon2fArr>     localFrontUmbra;// polygons of the local front umbra for each obstacle
Array<Polygon2fArr>     frontPenumbra;  // polygons of the maximal front umbra for each obstacle
Array<Polygon2fArr>     globalFrontUmbra; // polygons of the global front umbra for each obstacle
int 			specialOb=1;	// animation of refinement of umbral bitangent
					// will be performed for obstacle[specialOb!=0]
BezierCurve2f		specialHodo;	// hodograph of obstacle[specialOb]					
BezierCurve2f		lightHodo;	// hodograph of light
int			specialUmb=0;	// index of umbral bitangent on specialOb to animate
UmbralBitang		specialUmbTang; // the umbral bitangent being animated
float                   epsIntersect = .001;  // accuracy of intersection computation
float                   epsIntersectSmall = .000001; // accuracy for defineGlobalFrontUmbra intersection (testing if outer piercing sweep bitangent hits A)
float                   closeEps   = -1; // used to filter intersections near pt of tangency
float                   featureSize= .05;// the smallest recognizable feature;
                                         // points closer than this are indistinguishable;
                                         // used to filter intersections near pt of tangency
float                   sameSideEps= .3;

int 			nRefined;	
float 			tAnimate;	// present parameter for sweeping tangent
// int			pause=100;	// pause this many steps in animation
int 			Pause=1;	// flag: are you pausing in sweep animation?
float color[][3]  = {{0,0,0}, {.8,.8,.8}, {0.411765, 0.411765, 0.411765},  // Black, -, DimGrey
		     {1,0,0}, {1,.5,.5}, {1, .7, .7},                     // Red, my medium red, my pale red
		     {0,0,1}, {.3,.3,1}, {0.690196, 0.886275, 1},         // Blue, -, LightSkyBlue1
		     {0,1,0}, {.3,1,.3}, {0.603922, 1, 0.603922},         // Green, PaleGreen1
		     {1,0,1}, {1,.3,1},  {1,.7, 1},
		     {0,1,1}, {.3,1,1},  {.7,1,1},
		     {1,1,0}, {1,1,.3},  {1,1,.7}};
int			obstacleWin;	// primal window identifier 
int       		nPtsPerSegment = PTSPERBEZSEGMENT;

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  glClearColor (1.0, 1.0, 1.0, 1.0);

  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT,  GL_FASTEST);
  glEnable (GL_POINT_SMOOTH);
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);

  transxob = transyob = 0.0;
  zoomob = .85;
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

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 				    break; // ESCAPE
  case '1':	DRAWBITANG          = !DRAWBITANG;	    break;
  case '2': 	DRAWONEBITANG       = !DRAWONEBITANG;	    break;
  case '3':     DRAWLOCALBACKUMBRA  = !DRAWLOCALBACKUMBRA;  break;
  case '4':     DRAWITANG           = !DRAWITANG;	    break;
  case '5': 	DRAWGLOBALBACKUMBRA = !DRAWGLOBALBACKUMBRA; break;
  case '6':	ANIMATEINNERSWEEP   = !ANIMATEINNERSWEEP;		
  		if (ANIMATEINNERSWEEP)
		 {
		  specialUmbTang = sweptInner[specialOb][specialUmb];
		  tAnimate = specialUmbTang.tRefine[0]; 
		  nRefined = 1;
		  Pause = 1;
		 }
		 					    break;
  case '7':	ANIMATEOUTERSWEEP   = !ANIMATEOUTERSWEEP;		
  		if (ANIMATEOUTERSWEEP)
		 {
		  specialUmbTang = sweptOuter[specialOb][specialUmb];
		  tAnimate = specialUmbTang.tRefine[0];
		  nRefined = 1;
		  Pause = 1;
		 }					    break;
  case '8':	DRAWINNERUMBTANG    = !DRAWINNERUMBTANG;    break;
  case '9':	DRAWOUTERUMBTANG    = !DRAWOUTERUMBTANG;    break;
  case '0': 	UMBRAFILLED         = !UMBRAFILLED;	    break;
  case ' ':	Pause = 0;               		    break; // stop pausing
  case 8:	nextbitang = mod(nextbitang-1, bitang[1].getn());  // BACKSPACE    
                                                            break; 
  case 9:	nextbitang++;                                      // TAB
  		if (nextbitang == bitang[nextcurve].getn())
		 {
		  nextbitang = 0; 
		  nextcurve++;
		  while (nextcurve < bitang.getn() && 
		  	 bitang[nextcurve].getn() == 0) 
		    nextcurve++;
		  if (nextcurve == bitang.getn()) nextcurve = 1;
		 } 	
		                                            break;
  case 'a':   	LABEL               = !LABEL;		    break;
  case 'b': 	DRAWLOCALBACKUMBRA  = !DRAWLOCALBACKUMBRA;  break;
  case 'B': 	DRAWGLOBALBACKUMBRA = !DRAWGLOBALBACKUMBRA; break;
    /* if (!DRAWGLOBALBACKUMBRA)		// turn global umbra on/off
    		 {
		  DRAWGLOBALBACKUMBRA = 1; DRAWLOCALBACKUMBRA = 0;
		 }
		else 
		 {
		  DRAWGLOBALBACKUMBRA = 0; DRAWLOCALBACKUMBRA = 0;
		  } */
  case 'c':	ANIMATEINPIERCESWEEP  = !ANIMATEINPIERCESWEEP;		
  		if (ANIMATEINPIERCESWEEP)
		 {
		  specialUmbTang = sweptInPierce[specialOb][specialUmb];
		  tAnimate = specialUmbTang.tRefine[0]; 
		  nRefined = 1;
		  Pause = 1;
		 }
		break;
  case 'd':     DRAWDIRECTBITANG    = !DRAWDIRECTBITANG;    break;
  case 'D':     DRAWDIRECTVE        = !DRAWDIRECTVE;        break;
  case 'e':     DRAWSELFBITANG      = !DRAWSELFBITANG;      break;
  case 'f':	DRAWLOCALFRONTUMBRA = !DRAWLOCALFRONTUMBRA; break;
  case 'F': 	DRAWGLOBALFRONTUMBRA= !DRAWGLOBALFRONTUMBRA;break;
  case 'i':     DRAWINDIRECTBITANG  = !DRAWINDIRECTBITANG;  break;
  case 'I':     DRAWINDIRECTVE      = !DRAWINDIRECTVE;      break;
  case 'j':	DRAWITANG           = !DRAWITANG;           break;
  case 'l':	LABELFRONTUMBRA     = !LABELFRONTUMBRA;	    break;
  case 'L':     LABELBACKUMBRA      = !LABELBACKUMBRA;	    break;
  case 'm':	DRAWMAXBACKUMBRA    = !DRAWMAXBACKUMBRA;    break;
  case 'M':     DRAWMAXFRONTUMBRA   = !DRAWMAXFRONTUMBRA;   break;
  case 'o':	DRAWOTANG           = !DRAWOTANG;           break;
  case 'O':     ONLYFIRST           = !ONLYFIRST;           break;
  case 'p':	DRAWOPTANG          = !DRAWOPTANG;	    break;
  case 'P':	DRAWIPTANG          = !DRAWIPTANG;	    break;
  case 'r': 	UMBRAFILLED         = !UMBRAFILLED;	    break;
  case 'R':     LABELRESPONSIBLE    = !LABELRESPONSIBLE;    break;
  case 's':	SHADEFRONTUMBRA     = !SHADEFRONTUMBRA;	    break;
  case 't':     DRAWBACKPENUMBRA    = !DRAWBACKPENUMBRA;    break;
  case 'T':     DRAWFRONTPENUMBRA   = !DRAWFRONTPENUMBRA;   break;
  case 'v':     ANIMATEOUTPIERCESWEEP = !ANIMATEOUTPIERCESWEEP; 
                if (ANIMATEOUTPIERCESWEEP)
		  {
		    specialUmbTang = sweptOutPierce[specialOb][specialUmb];
		    tAnimate = specialUmbTang.tRefine[0];
		    nRefined = 1;
		    Pause = 1;
		  }
                                                            break;
  case 'x':	ANIMATEOUTERSWEEP   = !ANIMATEOUTERSWEEP;		
  		if (ANIMATEOUTERSWEEP)
		 {
		  specialUmbTang = sweptOuter[specialOb][specialUmb];
		  tAnimate = specialUmbTang.tRefine[0]; 
		  cout << "tAnimate = " << tAnimate << endl;
		  nRefined = 1;
		  Pause = 1;
		 }
		break;
  case 'z':	ANIMATEINNERSWEEP   = !ANIMATEINNERSWEEP;		
  		if (ANIMATEINNERSWEEP)
		 {
		  specialUmbTang = sweptInner[specialOb][specialUmb];
		  tAnimate = specialUmbTang.tRefine[0]; 
		  nRefined = 1;
		  Pause = 1;
		 }
		 					    break;
  default:                                                  break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 0: 	UMBRAFILLED         = !UMBRAFILLED;	    break;
  case 1:	DRAWBITANG          = !DRAWBITANG;	    break;
  case 2:	DRAWOTANG           = !DRAWOTANG;	    break;
  case 3: 	DRAWONEBITANG       = !DRAWONEBITANG;	    break;
  case 4:     	DRAWITANG           = !DRAWITANG;	    break;
  case 5: 	if (!DRAWGLOBALBACKUMBRA)		// turn global umbra on/off
   		 {
		  DRAWGLOBALBACKUMBRA = 1; DRAWLOCALBACKUMBRA = 0;
		 }
		else 
		 {
		  DRAWGLOBALBACKUMBRA = 0; DRAWLOCALBACKUMBRA = 0;
		 }
		 					    break;
  case 6:	ANIMATEINNERSWEEP   = !ANIMATEINNERSWEEP;		
  		if (ANIMATEINNERSWEEP)
		 {
		  specialUmbTang = sweptInner[specialOb][specialUmb];
		  tAnimate = specialUmbTang.tRefine[0]; 
		  nRefined = 1;
		  Pause = 1;
		 }
		 					    break;
  case 7:	ANIMATEOUTERSWEEP   = !ANIMATEOUTERSWEEP;		
  		if (ANIMATEOUTERSWEEP)
		 {
		  specialUmbTang = sweptOuter[specialOb][specialUmb];
		  tAnimate = specialUmbTang.tRefine[0]; 
		  nRefined = 1;
		  Pause = 1;
		 }
		break;
  case 8:	DRAWINNERUMBTANG    = !DRAWINNERUMBTANG;    break;
  case 9:	DRAWOUTERUMBTANG    = !DRAWOUTERUMBTANG;    break;
  case 10:	ONLYFIRST           = !ONLYFIRST;           break;
  case 11: 	if (!DRAWLOCALBACKUMBRA)		// turn local umbra on/off
   		 {
		  DRAWLOCALBACKUMBRA = 1; DRAWGLOBALBACKUMBRA = 0;
		 }
		else 
		 {
		  DRAWLOCALBACKUMBRA = 0; DRAWGLOBALBACKUMBRA = 0; 
		 }
		 					    break;
  case 12:	DRAWMAXBACKUMBRA    = !DRAWMAXBACKUMBRA;    break;
  case 13: 	LABEL               = !LABEL;	            break;
  case 14:     	DRAWOPTANG          = !DRAWOPTANG;	    break;
  case 15:	DRAWLOCALFRONTUMBRA = !DRAWLOCALFRONTUMBRA; break;
  case 16: 	DRAWGLOBALFRONTUMBRA= !DRAWGLOBALFRONTUMBRA;break;
  case 19:	SHADEFRONTUMBRA     = !SHADEFRONTUMBRA;	    break;
  case 20:      LABELRESPONSIBLE    = !LABELRESPONSIBLE;    break;
  case 21:     	DRAWIPTANG          = !DRAWIPTANG;	    break;
  case 22:      DRAWMAXFRONTUMBRA   = !DRAWMAXFRONTUMBRA;   break;
  case 23:      DRAWORIGIN          = !DRAWORIGIN;          break;
  case 24:      LETTERLABEL         = !LETTERLABEL;         break;
  case 25:      DRAWONELATE         = !DRAWONELATE;         break;
  case 26:      LABELLIGHT          = !LABELLIGHT;          break;
  case 28:      DRAWPIERCEUMBTANG   = !DRAWPIERCEUMBTANG;   break;
  case 29:      DRAWONEINTERM       = !DRAWONEINTERM;       break;
  case 30:      DRAWBACKPENUMBRA    = !DRAWBACKPENUMBRA;    break;
  case 31:      DRAWFRONTPENUMBRA   = !DRAWFRONTPENUMBRA;   break;
  case 32:	ANIMATEINPIERCESWEEP  = !ANIMATEINPIERCESWEEP;		
  		if (ANIMATEINPIERCESWEEP)
		 {
		  specialUmbTang = sweptInPierce[specialOb][specialUmb];
		  tAnimate = specialUmbTang.tRefine[0]; 
		  nRefined = 1;
		  Pause = 1;
		 }
		                                            break;
  case 33:	ANIMATEOUTPIERCESWEEP = !ANIMATEOUTPIERCESWEEP; 
                if (ANIMATEOUTPIERCESWEEP)
		  {
		    specialUmbTang = sweptOutPierce[specialOb][specialUmb];
		    tAnimate = specialUmbTang.tRefine[0];
		    nRefined = 1;
		    Pause = 1;
		  }
  case 40:      DRAWDIRECTBITANG    = !DRAWDIRECTBITANG;    break;
  case 41:      DRAWDIRECTVE        = !DRAWDIRECTVE;        break;
  case 42:      DRAWINDIRECTBITANG  = !DRAWINDIRECTBITANG;  break;
  case 43:      DRAWINDIRECTVE      = !DRAWINDIRECTVE;      break;
  case 44:      DRAWSELFBITANG      = !DRAWSELFBITANG;      break;
  case 100:     DRAWOBSTACLE        = !DRAWOBSTACLE;        break;
  default:   					            break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/
/*
void drawLocalUmbra (UmbralBitangArr &ut)
{
  glBegin(GL_LINE_STRIP);
//  glBegin(GL_POLYGON);
  glVertex2f (ut[0].umbraOb[0],      ut[0].umbraOb[1]);
  glVertex2f (ut[0].umbraExtreme[0], ut[0].umbraExtreme[1]);
  int coord1 = (fabs(ut[0].umbraExtreme[0]) == radiusRoom ? 0 : 1);
  int coord2 = (fabs(ut[1].umbraExtreme[0]) == radiusRoom ? 0 : 1);
  if (coord1 != coord2)		// round the corner
    if (coord1 == 0)
         glVertex2f (ut[0].umbraExtreme[coord1], ut[1].umbraExtreme[coord2]);
    else glVertex2f (ut[1].umbraExtreme[coord2], ut[0].umbraExtreme[coord1]);
  glVertex2f (ut[1].umbraExtreme[0], ut[1].umbraExtreme[1]);
  glVertex2f (ut[1].umbraOb[0],      ut[1].umbraOb[1]);
  glEnd();
}
*/

/******************************************************************************
          global = 1 iff pierce is organized globally, 
          one per obstacle rather than one per obstacle pair.
******************************************************************************/

void drawFront (Array<UmbralBitangArr> &pierce, Array<Polygon2fArr> &frontShadow, 
		int global, int coloroffset)
{
  int i,k;
  for (i=0; ONLYFIRST ? i<2 : i<nOb; i++)
   {
     glColor3fv (color[3*i+coloroffset]);
     int index = (global ? i : i*nOb);
     for (k=0; k<pierce[index].getn(); k++)
       if (UMBRAFILLED)
	 frontShadow[i][k].drawTriangulation();
       else
	{
	 pierce[index][k].drawObExtreme();
	 V2f midpt; midpt.midPt (pierce[index][k].umbraOb, pierce[index][k].umbraExtreme);
	 glRasterPos2f (midpt[0], midpt[1]); 	// label 
	 glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, '0');    // object blocked
	 if (LABELRESPONSIBLE)   // object responsible for this blockage
	  {
	   glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, '(');
	   glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, i+'0');
	   glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, ')');
	  }
	}
   }
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
//glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  
  glColor3fv (Red);			// bounding room
  room.draw(1);

  // draw the minimum feature size to scale in this room, at bottom right outside room
  glColor3fv (Red);
  glLineWidth (3.0);
  glBegin (GL_LINES);
  glVertex2f (radiusRoom-featureSize, -radiusRoom-.1);
  glVertex2f (radiusRoom,             -radiusRoom-.1);
  glEnd();
  glLineWidth (1.0);

  if (DRAWORIGIN)
    {
      glColor3fv (Black);
      glBegin(GL_POINTS);
      glVertex2f (0,0);
      glEnd();
    }
  if (DRAWBACKPENUMBRA)
   {
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
      {
	glColor3fv (color[3*i+2]);
	backPenumbra[i].drawTriangulation(); 
	glColor3fv (White);
	localBackUmbra[i].drawTriangulation();
	for (k=0; k<outerpierce[i].getn(); k++)
	  localFrontUmbra[i][k].drawTriangulation();
      }
   }
  if (DRAWMAXBACKUMBRA)			// maximal size of A's umbra is bounded
   {					// by late inner bitangents of A and L
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
    {
      if (UMBRAFILLED)
	{
	  glColor3fv (color[3*i+2]);
	  backPenumbra[i].drawTriangulation(); 
	  // white out the interior of the curve between points of tangency:
	  // for each pair of curve samples A and B, painting the triangle
	  // ABC where C is the curve's centroid
	  // (this will work when the centroid is in the kernel)
	}
      else
	{
	  glColor3fv (color[3*i+2]);
	  // backPenumbra[i].draw(3);
	  for (j=0; j<2; j++)             // draw and label late inner bitangent
	   {
	    inner[i][j].drawObExtreme();
	    V2f midpt;  midpt.midPt (inner[i][j].umbraOb, inner[i][j].umbraExtreme);
	    glRasterPos2f (midpt[0], midpt[1]); 	// label 
	    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10,'0'); // object partially blocked
	    if (LABELRESPONSIBLE) // object responsible for this blockage
	      {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '(');
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, i+'0');
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ')');
	      }
	   }       
	}
    }
   } 
  if (DRAWGLOBALBACKUMBRA)
   {
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
     {
      glColor3fv (color[3*i+1]);
      if (UMBRAFILLED)
	  globalBackUmbra[i].drawTriangulation();
      else
	{
	  globalBackUmbra[i].draw(1);
//        drawLocalUmbra (sweptInner[i]);
	  glLineWidth (3.0);    // white out the chord
	  glColor3fv (White);
	  glBegin(GL_LINES);
	  glVertex2f (sweptInner[i][0].umbraOb[0], sweptInner[i][0].umbraOb[1]); 
	  glVertex2f (sweptInner[i][1].umbraOb[0], sweptInner[i][1].umbraOb[1]);
	  glEnd();
	  glLineWidth (1.0);
	}
     }
   }
  if (DRAWLOCALBACKUMBRA)			// individual umbras, for now
   {
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
    {
      glColor3fv (color[3*i]);
      if (UMBRAFILLED)
	{
	  localBackUmbra[i].drawTriangulation();
	  // white out obstacle interior from outer[i][0].umbraOb to outer[i][1].umbraOb
	}
      else 
	{
	  // localBackUmbra[i].draw(3);
	  for (j=0; j<2; j++)             // draw and label late outer bitangent
	    {
	      outer[i][j].drawObExtreme();
	      V2f midpt;  midpt.midPt (outer[i][j].umbraOb, outer[i][j].umbraExtreme);
	      glRasterPos2f (midpt[0], midpt[1]); 	// label 
	   // glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '0');    // object blocked
      	      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '0');    // object blocked
	      if (LABELRESPONSIBLE) // object responsible for this blockage
		{
		  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '(');
		  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, i+'0');
		  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ')');
		}
	    }
	}
    }
//drawLocalUmbra (outer[i]);
   }
  if (DRAWFRONTPENUMBRA)
   {
     drawFront (innerpierce, frontPenumbra, 0, 2);
     glColor3fv (White);                             // but not the front umbra
     for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
       for (k=0; k<outerpierce[i].getn(); k++)
	 localFrontUmbra[i][k].drawTriangulation();
   }
  if (DRAWMAXFRONTUMBRA)	    // maximal size of A's front umbra is bounded
   {				    // by late inner piercing bitangents of A and L
     //     glColor3f (.7,.7,.7);
     drawFront (innerpierce, frontPenumbra, 0, 2);
   } 
  if (DRAWGLOBALFRONTUMBRA)
    {
      // glColor3f (.5,.5,.5);
      // drawFront (sweptInPierce, globalFrontUmbra, 1, 1);
      for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
	for (j=0; j<globalFrontUmbra[i].getn(); j++)
	  {
	    glColor3fv (color[3*i+1]);
	    if (UMBRAFILLED) globalFrontUmbra[i][j].drawTriangulation();
	    else             globalFrontUmbra[i][j].draw(1);
	  }
    }
  if (DRAWLOCALFRONTUMBRA)
   {
  // glColor3fv (Black);
     drawFront (outerpierce, localFrontUmbra, 1, 0);
   }
  glColor3fv (color[0]);		// light
  // glLineWidth(3.0);
  obstacle[0].draw();
  // glLineWidth(1.0);
  if (LABEL)
   {
     for (i=0; i<nOb; i++)
       {
	 glRasterPos2f (obCentroid[i][0], obCentroid[i][1]);
	 char iAlpha[3]; itoa (i, iAlpha);
	 glutBitmapCharacter(GLUT_BITMAP_9_BY_15, iAlpha[0]);
       }
   }
  if (LETTERLABEL)
    {
      for (i=1; i<nOb; i++)
	{
	  glRasterPos2f (obCentroid[i][0], obCentroid[i][1]);
	  glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'B' + i-1);
	}
    }
  if (LABELLIGHT)
    {
      glRasterPos2f (obCentroid[0][0], obCentroid[0][1]);
      glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'A');      
    }
  if (LABELFRONTUMBRA)
   {
    glColor3fv (Black);	// 'U' next to each front umbral bitangent
    for (i=1; i<nOb; i++)
      for (j=0; j<outerpierce[i].getn(); j++)
	outerpierce[i][j].labelFrontUmbra(obstacle);
   }
  if (LABELBACKUMBRA)
   {
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)	// 'U' in each local back umbra
     {
      V2f c;
      localBackUmbra[i].centroid(c);
      glRasterPos2f (c[0], c[1]);
      glutBitmapCharacter (GLUT_BITMAP_TIMES_ROMAN_10, 'u');
     }
// penumbra is harder, because inner boundary is jagged, so delay 
   }
  if (SHADEFRONTUMBRA)		// implicitly assuming a pseudoconvex region
   {
    glColor3fv (Black);
//    for each piercing tangent defining a region of the local front umbra
//      shade the region defined by its intermediate segment
      
   }
  if (DRAWOBSTACLE)
    {
      for (i=1; i<nOb; i++)		// obstacles (drawn at end, so that they
	{				// overwrite the global umbra whiteout hack)
	  glColor3fv (color[3*i%14]);
	  obstacle[i].draw(); 
	}
    }
  if (DRAWONEBITANG)
   {
    if (nextbitang==0) glColor3fv(Red); else glColor3fv (Black);
    bitang[nextcurve][nextbitang].draw ();
   }
  if (DRAWONELATE)
    {
      glColor3fv (Black);
      outer[1][0].drawObExtreme ();
    }
  else if (DRAWBITANG)
   {
    glColor3fv (Black);
    for (i=0; i<nOb; i++)
      for (j=i+1; j<nOb; j++)
        for (k=0; k<bitang[i*nOb+j].getn(); k++)
	  bitang[i*nOb+j][k].draw ();
   }
  if (DRAWSELFBITANG)
    {
      glColor3fv (Black);
      for (i=0; i<nOb; i++)
	for (j=0; j<selfbitang[i].getn(); j++)
	  selfbitang[i][j].draw();
    }
  if (DRAWONEINTERM)
    {
      glColor3fv (Black);
      if (outerpierce[1].getn() > 0)
	outerpierce[1][0].drawObExtreme();
    }
  if (DRAWDIRECTBITANG)                // bitangents associated with direct visual events
    {
      glColor3fv (Red);
      for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
	for (j=0; j<direct[i].getn(); j++)
	  direct[i][j].draw();
    }
  if (DRAWDIRECTVE)                    // direct visual events
    {
      glColor3fv (Purple);
      for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
	for (j=0; j<direct[i].getn(); j++)
	  direct[i][j].drawObExtreme();
    }
  if (DRAWINDIRECTBITANG)
    {
      glColor3fv (Blue);
      for (i=1; i<nOb; i++)
	for (j=i; ONLYFIRST ? j<2 : j<nOb; j++)
	  for (k=0; k<indirect[i*nOb+j].getn(); k++)
	    indirect[i*nOb+j][k].draw ();
    }
  if (DRAWINDIRECTVE)
    {
      glColor3fv (Green);
      for (i=1; i<nOb; i++)
	for (j=i; ONLYFIRST ? j<2 : j<nOb; j++)
	  for (k=0; k<indirect[i*nOb+j].getn(); k++)
	    indirect[i*nOb+j][k].drawObExtreme ();
    }
  if (DRAWOTANG)			// outer bitangents
   {
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
      {
	glColor3fv (color[3*i]);
	for (j=0; j<2; j++)
	  outer[i][j].draw ();
      }
   }
  if (DRAWITANG)			// inner bitangents
   {
    for (i=0; i<nOb; i++)
      for (j=i+1; ONLYFIRST ? j<2 : j<nOb; j++)
	{
	  glColor3fv (color[3*j+1]);
	  for (k=0; k<2; k++)
	    inner[i*nOb+j][k].draw ();
	}
   }
  if (DRAWOPTANG)			// outer piercing bitangents
   {
    glColor3fv (Blue);
    if (ONLYFIRST)         // just ones that pierce obstacle[1] from light
      {
	  for (k=0; k<outerpierce[1].getn(); k++)
	    {
	      outerpierce[1][k].drawObExtreme();
	      outerpierce[1][k].draw();
	    }
      }
    else
      {
	for (i=1; i<nOb; i++)
	  for (j=0; j<outerpierce[i].getn(); j++)
	    {
	      outerpierce[i][j].drawObExtreme();
	      outerpierce[i][j].draw();
	    }
      }
   }
  if (DRAWIPTANG)			// inner piercing bitangents
   {
    glColor3fv (Blue);
    if (ONLYFIRST)         // just ones that pierce obstacle[1]
      {
	for (i=0; i<nOb; i++)
	  for (k=0; k<innerpierce[nOb+i].getn(); k++)
	    {
	      innerpierce[nOb+i][k].drawObExtreme();
	      innerpierce[nOb+i][k].draw();
	    }
      }
    else
      {
	for (i=0; i<nOb; i++)
	  for (j=0; j<nOb; j++)
	    for (k=0; k<innerpierce[i*nOb+j].getn(); k++)
	      {
		innerpierce[i*nOb+j][k].drawObExtreme();
		innerpierce[i*nOb+j][k].draw();
	      }
      }
   }
  if (DRAWINNERUMBTANG)			// inner umbral bitangents
   {
    glColor3fv (Green);
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
      for (j=0; j<2; j++)
        sweptInner[i][j].draw ();
   }
  if (DRAWOUTERUMBTANG)			// outer umbral bitangents
   {
    glColor3fv (Blue);
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
      for (j=0; j<2; j++)
        if (!sweptOuter[i][j].abandon)
          sweptOuter[i][j].draw ();
   }
  if (DRAWPIERCEUMBTANG)		// piercing umbral bitangents
   {
    glColor3fv (Red);
    for (i=1; ONLYFIRST ? i<2 : i<nOb; i++)
      for (j=0; j<sweptInPierce[i].getn(); j++)
        sweptInPierce[i][j].draw ();
   }
  if (ANIMATEINNERSWEEP   // animate the inner sweep of the umbral bitangent
      || ANIMATEOUTERSWEEP
      || ANIMATEINPIERCESWEEP
      || ANIMATEOUTPIERCESWEEP)
   { 	
    glColor3fv (Black);
    if (Pause) glLineWidth(3.0);
    if (ANIMATEOUTERSWEEP || ANIMATEOUTPIERCESWEEP) 
         obstacle[0].drawTangent (tAnimate, lightHodo, 1);
    else obstacle[specialOb].drawTangent (tAnimate, specialHodo, 1);
    if (Pause) glLineWidth(1.0);
    if (!Pause && nRefined < specialUmbTang.nRefine)	// not finished
     {
       // refine the bitangent
       int found=0;
       float goal = specialUmbTang.tRefine[nRefined];
       float startpos = specialUmbTang.tRefine[nRefined-1];
       if (specialUmbTang.refineDirIsForward)
	 {
	   tAnimate+=.001;
	   if (goal > startpos && tAnimate > goal) found = 1;
	   if (goal < startpos && tAnimate < startpos && tAnimate > goal) found = 1;
	   if (tAnimate > obstacle[ANIMATEOUTERSWEEP 
				   ? 0 : specialOb].getLastKnot())
	       tAnimate = obstacle[ANIMATEOUTERSWEEP || ANIMATEOUTPIERCESWEEP 
				   ? 0 : specialOb].getKnot(0);
	 }
       else 
	 {
	   tAnimate-=.001; 
	   if (goal < startpos && tAnimate < goal) found = 1;
	   if (goal > startpos && tAnimate > startpos && tAnimate < goal) found = 1;
	   if (tAnimate < obstacle[ANIMATEOUTERSWEEP || ANIMATEOUTPIERCESWEEP 
				   ? 0 : specialOb].getKnot(0))
	       tAnimate = obstacle[ANIMATEOUTERSWEEP || ANIMATEOUTPIERCESWEEP 
				   ? 0 : specialOb].getLastKnot();
	 }
       if (found)
	 {
	   tAnimate = specialUmbTang.tRefine[nRefined];
	   nRefined++;
	   Pause = 1;
	 }
     }
   }
  /*  if (ANIMATEOUTERSWEEP)   // animate the outer sweep of the umbral bitangent
   {
    glColor3fv (Black);
    if (Pause) glLineWidth(3.0);
    obstacle[0].drawTangent (tAnimate, lightHodo, 1);
    if (Pause) glLineWidth(1.0);
    if (!Pause && nRefined < specialUmbTang.nRefine)	// not finished
     {
       // refine the bitangent
       int found = 0;
       float goal = specialUmbTang.tRefine[nRefined];
       float startpos = specialUmbTang.tRefine[nRefined-1];
       if (specialUmbTang.refineDirIsForward)
	 {
	   tAnimate+=.001;
	   if (goal > startpos && tAnimate > goal) found = 1;
	   if (goal < startpos && tAnimate < startpos && tAnimate > goal) found = 1;
	   if (tAnimate > obstacle[0].getLastKnot())
	     tAnimate = obstacle[0].getKnot(0);
	 }
       else 
	 {
	   tAnimate-=.001; 
	   if (goal < startpos && tAnimate < goal) found = 1;
	   if (goal > startpos && tAnimate > startpos && tAnimate < goal) found = 1;
	   if (tAnimate < obstacle[0].getKnot(0))
	     tAnimate = obstacle[0].getLastKnot();
	 }
       if (found)
	 {
	   tAnimate = specialUmbTang.tRefine[nRefined];
	   nRefined++;
	   Pause = 1;
	 }
     }
     } */
     
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
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
  scaleToUnitSquare (Pt);
  obstacle.allocate(Pt.getn());
  obCentroid.allocate (Pt.getn());
  for (i=0; i<Pt.getn(); i++)
   { 
    obstacle[i].fitClosed (Pt[i]);
    obstacle[i].prepareDisplay (nPtsPerSegment);
    obCentroid[i].clear();
    for (j=0; j<Pt[i].getn(); j++) obCentroid[i] += Pt[i][j];
    obCentroid[i] /= Pt[i].getn();
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
// cout << "obduala[" << i << "]:" << endl;  obduala[i].print();    
    obdualb[i].createB(obstacle[i], i);
// cout << "obdualb[" << i << "]:" << endl;  obdualb[i].print();    
   }
}

/******************************************************************************
	Intersect tangential curves to find bitangents.
	Ensure that bitangents are found from light to obstacle
	(i.e., light first so that index1 and param1 of CommonTangent
	refer to the point of bitangency with the light):
	this is needed in outer.
******************************************************************************/

void buildBitangent      (Array<TangentialCurve> &obduala,
		          Array<TangentialCurve> &obdualb,
		          float epsIntersect, float featureSize,
		          Array<UmbralBitangArr> &bitang)
{
  CommonTangentArr bitangA;	// bitangents from a-space 
  CommonTangentArr bitangB;	// bitangents from b-space 
  CommonTangentArr bitangAB;    // bitangents from both dual spaces
  bitang.allocate(nOb*nOb);
  for (int i=0; i<nOb; i++)
    for (int j=i+1; j<nOb; j++)		// bitangents between i and j
     {
       cout << "Computing bitangents between " << i << " and " << j << endl;
      obduala[i].intersect (obduala[j], bitangA, epsIntersect);
      obdualb[i].intersect (obdualb[j], bitangB, epsIntersect); 
      bitangAB.allocate (bitangA.getn() + bitangB.getn());
      int k;
      for (k=0; k<bitangA.getn(); k++)  bitangAB[k] = bitangA[k];
      for (k=0; k<bitangB.getn(); k++)  bitangAB[bitangA.getn() + k] = bitangB[k];
      IntArr foo; 	bitangAB.bubbleSort (foo);
      bitangAB.deleteApproxDuplicate(featureSize);
      bitang[i*nOb+j].allocate(bitangAB.getn());
      for (k=0; k<bitangAB.getn(); k++) bitang[i*nOb+j][k] = bitangAB[k];
cout << "Bitangents from obstacle " << i << " to obstacle " << j << ":" << endl;
 for (k=0; k<bitang[i*nOb+j].getn(); k++) cout << bitang[i*nOb+j][k] << endl;
     }
}

/******************************************************************************
	Intersect tangential curves to find self-bitangents.
******************************************************************************/

void buildSelfBitangent  (Array<TangentialCurve> &obduala,
		          Array<TangentialCurve> &obdualb,
		          float epsIntersect, float featureSize,
		          Array<UmbralBitangArr> &selfbitang)
{
  cout << "Building self-bitangents" << endl;
  int i,j;
  CommonTangentArr bitangA;	// selfbitangents from a-space 
  CommonTangentArr bitangB;	// selfbitangents from b-space 
  selfbitang.allocate(nOb);     // one set of self-bitangents per curve

     // self-intersect tangential curves to find bitangents
  for (i=0; i<nOb; i++)
   {
    obduala[i].selfIntersect (bitangA, epsIntersect);
    obdualb[i].selfIntersect (bitangB, epsIntersect);
    selfbitang[i].allocate(bitangA.getn() + bitangB.getn());
    for (j=0; j<bitangA.getn(); j++) selfbitang[i][j] = bitangA[j];
    for (j=0; j<bitangB.getn(); j++) selfbitang[i][bitangA.getn() + j] = bitangB[j];
    IntArr foo;  selfbitang[i].bubbleSort (foo);
    selfbitang[i].deleteApproxDuplicate (featureSize);
   }
}

/******************************************************************************
	Filter bitangents down to bitangents that define direct visual events
        (of distinguished object).
******************************************************************************/

void buildDirectBitang (Array<UmbralBitangArr> &bitang,
			Array<BezierCurve2f>   &obstacle,
			float epsIntersect, float closeEps, float featureSize, 
			float sameSideEps,
			Array<UmbralBitangArr> &direct)
{  
  int i,j;
  direct.allocate(nOb); 
  for (i=1; i<nOb; i++)				// each obstacle
   {
    direct[i].allocate(MAXDIRECT);
    int nd=0;  V2f umbraOb, umbraExtreme; int ob; float tOb;
    for (j=0; j<bitang[i].getn(); j++)	// each bitangent from light to obstacle
     {
      cout << endl << "Bitangent " << j << " of " << bitang[i].getn() 
	   << " from obstacle " << i << endl;     
      if (bitang[i][j].direct(obstacle, epsIntersect, closeEps, featureSize, 
			      sameSideEps, radiusRoom, 
			      umbraOb, umbraExtreme, ob, tOb))
	{
	  assert (nd < MAXDIRECT);
	  direct[i][nd++].create (bitang[i][j], umbraOb, umbraExtreme, ob, tOb);
	}
     }
    direct[i].shrink (nd);
   }
}

/******************************************************************************
	Filter bitangents down to bitangents that define indirect visual events
        (of distinguished object).
******************************************************************************/

void buildIndirectBitang (Array<UmbralBitangArr> &bitang,
			  Array<UmbralBitangArr> &selfbitang,
			  Array<BezierCurve2f>   &obstacle,
			  float epsIntersect, float closeEps, float featureSize, 
			  float sameSideEps,
			  Array<UmbralBitangArr> &indirect)
{  
  int i,j,k;
  indirect.allocate(nOb*nOb); 
  for (i=1; i<nOb; i++)
    for (j=i; j<nOb; j++)
     {
      indirect[i*nOb+j].allocate(MAXINDIRECT);
      int ni=0;  V2f umbraOb, umbraExtreme; int ob; float tOb;
      if (i==j)
	for (k=0; k<selfbitang[i].getn(); k++)
	 {
	  cout << endl << "Considering self-bitangent " << k << " of " << i << endl
	       << selfbitang[i][k];
	  if (selfbitang[i][k].indirect (obstacle, epsIntersect, closeEps, featureSize, 
					 sameSideEps, radiusRoom, 
					 umbraOb, umbraExtreme, ob, tOb))
	   {
	    assert (ni < MAXINDIRECT);
	    indirect[i*nOb+j][ni++].create (selfbitang[i][k], umbraOb, umbraExtreme, 
					    ob, tOb);
	   }
	 }
      else
       {
	indirect[j*nOb+i].allocate(MAXINDIRECT);
	for (k=0; k<bitang[i*nOb+j].getn(); k++)
	 {
	  cout << endl << "Considering bitangent " << k << " between " << i 
	       << " and " << j << endl << bitang[i*nOb+j][k];
	  if (bitang[i*nOb+j][k].indirect (obstacle, epsIntersect, closeEps, featureSize, 
					   sameSideEps, radiusRoom, 
					   umbraOb, umbraExtreme, ob, tOb))
	   {
	    assert (ni < MAXINDIRECT);
	    indirect[i*nOb+j][ni].create          (bitang[i*nOb+j][k], umbraOb, 
						   umbraExtreme, ob, tOb);
	    indirect[j*nOb+i][ni++].createReverse (bitang[i*nOb+j][k], umbraOb, 
						   umbraExtreme, ob, tOb);
	   }
	 } 
       }
      indirect[i*nOb+j].shrink(ni);
      if (i!=j) indirect[j*nOb+i].shrink(ni);
     }
}

/******************************************************************************
	Filter bitangents down to outer bitangents.
	Only compute between each obstacle and light, not between obstacles.
******************************************************************************/

void buildOuterBitang  (Array<UmbralBitangArr> &bitang,
			Array<BezierCurve2f>   &obstacle,
			float epsIntersect, float closeEps, float featureSize, 
			float sameSideEps,
			Array<UmbralBitangArr> &outer)
{  
  int i,j;
  outer.allocate(nOb); 
  for (i=1; i<nOb; i++)				// each obstacle
   {
    outer[i].allocate(MAXDIRECT);
    int npo=0;  V2f umbraOb, umbraExtreme; int ob; float tOb;
    for (j=0; j<bitang[i].getn(); j++)	// each bitangent from light to obstacle
     {
      cout << endl << "Bitangent " << j << " of " << bitang[i].getn() 
	   << " from obstacle " << i << endl;     
      if (bitang[i][j].outer(obstacle, epsIntersect, closeEps, featureSize, 
			     sameSideEps, radiusRoom, 
			     umbraOb, umbraExtreme, ob, tOb))
	{
	  assert (npo < MAXDIRECT);
	  outer[i][npo++].create (bitang[i][j], umbraOb, umbraExtreme, ob, tOb);
	}
	  
     }
    //    assert (npo==2);
    // trim L-outer pair of obstacle[i] if they intersect
    V2f hit;
    if (outer[i][0].intersect (outer[i][1], hit, .001))
     {
      outer[i][0].trim (hit);  // trim umbral bitangents at intersection
      outer[i][1].trim (hit);
     }
   }  
}

/******************************************************************************
	Filter bitangents down to inner bitangents.
	Compute inner bitangents between all pairs of obstacles,
	not just between obstacles and light.

  ->bitang:       bitangents of the scene
  ->obstacle:     curves of the scene
  ->epsIntersect: accuracy at which intersections are made
  ->closeEps:     
  ->sameSideEps:  parameter jump to get away from tangent and test side the curve lies on
  <-inner:        inner bitangents of the scene
******************************************************************************/

void buildInnerBitang  (Array<UmbralBitangArr> &bitang,
			Array<BezierCurve2f>   &obstacle,
			float epsIntersect, float closeEps, float featureSize,
			float sameSideEps,
			Array<UmbralBitangArr> &inner)
{  
  int i,j,k;
  inner.allocate(nOb*nOb); 
  for (i=0; i<nOb; i++)
    for (j=i+1; j<nOb; j++)
     {
      inner[i*nOb+j].allocate(2);
      inner[j*nOb+i].allocate(2);
      int npi=0;  V2f umbraOb, umbraExtreme; int ob; float tOb;
      for (k=0; k<bitang[i*nOb+j].getn(); k++)
       {
	 cout << endl << "Considering bitangent " << k << " between " << i 
	      << " and " << j << endl;
	 cout << bitang[i*nOb+j][k];
        if (bitang[i*nOb+j][k].inner (obstacle, epsIntersect, closeEps, featureSize, 
				      sameSideEps, radiusRoom, 
				      umbraOb, umbraExtreme, ob, tOb))
	 {
	  assert (npi<2);
	  inner[i*nOb+j][npi].create (bitang[i*nOb+j][k], umbraOb, umbraExtreme, ob, tOb);
	  inner[j*nOb+i][npi++].createReverse (bitang[i*nOb+j][k], umbraOb, umbraExtreme, 
					       ob, tOb);
//	  inner[i*nOb+j][npi].createCT   (bitang[i*nOb+j][k]);
//	  inner[j*nOb+i][npi++].createCTReverse (bitang[i*nOb+j][k]);
	 }
       }
//      assert (npi==2);	// violated by maxumbra.pts so temporarily don't check
     }
}

/******************************************************************************
	Filter bitangents down to inner piercing bitangents,
        between all pairs of obstacles.

        REFINEMENT: It would be better to avoid testing this condition on
        all bitangents that are already known to be inner or outer.
******************************************************************************/

void buildInnerPierce (Array<UmbralBitangArr> &bitang,
		       Array<BezierCurve2f>   &obstacle,
		       float epsIntersect, float closeEps, float sameSideEps, 
		       Array<UmbralBitangArr> &pierce)
{ 
cout << "Computing inner piercing bitangents" << endl;
  int i,j,k;
  pierce.allocate(nOb*nOb); 
  for (i=0; i<nOb; i++)			// note: no longer symmetric
    for (j=i+1; j<nOb; j++)
     {
      if (i==0)
        pierce[j*nOb+i].allocate(10);
      else
       {
	 pierce[i*nOb+j].allocate(10);
	 pierce[j*nOb+i].allocate(10);
       }
      int npp1=0,npp2=0;  V2f umbraOb, umbraExtreme;  int ob; float tOb, tExtreme;
      for (k=0; k<bitang[i*nOb+j].getn(); k++)
       {
	 cout << endl << "Considering bitangent " << k << " between " << i 
	      << " and " << j << endl;
      // bitang[i*nOb+j][k].print();
	 if (i!=0 && 
	     bitang[i*nOb+j][k].pierce (obstacle, i, j, epsIntersect, closeEps, sameSideEps, 
					radiusRoom, umbraOb, umbraExtreme, ob, tOb, 
					tExtreme, 1))
	   pierce[i*nOb+j][npp1++].create (bitang[i*nOb+j][k], umbraOb, umbraExtreme, 
					   ob, tOb, tExtreme);
	 if (bitang[i*nOb+j][k].pierce (obstacle, j, i, epsIntersect, closeEps, sameSideEps, 
					radiusRoom, umbraOb, umbraExtreme, ob, tOb, 
					tExtreme, 1))
	   pierce[j*nOb+i][npp2++].create (bitang[i*nOb+j][k], umbraOb, umbraExtreme, 
					   ob, tOb, tExtreme);
       }
      cout << "Now shrinking" << endl;
      pierce[i*nOb+j].shrink(npp1);
      pierce[j*nOb+i].shrink(npp2);
     }
}

/******************************************************************************
	Filter bitangents down to outer piercing bitangents between light 
        and any other obstacle.

        REFINEMENT: It would be better to avoid testing this condition on
        all bitangents that are already known to be inner or outer.
******************************************************************************/

void buildOuterPierce (Array<UmbralBitangArr> &bitang,
		       Array<BezierCurve2f>   &obstacle,
		       float epsIntersect, float closeEps, float sameSideEps, 
		       Array<UmbralBitangArr> &pierce)
{ 
cout << "Computing outer piercing bitangents" << endl;
  int i,j;
  pierce.allocate(nOb); 
  for (i=1; i<nOb; i++)
   {
    pierce[i].allocate(10);
    int npp1=0;  V2f umbraOb, umbraExtreme;  int ob; float tOb, tExtreme;
    for (j=0; j<bitang[i].getn(); j++)
     {
      cout << endl << "Considering bitangent " << j 
	   << " from light to obstacle " << i << endl;
      if (bitang[i][j].pierce (obstacle, i, 0, epsIntersect, closeEps, sameSideEps, radiusRoom, 
			       umbraOb, umbraExtreme, ob, tOb, tExtreme, 0))
       {
	assert (npp1 < 10);
	pierce[i][npp1++].create (bitang[i][j], umbraOb, umbraExtreme, ob, tOb, tExtreme);
       }
     }
    cout << "Now shrinking" << endl;
    pierce[i].shrink(npp1);
   }
}

/******************************************************************************
	Inner sweep from outer bitangents.
	Umbral bitangent is refined based on intersections with obstacles.

	[Note: we can't move this refinement into a member function of
	UmbralBitang, since it requires inner, which is not declarable
	as the prototype (as an array of an array)!?]

    ->sameSideEps: parameter jump to get away from tangent 
                   and test side the curve lies on
******************************************************************************/

void innerSweep (Array<UmbralBitangArr> &outer,
		 Array<UmbralBitangArr> &inner,
		 Array<BezierCurve2f>   &obstacle,
		 float                   sameSideEps,
		 Array<UmbralBitangArr> &sweptInner)
{
cout << endl << "Entering innerSweep" << endl;
  sweptInner.allocate(nOb);
  for (int i=1; i<nOb; i++)			// each non-light obstacle
   {
    sweptInner[i].allocate(2);
    for (int j=0; j<2; j++) 			// each umbral bitangent
     {
cout << "Refining umbral bitangent " << j << " from obstacle " << i << endl;
      sweptInner[i][j] = outer[i][j];
      sweptInner[i][j].nRefine = 1;	// 1st position of umbral bitangent
      sweptInner[i][j].tRefine[0] = sweptInner[i][j].param2;
cout << "Original tRefine = " << sweptInner[i][j].param2 << endl;
      int lightDirForward = sweptInner[i][j].lightDirectionIsForward (obstacle, sameSideEps);
      sweptInner[i][j].refineDirIsForward = lightDirForward;
cout << "lightDirForward: " << lightDirForward << endl;      
      int done=0;
      while (!done)	// may skip across several obstacles
       {
	done = 1;	// assume that no intersections will be found
        BezierCurve2f earlyUmbTang;	// early part of umbral bitangent
	int success = sweptInner[i][j].createSegBetweenLandA (obstacle, earlyUmbTang);
	cout << "earlyUmbTang = " << earlyUmbTang << endl;
     // if success = 0, we have swept past the light: don't worry about further refinement
        for (int k=1; success && done && k<nOb; k++)	// every other obstacle
	  if (k != sweptInner[i][j].index1 && 
	      k != sweptInner[i][j].index2)
           {
cout << "Are there intersections with obstacle " << k << "?" << endl;	   
            int nHit; V2fArr foopt;  FloatArr tfoo,btfoo;
            earlyUmbTang.intersectInterior (obstacle[k], nHit, foopt, 
					    tfoo, btfoo, .0000001);
cout << nHit << " intersections" << endl;	   
	    if (nHit>0)
	     {
cout << "Need to replace" << endl;
	      done = 0;
	      UmbralBitangArr candidate(4);
	      candidate[0] = inner[i*nOb+k][0];
	      candidate[1] = inner[i*nOb+k][1];
	      candidate[2] = inner[i*nOb][0];
	      candidate[3] = inner[i*nOb][1];
cout << "Replacement candidates: " << endl << candidate[0] << candidate[1];
	      sweptInner[i][j].replaceByClosestInSweepDirection (candidate, obstacle, 
						   i, lightDirForward, radiusRoom, 1, 1);
	   // sweptInner[i][j].replaceByInner (i, k, lightDirForward, inner[i*nOb+k], 
	   //                                  obstacle, radiusRoom);
	     }
           }
       }
      cout << endl << "INNER UMBRAL BITANGENT " << j << " OF OBSTACLE " << i 
	   << endl << sweptInner[i][j];
     }
   }
cout << "Exiting innerSweep" << endl;
}

/******************************************************************************
	Outer sweep, refining 'outer' to 'sweptOuter'.

  -> outer:      outer bitangents of scene (for each obstacle to light)
  -> sweptInner: bitangents generated by inner sweeps
  -> obstacle:   curves in the scene
  -> sameSideEps: parameter jump to get away from tangent 
                  and test side the curve lies on
  <- sweptOuter: bitangents generated by outer sweeps, refining outer bitangents
******************************************************************************/

void outerSweep (Array<UmbralBitangArr> &outer,
		 Array<UmbralBitangArr> &sweptInner,
		 Array<BezierCurve2f>   &obstacle,
		 float                   sameSideEps,
		 Array<UmbralBitangArr> &sweptOuter)
{
                       cout << endl << "Entering outerSweep" << endl;
  float epsIntersect = .0000001;
  sweptOuter.allocate(nOb);
  for (int i=1; i<nOb; i++)			// each non-light obstacle
   {
                       cout << "Refining umbral bitangents from obstacle " << i << endl;
    sweptOuter[i].allocate(2);
    for (int j=0; j<2; j++) 			// each umbral bitangent
     {
                       cout << "Umbral bitangent " << outer[i][j] << endl;      
      assert (outer[i][j].index1 == 0);
      sweptOuter[i][j] = outer[i][j];
      sweptOuter[i][j].abandon = 1;	
      sweptOuter[i][j].nRefine = 1;	// 1st position of umbral bitangent
      sweptOuter[i][j].tRefine[0] = outer[i][j].param1;
                       cout << "Original tRefine = " << sweptOuter[i][j].param1 << endl;
      int sweepDirForward = !outer[i][j].ADirectionIsForward (obstacle, sameSideEps); 
      // away from A
                       cout << "sweepDirForward on light = " << sweepDirForward << endl;
      sweptOuter[i][j].refineDirIsForward = sweepDirForward;
      UmbralBitang G = sweptInner[i][j];	// already computed
      int done=0;  int first = 1;
      while (!done)	// may skip across several obstacles
       {
	done = 1;
        BezierCurve2f earlyUmbTang; // segment of umbral bitang between light and G
	int success = sweptOuter[i][j].createSegBetweenLandG (obstacle, first, G, 
							      earlyUmbTang);
	// success=0 iff we have swept past the umbra bitangent G that we are looking for
        for (int k=1; success && done && k<nOb; k++)	// every other obstacle
	  if (k != sweptOuter[i][j].index2)	// index1 is the light
           {
	           cout << "Are there intersections with obstacle " << k << "?" << endl;
            int nHit; V2fArr foopt;  FloatArr tfoo,btfoo;
            earlyUmbTang.intersectInterior (obstacle[k], nHit, foopt, tfoo, btfoo, 
					    epsIntersect);
	    if (nHit>0)
	     {
	           cout << "Yes, there are intersections so we need to refine." << endl;
	      done = 0;
	      sweptOuter[i][j].abandon = 0;	// sweep is nontrivial
	      UmbralBitangArr candidate(2);
	      candidate[0] = outer[k][0];
	      candidate[1] = outer[k][1]; 
       	           cout << "abandon on entering replaceByClosest: " 
			<< sweptOuter[i][j].abandon << endl;
	      sweptOuter[i][j].replaceByClosestInSweepDirection(candidate, obstacle, 0, 
						     sweepDirForward, radiusRoom, 1, 0); 
	   // WHY do we set umbraExtreme (resetUmbraExtreme flag is set to 1), 
	   // since this bitangent is only used for clipping?
	           cout << "abandon on exiting: " << sweptOuter[i][j].abandon << endl;
	   // sweptOuter[i][j].replaceByOuter(k,ADirForward,outer[k],obstacle,radiusRoom);
	     }
           }
       }
                   cout << endl << "OUTER UMBRAL BITANGENT " << j << " OF OBSTACLE " << i 
			<< endl << sweptOuter[i][j];
		   cout << "tRefine = " << sweptOuter[i][j].tRefine << endl;
     }
   }
                   cout << "Exiting outerSweep" << endl;
}

/******************************************************************************
    Inner piercing sweep phase of algorithm.

    Method: Starting at an outer piercing bitangent of A and L, 
            sweep on A towards L until the bitangent is free.
            The refined bitangent will be an inner or inner piercing bitangent.

  -> outerpierce:   outer piercing bitangents
  -> innerpierce:   inner piercing bitangents
  -> inner:         inner          bitangents
  -> sweptInner:    bitangents generated from inner sweep;
                    necessary if front umbra merges with back umbra; 
                    in which case, a sweptInner bitangent is replaced
  -> obstacle:      curves in scene
  <- sweptInPierce: bitangents generated by inner piercing sweeps;
                    each bounds a component of the global front umbra

******************************************************************************/

void innerPiercingSweep (Array<UmbralBitangArr> &outerpierce,
			 Array<UmbralBitangArr> &innerpierce,
			 Array<UmbralBitangArr> &inner,
			 Array<UmbralBitangArr> &sweptInner,
			 Array<BezierCurve2f>   &obstacle,
			 float                   sameSideEps,
			 Array<UmbralBitangArr> &sweptInPierce)
{
                           cout << "Entering innerPiercingSweep" << endl;
  float epsIntersect = .0000001; // epsilon used in intersection
  sweptInPierce.allocate(nOb);
  for (int i=1; i<nOb; i++) // every nonlight obstacle
   {
                   cout << "Refining front umbral bitangents from obstacle " << i << endl;
	           cout << "Obstacle " << i << " has " << outerpierce[i].getn() 
			<< " piercing bitangents." << endl;
    sweptInPierce[i].allocate (outerpierce[i].getn()); // # outer piercing bitangs of A/L
    for (int j=0; j<outerpierce[i].getn(); j++) // each outer piercing bitang T of A and L
     {
      sweptInPierce[i][j] = outerpierce[i][j];
      sweptInPierce[i][j].nRefine = 1;  // 1st position of umbral bitangent
                           assert (sweptInPierce[i][j].index1 == 0);
      sweptInPierce[i][j].tRefine[0] = sweptInPierce[i][j].param2;
                   cout << "Original tRefine = " << sweptInPierce[i][j].param2 << endl;
      int lightDirForward = 
	sweptInPierce[i][j].lightDirectionIsForward (obstacle, sameSideEps);
                   cout << "lightDirForward: " << lightDirForward << endl;      
      sweptInPierce[i][j].refineDirIsForward = lightDirForward;
      int done=0;
      while (!done)	// may skip across several obstacles
       {
	done = 1;	// assume that no intersections will be found
        BezierCurve2f earlyUmbTang;	// part of umbral bitangent between A and L
	int success = sweptInPierce[i][j].createSegBetweenLandA (obstacle, earlyUmbTang);
        // if success=0, we have swept past the lite: don't worry about further refinement
        // THIS CASE OF SUCCESS=0 MAY NEED FURTHER THINKING: check with an example
        for (int k=1; success && done && k<nOb; k++)	// every other obstacle
	  if (k != sweptInPierce[i][j].index1 && 
	      k != sweptInPierce[i][j].index2)
         {
	           cout << "Does sweeping tangent intersect obstacle " << k<<"?"<<endl;
	  int nHit; V2fArr foopt;  FloatArr tfoo,btfoo;
	  earlyUmbTang.intersectInterior (obstacle[k], nHit, foopt, 
					  tfoo, btfoo, epsIntersect);
	  if (nHit>0)       // sweeping tangent does intersect obstacle k
	   {
	           cout << "Yes." << endl;
            done = 0;       // sweep is not done
	    UmbralBitangArr candidate(innerpierce[i*nOb+k].getn() + // build candidate set
				      inner[i*nOb+k].getn() +       // for next position
				      innerpierce[i*nOb].getn() +   // of sweeping tangent
				      inner[i*nOb].getn());
	    int nCandidate=0, m;
	    for (m=0; m<innerpierce[i*nOb+k].getn(); m++) // 1: inner pierc bitangs of A/B
	      candidate[nCandidate++] = innerpierce[i*nOb+k][m];
	    for (m=0; m<inner[i*nOb+k].getn(); m++)       // 2: inner bitangs of A/B
	      candidate[nCandidate++] = inner[i*nOb+k][m];
	    for (m=0; m<innerpierce[i*nOb].getn(); m++)   // 3: inner pierc bitangs of A/L
	      candidate[nCandidate++] = innerpierce[i*nOb][m];
	    for (m=0; m<inner[i*nOb].getn(); m++)         // 4: inner bitangs of A/L 
	      candidate[nCandidate++] = inner[i*nOb][m];
	    int chosenCandidate =
	      sweptInPierce[i][j].replaceByClosestInSweepDirection (candidate, obstacle, 
						    i, lightDirForward, radiusRoom, 0, 1);

	    /*	    START HERE
	    if (category 2)
	    shutdown this component of global front umbra */

/*
	    if (chosenCandidate >= innerpierce[i*nOb+k].getn() &&     // category 2
		chosenCandidate <  innerpierce[i*nOb+k].getn() + inner[i*nOb+k].getn())
	     {
	       cout << "Merges with back umbra" << endl;
	      // NOT DEBUGGED YET
	      // front umbra merges with back umbra;
	      // present bitangent should replace a boundary of the global back umbra
	      // (sweptInner[i][0] or sweptInner[i][1])
	      sweptInPierce[i][j].ignore = 1; // this is no longer a front umbra component
	      candidate.allocate(2); candidate[0]=outer[i][0]; candidate[1]=outer[i][1];
	      // sweep on light, away from A, starting at outer piercing bitangent
	      int ADirForward = 
		outerpierce[i][j].ADirectionIsForward (obstacle, sameSideEps);
	      int closer = 
		outerpierce[i][j].findClosestInSweepDirection (candidate, obstacle, 
							       0, ADirForward);
	      sweptInner[i][closer] = sweptInPierce[i][j];
	     }
*/
	    if (chosenCandidate >= 
		innerpierce[i*nOb+k].getn() + inner[i*nOb+k].getn())  // category 3 or 4
	     {
	      done = 1; // we're done, since we have swept past the light
		/*
	      add this bitangent as an umbral bitangent;
//	      if (chosenCandidate >= innerpierce[i*nOb+k].getn() + 
//		                     inner[i*nOb+k].getn() +
//		                     innerpierce[i*nOb].getn())
//	       {
//		// from 4th category (inner bitang of A/L)
//		  sweptInPierce[i][j].ignore = 1; NO, don't ignore
//		 // IT SHOULD BECOME A NEW UMBRAL BITANGENT
//	       }
*/
	     }
	   }
	 }
       }
      cout << endl << "FRONT UMBRAL BITANGENT " << j << " OF OBSTACLE " << i 
	   << endl << sweptInPierce[i][j];
     }
   }
cout << "Exiting innerPiercingSweep" << endl;
}

/******************************************************************************
	Outer piercing sweep, refining 'outerpierce' to 'sweptOutPierce'.

-> outerpierce:    outer piercing bitangents of scene (for each obstacle to light)
-> outer:          outer bitangents of scene
-> sweptInPierce:  bitangents generated by inner piercing sweeps
-> obstacle:       curves in the scene
-> radiusRoom
-> epsIntersect:            intersection accuracy
-> closeEps:       'nhood to ignore around tangent point'
<- sweptOutPierce: bitangents generated by outer piercing sweeps, 
                   refining outer piercing bitangents
******************************************************************************/

void outerPiercingSweep (Array<UmbralBitangArr> &outerpierce,
			 Array<UmbralBitangArr> &outer,
			 Array<UmbralBitangArr> &sweptInPierce,
			 Array<BezierCurve2f>   &obstacle,
			 float radiusRoom, float epsIntersect, 
			 float closeEps, float sameSideEps,
			 Array<UmbralBitangArr> &sweptOutPierce)
{
                       cout << endl << "Entering outerPiercingSweep" << endl;
  sweptOutPierce.allocate(nOb);
  for (int i=1; i<nOb; i++)			// each non-light obstacle
   {
                       cout << "Refining umbral bitangents from obstacle " << i << endl;
    sweptOutPierce[i].allocate(outerpierce[i].getn());
    for (int j=0; j<outerpierce[i].getn(); j++) // each component of front umbra
     if (sweptInPierce[i][j].ignore == 1)
       sweptOutPierce[i][j].abandon = 1;
     else
     {
                     cout << "Umbral bitangent " << outerpierce[i][j] << endl;      
      sweptOutPierce[i][j] = outerpierce[i][j];
      sweptOutPierce[i][j].abandon = 1;
      sweptOutPierce[i][j].nRefine = 1;	// 1st position of umbral bitangent
      sweptOutPierce[i][j].tRefine[0] = sweptOutPierce[i][j].param1;
                     cout << "Original tRefine = " << sweptOutPierce[i][j].param1 << endl;
      int sweepDirForward = 
	!sweptOutPierce[i][j].ADirectionIsForward (obstacle, sameSideEps); // away from A
                     cout << "sweepDirForward on light = " << sweepDirForward << endl;
      sweptOutPierce[i][j].refineDirIsForward = sweepDirForward;
      UmbralBitang G = sweptInPierce[i][j];
      int done=0;  int first = 1;
      while (!done)	// may skip across several obstacles
       {
	done = 1;
        BezierCurve2f earlyUmbTang; // segment of umbral bitang between light and G
	int success=sweptOutPierce[i][j].createSegBetweenLandG (obstacle, first, G,
								earlyUmbTang);
	// success = 0 iff we have swept past the umbra bitang G that we are looking for
        for (int k=1; success && done && k<nOb; k++)	// every other obstacle
	  if (k != sweptOutPierce[i][j].index2)	// index1 is the light
           {
	             cout << "Are there intersections with obstacle " << k << "?" << endl;
            int nHit; V2fArr foopt;  FloatArr tfoo,btfoo;
            earlyUmbTang.intersectInterior (obstacle[k], nHit, foopt, tfoo, btfoo, 
					    .0000001);
	    // SHOULD PASS THIS EPSILON VALUE AS A CONTROLLABLE PARAMETER
	    if (nHit>0)
	     {
	             cout << "Yes, there are intersections so we need to refine." << endl;
	      done = 0;
	      sweptOutPierce[i][j].abandon = 0;	// sweep is nontrivial
	      UmbralBitangArr candidate(4);
	      candidate[0] = outer[k][0];  // outer bitangents of B and L
	      candidate[1] = outer[k][1];
	      candidate[2] = outer[i][0];  // outer bitangents of A and L 
	      candidate[3] = outer[i][1];  // (just to easily check for sweeping past G)
	      cout << "(i,k) = (" << i << "," << k << ")" << endl;
	      cout << "candidate = " << candidate << endl;
       	             cout << "abandon on entering replaceByClosest: " 
			  << sweptOutPierce[i][j].abandon << endl;
	      int chosenCandidate = 
		sweptOutPierce[i][j].replaceByClosestInSweepDirection(candidate, obstacle,
						    0, sweepDirForward, radiusRoom, 0, 0);
	      cout << "chosenCandidate = " << chosenCandidate << endl;
	      cout << "New bitangent in outer piercing sweep: " << sweptOutPierce[i][j];
	      // Note: we don't reset umbraExtreme using this function, since we want to 
	      // set it to first curve intersection, not room intersection
	             cout << "abandon on exiting: " << sweptOutPierce[i][j].abandon<<endl;
	      if (chosenCandidate == 2 || chosenCandidate == 3) // outer bitang of A/L
	       {
		 // I DON'T THINK ABANDONING IS RIGHT: OUTER BITANG OF A AND L MAY STILL CUT;
		 // YES IT IS A BOUNDARY, BUT STILL POTENTIALLY A PRODUCTIVE BOUNDARY
		 // sweptOutPierce[i][j].abandon = 1; // we have swept past A: ignore it
		done = 1;
		cout << "Have swept past A" << endl;
	       }
	      else
	       { 
		// have we swept past?
		BezierCurve2f bitangSeg; V2f A,B; float tOb1, tOb2;
		sweptOutPierce[i][j].createBitangInsideSquare (radiusRoom, bitangSeg, 
					       A, B, tOb1, tOb2);  // to get A and B
		Line2f L1(sweptInPierce[i][j].umbraOb, 
			  sweptInPierce[i][j].umbraExtreme); // intermediate segment
		Line2f L2(sweptOutPierce[i][j].pt[0],        // pt of tangency with light
			  A.dist(sweptOutPierce[i][j].pt[0]) < 
			  A.dist(sweptOutPierce[i][j].pt[1]) ? B : A); // pt of room 
		                                          // on opposite side from light)
		V2f hit; float tHit; 
		if (!L1.intersect (L2, hit, tHit, 1)) // no intersection
		 {
		  sweptOutPierce[i][j].abandon = 1; // have swept past inner piercing 
		                                    // sweep tang: ignore it
		  done = 1;
		  cout << "Rejecting: have swept past inner piercing sweep" << endl;
		  cout << sweptOutPierce[i][j].abandon << endl;
		 }
		else
		 // set umbraExtreme
		 obstacle[i].shootRay (sweptOutPierce[i][j].pt[1], // pt of bitang on A or B  
				       A.dist(sweptOutPierce[i][j].pt[0]) < 
				       A.dist(sweptOutPierce[i][j].pt[1]) ? B : A, // pt of room on opposite side from light
				       sweptOutPierce[i][j].umbraExtreme,  
				       sweptOutPierce[i][j].tExtreme, epsIntersect, 
				       closeEps);
	       }
	     }
           }
       }
                  cout << endl << "OUTER UMBRAL BITANGENT " << j << " OF OBSTACLE " << i 
		       << endl << sweptOutPierce[i][j];
     }
   }
                  cout << "Exiting outerPiercingSweep" << endl;
}

// OUTPUT OF each outer piercing sweep IS A BITANGENT, WHICH WILL BE USED TO CLIP INNER PIERCING SWEEP'S BITANGENT LATER

/******************************************************************************
  Define the room polygon (in which the scene is set).
******************************************************************************/

void defineRoom (float radiusRoom, Polygon2f &room)
{
  V2fArr roomPt(4);
  roomPt[0][0] = radiusRoom; roomPt[0][1] = radiusRoom;
  roomPt[1][0] =-radiusRoom; roomPt[1][1] = radiusRoom;
  roomPt[2][0] =-radiusRoom; roomPt[2][1] =-radiusRoom;
  roomPt[3][0] = radiusRoom; roomPt[3][1] =-radiusRoom;
  room.create (roomPt);
}

/******************************************************************************
START HERE
PROBLEM WITH SOFTWARE ON SURROUNDINGSPIRAL: ONCE IT ENCASES, THERE IS ONLY ONE BITANGENT,
NOT TWO.
  Define the back umbra/penumbra of each obstacle.
  This region is bounded by bitangents, as well as the curve

  Typical case (A does not surround L):
  The room is intersected with the inner halfspaces of the bitangents
  and the obstacle's chord.
  Then the obstacle's chord is replaced by curve samples on the correct
  segment spanning the chord.
  The incorrect segment is found by casting a ray from the light to the obstacle.
  The correct segment is the other one.

  Atypical case (A surrounds L):
  This case can be recognized by an outer bitangent that intersects A.
  Let T1 and T2 be the two rays along the outer bitangents of A and L.
  Let P1 and P2 be their intersections with the room and P their intersection.
  Let A1 and A2 be the points of tangency of T1 and T2 with A.
  The room is intersected with the inside halfspace of the line P1P2.
  A1, P, and A2 are then inserted into the polygon between P1 and P2.
  Then A1 P A2 is replaced by the correct curve segment between A1 and A2.
  The incorrect segment is found by casting a ray from the light to the obstacle.
******************************************************************************/

void defineBackUmbra (Array<UmbralBitangArr> &ut, Array<Polygon2f> &backUmbra, float epsIntersect, float sameSideEps)
{
  cout << endl << "Entering defineBackUmbra" << endl;
  int i,j;
  backUmbra.allocate(nOb);
  //  V2f typicalPtOutside;		// typical pt outside present halfspace
  //  obstacle[0].eval (obstacle[0].getKnot(0), typicalPtOutside); // any pt of light
  for (i=1; i<nOb; i++)     // each obstacle has a local umbra
   {
    cout << "Obstacle " << i << endl;
    Line2f L;
      /* OLD VERSION: ONLY WORKS WHEN A DOES NOT SURROUND L 
      if (no intersections of outer bitangent with A)  // A does not surround L
       {
        backUmbra[i] = room;			// start with room
        // repeatedly trim by a line, making sure that line is oriented properly (otherwise reversing)
        // this elegantly handles the issue of turning corners on the boundary of the room
	// trim by obstacle's chord
	L.create (ut[i][0].umbraOb, ut[i][1].umbraOb);
	if (L.inside (typicalPtOutside)) 	L.create (ut[i][1].umbraOb, ut[i][0].umbraOb);  // reverse it
	backUmbra[i].halfspaceIntersect (L);
	// trim by both outer bitangents
	V2f typicalPtInside;		// typical pt inside present halfspace
	obstacle[i].eval (obstacle[i].getKnot(1), typicalPtInside); // any pt of obstacle
	for (j=0; j<2; j++)
	  {
	    L.create (ut[i][j].umbraOb, ut[i][j].umbraExtreme);
	    if (!L.inside (typicalPtInside))  L.create (ut[i][j].umbraExtreme, ut[i][j].umbraOb);
	    backUmbra[i].halfspaceIntersect (L);
	  }
	// now add curve samples replacing chord
	V2f hit;  float tHit;  // parameter value of point on curve segment *not* to be sampled
	obstacle[i].shootRay (obCentroid[0], obCentroid[i], hit, tHit, epsIntersect, epsIntersect);
	int posInsert=0;       // index s.t. curve samples will be added between posInsert and posInsert+1
	// replacing the obstacle's chord with sample points
	int nLU = backUmbra[i].getn();
	while (!((backUmbra[i][posInsert].approxeq (ut[i][0].umbraOb, .001) && 
		  backUmbra[i][(posInsert+1)%nLU].approxeq (ut[i][1].umbraOb, .001)) ||
		 (backUmbra[i][posInsert].approxeq (ut[i][1].umbraOb, .001) && 
		  backUmbra[i][(posInsert+1)%nLU].approxeq (ut[i][0].umbraOb, .001))))
	  posInsert++;
	V2fArr sample;  // curve samples replacing chord
	if ((ut[i][1].tOb < ut[i][0].tOb && 
	     tHit > ut[i][1].tOb && 
	     tHit < ut[i][0].tOb) ||
	    (ut[i][1].tOb > ut[i][0].tOb && // wraparound from ut[i][1].tOb to ut[i][0].tOb
	     (tHit > ut[i][1].tOb ||  
	      tHit < ut[i][0].tOb)))
	  // tHit is on the segment (ut[i][1].tOb, ut[i][0].tOb) so choose other
	  obstacle[i].sampleSegment (ut[i][0].tOb, ut[i][1].tOb, sample, .01);
	else obstacle[i].sampleSegment (ut[i][1].tOb, ut[i][0].tOb, sample, .01);
	if (backUmbra[i][posInsert].dist (sample[0]) > 
	    backUmbra[i][posInsert].dist (sample[sample.getn()-1]))
	  sample.reverseIt();      // curve was sampled in wrong direction (wrt polygon)	backUmbra[i].insert (sample, posInsert);  // insert samples into backUmbra[i]
	backUmbra[i].triangulate ();
       }
      else  // A surrounds L
       {
      */  // NEW VERSION: WORKS IN ALL CASES, WHETHER A SURROUNDS L OR NOT
    // allocate more vertices than ever necessary
    V2fArr backUmbraPt(obstacle[i].getnSample()+1000);
    backUmbraPt[0] = ut[i][0].umbraExtreme;
    backUmbraPt[1] = ut[i][0].umbraOb;
    int nVert = 2;  // vertex count

    // now add curve samples
    V2f hit;  float tHit;  // parameter value of pt on curve segment *not* to be sampled
    V2f ptOnLight;     obstacle[0].getSample(0, ptOnLight);
    V2f ptOnObstacle;  obstacle[i].eval((ut[i][0].tOb + ut[i][1].tOb) / 2., ptOnObstacle);
             // no potential for ambiguity
    cout << "ut[i][0].tOb" << ut[i][0].tOb << endl;
    cout << "ut[i][1].tOb" << ut[i][1].tOb << endl;
    cout << "ptOnLight = " << ptOnLight << endl;
    cout << "ptOnObstacle parameter = " << (ut[i][0].tOb + ut[i][1].tOb) / 2. << endl;
    cout << "ptOnObstacle = " << ptOnObstacle << endl;
    obstacle[i].shootRay (ptOnLight, ptOnObstacle, hit, tHit, epsIntersect, epsIntersect);
    cout << "Got back from shootRay" << endl;
    cout << "tHit = " << tHit << endl;
    V2fArr sample;  // curve samples
    if ((ut[i][1].tOb < ut[i][0].tOb && tHit > ut[i][1].tOb && tHit < ut[i][0].tOb) ||
	(ut[i][1].tOb > ut[i][0].tOb && (tHit > ut[i][1].tOb ||  tHit < ut[i][0].tOb))) 
                                 // wraparound from ut[i][1].tOb to ut[i][0].tOb
	// tHit is on the segment (ut[i][1].tOb, ut[i][0].tOb) so choose other
	 obstacle[i].sampleSegment (ut[i][0].tOb, ut[i][1].tOb, sample, .05); // was .01
    else obstacle[i].sampleSegment (ut[i][1].tOb, ut[i][0].tOb, sample, .05);
    cout << "Sample = " << sample << endl;
    if (backUmbraPt[1].dist (sample[0]) > 
	backUmbraPt[1].dist (sample[sample.getn()-1]))
      sample.reverseIt();      // curve was sampled in wrong direction (wrt polygon)
    if (sample.getn() + 8 > backUmbraPt.getn())
      {
	cout << "backUmbraPt needs to be enlarged!" << endl;
	exit(-1);
      }
    for (j=0; j<sample.getn(); j++) backUmbraPt[nVert++] = sample[j];
    cout << "Added samples" << endl;

    backUmbraPt[nVert++] = ut[i][1].umbraOb;
    if (ut[i][0].umbraExtreme != ut[i][1].umbraExtreme)
     { // not an umbra closed by an outer bitang intersection (so umbra touches the room)
      cout << "Walking around the room" << endl;
      backUmbraPt[nVert++] = ut[i][1].umbraExtreme;

      // finally, add room vertices between ut[i][1].umbraExtreme & ut[i][0].umbraExtreme
      int iEdge = -1;  // ut[i][1].umbraExtreme lies on [room[iEdge],room[iEdge+1]]
      do {
	iEdge = (iEdge+1)%room.getn();
	L.create (room[iEdge], room[(iEdge+1)%room.getn()]);
      } while (!L.insideLine (ut[i][1].umbraExtreme, .01));
      cout << "2nd umbraExtreme " << ut[i][1].umbraExtreme << " lies on edge " 
	   << room[iEdge] 
	   << "," << room[(iEdge+1)%room.getn()] << endl;
      V2f typicalPtInside;
      float tInside = (ut[i][1].index1 == i ? ut[i][1].param1 : ut[i][1].param2) + sameSideEps;
      if (tInside > obstacle[i].getLastKnot()) tInside = obstacle[i].getKnot(0) + sameSideEps;
      obstacle[i].eval (tInside, typicalPtInside); // pt of A a step from pt of tangency
      cout << "Typical point on A = " << typicalPtInside << endl;
      Line2f outerBitang (ut[i][1].umbraOb, ut[i][1].umbraExtreme);
      if (!outerBitang.inside (typicalPtInside))
       { cout << "flipping" << endl; 
         outerBitang.create (ut[i][1].umbraExtreme, ut[i][1].umbraOb); }
      int walkingDir;
      if (outerBitang.inside(room[iEdge])) 
	{ walkingDir = -1; L.create(ut[i][1].umbraExtreme, room[iEdge]); }
      else { walkingDir=1; L.create(ut[i][1].umbraExtreme, room[(iEdge+1)%room.getn()]); }
      cout << "walkingDir = " << walkingDir << endl;
      while (!L.insideLine (ut[i][0].umbraExtreme, .01))
       {
	backUmbraPt[nVert++] = room[walkingDir == 1 ? (iEdge+1)%room.getn() : iEdge];
	cout << "Added " << backUmbraPt[nVert-1] << endl;
	if (walkingDir == 1) iEdge = (iEdge+1)%room.getn();
	else iEdge = mod(iEdge-1, room.getn());
	L.create (room[iEdge], room[(iEdge+1)%room.getn()]);
       }
     }
    backUmbraPt.shrink (nVert);
    backUmbra[i].create (backUmbraPt);
    cout << "backUmbra[" << i << "] = " << backUmbra[i] << endl;
    backUmbra[i].triangulate();
   }
  cout << "Exiting defineBackUmbra" << endl;
}

/******************************************************************************
  Define the front umbra/penumbra of each obstacle.
  This region is bounded by piercing bitangents, as well as the curve.

-> ut:         the bitangents that define the front (pen)umbrae
                 front umbra: outer piercing bitangents
<- frontUmbra: the polygons that define the front (pen)umbrae
-> sameSideEps: epsilon for testing the side of tangent the curve lies on
-> global:     flag: 1 iff ut is organized globally, one per obstacle 
                     rather than one per obstacle pair
******************************************************************************/

void defineFrontUmbra (Array<UmbralBitangArr> &ut, Array<Polygon2fArr> &frontUmbra, 
		       float sameSideEps, int global)
{
                         cout << "Entering defineFrontUmbra" << endl;
  int i,j;		       
  frontUmbra.allocate(nOb);
  for (i=1; i<nOb; i++)          // for each obstacle
   {
                         cout << "Processing obstacle " << i << endl;
    // sweptInPierce is built differently: one per obstacle, rather than 1 per 
    // obstacle pair: need to process differently
    int index; if (global) index = i; else index = i*nOb;
    frontUmbra[i].allocate(ut[index].getn());
    for (j=0; j<ut[index].getn(); j++)  // for each piercing bitangent of the obstacle
      if (!(global && ut[index][j].ignore))  // don't build a polygon for ignoreable 
	                          // front umbral bitangs (which are necessarily global)
       {
	            cout << "Processing piercing bitangent " << j << endl << ut[index][j];
	V2fArr sample;
	ut[index][j].sampleInsidePiercing (obstacle, sample, .01, sameSideEps);
	/*
	// build a line whose inside halfspace contains the desired segment
	Line2f L(ut[index][j].umbraOb, ut[index][j].umbraExtreme);
	
	// verify that line's inside contains A, and correct if necessary
	float parInside = ut[index][j].tOb + sameSideEps; // tOb is pt of tangency
	if (parInside > obstacle[i].getLastKnot()) 
	parInside = obstacle[i].getKnot(0) + sameSideEps;
	V2f ptInside;  obstacle[i].eval (parInside, ptInside);  
	  if (!L.inside (ptInside))  
	  L.create (ut[index][j].umbraExtreme, ut[index][j].umbraOb); // reverse if necessary
	  
	  float parInPosDir = ut[index][j].tExtreme + sameSideEps;
	  if (parInPosDir > obstacle[i].getLastKnot()) 
	  parInPosDir = obstacle[i].getKnot(0) + sameSideEps;
	  V2f ptInPosDir; obstacle[i].eval (parInPosDir, ptInPosDir);
	  if (L.inside (ptInPosDir))
	  obstacle[i].sampleSegmentClosed (ut[index][j].tExtreme, ut[index][j].tOb,      sample, .01);
	  else obstacle[i].sampleSegmentClosed (ut[index][j].tOb,      ut[index][j].tExtreme, sample, .01);
	*/

	frontUmbra[i][j].create (sample);
	frontUmbra[i][j].triangulate();
       }
   }
  cout << "Exiting defineFrontUmbra" << endl;
}

/******************************************************************************
	Define the line L from the umbral bitangent u
	so that ptInside lies inside L.
******************************************************************************/

void defineInsideHalfspace (UmbralBitang &u, V2f &ptInside, Line2f &L)
{
  L.create (u.umbraOb, u.umbraExtreme);
  if (!L.inside (ptInside))		// inside is wrong! swap it
    L.create (u.umbraExtreme, u.umbraOb);
}

/******************************************************************************
	Define the boundary polygon of each obstacle A's global back umbra, as follows.
	Start with a bounding square (representing the room)
	of radius R centered at the origin.  (R is defined at the command line.)
	Intersect this bounding square with the inside of the following halfspaces:
	1) obstacle chord (connecting points of bitangency
			   with the two inner umbral bitangents of A)
	2) inner umbral bitangent #1 of A
	3) inner umbral bitangent #2 of A
	4) outer umbral bitangent #1 of A
	5) outer umbral bitangent #2 of A
******************************************************************************/

void defineGlobalBackUmbra (Array<BezierCurve2f> &obstacle, Polygon2f &room,
			    Array<UmbralBitangArr> &sweptInner,
			    Array<UmbralBitangArr> &sweptOuter,
			    Array<Polygon2f> &globalBackUmbra, float epsIntersect)
{
                      cout << "Entering defineGlobalBackUmbra" << endl;
  Line2f L;
  V2f typicalPtOutside;		// typical pt outside present halfspace
  obstacle[0].eval (obstacle[0].getKnot(0), typicalPtOutside); // any pt of light
  globalBackUmbra.allocate (nOb);
  for (int i=1; i<nOb; i++)		// each umbra
   {
                      cout << endl << "Defining umbral polygon of obstacle " << i << endl;
    V2f typicalPtInside;		// typical pt inside present halfspace
    obstacle[i].eval (obstacle[i].getKnot(1), typicalPtInside); // any pt of obstacle
    globalBackUmbra[i] = room;			// start with room
                      cout << "Room = " << globalBackUmbra[i] << endl;
    L.create (sweptInner[i][0].umbraOb, 	// obstacle chord
    	      sweptInner[i][1].umbraOb);
    if (L.inside (typicalPtOutside)) 	// inside is wrong! swap it
      L.create (sweptInner[i][1].umbraOb, sweptInner[i][0].umbraOb);
                      cout << "Clipping by obstacle chord halfspace " << L << endl;    
    globalBackUmbra[i].halfspaceIntersect (L);
                      cout << "Umbral polygon = " << globalBackUmbra[i] << endl;
                      cout << "Clipping by inner umbral bitangent halfspace #1 " 
			   << sweptInner[i][0] << endl;
    defineInsideHalfspace (sweptInner[i][0], typicalPtInside, L);
    globalBackUmbra[i].halfspaceIntersect (L);  
                      cout << "Umbral polygon = " << globalBackUmbra[i] << endl;
		      cout << "Clipping by inner umbral bitangent halfspace #2 " 
			  << sweptInner[i][1] << endl; 
    defineInsideHalfspace (sweptInner[i][1], typicalPtInside, L);
                      cout << "L = " << L << endl;
    globalBackUmbra[i].halfspaceIntersect (L);
                      cout << "Umbral polygon = " << globalBackUmbra[i] << endl;
    if (!sweptOuter[i][0].abandon)
     {
                      cout << "Clipping by outer umbral bitangent halfspace #1 " 
			   << sweptOuter[i][0] << endl;
      defineInsideHalfspace (sweptOuter[i][0], typicalPtInside, L);  
      globalBackUmbra[i].halfspaceIntersect (L);
                      cout << "Umbral polygon = " << globalBackUmbra[i] << endl;
     }

    if (!sweptOuter[i][1].abandon)
     {
                      cout << "Clipping by outer umbral bitangent halfspace #2 " 
			   << sweptOuter[i][1] << endl;
      defineInsideHalfspace (sweptOuter[i][1], typicalPtInside, L);
      globalBackUmbra[i].halfspaceIntersect (L);
                      cout << "Umbral polygon = " << globalBackUmbra[i] << endl;
     } 

    // now add curve samples replacing chord
                      cout << "Adding curve samples" << endl;
    V2f hit;  float tHit;  // parameter value of pt on curve segment *not* to be sampled
    obstacle[i].shootRay (obCentroid[0], obCentroid[i], hit, tHit, epsIntersect, epsIntersect);
                      cout << "Finding insertion site" << endl;
    int posInsert=0;       // index s.t. curve samples will be added between posInsert 
                           // and posInsert+1,
                           // replacing the obstacle's chord with sample points
    float epsClose = .01; // was .001
    int nLU = globalBackUmbra[i].getn();
                      cout << "globalBackUmbra = " << globalBackUmbra[i] << endl;
		      cout << "sweptInner[i][0].umbraOb"<<sweptInner[i][0].umbraOb<<endl;
		      cout << "sweptInner[i][1].umbraOb"<<sweptInner[i][1].umbraOb<<endl;
    while (!((globalBackUmbra[i][posInsert].approxeq(sweptInner[i][0].umbraOb,epsClose) &&
	      globalBackUmbra[i][(posInsert+1)%nLU].approxeq (sweptInner[i][1].umbraOb,
							      epsClose)) 
	  || (globalBackUmbra[i][posInsert].approxeq(sweptInner[i][1].umbraOb,epsClose) &&
	      globalBackUmbra[i][(posInsert+1)%nLU].approxeq (sweptInner[i][0].umbraOb, 
							      epsClose))))
      {
        posInsert++;
      }
 cout << "Sampling" << endl;
 cout << "sweptInner[i][0].tOb: " << sweptInner[i][0].tOb << endl;
 cout << "sweptInner[i][1].tOb: " << sweptInner[i][1].tOb << endl;
 cout << "tHit: " << tHit << endl;
    V2fArr sample;  // curve samples replacing chord
    if ((sweptInner[i][1].tOb < sweptInner[i][0].tOb && 
	 tHit > sweptInner[i][1].tOb && 
	 tHit < sweptInner[i][0].tOb) ||
	(sweptInner[i][1].tOb > sweptInner[i][0].tOb && 
	 (tHit > sweptInner[i][1].tOb ||  
	  tHit < sweptInner[i][0].tOb)))
         obstacle[i].sampleSegment(sweptInner[i][0].tOb,sweptInner[i][1].tOb,sample,.01);
    else obstacle[i].sampleSegment(sweptInner[i][1].tOb,sweptInner[i][0].tOb,sample,.01);
cout << "Checking for reversal" << endl;
    if (globalBackUmbra[i][posInsert].dist (sample[0]) > 
	globalBackUmbra[i][posInsert].dist (sample[sample.getn()-1]))
      sample.reverseIt();      // curve was sampled in wrong direction (wrt polygon)
    globalBackUmbra[i].insert (sample, posInsert);  // insert samples 
cout << "Triangulating" << endl;
    globalBackUmbra[i].triangulate ();
    cout << globalBackUmbra[i] << endl;
   }
cout << "Exiting defineGlobalBackUmbra" << endl;
}

/******************************************************************************
        Define the global front umbra.
        Each outer piercing bitangent T potentially defines a component.
        (A component may merge with the back umbra, disappearing in the process.)
        A component is built from the inner piercing sweep of T,
        potentially the outer piercing sweep of T (if it clips meaningfully),
        and a curve segment.

-> sweptInPierce:    the bitangents generated by inner piercing sweeps
-> sweptOutPierce:   the bitangents generated by outer piercing sweeps
<- globalFrontUmbra: the polygons defining each component of the global front umbra
-> sameSideEps:      
******************************************************************************/

void
defineGlobalFrontUmbra (Array<BezierCurve2f>   &obstacle,
			Array<UmbralBitangArr> &sweptInPierce,
			Array<UmbralBitangArr> &sweptOutPierce,
			Array<Polygon2fArr>    &globalFrontUmbra, 
			float sameSideEps, float epsIntersectSmall)
{
  cout << "Entering defineGlobalFrontUmbra" << endl;
  int i,j;
  globalFrontUmbra.allocate (nOb);
  for (i=1; i<nOb; i++)
   {
     cout << "Processing obstacle " << i << endl;
    int nComp = 0;  // # of components in gfu of curve i
    for (j=0; j<sweptInPierce[i].getn(); j++)
      if (!sweptInPierce[i][j].ignore)
	nComp++;  // this component didn't merge with back umbra
    globalFrontUmbra[i].allocate(nComp);  
    cout << nComp << " components to global front umbra" << endl;
    nComp=0;
    for (j=0; j<sweptInPierce[i].getn(); j++)  // for each inner front umbra bitangent
      if (!sweptInPierce[i][j].ignore)
       {
	 cout << "Processing piercing bitangent "<< j << endl << sweptInPierce[i][j];
	V2fArr sample;
	if (sweptOutPierce[i][j].abandon) // no outer piercing sweep bitangent
	  {
	    cout << "No outer piercing sweep bitangent" << endl;
	    // case 1: if inner piercing sweep bitangent pierces A,
	    // the front umbra is the region defined by this piercing bitangent:
	    // i.e., between the pt of bitangency and the pt of intersection with A
	    if (sweptInPierce[i][j].piercing(radiusRoom))
	      {
		cout << "Umbra is fully defined by inner piercing sweep's piercing bitang"
		     << endl;
		cout << "Sampling" << endl;
		cout << "tExtreme = " << sweptInPierce[i][j].tExtreme << endl;
		sweptInPierce[i][j].sampleInsidePiercing (obstacle, sample, .01, 
							  sameSideEps);
		cout << "Samples = " << sample << endl;
		globalFrontUmbra[i][nComp].create (sample);
		globalFrontUmbra[i][nComp++].triangulate();
	      }
	    // case 2: otherwise, merges with the global back umbra
	    // NOT IMPLEMENTED YET
	    else
	      {
		cout << "Inner piercing sweep bitang is not piercing: "
		     << "front umbra merges with back umbra" << endl;
		/*		START HERE: SET THE FRONT UMBRA TO EMPTY IN THIS CASE?
		  NO: THIS CASE WILL NEVER OCCUR BECAUSE WE ELIMINATE IT IN 
		  INNERPIERCINGSWEEP */
		// --------------------------------------------------------
		// --------------------------------------------------------
	      }
	  }
	else 
	 {
	   cout << "Has an outer piercing sweep bitangent" << endl;
	  BezierCurve2f Tcurve;  
	  Tcurve.createLine (sweptOutPierce[i][j].umbraOb,
			     sweptOutPierce[i][j].umbraExtreme);
	  int nHit; V2fArr hit; FloatArr tHit, atHit;
	  Tcurve.intersect (obstacle[i], nHit, hit, tHit, atHit, epsIntersectSmall); 
	  assert (sweptOuter[i][j].index1 == 0);
	  cout << "sweptOuter[i][j] = " << sweptOuter[i][j] << endl;
	  if (nHit > 0)
	  //	  if (sweptOuter[i][j].index2 != i && nHit > 0)
	    // outer piercing sweep bitang involves L and B (not L and A) and
	    // outer piercing sweep bitang intersects A
	   {
	    cout << "outer piercing sweep bitangent intersects A" << endl;
	    // case 1: if inner piercing sweep bitangent S intersects 
	    // outer piercing sweep bitangent T before it pierces A:
	    //   - compute intersection point P of these two bitangents
	    //   - front umbra is defined by the:
	    //    a) seg of A between pt of bitang of S with A & pt of bitang of T with A
	    //       (inside both bitangents) and
	    //    b) 2 line segments between these 2 pts of bitang and intersection pt P
	    Line2f L1(sweptInPierce[i][j].umbraOb, sweptInPierce[i][j].umbraExtreme);
	    Line2f L2(sweptOutPierce[i][j].umbraOb,sweptOutPierce[i][j].umbraExtreme);
	    V2f hitt; float tHitt; 
	    if (L1.intersect (L2, hitt, tHitt, 1))
	     {
	      cout << "S intersects T before it pierces A" << endl; 
	      cout << "Sampling to outer piercing bitangent" << endl;
	      cout << "tHit = " << tHit << endl;
	      // assumes tHit are in increasing order along the line sweptOutPierce[i][j]
	      // sample up to intersection of outer piercing bitangent with A, atHit[0]
	      sweptInPierce[i][j].sampleInside (obstacle, atHit[0], sweptOutPierce[i][j],
						sample, .1, sameSideEps);
	      sample.append (hitt);  // add this intersection to end of sample
	      globalFrontUmbra[i][nComp].create (sample);
	      cout << "Samples = " << sample << endl;
	      globalFrontUmbra[i][nComp++].triangulate(); 
	     }
	    // case 2: otherwise (S pierces A before it intersects T)
	    // same as case 1 of if condition
	    else
	     {
	      cout << "S pierces A before it intersects T" << endl;
	      cout << "Umbra is fully defined by inner piercing sweep's piercing bitang"
		   << endl;
	      sweptInPierce[i][j].sampleInsidePiercing (obstacle, sample, .01,
							sameSideEps);
	      globalFrontUmbra[i][nComp].create (sample);
	      globalFrontUmbra[i][nComp++].triangulate();
	     }
	   }
	  else // outer piercing sweep bitangent does not intersect A
	   {
	    cout << "T does not intersect A" << endl;
	    // case 1: S is piercing, same as case 1 of if condition
	    if (sweptInPierce[i][j].piercing(radiusRoom))
	     {
	      cout << "S is piercing" << endl;
	      cout << "Umbra is fully defined by inner piercing sweep's piercing bitang"
		   << endl;
	      sweptInPierce[i][j].sampleInsidePiercing (obstacle, sample, .01, 
							sameSideEps);
	      globalFrontUmbra[i][nComp].create (sample);
	      globalFrontUmbra[i][nComp++].triangulate();
	     }
	    // case 2: S is not piercing
	    // merges with global back umbra 
	    else
	     {
	      cout << "S is not piercing" << endl;
	      cout << "Merges with back umbra" << endl;
	     }
	   }
	 }
       }
   }
  cout << "Exiting defineGlobalFrontUmbra" << endl;
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       i;
  int       ArgsParsed=0;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'c': closeEps = atof(argv[ArgsParsed++]);            break;
      case 'F': featureSize = atof(argv[ArgsParsed++]);         break;
      case 'e': epsIntersect = atof(argv[ArgsParsed++]);	break;
      case 'E': epsIntersectSmall = atof(argv[ArgsParsed++]);	break;
      case 'f': sameSideEps = atof(argv[ArgsParsed++]);         break;
      case 's': specialOb = atoi(argv[ArgsParsed++]);		break;
      case 'u': specialUmb = atoi (argv[ArgsParsed++]);		break;
      case 'r': radiusRoom = atof (argv[ArgsParsed++]);		break;
      case 'w': SURROUND=1;                                     break;
      case 'S': SCENEINPUT = 1;                                 break;
      case 't': COMPUTETANGONLY=1;                              break;
      case 'P': level = atoi(argv[ArgsParsed++]);               break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  if (closeEps == -1)  closeEps = .1/radiusRoom;  // not set by 'c' parameter
  cout << "closeEps = " << closeEps << endl;
  defineRoom (radiusRoom, room);
	cout << "Inputting curves" << endl;    
  if (SCENEINPUT)
    {
      Scene2d scene;
      ifstream infile;  infile.open(argv[argc-1]);
      scene.read (infile);
      cout << "Finished reading" << endl;
      infile.close();
      scene.build (nPtsPerSegment);
      cout << "Finished building" << endl;
      scene.extract (obstacle);
      for (i=0; i<obstacle.getn(); i++) obstacle[i].prepareDisplay (nPtsPerSegment);
      obCentroid.allocate (obstacle.getn());
      for (i=0; i<obstacle.getn(); i++) scene.centroid (i, obCentroid[i]);
    }
  else inputCurves(argv[argc-1], obstacle);
  for (i=0; i<obstacle.getn(); i++) cout << "Obstacle " << i << ":" << obstacle[i]<<endl;
  nOb = obstacle.getn(); 
  assert (specialOb >= 1 && specialOb < nOb);
  assert (specialUmb == 0 || specialUmb == 1);
	cout << "Creating hodographs" << endl;    
  specialHodo.createHodograph (obstacle[specialOb]);
  lightHodo.createHodograph   (obstacle[0]);

  /*  V2f foo;
  obstacle[0].eval (2.70535, foo);
  cout << "Point at 2.70535 on light is " << foo << endl;
  */

  if (level >= 1)
    {
        cout << "Building tangential curves" << endl; 
      buildTangentialCurves (obstacle);
	cout << "Building bitangents" << endl;    
      buildBitangent     (obduala, obdualb, epsIntersect, featureSize, bitang);
      buildSelfBitangent (obduala, obdualb, epsIntersect, featureSize, selfbitang);
    }
  if (level >= 2)
   {
        cout << "Building bitangents associated with direct visual events" << endl;
    buildDirectBitang (bitang, obstacle, epsIntersect, closeEps, featureSize, 
		       sameSideEps, direct);
    for (i=1; i<obstacle.getn(); i++)
      cout<< "Obstacle " << i << ": " << direct[i].getn() << " direct bitangents" << endl;
    
        cout << "Building bitangents associated with indirect visual events" << endl;
    buildIndirectBitang (bitang, selfbitang, obstacle, epsIntersect, closeEps, 
			 featureSize, sameSideEps, indirect);
	cout << "Building outer bitangents" << endl;    
    buildOuterBitang (bitang, obstacle, epsIntersect, closeEps, featureSize, sameSideEps, 
		      outer); // just to light
    cout << "Outer bitangents = " << outer << endl;
   }
  if (level >= 3)
   {
	cout << "Building inner bitangents" << endl;
    buildInnerBitang (bitang, obstacle, epsIntersect, closeEps, featureSize, sameSideEps, inner); // all pairs
   }
  if (level >= 4)
   {
	cout << "Building outer piercing bitangents" << endl;   
    buildOuterPierce (bitang, obstacle, epsIntersect, closeEps, sameSideEps, 
		      outerpierce); // just to light
   }
  if (level >= 5)
   {
	cout << "Building inner piercing bitangents" << endl;    
    buildInnerPierce (bitang, obstacle, epsIntersect, closeEps, sameSideEps, 
		      innerpierce); // all pairs
   }
  if (level >= 6)
    {
        cout << "Building local back umbra" << endl;
      defineBackUmbra (outer, localBackUmbra, epsIntersect, sameSideEps);
    }
  if (level >= 7)
    {
        cout << "Building local back penumbra" << endl;
      defineBackUmbra (inner, backPenumbra,   epsIntersect, sameSideEps);
    }
  if (level >= 8)
    {
        cout << "Building local front umbra" << endl;
      defineFrontUmbra (outerpierce, localFrontUmbra, .01, 1);
    }
  if (level >= 9)
    {
      // SHOULD CHANGE .01 TO SAMESIDEEPS
        cout << "Building local front penumbra" << endl;
      defineFrontUmbra (innerpierce, frontPenumbra,   .01, 0);
    }
  if (level >= 10)
   {
    if (SURROUND) fatalError("Theory not ready for inner sweep in surrounding case yet");
    else
      {
	// pair of sweeps starting at outer bitangents
	cout << "Inner sweep" << endl; 
	innerSweep (outer, inner, 
		    obstacle, sameSideEps, sweptInner);
      }
   }
  if (level >= 11)
   {
    if (SURROUND) fatalError("Theory not ready for outer sweep in surrounding case yet");
    else
      {
	    cout << "Outer sweep" << endl;
        outerSweep (outer, sweptInner, 
		    obstacle, sameSideEps, sweptOuter);
      }
   }
  if (level >= 12)
   {
    if (SURROUND) fatalError("Theory not ready for innerpiercing sweep in surround case");
    else
     {
	  // pair of sweeps starting at outer piercing bitangents
	    cout << "Inner piercing sweep" << endl;
      innerPiercingSweep (outerpierce, innerpierce, inner, sweptInner, 
			  obstacle, sameSideEps, sweptInPierce); 
     }
   }
  if (level >= 13)
   {
    if (SURROUND) fatalError("Theory not ready for outerpiercing sweep in surround case");
    else
      {
	    cout << "Outer piercing sweep" << endl;
        outerPiercingSweep (outerpierce, outer, sweptInPierce, 
			    obstacle, radiusRoom, epsIntersect, 
			    closeEps, sameSideEps, sweptOutPierce);
	    cout << "Outer piercing bitangents abandon flag:" << endl;
	    for (int i=0; i<sweptOutPierce.getn(); i++)
	      for (int j=0; j<sweptOutPierce[i].getn(); j++)
	        cout << sweptOutPierce[i][j].abandon << " ";
	    cout << endl;
      }
   }
  if (level >= 14)
   {
    if (SURROUND) fatalError("Theory not ready for global back umbra in surround case");
    else
     {	  
	  cout << "Building global back umbra" << endl;
      defineGlobalBackUmbra (obstacle, room, sweptInner, sweptOuter, globalBackUmbra,
			     epsIntersect);
      }
   }
  if (level >= 15)
   {
    if (SURROUND) fatalError("Theory not ready for global front umbra in surround case");
    else
     {	  
	    cout << "Building global front umbra" << endl;
	    // cout << sweptInPierce[1].getn() << endl;
	    // cout << "sweptInPierce[1][0].umbraOb: " << sweptInPierce[1][0].umbraOb << endl;
      defineGlobalFrontUmbra (obstacle, sweptInPierce, sweptOutPierce, globalFrontUmbra, 
			      sameSideEps, epsIntersectSmall);
   // defineFrontUmbra (sweptInPierce, globalFrontUmbra, .01, 1); 
      }
   }

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowPosition (0,0);
  glutInitWindowSize (900,900);
  char titlebar[100]; 
  strcpy (titlebar, "Umbra (");  
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
  glutAddMenuEntry ("Obstacles",                                100);
  glutAddMenuEntry ("Bitangents [1]", 				1);
  glutAddMenuEntry ("Self-bitangents", 				44);

  glutAddMenuEntry ("Direct bitangents      [d]",               40);
  glutAddMenuEntry ("Direct visual events   [D]",               41);
  glutAddMenuEntry ("Indirect bitangents    [i]",               42);
  glutAddMenuEntry ("Indirect visual events [I]",               43);

  glutAddMenuEntry ("One bitangent at a time [2]",		3);
  glutAddMenuEntry ("Late segment of a bitangent",              25);
  glutAddMenuEntry ("Intermediate segment of a bitangent",      29);  
  glutAddMenuEntry ("Outer bitangents with L [o]",      	2);
  glutAddMenuEntry ("Inner bitangents [j, CHANGED]",		4);
  glutAddMenuEntry ("Outer piercing bitangents [p]",		14);
  glutAddMenuEntry ("Inner piercing bitangents [P]",		21);
  glutAddMenuEntry (" ",                                        99);
  glutAddMenuEntry ("Local front umbra [f]",			15);
  glutAddMenuEntry ("Maximal front umbra [M]",                  22);
  glutAddMenuEntry ("Global front umbra [F]",			16);
  glutAddMenuEntry ("Local back umbra [b]",			11);
  glutAddMenuEntry ("Maximal back umbra [m]",			12);
  glutAddMenuEntry ("Global back umbra [B]",			5);
  glutAddMenuEntry ("Back penumbra [t]",                        30);
  glutAddMenuEntry ("Front penumbra [T]",                       31);
  glutAddMenuEntry (" ",                                        99);
  glutAddMenuEntry ("Label front umbra [l]",			17);
  glutAddMenuEntry ("Label back umbra [L]",			18);
  glutAddMenuEntry ("Numerical labels [a]",			13);
  glutAddMenuEntry ("Letter labels",                            24);
  glutAddMenuEntry ("Label the light",                          26);
  glutAddMenuEntry ("Label object responsible for blockage [R]",20);
  glutAddMenuEntry ("Shade local front umbra [s]",		19);
  glutAddMenuEntry ("Filled/unfilled [r]",			0);
  glutAddMenuEntry (" ",                                        99);
  glutAddMenuEntry ("Animate inner sweep of umbral bitangent [z]", 	6);
  glutAddMenuEntry ("Animate outer sweep of umbral bitangent [x]", 	7);
  glutAddMenuEntry ("Animate inner piercing sweep of umbral bitangent [c]", 32);
  glutAddMenuEntry ("Animate outer piercing sweep of umbral bitangent [v]", 33);
  glutAddMenuEntry ("Inner umbral bitangents", 			8);
  glutAddMenuEntry ("Outer umbral bitangents (not abandoned)", 	9);
  glutAddMenuEntry ("Piercing umbral bitangents",               28);
  glutAddMenuEntry ("Only for first obstacle [O]",		10);
  glutAddMenuEntry ("Draw origin",                              23);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
