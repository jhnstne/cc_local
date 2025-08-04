/*
  File:          dual.c++ 
  Author:        J.K. Johnstone 
  Created:	 18 August 1999 
  Last Modified: 14 September 1999 
  Purpose:       Use dual curves to compute a visibility graph amongst
		 curved obstacles in the plane, and then interactively
		 compute shortest paths between a source and
		 destination (source and destination position being
		 interactively controlled by the mouse).  
  Input: 	 n obstacles.  Each obstacle is represented by an array
		 of sample points, defining a closed Bezier spline
		 through interpolation.  Thus, since this looks like
		 the contour format, and since we are collecting the
		 test data from the tablet as contours, the input file
		 IS a .rawctr file.  
  Output: 	 V-graph (and interactive motion).
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
#include "Contour.h"
#include "Vector.h"
#include "Polygon.h"
#include "BezierCurve.h"
#include "RatBezierCurve.h"

#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-j] (just display obstacles)" << endl;
  cout << "\t[-n] (don't output V-graph to file)" << endl;
  cout << "\t[-d displaydensity of Bezier segment]" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.rawctr" << endl;
 }

static GLfloat   zoomob, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1;	// first MOUSEX reading?
static int	 oldx;		// previous value of MOUSEX

static GLboolean JUSTVIEWING=0;		// just viewing: don't process further
static GLboolean NEWFILE=1;		// output processed data to new file?
static GLboolean DRAWWIRE=1;		// draw polygons as wireframes?
static GLboolean DRAWPOLYGONOB=0;	// draw input polygonal obstacles?
static GLboolean DRAWCURVEOB=1;		// draw curved obstacles?
static GLboolean DRAWALLTANGFORINTS=0;	// draw all tangents assoc w. intersections?
static GLboolean DRAWCOMMONTANG=1;	// draw common tangents?
static GLboolean DRAWDUALCTRLPOLY=0;	// draw dual control polygons?
static GLboolean DRAWDUALCURVE=1;	// draw dual curves?
static GLboolean DRAWTANG=0;		// draw tangent (for spinning)?
static GLboolean DRAWHIT=1;		// draw dual intersections?
static GLboolean rotateOb=0;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'ccw' direction?

// static GLboolean DEBUG = 1;

RatBezierCurve2f 	rat1, rat2;
V2fArr			rat1pt;
RatBezierCurve2f  	d1, d1top, d1bot, d11, d12;

int 	    	   	n=0;		// # of obstacles
Array<Polygon2f>   	obstaclePoly;
Array<BezierCurve2f> 	obstacle;
BezierCurve2f 		hodo0;		// hodograph of 1st obstacle, 
					// for interactive tangent display
BezierCurve2f		hodo1;		// hodo of 2nd obstacle, for removing common tangent mistakes
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
Array<RatBezierCurve2f>	obdual;		// dual curves of obstacles
Array<RatBezierCurve2f> obdualreflex;	// reflection of duals
int			nHit;		
V2fArr			hit;		// intersections of duals
FloatArr		tHit0;		// param values of intersections on obdual[0]
FloatArr		tHit1;		// param values of intersection on obdual[1]
int			nHitTrue;	// analogous, after filtering impostor common tangents
V2fArr 			hitTrue;
FloatArr 		tHit0True;
FloatArr		tHit1True;
int			nHitVis;	// analogous, for visible tangents
V2fArr			hitVis;
FloatArr		tHit0Vis;
FloatArr		tHit1Vis;
int			obstacleWin;	// identifier for left obstacle window
int			dualWin;	// identifier for right dual window

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  glClearColor (1.0, 1.0, 1.0, 1.0);

  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glEnable (GL_POINT_SMOOTH); 			// too slow for rotation 
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);
//  glLineWidth(2.0);
//  glLineStipple (1, 0xAAAA);

  zoomob = 1.75;
  zoomdual = .025; 
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void RotateOb (void)
{
  if (spinCCW)
   {
    tActive += tDelta;
    if (tActive > obstacle[0].getKnot (obstacle[0].getnKnot() - 1))
      tActive = obstacle[0].getKnot(0);
   }
  else
   {
    tActive -= tDelta;
    if (tActive < obstacle[0].getKnot(0))
      tActive = obstacle[0].getKnot (obstacle[0].getnKnot() - 1);
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) {
    if (rotateOb)
      glutIdleFunc (NULL);
  }
  else if (rotateOb)
    glutIdleFunc (RotateOb);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); break;					// ESCAPE
  case 'r':  	rotateOb = !rotateOb;
 	   	if (rotateOb) glutIdleFunc (RotateOb); else glutIdleFunc (NULL); break;
  case 'b':  	spinCCW = !spinCCW;     break;
  case 't':     DRAWTANG = !DRAWTANG;	break;
  default:      break;
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
 	  leftMouseDown = firstx = 1;
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
	  middleMouseDown = firstx = 1; 
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
  oldx = x;  
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

void menuOb (int value)
{
  switch (value) {
  case 1:  rotateOb = !rotateOb;
 	   if (rotateOb) glutIdleFunc (RotateOb); else glutIdleFunc (NULL); break;
  case 2:  DRAWTANG = !DRAWTANG; 					break;
  case 3:  DRAWALLTANGFORINTS = !DRAWALLTANGFORINTS;
  	   if (DRAWALLTANGFORINTS) DRAWCOMMONTANG=0;			break;
  case 4:  DRAWCOMMONTANG = !DRAWCOMMONTANG;				
  	   if (DRAWCOMMONTANG) DRAWALLTANGFORINTS=0;			break;
  default:   								break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1:     DRAWDUALCURVE 	= !DRAWDUALCURVE; 		break;
  case 2:     DRAWDUALCTRLPOLY  = !DRAWDUALCTRLPOLY;		break;
  case 4:     DRAWHIT		= !DRAWHIT;			break;
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomob, zoomob, zoomob);
  
  if (DRAWWIRE)
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  else
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    
  // display polygonal obstacles
  if (DRAWPOLYGONOB)
   {
    glColor3fv (Chocolate);
    for (i=0; i<n; i++)  obstaclePoly[i].draw(1);
   }
   
  if (DRAWCURVEOB)
   {
    glColor3fv (Red);	obstacle[0].draw();
    glColor3fv (Blue);	obstacle[1].draw();
   }
   
  if (DRAWALLTANGFORINTS)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nHit; i++)
     {
      V2f pt0;  obstacle[0].eval (tHit0[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit1[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   }
  if (DRAWCOMMONTANG)			// only true common tangents
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nHitTrue; i++)
     {
      V2f pt0;  obstacle[0].eval (tHit0True[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit1True[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   }
   
  // draw tangent on first curve at tActive
  if (DRAWTANG)
   {
    glColor3fv (Red);
    obstacle[0].drawTangent (tActive, hodo0);
   }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayDual ()
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  
  if (DRAWWIRE)
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  else
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

  // display dual curves
  
  if (DRAWDUALCURVE)
   {
    glColor3fv (Red);	obdual[0].draw(); 
    glColor3fv (Blue);	obdual[1].draw();
    glColor3fv (Purple);  obdualreflex[1].draw();
   }
  if (DRAWDUALCTRLPOLY)
   {
    glColor3fv (Blue);  
    for (i=0; i<n; i++)  obdual[i].drawCtrlPoly(); 
   }
   
  if (DRAWHIT)
   {
    glColor3fv (Black);
    glBegin (GL_POINTS);
//    for (i=0; i<nHit; i++) glVertex2f (hit[i][0], hit[i][1]);
    for (i=0; i<nHit; i++)  obdual[0].drawPt (tHit0[i]);
    glEnd();
   }
  
  // draw point associated with tangent, on first dual at tActive
  if (DRAWTANG)
   {
    glColor3fv (Red);
    obdual[0].drawPt(tActive);
   }
   
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  int       nPtsPerSegment = PTSPERBEZSEGMENT;
  ifstream  infile;
  ofstream  outfile;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'j': JUSTVIEWING=1; 					break;
      case 'n': NEWFILE=0; 					break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }  

  /************************INPUT************************************/
  
  infile.open(argv[argc-1]);
  // read data polygons as contours (glorified polygons with tablet-input software already written)
  // don't want to read as Body, otherwise contours will be hidden as private objects
  string filePrefix;  int format;  readFileName (infile, filePrefix, format);
  string comment;  readComment (infile, comment);
  getLeftBrace (infile);  string id;  infile >> id;  assert (id == "BODY");
  string name;  infile >> name;
  getLeftBrace (infile);  infile >> id;  assert (id == "SECTION"); infile >> name;
  infile >> id;  assert (id == "Z");  int zval;  infile >> zval;
  int mark = infile.tellg();  			// count the contours
  while (!tryToGetRightBrace(infile))		// look for ending brace
   {
    getLeftBrace(infile);  infile >> id;  assert (id == "CONTOUR");
    skipToMatchingRightBrace(infile); n++;
   }
  assert(n>0);  Array<Contour> ctrData(n);
  infile.seekg(mark);
  for (int i=0; i<n; i++)
   {
    getLeftBrace(infile);
    infile >> id;  infile >> name; 
    infile >> id;  assert (id == "COLOR");  int color;  infile >> color;
    mark = infile.tellg();  int nPt=0; V2f foo;	// count the points
    while (!tryToGetLeftBrace(infile) && !tryToGetRightBrace(infile))
     { 
      infile >> foo[0] >> foo[1];     nPt++;
     }
    assert(nPt>0);  V2f *Pt; Pt = new V2f[nPt];
    infile.seekg(mark);
    infile >> Pt[0][0] >> Pt[0][1];
    for (int j=1; j<nPt; j++)
     {
      infile >> Pt[j][0] >> Pt[j][1];
      if (Pt[j]==Pt[j-1]) { j--; nPt--; }	// skip duplicate
      if (j==nPt-1 && Pt[j]==Pt[0]) nPt--;	// skip duplicate at end, too
     }
    ctrData[i].create(nPt,Pt);  delete [] Pt;
    getRightBrace(infile);
   }
  getRightBrace(infile);
  getRightBrace(infile);
  
  // store for future display and use as polygonal obstacles
  obstaclePoly.allocate(n);
  for (i=0; i<n; i++)  obstaclePoly[i] = ctrData[i];

  // generate curved obstacles by interpolation
  obstacle.allocate(n);
  for (i=0; i<n; i++)  
   { 
    obstacle[i].fit (obstaclePoly[i]);  
    obstacle[i].prepareDisplay (nPtsPerSegment*3);
// obstacle[i].print();
   }
   
  /**************************DUALIZING**********************************/

  if (!JUSTVIEWING)			
   {
    obdual.allocate(n);
    obdualreflex.allocate(n);
    for (i=0; i<n; i++)  
     {
      obdual[i].dualOf (obstacle[i]);
      obdual[i].prepareDisplay (nPtsPerSegment*3);
      obdualreflex[i].reflect (obdual[i]);	// reflect dual about origin
      obdualreflex[i].prepareDisplay (nPtsPerSegment*3);
     }

    int nHitOrig, nHitReflex;	V2fArr hitOrig, hitReflex;
    FloatArr tHitOrig0, tHitOrig1, tHitReflex0, tHitReflex1;
    obdual[0].intersect (obdual[1], nHitOrig, hitOrig, tHitOrig0, tHitOrig1,.0000001);
    obdual[0].intersect (obdualreflex[1], nHitReflex, hitReflex, tHitReflex0, 
    			 tHitReflex1, .0000001);
    nHit = nHitOrig + nHitReflex;	hit.allocate (nHit);  
    tHit0.allocate(nHit);  tHit1.allocate(nHit);
    for (i=0; i<nHitOrig;  i++)  
     { hit[i] = hitOrig[i];  tHit0[i] = tHitOrig0[i]; tHit1[i] = tHitOrig1[i];}
    for (i=0; i<nHitReflex; i++) 
     { hit[i+nHitOrig] = hitReflex[i]; 
       tHit0[i+nHitOrig] = tHitReflex0[i]; tHit1[i+nHitOrig] = tHitReflex1[i]; }
       
cout << nHit << " intersections: " << endl;
for (i=0; i<nHit; i++)
  cout << "(" << hit[i][0] << "," << hit[i][1] << ") with parameter value on obstacle[0] of " 
       << tHit0[i] << " and obstacle[1] of " << tHit1[i] << endl;
       
    hodo0.createHodograph (obstacle[0]);
    hodo1.createHodograph (obstacle[1]);

    /************ remove common tangent mistakes **************/
    nHitTrue = 0;  hitTrue.allocate (nHit);  
    tHit0True.allocate(nHit);  tHit1True.allocate(nHit);
    for (i=0; i<nHit; i++)
     {
      // compare proposed common tangent with tangents at either end
      V2f pt0;   obstacle[0].eval (tHit0[i], pt0);	
      V2f pt1;   obstacle[1].eval (tHit1[i], pt1);
      V2f commonTang; for (int j=0; j<2; j++) commonTang[j] = pt1[j]-pt0[j];
      V2f tang0; hodo0.eval (tHit0[i], tang0);
      V2f tang1; hodo1.eval (tHit1[i], tang1);
      float angle0;  angle0 = commonTang.angle (tang0);
      float angle1;  angle1 = commonTang.angle (tang1);
cout << angle0 << " " << angle1 << endl;
      float radeps = .1;
      if ((angle0 < radeps || angle0 > M_PI-radeps) &&	// vectors should match 
	  (angle1 < radeps || angle1 > M_PI-radeps))
       {
        hitTrue  [nHitTrue]   = hit[i];
	tHit0True[nHitTrue]   = tHit0[i];
	tHit1True[nHitTrue++] = tHit1[i];
       }
     }

    /************ remove invisible common tangents **************/
    nHitVis = 0;  hitVis.allocate (nHitTrue);
    tHit0Vis.allocate(nHitTrue);  tHit1Vis.allocate (nHitTrue);
    for (i=0; i<nHitTrue; i++)
     {
      // 
      V2f pt0;   obstacle[0].eval (tHit0True[i], pt0);	
      V2f pt1;   obstacle[1].eval (tHit1True[i], pt1);
     }
   
    tActive = obstacle[0].getKnot(0);		// start at beginning
    tDelta = obstacle[0].getKnot (obstacle[0].getnKnot()-1) / 2000.;

    // intersect duals to find common tangents
    // store common tangents as obstacle index/parameter value quartets
    // check each common tangent for visibility
    //
    // cout << "Creating V-graph" << endl;
    // V-graph G = (V,E): V = {A,B} + points of common tangency 
    // (between 2 obstacles, the same obstacle, and A/B and obstacle);
    // E = common tangents + curved segments between vertices on the same obstacle
    // (not including segments between vertices that are already connected by a tangent)
    // cout << "Finished V-graph" << endl;
   }

  if (NEWFILE) 			// output V-graph
   {
    string vGraphFile = filePrefix + ".vgraph";
    outfile.open(vGraphFile.c_str());
    outfile << "{ " << vGraphFile << " }\n";
    // write polygonal obstacles out (repeat input) and then topology of Vgraph
   }

  //  through mouse controls outside main:
  //    input source and destination through mouse; compute shortest path
      
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowPosition (164,20);
  glutInitWindowSize (545,545);
  char titlebar[100]; 
  strcpy (titlebar, "Curves (");  
  strcat (titlebar, filePrefix.c_str());  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Tangents associated w. all intersections", 3);
  glutAddMenuEntry ("Common tangents", 4);
  glutAddMenuEntry ("Tangent [t]", 2);
  glutAddMenuEntry ("Spin tangent on first curve [r]", 1);
  glutAddMenuEntry ("Reverse direction of spin [b]", 5);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (724,30);
  glutInitWindowSize (545,545);
  strcpy (titlebar, "Dual curves");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Dual curves", 1);
  glutAddMenuEntry ("Dual control polygons", 2);
  glutAddMenuEntry ("Dual intersections", 4);
  glutAddMenuEntry ("Spin point on first dual [r]", 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

  // test intersection of rational Bezier curves
/*  V2f knot(0,1);  V3f wt(1,1,1); V2fArr cpt1(3), cpt2(3);
  cpt1[0][0] = cpt1[0][1] = 0.;
  cpt1[1][0] = cpt1[1][1] = 1.;
  cpt1[2][0] = 2.; cpt1[2][1] = 0.;
  cpt2[0][0] = 1.; cpt2[0][1] = -1.;
  cpt2[1] = cpt1[1];
  cpt2[2][0] = 2.; cpt2[2][1] = 1.;
  rat1.createRat (2, 2, 1, cpt1, wt, knot);
  rat2.createRat (2, 2, 1, cpt2, wt, knot);
  rat1.prepareDisplay (nPtsPerSegment*3);
  rat2.prepareDisplay (nPtsPerSegment*3);
  int rat1rat2n;
  rat1.intersect (rat2, rat1rat2n, rat1pt, .0001);
  cout << rat1rat2n << " intersections." << endl;
  cout << "Intersection: " << rat1pt[0][0] << " " << rat1pt[0][1] << endl; */
  
  // test subdivision : evaluate at same knot in many levels of subdivision
/*  V2fArr cpt1(3);
  cpt1[0][0] = cpt1[0][1] = 0.;
  cpt1[1][0] = cpt1[1][1] = 1.;
  cpt1[2][0] = 2.; cpt1[2][1] = 0.;
  V2f knot(0,1);  V3f wt(1,1,1); 
  RatBezierCurve2f r11, r111, foo;
  rat1.createRat (2, 2, 1, cpt1, wt, knot);
  rat1.subdivide (r11, foo);
  r11.subdivide (r111, foo);
  V2f pt; 
  rat1.eval (0.115112, pt);  cout << "(" << pt[0] << "," << pt[1] << ")" << endl;
  r11.eval (0.115112, pt);  cout << "(" << pt[0] << "," << pt[1] << ")" << endl;
  r111.eval (0.115112, pt);  cout << "(" << pt[0] << "," << pt[1] << ")" << endl; */
  
    // test subdivision of rational Bezier
/*    obdual[1].extract (1, d1);
    d1.subdivide (d1top, d1bot);
    d1bot.subdivide (d11, d12);
    d1.prepareDisplay (nPtsPerSegment*3);
    d11.prepareDisplay (nPtsPerSegment*3);
    d12.prepareDisplay (nPtsPerSegment*3); */
  
  // test of intersection (in displayOb)
/*  glColor3fv (Black);
  rat1.draw();  rat2.draw();
  glColor3fv (Red);
  glBegin(GL_POINTS);
  glVertex2f (rat1pt[0][0], rat1pt[0][1]);
  glEnd();  */

