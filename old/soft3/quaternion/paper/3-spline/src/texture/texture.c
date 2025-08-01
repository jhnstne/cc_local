/*
  File: texture.c
  Authors: J.K. Johnstone
  Created: 21 January 1998
  Last Modified: 21 January 1998
  Purpose: Visualize maps to the sphere, by visualizing S^n -> S^n
  History:
*/

#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <REAL.h>			/* cbin */
#include <Misc.h>
#include <Vec.h>

static char *RoutineName;
static void usage()
 {
  (void)
   fprintf(stderr, "usage: %s\n", RoutineName);
   fprintf(stderr, "\t[-l] <# of lines of latitude in northern hemisphere (default: 20)>\n");
   fprintf(stderr, "\t[-h] (this help message)\n");
 }

static GLfloat   transx, transy, transz, rotx, roty, rotz, zoom;
static GLfloat	 StretchZ=1.0;		/* additional scaling of z (for elevation maps) */
static GLfloat   initrotx, initroty, initrotz;
static int 	 panLeft=0;  		/* control panning for 3d effect */
static int 	 panRight=1;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean rightMouseDown=0;
static GLboolean firstx=1;	/* first MOUSEX reading? */
static GLboolean firsty=1;
static int	 oldx,oldy;	/* previous value of MOUSEX and MOUSEY */
static GLboolean rotate=0;	/* start rotating? */
static GLboolean PAN=0;	 	/* rotate object back and forth for 3d effect? */

V3r		 white =  {1.0, 1.0, 1.0};
V3r		 black =  {0.0, 0.0, 0.0};
V3r		 red =    {1.0, 0.0, 0.0};
V3r 		 green =  {0.0, 1.0, 0.0};
V3r		 blue =   {0.0, 0.0, 1.0};
V3r     	 yellow = {1.0, 1.0, 0.0};
V3r		 magenta= {1.0, 0.0, 1.0};
V3r		 cyan   = {0.0, 1.0, 1.0};
static GLfloat ColorMap[8][3] =
{
        {0.0,0.0,0.0},		/* black */
        {1.0,0.0,0.0},		/* red */
        {0.0,1.0,0.0},		/* green */
        {0.0,0.0,1.0},		/* blue */
        {1.0,1.0,0.0},		/* yellow */
        {0.0,1.0,1.0},  	/* cyan */
        {1.0,0.0,1.0},  	/* magenta */
        {0.5,0.5,1.0}		/* light blue? */
};

V3r **S2;	/* S2[i][j] = jth pt on ith line of latitude of the sphere S2 */
REAL **dist;	/* dist[i][j] = dist(S2[i][j], M(S2[i][j])) */
		/* where M is the map of \Re^n to S^{n-1} being visualized */
int lat=20;	/* # of lines of latitude in strict northern hemisphere */

V3r **plane;	/* plane[i][j] = jth pt on ith grid line of the hyperplane x_3=0 */
REAL **distplane; /* dist[i][j] = dist(plane[i][j], inv(plane[i][j])) */
int SPHERE=1;
int INVERSION=0;

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  GLfloat mat_ambient[]    = {0.1745, 0.01175, 0.01175}; 
  GLfloat mat_specular[]   = {0.1, 0.1, 0.1, 1.0};
  GLfloat mat_diffuse[]    = {0.61424, 0.04136, 0.04136, 1.0}; 	/* red */
  GLfloat mat_emission[]   = {0.1, 0.1, 0.1, 1.0};
  GLfloat high_shininess[] = { 0.6 * 128.0 };
  GLfloat light0_position[] = {0.0, 0.0, 5.0, 0.0};
  GLfloat light1_position[] = {0.0, 0.0, -5.0, 0.0};
  GLfloat light2_position[] = {5.0, 0.0, 0.0, 0.0};
  GLfloat light3_position[] = {-5.0, 0.0, 0.0, 0.0};
  GLfloat light4_position[] = {0.0, 5.0, 0.0, 0.0};
  GLfloat light5_position[] = {0.0, -5.0, 0.0, 0.0};
  GLfloat light_ambient[]  = {0.2, 0.2, 0.2, 1.0};
  GLfloat light_diffuse[]  = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_specular[]  = {1.0, 1.0, 1.0, 1.0};

  glClearColor (1.0, 1.0, 1.0, 1.0);
  glEnable(GL_DEPTH_TEST); 
  glShadeModel (GL_SMOOTH);

  glMaterialfv (GL_FRONT, GL_AMBIENT,   mat_ambient);
  glMaterialfv (GL_FRONT, GL_DIFFUSE,   mat_diffuse);
  glMaterialfv (GL_FRONT, GL_SPECULAR,  mat_specular);
  glMaterialfv (GL_FRONT, GL_SHININESS, high_shininess);
  glMaterialfv (GL_FRONT, GL_EMISSION,  mat_emission); 
  glColorMaterial (GL_FRONT, GL_DIFFUSE);         /* set material using color */ 
  glEnable (GL_COLOR_MATERIAL); 

/*  glLightModeli (GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE); */  /* more expensive */
  glLightfv (GL_LIGHT0, GL_POSITION, light0_position);  
  glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv (GL_LIGHT1, GL_POSITION, light1_position); 
  glLightfv (GL_LIGHT1, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT1, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT1, GL_SPECULAR, light_specular);
  glLightfv (GL_LIGHT2, GL_POSITION, light2_position); 
  glLightfv (GL_LIGHT2, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT2, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT2, GL_SPECULAR, light_specular);
  glLightfv (GL_LIGHT3, GL_POSITION, light3_position); 
  glLightfv (GL_LIGHT3, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT3, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT3, GL_SPECULAR, light_specular);
  glLightfv (GL_LIGHT4, GL_POSITION, light4_position); 
  glLightfv (GL_LIGHT4, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT4, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT4, GL_SPECULAR, light_specular);
  glLightfv (GL_LIGHT5, GL_POSITION, light5_position); 
  glLightfv (GL_LIGHT5, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT5, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT5, GL_SPECULAR, light_specular);
/*  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glEnable (GL_LIGHT1); 
  glEnable (GL_LIGHT2);
  glEnable (GL_LIGHT3);
  glEnable (GL_LIGHT4);
  glEnable (GL_LIGHT5); */

  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_POINT_SMOOTH); 		/* too slow for rotation */
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glEnable (GL_LINE_SMOOTH);
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glPointSize (5.0);

  transx = 0.0;  transy = 0.0;  transz = 0.0;
    rotx = initrotx = 450;
    roty = initroty = 20;
    rotz = initrotz = 20;
  zoom = .75;
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-(GLfloat)w/(GLfloat)h, (GLfloat)w/(GLfloat)h, -1.0, 1.0, -5.0, 5.0);
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

void Pan (void)
{
  if (panLeft)
   {
    rotz += 0.1;
    if (rotz > 360.0)
      rotz -= 360.0;
    panLeft++;
    if (panLeft==200)
     {
       panLeft=0;
       panRight=1;
     }
   }
  else if (panRight)
   {
    rotz -= 0.1;
    if (rotz < 0.0)
      rotz += 360.0;
    panRight++;
    if (panRight==200)
     {
      panRight=0;
      panLeft=1;
     }
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) {
    if (rotate || PAN)
      glutIdleFunc (NULL);
  }
  else if (rotate)
    glutIdleFunc (Rotate);
  else if (PAN)
    glutIdleFunc (Pan);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); break;		/* ESCAPE */
  case 'p':	PAN = !PAN;		/* toggle pan */
		if (PAN) glutIdleFunc (Pan); else glutIdleFunc (NULL); break;
  case 'r':     rotate = !rotate;
  	        if (rotate) glutIdleFunc (Rotate); else glutIdleFunc (NULL); break;
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
    if (zoom < 0.0) zoom = 0.0;
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
    
    printf("(rotx,roty) = (%f,%f)\n", rotx,roty);
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
    case 1:     break;
    default:    break;
    }
  glutPostRedisplay();
}

/**********************************************************
	Draw a sphere using conventional parameterization.
***********************************************************/

static void mySphere()
{
  int i,j;
  
  glColor3fv (red);
  glBegin(GL_TRIANGLE_FAN);
  glNormal3f (0.,0.,-1.);
  glVertex3f (0.,0.,-1.);	/* south pole */
  for (i=0; i<4*lat+1; i++)
   {
    glNormal3fv (S2[0][i]);
    glVertex3fv (S2[0][i]);
   }
  glEnd();

  for (i=0; i<2*lat-2; i++)
   {
    glBegin(GL_QUAD_STRIP);
    for (j=0; j<4*lat+1; j++)
     {
      glNormal3fv (S2[i][j]);
      glVertex3fv (S2[i][j]);
      glNormal3fv (S2[i+1][j]);
      glVertex3fv (S2[i+1][j]);
     }
    glEnd();
   }

  glBegin(GL_TRIANGLE_FAN);
  glNormal3f (0.,0.,1.);
  glVertex3f (0.,0.,1.);	/* north pole */
  for (i=0; i<4*lat+1; i++)
   {
    glNormal3fv (S2[2*lat-2][i]);
    glVertex3fv (S2[2*lat-2][i]);
   }
  glEnd();
}

/*********************************************
	Apply the most natural map to sphere
	to the point p.
*********************************************/

void map2sphereM (V3r p, V3r pimg)
{
  REAL denom;

  /* (a,b,c) = 1/(a^2+b^2+c^2) (a^2 + b^2 - c^2, 2ac, 2bc) */
  /* rational map to S^2 with (a_1,...,a_3) = (x_1,x_2,x_3) */
  
  denom = p[0]*p[0] + p[1]*p[1] + p[2]*p[2];
  pimg[0] = (p[0]*p[0] + p[1]*p[1] - p[2]*p[2])/denom;
  pimg[1] = 2*p[0]*p[2]/denom;
  pimg[2] = 2*p[1]*p[2]/denom;
}

/*********************************************
	Apply `stereographic projection' 
	(in 3d version) to the point p.
*********************************************/

void map2sphereStereo (V3r p, V3r pimg)
{
  REAL denom;

  /* (a,b,c) = 1/(a^2+b^2+1) (a^2 + b^2 - 1, 2a, 2b) */
  /* rational map to S^2 with (a_1,...,a_3) = (x_1,x_2,1) */
  
  denom = p[0]*p[0] + p[1]*p[1] + 1;
  pimg[0] = (p[0]*p[0] + p[1]*p[1] - 1)/denom;
  pimg[1] = 2*p[0]/denom;
  pimg[2] = 2*p[1]/denom;
}

/*********************************************
	Apply inversion (wrt unit circle at origin)
	to the point p.
*********************************************/

void inversion (V3r p, V3r pimg)
{
  REAL denom;

  denom = p[0]*p[0] + p[1]*p[1];
  pimg[0] = p[0]/denom;
  pimg[1] = p[1]/denom;
  pimg[2] = 0;
}

/******************************************************************************/
/******************************************************************************/

void display (void)
{
  int i,j,k;

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transx, transy, transz);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  glScalef  (zoom, zoom, StretchZ * zoom);
  
/*  glPolygonMode (GL_FRONT, GL_LINE);
  glPolygonMode (GL_BACK, GL_LINE);  */
  
/* mySphere(); */

/*  glColor3fv (red);
    glutWireTeapot (2.0); */

  if (INVERSION)
   {
    for (i=0; i<2*lat; i++)
     {
      glBegin(GL_QUAD_STRIP);
      for (j=0; j<2*lat+1; j++)
       {
        glColor3f (distplane[i][j], distplane[i][j], distplane[i][j]);  
        glNormal3f (0.,0.,1.);
        glVertex3fv (plane[i][j]);
        glColor3f (distplane[i+1][j], distplane[i+1][j], distplane[i+1][j]);  
        glNormal3f (0.,0.,1.);
        glVertex3fv (plane[i+1][j]);
       }
      glEnd();
     }
   }
  
if (SPHERE)
 {
  glBegin(GL_TRIANGLE_FAN);
  glNormal3f (0.,0.,-1.);
  glVertex3f (0.,0.,-1.);	
  for (i=0; i<4*lat+1; i++)
   {
    glColor3f (dist[0][i], dist[0][i], dist[0][i]);
    glNormal3fv (S2[0][i]);
    glVertex3fv (S2[0][i]);
   }
  glEnd();

  for (i=0; i<2*lat-2; i++)
   {
    glBegin(GL_QUAD_STRIP);
    for (j=0; j<4*lat+1; j++)
     {
      glColor3f (dist[i][j], dist[i][j], dist[i][j]);  
      glNormal3fv (S2[i][j]);
      glVertex3fv (S2[i][j]);
      glColor3f (dist[i+1][j], dist[i+1][j], dist[i+1][j]);  
      glNormal3fv (S2[i+1][j]);
      glVertex3fv (S2[i+1][j]);
     }
    glEnd();
   }

  glBegin(GL_TRIANGLE_FAN);
  glNormal3f (0.,0.,1.);
  glVertex3f (0.,0.,1.);	
  for (i=0; i<4*lat+1; i++)
   {
    glColor3f (dist[2*lat-2][i], dist[2*lat-2][i], dist[2*lat-2][i]);   
/*    glColor3f (1-dist[2*lat-2][i], 0, dist[2*lat-2][i]);   */
    glNormal3fv (S2[2*lat-2][i]);
    glVertex3fv (S2[2*lat-2][i]);
   }
  glEnd(); 
 }

  glPopMatrix();
  glutSwapBuffers ();
}

/******************************************************************************/
/******************************************************************************/

int main(int argc, char **argv)
{
  int   ArgsParsed=0;
  FILE  *fp;
  int   i,j;
  REAL  d;	/* degree change between lines of latitude (or longitude) */
  V2r   *circ;	/* points on a generic circle */
  REAL  u,v;	/* parameters of sphere */
  REAL  c,s;	/* cosine, sine */
  V3r   **imgS2;  /* image of S2 under map to sphere */
  V3r   **imgplane;  /* image of plane under inversion */
  REAL  maxdist;
  REAL  x,y,foo;

  RoutineName = argv[ArgsParsed++];
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
     {
     switch (argv[ArgsParsed++][1])
      {
       case 'l': lat = atoi(argv[ArgsParsed++]); break;
       case 'h': 
       default:
		 usage(); 
                 printf("\n  where ...\n\n\t-h\t: this help message\n");
                 exit(-1);
      }
    }
   else ArgsParsed++;
  }  

  /************************************************************/
  
  		/* store sample points on sphere */
  
  S2 = (V3r **) malloc ((2*lat-1) * sizeof (V3r *)); /* 2*lat-1 lines of latitude */
  				/* N.B. not counting north and south pole */
  for (i=0; i<2*lat-1; i++)
    S2[i] = (V3r *) malloc ((4*lat+1) * sizeof(V3r));	/* pts on line of lat */

  circ = (V2r *) malloc ((4*lat+1) * sizeof(V2r));
  d = PI/(2*lat);		/* 90 deg/lat */
  circ[0][0] = circ[4*lat][0] = 1.0;	/* want exact endpoints */
  circ[0][1] = circ[4*lat][1] = 0.0;
  for (i=1; i<4*lat; i++) 
   {
    circ[i][0] = cos (i*d);
    circ[i][1] = sin (i*d);
   }
  
  for (i=0; i<2*lat-1; i++)	/* S2(u,v) = (cos u cos v, cos u sin v, sin u) */
   {				/* u \in [-PI/2,PI/2], v \in [0,2PI] */
    u = PI/(-2) + (i+1)*d;
    c = cos(u); s = sin(u);
    for (j=0; j<4*lat+1; j++)
     {
      S2[i][j][0] = c * circ[j][0];
      S2[i][j][1] = c * circ[j][1];
      S2[i][j][2] = s;
     }
   }
     
   		/* compute image of all sample points under map to sphere */
  imgS2 = (V3r **) malloc ((2*lat-1) * sizeof (V3r *));
  for (i=0; i<2*lat-1; i++)
    imgS2[i] = (V3r *) malloc ((4*lat+1) * sizeof(V3r));
  for (i=0; i<2*lat-1; i++)
    for (j=0; j<4*lat+1; j++)
      map2sphereStereo (S2[i][j], imgS2[i][j]);
   
 		/* compute distance of sample points from their images */
  dist = (REAL **) malloc ((2*lat-1) * sizeof(REAL *));
  for (i=0; i<2*lat-1; i++)
    dist[i] = (REAL *) malloc ((4*lat+1) * sizeof(REAL));
  for (i=0; i<2*lat-1; i++)
    for (j=0; j<4*lat+1; j++)
      dist[i][j] = acos (Dot (S2[i][j], imgS2[i][j]));
		/*      dist[i][j] = (REAL) i/(2*lat-1);	*/
     
     		/* normalize distances to [0,1] */
  maxdist = 0.;
  for (i=0; i<2*lat-1; i++)
    for (j=0; j<4*lat+1; j++)
      if (dist[i][j] > maxdist)
        maxdist = dist[i][j];
  for (i=0; i<2*lat-1; i++)
    for (j=0; j<4*lat+1; j++)
     {
      dist[i][j] /= maxdist;
      printf("dist[%i][%i] = %f\n", i,j,dist[i][j]);
     }  

  /************************************************************/
  		/* store sample points on plane */
  
  plane = (V3r **) malloc ((2*lat+1) * sizeof (V3r *)); 
  for (i=0; i<2*lat+1; i++)
    plane[i] = (V3r *) malloc ((2*lat+1) * sizeof(V3r));
  x=-2.;
  foo = (REAL) 2.0/lat;
  for (i=0; i<2*lat+1; i++)
   {
    y=-2.;
    for (j=0; j<2*lat+1; j++)
     {
      plane[i][j][0] = x;
      plane[i][j][1] = y;
      plane[i][j][2] = 0.;
      printf("plane[%i][%i]=(%f,%f,%f)\n", i,j,x,y,0.);
      y+=foo;
     }
    x+=foo;
   }
     
   		/* compute image of all sample points under inversion */
  imgplane = (V3r **) malloc ((2*lat+1) * sizeof (V3r *));
  for (i=0; i<2*lat+1; i++)
    imgplane[i] = (V3r *) malloc ((2*lat+1) * sizeof(V3r));
  for (i=0; i<2*lat+1; i++)
    for (j=0; j<2*lat+1; j++)
     {
      if (fabs(plane[i][j][0]) > .1 || fabs(plane[i][j][1]) > .1)
        inversion(plane[i][j],imgplane[i][j]);
     }
   
 		/* compute distance of sample points from their images */
  distplane = (REAL **) malloc ((2*lat+1) * sizeof(REAL *));
  for (i=0; i<2*lat+1; i++)
    distplane[i] = (REAL *) malloc ((2*lat+1) * sizeof(REAL));
  for (i=0; i<2*lat+1; i++)
    for (j=0; j<2*lat+1; j++)
      if (fabs(plane[i][j][0]) > .1 || fabs(plane[i][j][1]) > .1)
        distplane[i][j] = Dist (plane[i][j], imgplane[i][j]);
      else 
        distplane[i][j] = 5.;	/* don't want distances to go to infinity */

     		/* normalize distances to [0,1] */
  maxdist = 0.;
  for (i=0; i<2*lat+1; i++)
    for (j=0; j<2*lat+1; j++)
      if (distplane[i][j] > maxdist)
        maxdist = distplane[i][j];
  for (i=0; i<2*lat+1; i++)
    for (j=0; j<2*lat+1; j++)
     {
      distplane[i][j] /= maxdist;
      printf("distplane[%i][%i] = %f\n", i,j,distplane[i][j]);
     }  

  /************************************************************/

  glutInitWindowPosition (0,0);
/*xmax = glutGet (GLUT_SCREEN_WIDTH);*/
/*ymax = glutGet (GLUT_SCREEN_HEIGHT);*/
/*glutInitWindowSize (500,(int) 500*ymax/xmax);	*/ /* preserve aspect ratio */
  glutInitWindowSize (800,800);
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow ("Map texture");
  glutDisplayFunc (display);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motion);
  glutReshapeFunc (reshape);
  glutVisibilityFunc (visibility);
  gfxinit();
  glutCreateMenu (menu);
  glutAddMenuEntry ("", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);
  glutMainLoop();
  return 0;             
}
