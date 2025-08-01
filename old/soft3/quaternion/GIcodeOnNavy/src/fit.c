/*************************************************************************
**									**
**	MODULE NAME 	:	fit_c.c					**
**									**
**	MODULE TYPE 	:	X11 curve-fitting demo program		**
**									**
**	MODULE AUTHOR 	:	Philip J. Schneider			**
**                                                                      **
        Last Modified: 6 February 1995

        By: K.R. Sloan 

**                                                                      **
**									**
**************************************************************************
*************************************************************************/
#include <stdio.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <malloc.h>
#include <math.h>
#include <string.h>

#include "2d.h"

#define	MAXPOINTS	1000
static	Point2		points[MAXPOINTS];
static	int		nPts = 0;
double	error = 16.0;

static  int		last_last_x = -42, last_last_y = -42;
static	int		last_x = -42, last_y = -42;

#define WINSIZE		400

static	Display 	*dpy;
static	int		screen;
static	Colormap	cmap;
static	XColor		actual, closest;
static	XEvent		event;
static	Window 		w, root;
static	GC 		gc, gcInvert, gcRed, gcGreen, gcBlue;


static	int interactive = 1;
static	int debug = 0;
static  int cp = 0;

extern	void	FitCurve();
static Point2 Bezier();


main(argc, argv)
    int 	argc;
    char 	*argv[];
{
    char		*display = (char *)0;
    int			i;
    unsigned long 	white, black;

    for (i = 0; i < argc; i++) {
	if (!(strncmp(argv[i], "-display", 8))) {
	    display = argv[i+1];
	    i++;
	    continue;
	}
	if (!(strncmp(argv[i], "-test", 5))) {
	    interactive = 0;
	    continue;
	}
	if (!(strncmp(argv[i], "-debug", 6))) {
	    debug = 1;
	    continue;
	}
	if (!(strncmp(argv[i], "-error", 6))) {
	    error = atof(argv[i+1]);
	    i++;
	    continue;
	}
	if (!(strncmp(argv[i], "-cp", 3))) {
	    cp = 1;
	    continue;
	}
    }

    if (!display) {
	extern char	*getenv();
	
	if (!(display = getenv("DISPLAY"))) {
	    (void)fprintf(stderr,
			  "%s : DISPLAY environment variable not set\n",
			  argv[0]);
	    (void)exit(-42);
	}
    }
    
    if (!(dpy = XOpenDisplay(display))) {
	(void)fprintf(stderr, "%s : Cannot open display %s\n",
		      argv[0], display);
	(void)exit(-42);
    }
    
    screen = DefaultScreen(dpy);
    root = RootWindow(dpy, screen);
    cmap = DefaultColormap(dpy, screen);
    
    white = WhitePixel(dpy, DefaultScreen(dpy));
    black = BlackPixel(dpy, DefaultScreen(dpy));
    
    w = XCreateSimpleWindow(dpy, root, 0, 0, WINSIZE, WINSIZE, 2, 
			    black, white);
    
    XChangeProperty(dpy, w, XA_WM_NAME, XA_STRING,
		    8, PropModeReplace, "INPUT", 6);
    XMapWindow(dpy, w);
    XSync(dpy, 0);
    
    {
	XGCValues	xgcv;
	
	xgcv.foreground = black;
	xgcv.background = white;
	gc = XCreateGC(dpy, w, GCForeground|GCBackground, &xgcv);
	
	xgcv.function = GXinvert;
	gcInvert = XCreateGC(dpy, w, GCForeground|GCBackground|GCFunction,
			     &xgcv);
	
	XAllocNamedColor(dpy, cmap, "red", &actual, &closest);
	xgcv.foreground = closest.pixel;
	xgcv.function = GXcopy;
	gcRed = XCreateGC(dpy, w, GCForeground|GCBackground|GCFunction,
			  &xgcv);
	
	XAllocNamedColor(dpy, cmap, "yellow", &actual, &closest);
	xgcv.foreground = closest.pixel;
	xgcv.function = GXcopy;
	gcGreen = XCreateGC(dpy, w, GCForeground|GCBackground|GCFunction,
			    &xgcv);
	
	XAllocNamedColor(dpy, cmap, "black", &actual, &closest);
	xgcv.foreground = closest.pixel;
	xgcv.function = GXcopy;
	gcBlue = XCreateGC(dpy, w, GCForeground|GCBackground|GCFunction,
			   &xgcv);
	
    }
    
    XSelectInput(dpy, w, 
		 (unsigned long)
		 (ButtonPressMask|ButtonReleaseMask|ExposureMask));

    if (interactive) {
	while(1) {
	    if (XPending(dpy)) {
		XNextEvent(dpy, &event);
		
		switch (event.type) {
		    case Expose : {
			break;
		    }
		    case ButtonPress : {
			XButtonPressedEvent *buttEvent;
			
			buttEvent = (XButtonPressedEvent *)&event;
			
			switch (buttEvent->button) {
			    case 1 : { /* Left button */
			        static void GetPoints();

				GetPoints();
				break;
			    }
			    case 2 : { /* Middle button */
				FitCurve(points, nPts, error);
			    break;
			    }
			    
			    case 3 : {
				nPts = 0;
				XClearWindow(dpy, w);
				break;
			    }
			}
			break;
		    }
		}
	    }
	}
    } else {
	static Point2	d2[26] = {
	    { 66.000000, 218.000000 },
	    { 66.000000, 207.000000 },
	    { 67.000000, 194.000000 },
	    { 69.000000, 181.000000 },
	    { 72.000000, 165.000000 },
	    { 75.000000, 155.000000 },
	    { 82.000000, 148.000000 },
	    { 89.000000, 144.000000 },
	    { 93.000000, 142.000000 },
	    { 100.000000, 142.000000 },
	    { 107.000000, 143.000000 },
	    { 120.000000, 145.000000 },
	    { 130.000000, 147.000000 },
	    { 140.000000, 149.000000 },
	    { 147.000000, 153.000000 },
	    { 151.000000, 155.000000 },
	    { 154.000000, 159.000000 },
	    { 157.000000, 161.000000 },
	    { 158.000000, 164.000000 },
	    { 158.000000, 166.000000 },
	    { 159.000000, 168.000000 },
	    { 160.000000, 170.000000 },
	    { 160.000000, 171.000000 },
	    { 161.000000, 172.000000 },
	    { 160.000000, 173.000000 },
	    { 159.000000, 173.000000 },
	};

	(unsigned)sleep((unsigned)5);

	FitCurve(d2, 26, error);

	while (1);
    }
}


static void GetPoints()
{
    while (1) {
	if (XPending(dpy)) {
	    XNextEvent(dpy, &event);
	    
	    if (event.type == ButtonRelease) {
		return;
	    }
	} else {
	    Window	root_return, child_return;
	    int		root_x_return, root_y_return,
	    		win_x_return, win_y_return;
	    unsigned int mask_return;
	    
	    XQueryPointer(dpy, w,
			  &root_return, &child_return,
			  &root_x_return, &root_y_return,
			  &win_x_return, &win_y_return,
			  &mask_return);

	    if (win_x_return != last_x || win_y_return != last_y) {
		if (win_x_return != last_last_x ||
		    win_y_return != last_last_y) {

		    static void AddPoint();
		    
		    AddPoint(win_x_return, win_y_return);

		    last_last_x = last_x;
		    last_last_y = last_y;

		    last_x = win_x_return;
		    last_y = win_y_return;
		}
	    }
	}
    }
}



static void AddPoint(x, y)
    short x, y;
{
    nPts++;
    points[nPts-1].x = (int)x; 
    points[nPts-1].y = (int)y;

    XFillRectangle(dpy, w, gcRed, x-1, y-1, 2, 2);
}



DrawBezierCurve(degree, pts)
    int		degree;
    Point2	*pts;
{
    int		steps =  500;
    int		i;
    Point2	pt;
    XPoint	xPt;

    /*
     *  Draw in the Bezier curves themselves 
     */
    for (i = 0; i < steps; i++) {
	pt = Bezier(degree, pts, (double)i/steps);
	xPt.x = (short)pt.x;
	xPt.y = (short)pt.y;

        XFillRectangle(dpy, w, gcGreen, xPt.x-1, xPt.y-1, 2, 2);
    }

    /*
     *  Optionally draw in the control points
     */
    if (cp) {
	for (i = 0; i <= degree; i++) {
	    xPt.x = (short)pts[i].x;
	    xPt.y = (short)pts[i].y;
	    
	    if (i % 3 == 0) {
		XFillRectangle(dpy, w, gc, xPt.x-2, xPt.y-2, 4, 4);
	    } else {
		XFillRectangle(dpy, w, gcBlue, xPt.x-2, xPt.y-2, 4, 4);
	    }
	}
	
	
	/*
	 *  Optionally draw in the control polygons
	 */
	for (i = 1; i <= degree; i++) {
	    XPoint	xPt1, xPt2;
	    
	    xPt1.x = (short)pts[i].x;
	    xPt1.y = (short)pts[i].y;
	    
	    xPt2.x = (short)pts[i-1].x;
	    xPt2.y = (short)pts[i-1].y;
	    
	    XDrawLine(dpy, w, gcBlue, xPt1.x, xPt1.y, xPt2.x, xPt2.y);
	}
    }

    XFlush(dpy);
}




/*****************************************************************
 *  TAG( Bezier )
 * 
 *  Evaluate a Bezier curve at a particular parameter value
 * 
 *  Inputs:
 *	d		The degree of the bezier curve
 *	V		Control points of bezier curve
 *	t		Solve at this parametric value
 *	
 *  Outputs:
 *	none
 *
 *  Returns:
 *	Point on curve as evaluated.
 *
 *  Algorithm:
 *	deCasteljau's algorithm
 */
static Point2 Bezier(degree, V, t)
    int		degree;		/* The degree of the bezier curve	*/
    Point2 	*V;		/* Pointer to array of control points	*/
    double 	t;		/* Parametric value to find point for	*/
{
    int 	i, j;		/* Index variables 			*/
    Point2 	Q;	        /* Point corresponding to parameter t	*/
    Point2 	*Vtemp;		/* Local copy of control points		*/

    Vtemp = (Point2 *)malloc((unsigned)((degree+1) * sizeof(Point2)));

    /* Copy array	*/
    for (i = 0; i <= degree; i++) {
	Vtemp[i] = V[i];
    }

    /* Triangle computation	*/
    for (i = 1; i <= degree; i++) {	
	for (j = 0; j <= degree-i; j++) {
	    Vtemp[j].x = (1.0 - t) * Vtemp[j].x + t * Vtemp[j+1].x;
	    Vtemp[j].y = (1.0 - t) * Vtemp[j].y + t * Vtemp[j+1].y;
	}
    }

    Q = Vtemp[0];
    (void)free((void *)Vtemp);

    return Q;
}

