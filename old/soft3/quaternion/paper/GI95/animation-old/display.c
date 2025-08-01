long initialize_3d_window(char *str, long x1, long x2, long y1)
{
	/* open a window with title STR with lower corner at (x1,y1) */
	/* and upper corner at (x2,y2), where y2 preserves aspect ratio. */
	/* Returns its id */
	/* (x1,x2,y1,y2)=(xmax/9, xmax*9/10, ymax/9, ymax*9/10) is common */

	extern int xmax,ymax,zmax;
	int wid;
	static unsigned short dashed = 0x0f0f;

	xmax = getgdesc(GD_XPMAX);
	ymax = getgdesc(GD_YPMAX);
	zmax = getgdesc(GD_ZMAX);
	prefposition(x1,x2,y1,y1+(xmax/ymax)*(x2-x1));
/*	foreground(); */
	wid = winopen(str);
	minsize(xmax/10, ymax/10);
	keepaspect(xmax, ymax);
	winconstraints();
	
	RGBmode();
/*   	blendfunction(BF_SA,BF_MSA); */ /* transparency */ 
	doublebuffer();
	gconfig();
	mmode(MVIEWING); 	/* double matrix mode since using lighting */
	zbuffer(TRUE);

	/* devices */
	qdevice(LEFTMOUSE);	/* record clicks of left mouse on event queue */
	qdevice(MIDDLEMOUSE);
	qdevice(RIGHTMOUSE);
	qdevice(MOUSEX); 
	qdevice(MOUSEY); 
	qdevice(ESCKEY);
	qdevice(WINFREEZE);
	qdevice(WINTHAW);
	qdevice(REDRAWICONIC);
	qenter(REDRAW, wid);
	
/*	deflinestyle(1,dashed); */

	/* light and material models */
	lmdef(DEFLMODEL, 1, 10, light_model);
	lmdef(DEFLIGHT, 1, 14, white_light);
	lmdef(DEFMATERIAL, 1, 11, green_plastic);
	
	lmdef(DEFLIGHT, 2, 14, other_light);

	lmbind(LMODEL,   1); 
	lmbind(LIGHT0,   1); 
	lmbind(LIGHT1,   2); 
	lmbind(MATERIAL, 1);

	return(wid);
}

long initialize_3d_window2(char *str, long x1, long x2, long y1)
{
	/* open a window with title STR, returning its id */

	extern int xmax,ymax,zmax;
	int wid;

	xmax = getgdesc(GD_XPMAX);
	ymax = getgdesc(GD_YPMAX);
	zmax = getgdesc(GD_ZMAX);
	prefposition(xmax/9, xmax*9/10, ymax/9, ymax*9/10);
/*	foreground(); */
	wid = winopen(str);
	minsize(xmax/10, ymax/10);
	keepaspect(xmax, ymax);
	winconstraints();
	
	RGBmode();
/*   	blendfunction(BF_SA,BF_MSA); */ /* transparency */ 
	doublebuffer();
	gconfig();
	mmode(MVIEWING); 	/* double matrix mode since using lighting */
	zbuffer(TRUE);

	/* devices */
	qdevice(LEFTMOUSE);	/* record clicks of left mouse on event queue */
	qdevice(ESCKEY);
	qdevice(WINFREEZE);
	qdevice(WINTHAW);
	qdevice(REDRAWICONIC);
	qenter(REDRAW, wid);

	/* light and material models */
	lmdef(DEFLMODEL,   1, 10, light_model);
	lmdef(DEFLIGHT,    1, 14, white_light);
	lmdef(DEFLIGHT,    2, 14, other_light);
	lmdef(DEFMATERIAL, 1, 11, material1_plastic);
	lmdef(DEFMATERIAL, 2, 11, material2_plastic);
	lmdef(DEFMATERIAL, 3, 11, material3_plastic);
	
	lmbind(LMODEL,   1);
	lmbind(MATERIAL, 2);
	lmbind(LIGHT0,   1);
	lmbind(LIGHT1,   2);

	return(wid);
}
