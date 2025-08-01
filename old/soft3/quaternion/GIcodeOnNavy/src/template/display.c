long initialize_3d_window(char *str)
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
	lmbind(MATERIAL, 1);
	lmbind(LIGHT0,   1);
	lmbind(LIGHT1,   2);

	return(wid);
}
