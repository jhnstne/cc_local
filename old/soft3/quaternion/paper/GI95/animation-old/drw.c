void draw_sphere(float x, float y, float z, float r)
{
	/* draw sphere */
	float params[4];
	
	sphmode(SPH_TESS,SPH_BILIN); 	/* longitude-latitude lines */
	sphmode(SPH_PRIM,SPH_LINE); 
	sphmode(SPH_DEPTH,10);
/*	sphmode(SPH_DEPTH,SPH_MAXDEPTH); */
	params[0]=x;
	params[1]=y;
	params[2]=z;
	params[3]=r;
	sphdraw( params );
}

void draw_point(float x, float y, float z)
{
	/* draw point */
	draw_sphere(x,y,z,PT_RADIUS);
}

void draw_point4dh_on_sphere(v4dh p)
{
	/* project onto x_3 = 0 */
	draw_sphere(p[1]/p[0],p[2]/p[0],p[4]/p[0],PT_RADIUS);
}

void prepare_draw_bspl_in_3d (const bspl_3d *bspl, 
			      double display_bspl[3][MAXDISPLAYPTS],
			      int *point_num)
{
	double display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS];
	int i;

	bspl_to_points(bspl->d, bspl->L, bspl->x1, bspl->knots, BSPL_DENSITY,
		       display_x1, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x2, bspl->knots, BSPL_DENSITY,
		       display_x2, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x3, bspl->knots, BSPL_DENSITY,
		       display_x3, point_num);
	for (i=0;i<*point_num;i++) {	/* collapse */
		display_bspl[0][i] = display_x1[i];
		display_bspl[1][i] = display_x2[i];
		display_bspl[2][i] = display_x3[i];
	}
}

void prepare_draw_bspl_in_4d (const bspl_4d *bspl, 
			      double display_bspl[4][MAXDISPLAYPTS],
			      int *point_num)
{
	double display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS], display_x4[MAXDISPLAYPTS];
	int i;

	bspl_to_points(bspl->d, bspl->L, bspl->x1, bspl->knots, BSPL_DENSITY,
		       display_x1, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x2, bspl->knots, BSPL_DENSITY,
		       display_x2, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x3, bspl->knots, BSPL_DENSITY,
		       display_x3, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x4, bspl->knots, BSPL_DENSITY,
		       display_x4, point_num);
	for (i=0;i<*point_num;i++) {	/* collapse */
		display_bspl[0][i] = display_x1[i];
		display_bspl[1][i] = display_x2[i];
		display_bspl[2][i] = display_x3[i];
		display_bspl[3][i] = display_x4[i];
	}
}

void draw_curve_in_3d(double display[3][MAXDISPLAYPTS], const int point_num)
{
	int i;
	double v[3];

	/* draw a Bezier or B-spline curve in 3-space */

	bgnline();
	for (i=0; i<point_num ;i++) {
		v[0]=display[0][i];
		v[1]=display[1][i];
		v[2]=display[2][i];
		v3d(v);
	}
	endline();
}

void draw_curve_in_4d(double display[4][MAXDISPLAYPTS], const int point_num)
{
	int i;
	double v[3];

	/* draw a Bezier or B-spline curve in 4-space */
	/* either project onto $x_2=0$ or perspective project */
	/* we do the former for simplicity at the present */

	setlinestyle(1); /* dashed */
	bgnline();
	for (i=0; i<point_num ;i++) {
		v[0]=display[0][i];
		v[1]=display[2][i];
		v[2]=display[3][i];
		v3d(v);
	}
	endline();
	setlinestyle(0); /* back to solid */
}

void draw_bspl_control_in_4d (const bspl_4d *bspl)
{
	/* draw control polygon */
	/* either project onto $x_2=0$ or perspective project */
	/* we do the former for simplicity at the present */

	int i;
	double v[3];
	bgnline();
	for (i=0; i<=bspl->L+2; i++) {
		v[0]=bspl->x1[i];
		v[1]=bspl->x3[i];
		v[2]=bspl->x4[i];
		v3d(v);
	}
	endline();
}

void prepare_draw_bez_in_3d (const bez_3d *bez, 
			     double display_bez[3][MAXDISPLAYPTS],
			     int *point_num)
{
	/* prepare to draw Bezier curve in 3-space */
	/* by generating POINT_NUM points on curve, in DISPLAY_BEZ */

	double display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS];
	int i;
	double v[3];
	
	
	for (i=0;i<bez->L;i++) {	/* for ith segment of Bezier spline */
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x1+(bez->d * i), 
 			 display_x1+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x2+(bez->d * i), 
			 display_x2+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x3+(bez->d * i),
			 display_x3+(BEZ_DENSITY * i));
	}

	/* collapse */
	*point_num = (bez->L * BEZ_DENSITY) + 1; /*each segment has BEZ_DENSITY+1
			points but the last one in ith segment = first one in 
			i+1st segment */
	for (i=0;i<*point_num;i++) {
		display_bez[0][i]=display_x1[i];
		display_bez[1][i]=display_x2[i];
		display_bez[2][i]=display_x3[i];
	}
}

void prepare_draw_bez_in_4d (const bez_4d *bez, 
			     double display_bez[4][MAXDISPLAYPTS],
			     int *point_num)
{
	/* prepare to draw Bezier curve in 4-space */
	/* by generating POINT_NUM points on curve, in DISPLAY_BEZ */

	double display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS], display_x4[MAXDISPLAYPTS];
	int i;
	double v[3];
	
	
	for (i=0;i<bez->L;i++) {	/* for ith segment of Bezier spline */
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x1+(bez->d * i), 
 			 display_x1+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x2+(bez->d * i), 
			 display_x2+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x3+(bez->d * i),
			 display_x3+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x4+(bez->d * i),
			 display_x4+(BEZ_DENSITY * i));
	}

	/* collapse */
	*point_num = (bez->L * BEZ_DENSITY) + 1; /*each segment has BEZ_DENSITY+1
			points but the last one in ith segment = first one in 
			i+1st segment */
	for (i=0;i<*point_num;i++) {
		display_bez[0][i]=display_x1[i];
		display_bez[1][i]=display_x2[i];
		display_bez[2][i]=display_x3[i];
		display_bez[3][i]=display_x4[i];
	}
}

void prepare_draw_ratbez_in_4d (const ratbez_4d *ratbez, 
			        double display_ratbez[4][MAXDISPLAYPTS],
			        int *point_num)
{
	/* prepare to draw rational Bezier curve in 4-space */
	/* by generating POINT_NUM points on curve, in DISPLAY_RATBEZ */

	double display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS], display_x4[MAXDISPLAYPTS];
	int i;
	double v[3];
	
	
	for (i=0;i<ratbez->L;i++) {	/* for ith segment of Bezier spline */
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x1+(ratbez->d * i),
			ratbez->weights+(ratbez->d * i),
 			display_x1+(BEZ_DENSITY * i));
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x2+(ratbez->d * i), 
			ratbez->weights+(ratbez->d * i),
			display_x2+(BEZ_DENSITY * i));
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x3+(ratbez->d * i),
			ratbez->weights+(ratbez->d * i),
			display_x3+(BEZ_DENSITY * i));
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x4+(ratbez->d * i),
			ratbez->weights+(ratbez->d * i),
			display_x4+(BEZ_DENSITY * i));
	}

	/* collapse */
	*point_num = (ratbez->L * BEZ_DENSITY) + 1; /*each segment has BEZ_DENSITY+1
			points but the last one in ith segment = first one in 
			i+1st segment */
	for (i=0;i<*point_num;i++) {
		display_ratbez[0][i]=display_x1[i];
		display_ratbez[1][i]=display_x2[i];
		display_ratbez[2][i]=display_x3[i];
		display_ratbez[3][i]=display_x4[i];
	}
}

void draw_curve_on_4d_sphere (double display[4][MAXDISPLAYPTS],
			      const int point_num)
{
	/* visualize curve on sphere by projecting onto $x_3=0$ */

	int i;
	double v[3],dist;

	bgnline();
	for (i=0; i<point_num; i++) {
		/* check that point is indeed on sphere */
		dist = 	sqrt(pow(display[0][i],2)+
		       	pow(display[1][i],2)+
			pow(display[2][i],2)+
			pow(display[3][i],2));
		if (fabs(dist-1.) > .1)
			printf("(%.2f,%.2f,%.2f,%.2f) is %f from sphere\n",
				display[0][i],display[1][i],display[2][i],
				display[3][i],dist-1.);
/*		else printf("(%.2f,%.2f,%.2f,%.2f) is OK\n",
*				display[0][i],display[1][i],display[2][i],
*				display[3][i]);
*/
		v[0] = display[0][i];
		v[1] = display[1][i];
		v[2] = display[3][i];
		v3d(v);
	}
	endline();
}

void draw_bez_control_on_4d_sphere (const bez_4d *bez)
{
        int i,n;
        double v[3],dist;

	n=bez->d * bez->L;
        bgnline();
        for (i=0; i<=n; i++) {
		if (i%(bez->d)==0) {
			/* this control point should lie on sphere */
			dist = 	sqrt(pow(bez->x1[i],2)+
				     pow(bez->x2[i],2)+
				     pow(bez->x3[i],2)+
				     pow(bez->x4[i],2));
			if (fabs(dist-1.) > .1)
				printf("control point (%.2f,%.2f,%.2f,%.2f) is %f from sphere\n",
				bez->x1[i],bez->x2[i],bez->x3[i],bez->x4[i],
				dist-1.);
/*			else printf("control point (%.2f,%.2f,%.2f,%.2f) is OK\n",
*				bez->x1[i],bez->x2[i],bez->x3[i],bez->x4[i]);
*/
		}
                v[0]=bez->x1[i];
                v[1]=bez->x2[i];
                v[2]=bez->x4[i];
                v3d(v);
        }
        endline();
}

void draw_ratbez_control_on_4d_sphere (const ratbez_4d *ratbez)
{
        int i,n;
        double v[3],dist;

	n=ratbez->d * ratbez->L;
	setlinestyle(1);
        bgnline();
        for (i=0; i<=n; i++) {
                v[0]=ratbez->x1[i];
                v[1]=ratbez->x2[i];
                v[2]=ratbez->x4[i];
                v3d(v);
        }
        endline();
	setlinestyle(0);
}

void draw_bez_control_in_4d (const bez_4d *bez)
{
        int i,n;
        double v[3];
	n=bez->d * bez->L;
        bgnline();
        for (i=0; i<=n; i++) {
                v[0]=bez->x1[i];
                v[1]=bez->x3[i];
                v[2]=bez->x4[i];
                v3d(v);
        }
        endline();
}

void draw_oriented_wire_cube (Qion q, V3d pos)
{
	/* draw a wireframe brick (one dimension longer) 
	   at position POS (reference vertex)
	   and orientation Q */
	/* in future: 	solid cube (using bgnmesh)
		      	cyclide (using CK)
			banana  (using Inventor or ...) */

	Matrix M;
	short v[3];

	pushmatrix();
	translate(pos[0],pos[1],pos[2]);
	quaternion_to_matrix(q,M);
	multmatrix(M);
	/* front face */
	bgnclosedline();
	v[0]=v[1]=v[2]=0;
	v3s(v);		/* (0,0,0) */
	v[0]=1;
	v3s(v);		/* (1,0,0) */
	v[2]=1;
	v3s(v);		/* (1,0,1) */
	v[0]=0;
	v3s(v);		/* (0,0,1) */
	endclosedline();

	/* back face */
	bgnclosedline();
	v[0]=v[2]=0; v[1]=2;
	v3s(v);		/* (0,2,0) */
	v[0]=1;
	v3s(v);		/* (1,2,0) */
	v[2]=1;
	v3s(v);		/* (1,2,1) */
	v[0]=0;
	v3s(v);		/* (0,2,1) */
	endclosedline();

	/* bottom face */
	bgnclosedline();
	v[0]=v[1]=v[2]=0;
	v3s(v);		/* (0,0,0) */
	v[0]=1;
	v3s(v);		/* (1,0,0) */
	v[1]=2;
	v3s(v);		/* (1,2,0) */
	v[0]=0;
	v3s(v);		/* (0,2,0) */
	endclosedline();

	/* top face */
	bgnclosedline();
	v[0]=v[1]=0; v[2]=1;
	v3s(v);		/* (0,0,1) */
	v[0]=1;
	v3s(v);		/* (1,0,1) */
	v[1]=2;
	v3s(v);		/* (1,2,1) */
	v[0]=0;
	v3s(v);		/* (0,2,1) */
	endclosedline();
	popmatrix();
}


/*
* void drawObject(const wireframe obj, const V3d pos, const Qion q)
* {
*/
	/* draw animated rigid object OBJ at POS in orientation Q */
	/* stub */
	/* wireframe of cube would be good starting place */

/*
* }
*/


	
