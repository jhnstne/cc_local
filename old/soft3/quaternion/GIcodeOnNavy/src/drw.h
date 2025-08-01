#define DENSITY	100		
#define BSPL_DENSITY	DENSITY		/* max points per B-spline segment */
#define BEZ_DENSITY	DENSITY		/* max points per Bezier segment */
#define MAXSEG		30		/* max segments in B-spline or Bezier */
#define MAXDISPLAYPTS	(BSPL_DENSITY * MAXSEG)

extern void 	draw_sphere(float x, float y, float z, float r);

extern void 	draw_point(float x, float y, float z);

extern void 	draw_point4dh_on_sphere(v4dh p);

extern void 	prepare_draw_bspl_in_3d (const bspl_3d *bspl, 
			      double display_bspl[3][MAXDISPLAYPTS],
			      int *point_num);

extern void 	prepare_draw_bspl_in_4d (const bspl_4d *bspl,
			double display_bspl[4][MAXDISPLAYPTS], int *point_num);

extern void 	draw_curve_in_3d(double display[3][MAXDISPLAYPTS], 
			const int point_num);

extern void 	draw_curve_in_4d(double display[4][MAXDISPLAYPTS], 
			const int point_num);

extern void 	draw_bspl_control_in_4d (const bspl_4d *bspl);

extern void 	prepare_draw_bez_in_3d (const bez_3d *bez, 
			double display_bez[3][MAXDISPLAYPTS],  int *point_num);

extern void 	prepare_draw_bez_in_4d (const bez_4d *bez, 
			double display_bez[4][MAXDISPLAYPTS],  int *point_num);

extern void 	prepare_draw_ratbez_in_4d (const ratbez_4d *ratbez, 
		        double display_ratbez[4][MAXDISPLAYPTS], int *point_num);

extern void 	draw_curve_on_4d_sphere (double display[4][MAXDISPLAYPTS],
			const int point_num);

extern void 	draw_bez_control_on_4d_sphere(const bez_4d *bez);

extern void 	draw_ratbez_control_on_4d_sphere (const ratbez_4d *ratbez);

extern void 	draw_oriented_wire_cube (Qion q, V3d pos);

/* extern void 	drawObject(const Object obj,const V3d pos,const Qion q) */ /* e.g., obj = cyclide or banana */ 

