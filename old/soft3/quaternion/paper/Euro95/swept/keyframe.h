#define MAXINST		20	/* max # of curve instances to be interpolated
				   by swept surface */
#define PT_RADIUS	.03

extern void viz_sweep     (double	display_sweepcurve[3][MAXDISPLAYPTS],
			   const int	sweepcurve_num,
			   const int	m,
			   const V3d	pos[MAXINST],
			   const ratbez_4d *ori,
			   const double scale[MAXINST]);


