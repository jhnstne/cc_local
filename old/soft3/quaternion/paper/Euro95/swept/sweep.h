#ifndef REAL
#define REAL float
#endif

#define MAXINST		20	/* max # of curve instances to be interpolated
				   by swept surface */
				/* may want to make equal to MAXFRAMES in */
				/* ../cbin/oricurve.h */
#define PT_RADIUS	.03

extern void viz_sweep    (REAL	display_sweepcurve[3][MAXDISPLAYPTS],
		   const int	sweepcurve_num,
		   const int	m,
		   const V3d	pos[MAXINST],
		   const bez_3d *poscurve_de,
		   REAL  displayPosBez[3][MAXDISPLAYPTS],
                   const int	posBezNum,
		   const ratbez_4d *ori,
		   const REAL scale[MAXINST],
		   const bez_1d	*scaleBez,
		   display_surf	*display_tpsurf,
		   tp_ratbez 	*surf);

extern void viz_keys    (REAL	display_sweepcurve[3][MAXDISPLAYPTS],
		   const int	sweepcurve_num,
		   const int	m,
		   const V3d	pos[MAXINST],
		   const bez_3d *poscurve_de,
		   REAL  displayPosBez[3][MAXDISPLAYPTS],
                   const int	posBezNum,
		   const ratbez_4d *ori,
		   const REAL scale[MAXINST],
		   const bez_1d	*scaleBez,
		   display_surf	*display_tpsurf,
		   tp_ratbez 	*surf);

extern void viz_iso    (REAL	display_sweepcurve[3][MAXDISPLAYPTS],
		   const int	sweepcurve_num,
		   const int	m,
		   const V3d	pos[MAXINST],
		   const bez_3d *poscurve_de,
		   REAL	displayPosBez[3][MAXDISPLAYPTS],
                   const int	posBezNum,
		   const ratbez_4d *ori,
		   const REAL scale[MAXINST],
		   const bez_1d	*scaleBez,
		   display_surf	*display_tpsurf,
		   tp_ratbez 	*surf);

extern void inputPosOriScale(unsigned int *n, 
			     V3d pos[], Qion q[], REAL scale[]);

extern void defineMkl (REAL M[3][3], 
		       ratbez_4d *ori, const int s, const int i, const int j);

extern void print_diagnostics(ratbez_3d *sweepcurve, bez_3d *poscurve_de, 
	    		  ratbez_4d *ori, bez_1d *scaleBez);
