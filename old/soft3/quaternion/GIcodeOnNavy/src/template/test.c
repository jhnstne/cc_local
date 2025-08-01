#include <stdio.h>
#include <gl/gl.h>

main()
{
	short rgbvec[3];
	
	prefsize(400,400);
	winopen("RGBcolor");
	RGBmode();
/*	RGBcolor(0,100,0); */
	rgbvec[0]=0;
	rgbvec[1]=200;
	rgbvec[2]=0;
	c3s(rgbvec);
	clear();
	sleep(10);
	gexit();
	return 0;
}
