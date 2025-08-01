mz = (Point . (norm . [0, 0, -1])(pos . [0, 0, -1]));
my = (Point . (norm . [0, -1, 0])(pos . [0, -1, 0]));
mx = (Point . (norm . [-1, 0, 0])(pos . [-1, 0, 0]));
pz = (Point . (norm . [0, 0, 1])(pos . [0, 0, 1]));
py = (Point . (norm . [0, 1, 0])(pos . [0, 1, 0]));
px = (Point . (norm . [1, 0, 0])(pos . [1, 0, 0]));

MMM = [mx, mz, my];
MMP = [pz, mx, my];
MPM = [mz, mx, py];
MPP = [py, mx, pz];
PMM = [mz, px, my];
PMP = [px, pz, my];
PPM = [mz, py, px];
PPP = [pz, px, py];

mesh = {
	MMM,
	MMP,
	MPM,
	MPP,
	PMM,
	PMP,
	PPM,
	PPP};
