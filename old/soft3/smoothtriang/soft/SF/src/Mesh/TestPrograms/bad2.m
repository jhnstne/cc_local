%

px = (Point . (pos . [1,0,0]) (norm . [1,0,0]));
py = (Point . (pos . [0,1,0]) (norm . [0,1,0]));
pz = (Point . (pos . [0,0,1]) (norm . [0,0,1]));
mx = (Point . (pos . [-1,0,0]) (norm . [-1,0,0]));
my = (Point . (pos . [0,-1,0]) (norm . [0,-1,0]));
mz = (Point . (pos . [0,0,-1]) (norm . [0,0,-1]));

MPM = [mx,py];
MMP = [mx];

mesh = {PMP,MPP,MMP,PPM,MPM,MMM};

