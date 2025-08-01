%
% Author: Charles Loop
% Purpose: An octahedron
%
%	-e 3 3 3 -o -1 -1 -1 -u 0 1 0

px = (Point . (pos . [1,0,0]) (norm . [1,0,0]));
py = (Point . (pos . [0,1,0]) (norm . [0,1,0]));
pz = (Point . (pos . [0,0,1]) (norm . [0,0,1]));
mx = (Point . (pos . [-1,0,0]) (norm . [-1,0,0]));
my = (Point . (pos . [0,-1,0]) (norm . [0,-1,0]));
mz = (Point . (pos . [0,0,-1]) (norm . [0,0,-1]));
p;

PPP = [px,py,pz];
PPM = [py,px,mz];
PMP = [pz,my,px];
PMM = [px,my,mz];
MPP = [mx,pz,py];
MPM = [mx,py,mz];
MMP = [mx,my,pz];
MMM = [mz,my,mx];

mesh = {p,PPP,PMP,MPP,MMP,PPM,PMM,MPM,MMM};
