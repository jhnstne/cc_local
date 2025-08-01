%
% Octahedron
%
%       -e 3 3 3 -o -1 -1 -1 -u 0 1 0
%       -e 4 3 0 -o -4 -3 0 -u 0 0 1

Ptop = (Point .  (pos .   [0, 0, 1])
		 (norm .  [0, 0, 1])
		 (sff .   (v0 .    [0, 1, 0])
			  (v1 .    [-1, 0, 0])
			  (m .    [
				    [-1, 0], 
				    [0, -1]])));

Pbot = (Point .  (pos .   [0, 0, -1])
		 (norm .  [0, 0, -1])
		 (sff .   (v0 .    [0, -1, 0])
			  (v1 .    [-1, 0, 0])
			  (m .    [
				    [-1, 0], 
				    [0, -1]])));

P1x0 = (Point .  (pos .   [1, 0, 0])
		 (norm .  [1, 0, 0])
		 (sff .   (v0 .    [0, 0, 1])
			  (v1 .    [0, -1, 0])
			  (m .    [
				    [-1, 0], 
				    [0, -1]])));

P1x1 = (Point .  (pos .   [0, 1, 0])
		 (norm .  [0, 1, 0])
		 (sff .   (v0 .    [0, 0, -1])
			  (v1 .    [-1, 0, 0])
			  (m .    [
				    [-1, 0], 
				    [0, -1]])));

P1x2 = (Point .  (pos .   [-1, 0, 0])
		 (norm .  [-1, 0, 0])
		 (sff .   (v0 .    [0, 0, -1])
			  (v1 .    [0, -1, 0])
			  (m .    [
				    [-1, 0], 
				    [0, -1]])));

P1x3 = (Point .  (pos .   [0, -1, 0])
		 (norm .  [0, -1, 0])
		 (sff .   (v0 .    [0, 0, 1])
			  (v1 .    [-1, 0, 0])
			  (m .    [
				    [-1, 0], 
				    [0, -1]])));

FBx0 = [ Pbot, P1x1, P1x0 ];
FBx1 = [ Pbot, P1x2, P1x1 ];
FBx2 = [ Pbot, P1x3, P1x2 ];
FBx3 = [ Pbot, P1x0, P1x3 ];
FTx0 = [ Ptop, P1x0, P1x1 ];
FTx1 = [ Ptop, P1x1, P1x2 ];
FTx2 = [ Ptop, P1x2, P1x3 ];
FTx3 = [ Ptop, P1x3, P1x0 ];
mesh = {
	FBx0, FTx0,
	FBx1, FTx1,
	FBx2, FTx2,
	FBx3, FTx3
};
