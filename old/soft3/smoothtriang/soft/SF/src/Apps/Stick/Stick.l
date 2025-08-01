.\" SCCSID: @(#)last.1	2.7	9/22/87
.TH Stick 1
.SH NAME
Stick \- convert SGP triangles to balls and sticks
.SH SYNTAX
.B Stick
<options>

.SH DESCRIPTION
The Stick
command converts a stream of SGP triangles into a ball and stick
representation.  The balls and sticks may be outputed in one of
three different formats.  If a triangle format is select,
the balls and sticks are approximated with
triangles; the balls are approximated with icosahedrons;
the sticks are cylinders approximated with eight triangles.
A portion of the tangent plane of a vertex is also drawn.

In an effort to avoid redundant output, a sphere (stick)
is only made for a vertex (edge) if that vertex (edge) hasn't
been output already.  A sphere is considered to have been seen before
if its center is within 1/10 the radius of another center.  A similar
test is used for determining if a stick has been seen before.  Also,
no triangles are generated for degenerate sticks.


.SH OPTIONS
.IP "\fB\-i x\fI
Select an input method.  'x' must be either 'sgp', 'dstruct', 
or 'segment'.
These select Ken Sloan triangle format or dstruct triangle format or
a special segment format.  The segment format expects lines containing
two x,y,z triples and an optional color, either an rgb triple or
two rgb triples (diffuse and specular) and a phong coefficient.
The default is Ken Sloan format.
.IP "\fB\-o x\fI
Select an output method.  'x' must be either 'sgp' or 'dstruct'.
These select Ken Sloan triangle format or dstruct triangle format.
The default is Ken Sloan format.
.IP "\fB\-b\fR
Suppress output of the balls.
.IP "\fB\-br\fI radius\fR
Sets the radius of the balls.
Default is 0.15.
.IP "\fB\-bm\fI material\fR
Select a material for the balls from the material library.
Default is RedPlastic.
.IP "\fB\-bc\fI spec rd gd bd rs gs bs\fR
Sets the color of the balls.  Note that is is preferable to use -bm.
.IP "\fB\-s\fR
Suppress output of sticks.
.IP "\fB\-sr\fI radius\fR
Sets the radius of the sticks.  Default is 0.05.
.IP "\fB\-sm\fI material\fR
Select a material for the sticks from the material library.
Default is BluePlastic.
.IP "\fB\-sc\fI spec rd gd bd rs gs bs\fR
Sets the color of the sticks.  Note that it is preferable to use -sm.
.IP "\fB\-n\fR
Suppress output of normals.
.IP "\fB\-sn\fI radius\fR
Sets the radius of the tangent planes.
Default is 0.45.
.IP "\fB\-nm\fI material\fR
Select a material for the tangent planes from the material library.
Default is YellowPlastic.
.IP "\fB\-nbm\fI material\fR
Select a material for the backside of the tangent planes 
from the material library.
Default is DarkGreenPlastic.
.IP "\fB\-nc\fI spec rd gd bd rs gs bs\fR
Sets the color of the tangent planes.  Note that it is preferable to use -sm.
.IP "\fB\-nbc\fI spec rd gd bd rs gs bs\fR
Sets the color of the backside of the tangent planes.
Note that it is preferable to use -sm.
.IP \fB\-Q\fR
Instead of eight triangles per stick, use sixteen.
.IP \fB\-a\fI scale\fR
Automatically scale the size of the balls, sticks, and normals based
on the first object read.  Additionally, the size is scaled by 'scale'.
Note: this currently only works on segment input.

.SH NOTES
While any number of -sm, -sc (-bm, -bc) may appear on the command line,
only the last one appearing is used.

If there are more than 1000 distinct vertices or edges, an error
message will be printed and duplicate triangle may be produced.

.SH DEFAULTS
The default stick radius is 0.05.  The default stick color is blue
plastic.

The default sphere radius is 0.15.  The default sphere color is
red plastic.

.SH EXAMPLE
Smash < octa.m | Fitter0 | Tess | Stick | Render -Q

.SH SEE\ ALSO
Smask, Fitter0, Tess, Render, material.h
