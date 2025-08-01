BASEDIR = ../..
M = Mesh
O = Operators
U = UserData
U2 = U2

libmesh.a: 
	(cd Mesh; make)
	(cd UserData; make)
	(cd Operators; make)
	rm -f libmesh.a
	ar q libmesh.a \
		$M/meshsym.o $M/mesh.o $M/y.tab.o $M/meshutils.o \
		$M/valid.o $M/meshAlloc.o $M/iterators.o \
		$M/debug.o \
		$U/meshGeometry.o $U/userData.o $U/userMalloc.o \
		$O/addremovemesh.o $O/addvertexedge.o \
		$O/centersplitface.o $O/joinvertices.o \
		$O/createface.o $O/destroyface.o $O/destroyvertex.o \
		$O/error.o $O/fixvertices.o $O/joinfaces.o \
		$O/joinhalfedge.o $O/removevertexedge.o \
		$O/splitedge.o $O/splitface.o $O/createvertex.o
	ar ts libmesh.a
	chmod -fR g+w .

libmesh2.a : Makefile
	(cd Mesh; make)
	(cd U2; make)
	(cd Operators; make)
	rm -f libmesh2.a
	ar q libmesh2.a \
		$M/meshsym.o $M/mesh.o $M/y.tab.o $M/meshutils.o \
		$M/valid.o $M/meshAlloc.o $M/iterators.o \
		$M/debug.o \
		${U2}/meshGeometry.o ${U2}/userData.o ${U2}/userMalloc.o \
		$O/addremovemesh.o $O/addvertexedge.o \
		$O/centersplitface.o $O/joinvertices.o \
		$O/createface.o $O/destroyface.o $O/destroyvertex.o \
		$O/error.o $O/fixvertices.o $O/joinfaces.o \
		$O/joinhalfedge.o $O/removevertexedge.o \
		$O/splitedge.o $O/splitface.o $O/createvertex.o
	ar ts libmesh2.a
	chmod -fR g+w .

man:
	cp Doc/mesh.3 Doc/userdata.3 ${BASEDIR}/man/man3
	cp Doc/meshformat.5 ${BASEDIR}/man/man5
	nroff -man Doc/mesh.3 > ${BASEDIR}/man/cat3/mesh.0
	nroff -man Doc/userdata.3 > ${BASEDIR}/man/cat3/userdata.0
	nroff -man Operators/operators.3 > ${BASEDIR}/man/cat3/operators.0
	nroff -man Doc/meshformat.5 > ${BASEDIR}/man/cat5/meshformat.0
	chmod -f g+w ${BASEDIR}/man/man3/mesh.3 ${BASEDIR}/man/cat3/mesh.0
	chmod -f g+w ${BASEDIR}/man/man3/userdata.3 \
		${BASEDIR}/man/cat3/userdata.0
	chmod -f g+w ${BASEDIR}/man/man3/operators.3 \
		${BASEDIR}/man/cat3/operators.0
	chmod -f g+w ${BASEDIR}/man/man5/meshformat.5 \
		${BASEDIR}/man/cat5/meshformat.0

include:
	cp Mesh/mesh.h UserData/userData.h Operators/operators.h \
		${BASEDIR}/include
	chmod -f g+w ${BASEDIR}/include/mesh.h ${BASEDIR}/include/userData.h \
		  ${BASEDIR}/include/operators.h

install: libmesh.a man include
	cp libmesh.a ${BASEDIR}/lib
	ar ts ${BASEDIR}/lib/libmesh.a
	chmod -f g+w ${BASEDIR}/lib/libmesh.a

#install: libmesh.a libmesh2.a man include
#	cp libmesh.a ${BASEDIR}/lib
#	ar ts ${BASEDIR}/lib/libmesh.a
#	chmod -f g+w ${BASEDIR}/lib/libmesh.a
#	cp libmesh2.a ${BASEDIR}/lib
#	ar ts ${BASEDIR}/lib/libmesh2.a
#	chmod -f g+w ${BASEDIR}/lib/libmesh2.a

clean:
	(cd Mesh; make clean)
	(cd UserData; make clean)
	(cd U2; make clean)
	(cd Operators; make clean)
	rm -f *.a
