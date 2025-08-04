#################################################################
#
#         .cshrc file
#
#         initial setups for both interactive and noninteractive
#         C-Shells
#
#################################################################
#
setenv PRINTER ps2
setenv TERM vt100
setenv EDITOR /ra/local/5emacs/bin/emacs
setenv DISPLAY light:0.0
setenv WWW_HOME "http://www.cis.uab.edu/"
setenv MOZILLA_HOME /hf/local/netscape4.7
setenv NETSCAPE_HOME /hf/local/netscape4.7
setenv OPENWINHOME /usr/openwin
setenv JAVA_HOME /hf/local/java
setenv JAVAHOME /hf/local/java
setenv NETSHOW_HOME_DIR /hf/local/netshow
setenv VOXWARE_HOME_DIR /hf/local/voxware/plugins
setenv XAPPLRESDIR $HOME/app-defaults
setenv NNTPSERVER juniper.cis.uab.edu
setenv GS_LIB /hf/local/share/ghostscript/6.01/lib
setenv GS_FONTS /hf/local/share/ghostscript/6.01/fonts
#
### FOR LaTeX2e  the new stuff
setenv MYTEXBIN "/hc/rd/tex_sol2.5/bin/sparc-solaris2.5"
setenv TEXMF /hc/rd/tex_sol2.5/texmf
setenv TEXINPUTS ":$TEXMF/tex/latex/oldCISinputs:$TEXMF/tex/latex:."
# setenv TEXINPUTS ":/ra/tex3.14/bin:/ra/tex3.14/lib:/ra/tex3.14/lib/inputs:.:/mb/jj/texlib"
setenv FONTDIR /hc/rd/tex_sol2.5/texmf/fonts
setenv TEXFONTS ":/hc/rd/tex_sol2.5/texmf/fonts/tfm/adobe/times"
# setenv TEXFONTS ":/ra/tex3.14/lib/fonts:/ra/tex3.14/lib/fonts/gf:/ra/tex3.14/lib/fonts/pk:/ra/tex3.14/lib/fonts/fb"
setenv MODE $TEXMF/metafont/misc/modes.mf
setenv FONTCACHEDIR /hc/rd/texfonts
setenv XTEX_MAKE_FONT "TeXtoXfont %n %d %b %m $MODE $FONTCACHEDIR"
setenv XTEX_TFM_PATH /hc/rd/texfonts/tfm/
setenv XTEX_FONT_PATH /hc/rd/texfonts/
# setenv TEXPOOL ":/ra/tex3.14/lib"
# setenv TEXFONTS_DVITPS_GF "/ra/tex3.14/lib/fonts/gf"
# setenv TEXFONTS_DVITPS_PK "/ra/tex3.14/lib/fonts/pk"
# setenv LATEXINPUTS "/ra/tex3.14/lib/inputs"
# setenv LATEXFORMATS "/ra/tex3.14/lib/tex/formats"
# setenv LATEXBIN "/ra/tex3.14/bin"
#
setenv INFOPATH /ra/local/info
setenv VTKDIR /mb/GRAIL/vtk
setenv VTKccEX ${VTKDIR}/graphics/examplesCxx
setenv VTKtclEX ${VTKDIR}/graphics/examplesTcl
setenv SUNVISION /da/sunvision_1.1
setenv XGLHOME /da/XGL_1.0.2
setenv VXHOME /usr/vx-mvx_1.0
setenv VXSERVERPATH $XGLHOME/vxserver:$SUNVISION/device/vx
setenv GOPHER_PRINTER "/ra/local/bin/nenscript -2 -G -P ps2 -r "
# IF I GET FRAMEMAKER:
# setenv FMHOME	/applic/utilities/frame
setenv WINDOWSYS Xwindows
#
setenv LD_LIBRARY_PATH ".\
		:$OPENWINHOME/lib\
		:/hf/local/lib/gcc-lib/sparc-sun-solaris2.6/egcs-2.91.66\
		:/hf/local/lib\
		:/usr/lib\
		:/usr/dt/lib\
		:/cb/cameron/vtk1.2/lib\
		:/mb/GRAIL/lib"
##
setenv CLASSPATH ".:$MOZILLA_HOME/java/classes/java40.jar:/hf/local/java/lib/tools.jar:/rc/proglang/java\
		 :/hf/local/jdk1.2.2/bin\
		 :/ra/local/netscape403/java/classes/ifc11.jar"
##
setenv MANPATH ":/usr/man:/hf/local/man:/usr/openwin/share/man:/hf/lang/SUNWspro/man:/usr/dt/share/man:."
## FOR SOLARIS2.6 GCC/g++  egcs-2.91.66 environment
#
setenv LIBRARY_PATH "/hf/local/lib/gcc-lib/sparc-sun-solaris2.6/egcs-2.91.66:/usr/lib:/usr/openwin/lib:"
setenv GCC_EXEC_PREFIX /hf/local/lib/gcc-lib/sparc-sun-solaris2.6/egcs-2.91.66
# setenv GCC_EXEC_PREFIX /ra/local/lib/gcc-lib/
# This is where GCC stores its private include files.
setenv GCC_INCLUDE_DIR /hf/local/lib/gcc-lib/sparc-sun-solaris2.6/egcs-2.91.66/include
# This is where g++ looks first for header files.
setenv GPLUS_INCLUDE_DIR /hf/local/lib/g++-include
# GNU CC searches this dir so that users can install header files.
setenv LOCAL_INCLUDE_DIR /hf/local/include
# This is the place other packages can install header files that GCC can use.
setenv TOOL_INCLUDE_DIR /hf/local/sparc-sun-solaris2.6/include
#
## END SOLARIS2.6/g++  egcs-2.91.66 environment
#
# FOR GNU CC C++ OBJC Solaris 2.x
#setenv GCC_EXEC_PREFIX /opt/gnu/lib/gcc-lib/sparc-sun-solaris2/cygnus-2.0.2/
#setenv LIBRARY_PATH /opt/gnu/lib/:.
#setenv C_INCLUDE_PATH /opt/gnu/lib
#setenv CPLUS_INCLUDE_PATH /wc/local/lib/g++-include/
#setenv OBJC_INCLUDE_PATH /wc/local/lib/gcc-lib/
#
set path=( .\
	$JAVAHOME/bin\
	/bin\
	/usr/bin\
	/sbin\
	/usr/sbin\
	$MOZILLA_HOME\
	/hf/lang/SUNWspro/bin\
	/hf/lang/SUNWspro/lib\
	/opt/NSCPcom\
	/hf/local/bin\
	/hf/local/lib\
	/hf/local/latex2html\
        /hf/local/lib/gcc-lib/sparc-sun-solaris2.6/egcs-2.91.66\
	/hf/local/lib/g++-include\
	/hf/local/xemacs-21.1.6/src\
	/hc/ra/local/bin\
	/usr/dt/bin\
	/usr/dt/lib\
	$OPENWINHOME/bin\
	$OPENWINHOME/bin/xview\
	$MYTEXBIN\
	/usr/ccs/bin\
	/usr/ucb\
	/usr/include\
	/usr/lib\
	/opt/SUNWits/Graphics-sw/xgl/lib\
	/usr/lib/lp/postscript\
	$HOME\
	  /cb/cameron/vtk1.2/bin \
	  /cb/Facets/bin \
	  /usr/dt/bin \
	  /hf/local/jdk1.2.2/bin \
	  /hf/local/latex2html \
	  /v/ingres \
	$HOME/bin)


##
#         cd path
#
set cdpath = (.. ~ /s / /usr)
#         set this for all shells
set nonomatch
umask 022
##biff y
set noclobber # new
#         skip remaining setup if not an interactive shell
if ($?USER == 0 || $?prompt == 0) exit
#          settings  for interactive shells
set prompt="`hostname`% "   # "`hostname`(\!)% "
set history=50 
set user = `whoami`
set mail = /var/mail/$user
set ignoreeof
set savehist=50
set notify
#set time=15

#         more aliases included with Interleaf

alias a alias
a pd	'pushd $cwd'
a po	popd
a cd 	'set old_dir = $cwd:q; chdir \!*;echo $cwd'
a back	'set temp_dir = $old_dir:q; cd $temp_dir:q; unset temp_dir'
a ho	hostname
a psa	"ps aux\!* |& sort +2rd -3 +8rb -9 +3rd |& more"
a pman 'neqn \!* | tbl | nroff -ms | colcrt'
a prv	printenv
a h     "history \!* 24"
a j     jobs -l
a hes   "echo hestia; rlogin hestia -l $USER"
alias telnet671 'telnet blazer6.cisulab.uab.edu'
a ghostview "cat www.ps"
alias c clear
alias cp            'cp -i'
alias mv            'mv -i'
alias pwd           'echo $cwd'
alias .             logout
alias sbp sbprolog $SPATH/modlib/\\\$readloop
alias intess 'source /wc/tess/t40/tcoms/intess'
alias tesstools 'sunview -s ~/.tesstools'
alias ls 'ls -F'
alias cdre 'cd /mb/jj/Research'
alias emacs 'emacs -nw'
alias man 'man -a -F'
alias cdclass 'cd /mb/jj/Class/680'
alias cdwww 'cd /rc/www/httpd/htdocs/info'
alias cdtex 'cd /ra/tex3.14/lib/inputs'
alias cdcontours 'cd /rc/GRAIL/ContoursRelease'
alias cdsurf 'cd /rc/GRAIL/SurfaceFitting'
alias cdmesa 'cd /cb/cameron/Mesa-1.2.8/demos'
alias cdglut 'cd /mb/jj/glut-3.0/lib/glut'
alias cdspline 'cd /mb/jj/Research/S3curve'
alias cdfacet 'cd /cb/Facets/src'
alias cdM 'cd /mb/jj/Research/S3curve'
alias cdgrant 'cd /mb/jj/Grant/NSF98'
alias mail /usr/ucb/Mail
alias nicemail dtmail
# usage: distill -sOutputFile=file.pdf file.ps: converts from PostScript to PDF
# then in GS>: GS>file.ps run
#              GS>quit
alias distill '/usr/local/bin/gs -I/usr/local/packages.d/gs/gs6.01 -q -dNOPAUSE -sDEVICE=pdfwrite'
alias cdwww 'cd /he/www/httpd/htdocs/info/faculty/jj'
