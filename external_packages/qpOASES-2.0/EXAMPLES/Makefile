##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  EXAMPLES/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   2.0
##	Date:      2007-2009
##



# select your operating system here!
include ../SRC/make_linux.mk
#include ../SRC/make_windows.mk


##
##	flags
##

IDIR        =  ../INCLUDE
SRCDIR      =  ../SRC
EXTRAS_DIR  =  EXTRAS

IFLAGS      =  -I. \
               -I${IDIR}

QPOASES_EXES = \
	example1${EXE} \
	example1a${EXE} \
	example1b${EXE} \
	example2${EXE} \
	example3${EXE} \
	example4${EXE} \
	exampleLP${EXE}

QPOASES_HEADERS = \
	${IDIR}/SQProblem.hpp \
	${IDIR}/QProblem.hpp \
	${IDIR}/QProblemB.hpp \
	${IDIR}/Bounds.hpp \
	${IDIR}/Constraints.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/CyclingManager.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp \
	${IDIR}/${EXTRAS_DIR}/SolutionAnalysis.hpp \
	${IDIR}/${EXTRAS_DIR}/OQPinterface.hpp



##
##	targets
##

all: ${QPOASES_EXES}


example1${EXE}: example1.${OBJEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB}

example1a${EXE}: example1a.${OBJEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB}

example1b${EXE}: example1b.${OBJEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB}

example2${EXE}: example2.${OBJEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB}

example3${EXE}: example3.${OBJEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB}

example4${EXE}: example4.${OBJEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_EXTRAS_LIB}

exampleLP${EXE}: exampleLP.${OBJEXT}
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LIB}


clean:
	${RM} -f *.${OBJEXT} ${QPOASES_EXES}

clobber: clean


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<


##
##	end of file
##