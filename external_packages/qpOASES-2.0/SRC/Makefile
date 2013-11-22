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
##	Filename:  SRC/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   2.0
##	Date:      2007-2009
##


# select your operating system here!
include make_linux.mk
#include make_windows.mk


##
##	flags
##

IDIR        =  ../INCLUDE
SRCDIR      =  ../SRC
EXTRAS_DIR  =  EXTRAS

IFLAGS      =  -I. \
               -I${IDIR}

QPOASES_OBJECTS = \
	SQProblem.${OBJEXT} \
	QProblem.${OBJEXT} \
	QProblemB.${OBJEXT} \
	Bounds.${OBJEXT} \
	Constraints.${OBJEXT} \
	SubjectTo.${OBJEXT} \
	Indexlist.${OBJEXT} \
	CyclingManager.${OBJEXT} \
	Utils.${OBJEXT} \
	MessageHandling.${OBJEXT}


QPOASES_EXTRAS_OBJECTS = \
	SolutionAnalysis.${OBJEXT} \
	OQPinterface.${OBJEXT}



##
##	targets
##

all: libqpOASES.${LIBEXT} libqpOASESextras.${LIBEXT}

withextras: libqpOASESextras.${LIBEXT}


libqpOASES.${LIBEXT}: ${QPOASES_OBJECTS}
	${AR} r $@ $?

libqpOASESextras.${LIBEXT}: ${QPOASES_OBJECTS} ${QPOASES_EXTRAS_OBJECTS}
	${AR} r $@ $?


SQProblem.${OBJEXT}: \
	SQProblem.cpp \
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
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} SQProblem.cpp

QProblem.${OBJEXT}: \
	QProblem.cpp \
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
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} QProblem.cpp

QProblemB.${OBJEXT}: \
	QProblemB.cpp \
	${IDIR}/QProblemB.hpp \
	${IDIR}/Bounds.hpp \
	${IDIR}/Constraints.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} QProblemB.cpp

Bounds.${OBJEXT}: Bounds.cpp \
	${IDIR}/Bounds.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} Bounds.cpp

Constraints.${OBJEXT}: \
	Constraints.cpp \
	${IDIR}/Constraints.hpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} Constraints.cpp

SubjectTo.${OBJEXT}: \
	SubjectTo.cpp \
	${IDIR}/SubjectTo.hpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} SubjectTo.cpp

Indexlist.${OBJEXT}: \
	Indexlist.cpp \
	${IDIR}/Indexlist.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} Indexlist.cpp

CyclingManager.${OBJEXT}: \
	CyclingManager.cpp \
	${IDIR}/CyclingManager.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} CyclingManager.cpp

Utils.${OBJEXT}: \
	Utils.cpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} Utils.cpp

MessageHandling.${OBJEXT}: \
	MessageHandling.cpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} MessageHandling.cpp

SolutionAnalysis.${OBJEXT}: \
	${EXTRAS_DIR}/SolutionAnalysis.cpp \
	${IDIR}/${EXTRAS_DIR}/SolutionAnalysis.hpp \
	${IDIR}/SQProblem.hpp \
	${IDIR}/QProblem.hpp \
	${IDIR}/QProblemB.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} ${EXTRAS_DIR}/SolutionAnalysis.cpp

OQPinterface.${OBJEXT}: \
	${EXTRAS_DIR}/OQPinterface.cpp \
	${IDIR}/${EXTRAS_DIR}/OQPinterface.hpp \
	${IDIR}/QProblem.hpp \
	${IDIR}/QProblemB.hpp \
	${IDIR}/Utils.hpp \
	${IDIR}/Constants.hpp \
	${IDIR}/Types.hpp \
	${IDIR}/MessageHandling.hpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} ${EXTRAS_DIR}/OQPinterface.cpp

clean:
	${RM} -f *.${OBJEXT}

clobber:
	${MAKE} clean; ${RM} -f *.${LIBEXT}


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<


##
##	end of file
##
