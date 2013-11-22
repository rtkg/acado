##
##    This file is part of ACADO Toolkit.
##
##    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
##    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
##    Developed within the Optimization in Engineering Center (OPTEC) under
##    supervision of Moritz Diehl. All rights reserved.
##
##    ACADO Toolkit is free software; you can redistribute it and/or
##    modify it under the terms of the GNU Lesser General Public
##    License as published by the Free Software Foundation; either
##    version 3 of the License, or (at your option) any later version.
##
##    ACADO Toolkit is distributed in the hope that it will be useful,
##    but WITHOUT ANY WARRANTY; without even the implied warranty of
##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
##    Lesser General Public License for more details.
##
##    You should have received a copy of the GNU Lesser General Public
##    License along with ACADO Toolkit; if not, write to the Free Software
##    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##
##


##
##	Filename:  src/code_generation/Makefile
##	Author:    Boris Houska and Hans Joachim Ferreau
##	Date:      2010
##


##
##	include global settings
##
LOCAL_PATH_PREFIX = ../..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk



##
##	flags
##

IFLAGS    =  -I. \
             ${HEADER_PATHS}

CPPFLAGS  =  ${IFLAGS} ${CPP_GLOBAL_FLAGS}



OBJECTS = \
	integrator_export.${OBJEXT} \
	condensing_export.${OBJEXT} \
	gauss_newton_export.${OBJEXT} \
	mpc_export.${OBJEXT} \
	export_index.${OBJEXT} \
	export_data_argument.${OBJEXT} \
	export_data.${OBJEXT} \
	export_argument_list.${OBJEXT} \
	export_function.${OBJEXT} \
	export_ode_function.${OBJEXT} \
	export_statement.${OBJEXT} \
	export_arithmetic_statement.${OBJEXT} \
	export_statement_block.${OBJEXT} \
	export_for_loop.${OBJEXT} \
	export_function_call.${OBJEXT} \
	export_statement_string.${OBJEXT}

##
##	targets
##

all: libacado_code_generation.${LIBEXT}


libacado_code_generation.${LIBEXT}: ${OBJECTS}
	${AR} $@ $?


%.${OBJEXT}: %.cpp
	echo "Creating" $@
	@ ${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<


clean:
	${RM} -f *.${LIBEXT} *.${OBJEXT}

clobber: clean



##
##	end of file
##
