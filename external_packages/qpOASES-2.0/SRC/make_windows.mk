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
##	Filename:  SRC/make_windows
##	Author:    Hans Joachim Ferreau
##	Version:   2.0
##	Date:      2009
##


##
##	definitions for compiling with Visual Studio under Windows
##

CPP = cl
AR  = ar
RM  = rm

OBJEXT = obj
LIBEXT = lib
EXE = .exe
DEF_TARGET =

CPPFLAGS = -nologo -EHsc -DWIN32 -Dsnprintf=_snprintf -D__NO_COPYRIGHT__

QPOASES_LIB         =  ${SRCDIR}/libqpOASES.${LIBEXT}
QPOASES_EXTRAS_LIB  =  ${SRCDIR}/libqpOASESextras.${LIBEXT}

##
##	end of file
##
