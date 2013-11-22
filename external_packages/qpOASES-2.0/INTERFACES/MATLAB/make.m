%%
%%	This file is part of qpOASES.
%%
%%	qpOASES -- An Implementation of the Online Active Set Strategy.
%%	Copyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.
%%
%%	qpOASES is free software; you can redistribute it and/or
%%	modify it under the terms of the GNU Lesser General Public
%%	License as published by the Free Software Foundation; either
%%	version 2.1 of the License, or (at your option) any later version.
%%
%%	qpOASES is distributed in the hope that it will be useful,
%%	but WITHOUT ANY WARRANTY; without even the implied warranty of
%%	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%%	See the GNU Lesser General Public License for more details.
%%
%%	You should have received a copy of the GNU Lesser General Public
%%	License along with qpOASES; if not, write to the Free Software
%%	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%%



%%
%%	Filename:  INTERFACES/MATLAB/make.m
%%	Author:    Hans Joachim Ferreau
%%	Version:   2.0
%%	Date:      2007-2009
%%



% consistency check
if ( exist( [pwd, '/make.m'],'file' ) == 0 )
	disp( 'ERROR: Run this make script directly within the directory' );
	disp( '       <qpOASES-inst-dir>/INTERFACES/MATLAB, please.' );
	return;
end


QPOASESPATH = '../../';

IFLAGS  = [ '-I. -I',QPOASESPATH,'INCLUDE',' ' ];

CPPFLAGS  = [ IFLAGS, '-D__cpluplus -D__MATLAB__ -O -DLINUX', ' ' ]; %% -O -D__DEBUG__ -D__SUPPRESSANYOUTPUT__

QPOASES_OBJECTS =	[	QPOASESPATH, 'SRC/SQProblem.cpp ',...
						QPOASESPATH, 'SRC/QProblem.cpp ',...
						QPOASESPATH, 'SRC/QProblemB.cpp ',...
						QPOASESPATH, 'SRC/Bounds.cpp ',...
						QPOASESPATH, 'SRC/Constraints.cpp ',...
						QPOASESPATH, 'SRC/SubjectTo.cpp ',...
						QPOASESPATH, 'SRC/Indexlist.cpp ',...
						QPOASESPATH, 'SRC/CyclingManager.cpp ',...
						QPOASESPATH, 'SRC/Utils.cpp ',...
						QPOASESPATH, 'SRC/MessageHandling.cpp ', ' ' ];


DEBUGFLAGS = '';
%DEBUGFLAGS = ' -g CXXDEBUGFLAGS=''$CXXDEBUGFLAGS -Wall -pedantic -Wshadow'' ';

NAME = 'qpOASES';
eval( [ 'mex -output ', NAME, ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ',QPOASES_OBJECTS] ] );
disp( [ NAME, '.', eval('mexext'), ' successfully created!'] );

NAME = 'qpOASES_sequence';
eval( [ 'mex -output ', NAME, ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ',QPOASES_OBJECTS] ] );
disp( [ NAME, '.', eval('mexext'), ' successfully created!'] );

NAME = 'qpOASES_sequenceSB';
eval( [ 'mex -output ', NAME, ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ',QPOASES_OBJECTS] ] );
disp( [ NAME, '.', eval('mexext'), ' successfully created!'] );

NAME = 'qpOASES_sequenceVM';
eval( [ 'mex -output ', NAME, ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ',QPOASES_OBJECTS] ] );
disp( [ NAME, '.', eval('mexext'), ' successfully created!'] );

path( path,pwd );


clear QPOASESPATH IFLAGS CPPFLAGS QPOASES_OBJECTS DEBUGFLAGS NAME



%%
%%	end of file
%%
