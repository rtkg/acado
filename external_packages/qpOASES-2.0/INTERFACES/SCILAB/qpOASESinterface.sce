//
//	This file is part of qpOASES.
//
//	qpOASES -- An Implementation of the Online Active Set Strategy.
//	Copyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.
//
//	qpOASES is free software; you can redistribute it and/or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//	qpOASES is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//	Lesser General Public License for more details.
//
//	You should have received a copy of the GNU Lesser General Public
//	License along with qpOASES; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//



//
//	Filename:  INTERFACES/SCILAB/qpOASESinterface.sci
//	Author:    Holger Diedam, Hans Joachim Ferreau
//	Version:   2.0
//	Date:      2007-2009
//



sharedlib = link( './libqpOASESinterface.so' );
addinter( sharedlib, 'qpOASESgateway', ["qpOASES","qpOASES_init","qpOASES_initSB","qpOASES_initVM","qpOASES_hotstart","qpOASES_hotstartSB","qpOASES_hotstartVM","qpOASES_cleanup","qpOASES_cleanupSB","qpOASES_cleanupVM"] );



//
//	end of file
//
