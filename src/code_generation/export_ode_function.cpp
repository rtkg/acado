/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



/**
 *    \file src/code_generation/export_ode_function.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2011
 */

#include <acado/code_generation/export_ode_function.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportODEfunction::ExportODEfunction( ) : ExportFunction( )
{
}


ExportODEfunction::ExportODEfunction(	const DifferentialEquation& _f,
										const char* _name
										) : ExportFunction( _name )
{
	f = _f;
}


ExportODEfunction::ExportODEfunction( const ExportODEfunction& arg ) : ExportFunction( arg )
{
	f = arg.f;
}


ExportODEfunction::~ExportODEfunction( )
{
}


ExportODEfunction& ExportODEfunction::operator=( const ExportODEfunction& arg )
{
	if( this != &arg )
	{
		ExportFunction::operator=( arg );
		f = arg.f;
	}

	return *this;
}



returnValue ExportODEfunction::init(	const DifferentialEquation& _f,
										const char* _name
										)
{
	f = _f;
	return ExportFunction::init( _name );
}



returnValue ExportODEfunction::exportDataDeclaration(	FILE* file,
														const char* _realString,
														const char* _intString,
														int _precision
														) const
{
	return f.exportHeader( file,name,_realString );
}


returnValue ExportODEfunction::exportForwardDeclaration(	FILE* file,
															const char* _realString,
															const char* _intString,
															int _precision
															) const
{
	return f.exportForwardDeclarations( file,name,_realString );
}


returnValue ExportODEfunction::exportDefinition(	FILE* file,
													const char* _realString,
													const char* _intString,
													int _precision
													) const
{
	return f.exportCode( file,name,_realString );
}



//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
