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
 *    \file src/code_generation/export_function_call.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010
 */

#include <acado/code_generation/export_function_call.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportFunctionCall::ExportFunctionCall(	const char* _fName,
										const ExportDataArgument& _argument1,
										const ExportDataArgument& _argument2,
										const ExportDataArgument& _argument3,
										const ExportDataArgument& _argument4,
										const ExportDataArgument& _argument5,
										const ExportDataArgument& _argument6,
										const ExportDataArgument& _argument7,
										const ExportDataArgument& _argument8,
										const ExportDataArgument& _argument9
										) : ExportStatement( )
{
	fName = 0;

	init( _fName,_argument1,_argument2,_argument3,
				 _argument4,_argument5,_argument6,
				 _argument7,_argument8,_argument9 );
}


ExportFunctionCall::ExportFunctionCall(	const ExportFunction& _f,
										const ExportDataArgument& _argument1,
										const ExportDataArgument& _argument2,
										const ExportDataArgument& _argument3,
										const ExportDataArgument& _argument4,
										const ExportDataArgument& _argument5,
										const ExportDataArgument& _argument6,
										const ExportDataArgument& _argument7,
										const ExportDataArgument& _argument8,
										const ExportDataArgument& _argument9
										) : ExportStatement( )
{
	fName = 0;

	init( _f,_argument1,_argument2,_argument3,
			 _argument4,_argument5,_argument6,
			 _argument7,_argument8,_argument9 );
}


ExportFunctionCall::ExportFunctionCall( const ExportFunctionCall& arg ) : ExportStatement( arg )
{
	fName = 0;
	init( arg.fName );

	f = arg.f;
	arguments = arg.arguments;
}


ExportFunctionCall::~ExportFunctionCall( )
{
	clear( );
}


ExportFunctionCall& ExportFunctionCall::operator=( const ExportFunctionCall& arg )
{
	if( this != &arg )
	{
		ExportStatement::operator=( arg );
		
		fName = 0;
		init( arg.fName );

		f = arg.f;
		arguments = arg.arguments;
	}

	return *this;
}

ExportStatement* ExportFunctionCall::clone( ) const
{
	return new ExportFunctionCall(*this);
}



returnValue ExportFunctionCall::init(	const char* _fName,
										const ExportDataArgument& _argument1,
										const ExportDataArgument& _argument2,
										const ExportDataArgument& _argument3,
										const ExportDataArgument& _argument4,
										const ExportDataArgument& _argument5,
										const ExportDataArgument& _argument6,
										const ExportDataArgument& _argument7,
										const ExportDataArgument& _argument8,
										const ExportDataArgument& _argument9
										)
{
	clear( );

	if ( _fName != 0 )
		setName( _fName );

	arguments.addArgument( 	_argument1,_argument2,_argument3,
							_argument4,_argument5,_argument6,
							_argument7,_argument8,_argument9 );

	return SUCCESSFUL_RETURN;
}


returnValue ExportFunctionCall::init(	const ExportFunction& _f,
										const ExportDataArgument& _argument1,
										const ExportDataArgument& _argument2,
										const ExportDataArgument& _argument3,
										const ExportDataArgument& _argument4,
										const ExportDataArgument& _argument5,
										const ExportDataArgument& _argument6,
										const ExportDataArgument& _argument7,
										const ExportDataArgument& _argument8,
										const ExportDataArgument& _argument9
										)
{
	clear( );

	f = _f;

	arguments.addArgument( 	_argument1,_argument2,_argument3,
							_argument4,_argument5,_argument6,
							_argument7,_argument8,_argument9 );

	return SUCCESSFUL_RETURN;
}



returnValue ExportFunctionCall::exportCode(	FILE *file,
											const char* _realString,
											const char* _intString,
											int _precision
											) const
{
	if ( fName != 0 )
	{
		acadoFPrintf( file,"%s( ", fName );
		arguments.exportCode( file,_realString,_intString,BT_FALSE );
		acadoFPrintf( file," );\n");
	}
	else
	{
		acadoFPrintf( file,"%s( ", f.name );
		arguments.exportCode( file,_realString,_intString,BT_FALSE );
		acadoFPrintf( file," );\n");
	}

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportFunctionCall::clear( )
{
	if ( fName != 0 )
	{
		delete[] fName;
		fName = 0;
	}
	
	arguments.clear( );

	return SUCCESSFUL_RETURN;
}



returnValue	ExportFunctionCall::setName(	const char* const _fName
											)
{
	if ( _fName == 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( fName == 0 )
		fName = new char[MAX_LENGTH_NAME+1];

	strncpy( fName,_fName,MAX_LENGTH_NAME );
	fName[MAX_LENGTH_NAME] = '\0';
	
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
