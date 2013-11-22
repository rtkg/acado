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
 *    \file src/code_generation/export_function.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2011
 */

#include <acado/code_generation/export_function.hpp>
#include <acado/code_generation/export_function_call.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


ExportFunction::ExportFunction(	const char* _name,
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
	name = 0;
	functionReturnValue = 0;

	init( _name,_argument1,_argument2,_argument3,
				_argument4,_argument5,_argument6,
				_argument7,_argument8,_argument9 );
}


ExportFunction::ExportFunction( const ExportFunction& arg )
{
	name = 0;

	init( arg.name );
	
	functionArguments = arg.functionArguments;
	functionBody = arg.functionBody;
	
	if ( arg.functionReturnValue != 0 )
		setReturnValue( *(arg.functionReturnValue) );
}


ExportFunction::~ExportFunction( )
{
	clear( );
}


ExportFunction& ExportFunction::operator=( const ExportFunction& arg )
{
	if( this != &arg )
	{
		init( arg.name );
		
		functionArguments = arg.functionArguments;
		functionBody = arg.functionBody;
		
		if ( arg.functionReturnValue != 0 )
			setReturnValue( *(arg.functionReturnValue) );
	}

	return *this;
}



returnValue ExportFunction::init(	const char* _name,
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

	if ( _name != 0 )
		setName( _name );

	addArgument( 	_argument1,_argument2,_argument3,
					_argument4,_argument5,_argument6,
					_argument7,_argument8,_argument9 );

	return SUCCESSFUL_RETURN;
}



returnValue ExportFunction::addArgument(	const ExportDataArgument& _argument1,
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
	return functionArguments.addArgument( 	_argument1,_argument2,_argument3,
											_argument4,_argument5,_argument6,
											_argument7,_argument8,_argument9 );
}


returnValue ExportFunction::addStatement(	const ExportStatement& _statement
												)
{
	return functionBody.addStatement( _statement );
}


returnValue ExportFunction::addStatement(	const char* const _statementString
											)
{
	ExportStatementString tmp( _statementString );
	return addStatement( tmp );
}


returnValue ExportFunction::addFunctionCall(	const char* _fName,
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
	ExportFunctionCall tmp( _fName, _argument1,_argument2,_argument3,
							_argument4,_argument5,_argument6,
							_argument7,_argument8,_argument9 );

	return addStatement( tmp );
}


returnValue ExportFunction::addComment(	const char* const _statementString
										)
{
	char tmp[1024];
	sprintf( tmp,"/* %s */",_statementString );
	return addStatement( tmp );
}


returnValue ExportFunction::addLinebreak( )
{
	ExportStatementString tmp( "\n" );
	return addStatement( tmp );
}




returnValue ExportFunction::exportDataDeclaration(	FILE* file,
													const char* _realString,
													const char* _intString,
													int _precision
													) const
{
	return SUCCESSFUL_RETURN;
}


returnValue ExportFunction::exportForwardDeclaration(	FILE *file,
														const char* _realString,
														const char* _intString,
														int _precision
														) const
{
	if ( functionReturnValue != 0 )
		acadoFPrintf( file,"%s", functionReturnValue->typeString );
	else
		acadoFPrintf( file,"void" );
	
	acadoFPrintf( file," %s( ", name );
	functionArguments.exportCode( file,_realString,_intString,BT_TRUE );
	acadoFPrintf( file," );\n");

	return SUCCESSFUL_RETURN;
}


returnValue ExportFunction::exportCode(	FILE *file,
										const char* _realString,
										const char* _intString,
										int _precision
										) const
{
	if ( functionReturnValue != 0 )
		acadoFPrintf( file,"%s", functionReturnValue->typeString );
	else
		acadoFPrintf( file,"void" );
	
	acadoFPrintf( file," %s( ", name );
	functionArguments.exportCode( file,_realString,_intString,BT_TRUE );
	acadoFPrintf( file," ){\n");

	functionBody.exportDataDeclaration( file );
	acadoFPrintf( file,"\n");
	functionBody.exportCode( file );
	
	if ( functionReturnValue != 0 )
		acadoFPrintf( file,"return %s;\n", functionReturnValue->name );
	acadoFPrintf( file,"}\n\n");

	return SUCCESSFUL_RETURN;
}



returnValue ExportFunction::setReturnValue(	const ExportData& _functionReturnValue
											)
{
	if ( functionReturnValue != 0 )
		*functionReturnValue = _functionReturnValue;
	else
		functionReturnValue = new ExportData( _functionReturnValue );

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportFunction::clear( )
{
	if ( name != 0 )
	{
		delete[] name;
		name = 0;
	}
	
	if ( functionReturnValue != 0 )
	{
		delete functionReturnValue;
		functionReturnValue = 0;
	}

	return SUCCESSFUL_RETURN;
}



returnValue	ExportFunction::setName(	const char* const _name
										)
{
	if ( _name == 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( name == 0 )
		name = new char[MAX_LENGTH_NAME+1];

	strncpy( name,_name,MAX_LENGTH_NAME );
	name[MAX_LENGTH_NAME] = '\0';
	
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
