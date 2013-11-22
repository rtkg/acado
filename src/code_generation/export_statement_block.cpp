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
 *    \file src/dynamic_system/dynamic_system.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/code_generation/export_statement_block.hpp>
#include <acado/code_generation/export_function_call.hpp>
#include <acado/code_generation/export_statement_string.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportStatementBlock::ExportStatementBlock( ) : ExportStatement( )
{
	nStatements = 0;
	statements  = 0;
}


ExportStatementBlock::ExportStatementBlock( const ExportStatementBlock& arg ) : ExportStatement( arg )
{
	nStatements = arg.nStatements;

	if ( arg.statements != 0 )
	{
		statements = (ExportStatement**) calloc( nStatements,sizeof(ExportStatement*) );
		for( uint i=0; i<nStatements; ++i )
			statements[i] = (arg.statements[i])->clone( );
	}
	else
	{
		nStatements = 0;
		statements = 0;
	}
}


ExportStatementBlock::~ExportStatementBlock( )
{
	clear( );
}


ExportStatementBlock& ExportStatementBlock::operator=( const ExportStatementBlock& arg )
{
	if ( this != &arg )
	{
		clear( );

		ExportStatement::operator=( arg );

		nStatements = arg.nStatements;
	
		if ( arg.statements != 0 )
		{
			statements = (ExportStatement**) calloc( nStatements,sizeof(ExportStatement*) );
			for( uint i=0; i<nStatements; ++i )
				statements[i] = (arg.statements[i])->clone( );
		}
		else
		{
			nStatements = 0;
			statements = 0;
		}
	}

	return *this;
}


ExportStatement* ExportStatementBlock::clone( ) const
{
	return new ExportStatementBlock(*this);
}



returnValue ExportStatementBlock::addStatement(	const ExportStatement& _statement
												)
{
	++nStatements;

	statements = (ExportStatement**) realloc( statements,nStatements*sizeof(ExportStatement*) );
	statements[ nStatements-1 ] = _statement.clone( );
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportStatementBlock::addStatement(	const char* const _statementString
												)
{
	ExportStatementString tmp( _statementString );
	return addStatement( tmp );
}


returnValue ExportStatementBlock::addFunctionCall(	const char* _fName,
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


returnValue ExportStatementBlock::addComment(	const char* const _statementString
												)
{
	char tmp[1024];
	sprintf( tmp,"/* %s */",_statementString );
	return addStatement( tmp );
}


returnValue ExportStatementBlock::addLinebreak( )
{
	ExportStatementString tmp( "\n" );
	return addStatement( tmp );
}



uint ExportStatementBlock::getNumStatements( ) const
{
	return nStatements;
}



returnValue ExportStatementBlock::exportDataDeclaration(	FILE *file,
															const char* _realString,
															const char* _intString,
															int _precision
															) const
{
	for( uint i=0; i<nStatements; ++i )
	{
		if ( statements[i]->exportDataDeclaration( file,_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );
	}

	return SUCCESSFUL_RETURN;
}



returnValue ExportStatementBlock::exportCode(	FILE* file,
												const char* _realString,
												const char* _intString,
												int _precision
												) const
{
	for( uint i=0; i<nStatements; ++i )
	{
		if ( statements[i]->exportCode( file,_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );
		
// 		if ( i<nStatements-1 )
// 			acadoFPrintf( file,"\n" );
	}

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportStatementBlock::clear( )
{
	if ( statements != 0 )
	{
		for( uint i=0; i<nStatements; ++i )
			delete statements[i];
		free( statements );
		
		nStatements = 0;
	}
	
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
