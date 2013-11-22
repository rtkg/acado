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
 *    \file src/code_generation/export_data_argument.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#include <acado/code_generation/export_data_argument.hpp>
// #include <acado/code_generation/export_index.hpp>


BEGIN_NAMESPACE_ACADO


static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5
static char exportDataString[1024];


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportDataArgument::ExportDataArgument( )
{
	name = 0;
	typeString = 0;
	
	init( 0,0,0,0 );
}


ExportDataArgument::ExportDataArgument(	const char* const _name,
										const char* const _typeString,
										uint _nRows,
										uint _nCols,
										BooleanType _callItByValue,
										const ExportIndex& _addressIdx
										)
{
	name = 0;
	typeString = 0;

	init( _name,_typeString,_nRows,_nCols,_callItByValue,_addressIdx );
}


ExportDataArgument::ExportDataArgument(	const char* const _name,
										const char* const _typeString,
										const Matrix& _data,
										BooleanType _callItByValue,
										const ExportIndex& _addressIdx
										)
{
	name = 0;
	typeString = 0;

	init( _name,_typeString,_data,_callItByValue,_addressIdx );
}


// ExportDataArgument::ExportDataArgument(	const ExportIndex& _arg
// 										)
// {
// 	name = 0;
// 	typeString = 0;
// 
// 	init( _arg.name,_arg.typeString,1,1,0,BT_TRUE );
// }


ExportDataArgument::ExportDataArgument( const ExportDataArgument& arg )
{
	name = 0;
	typeString = 0;

	init( arg.name, arg.typeString, arg.data, arg.callItByValue , arg.addressIdx );
}


ExportDataArgument::~ExportDataArgument( )
{
	clear( );
}


ExportDataArgument& ExportDataArgument::operator=( const ExportDataArgument& arg )
{
	if( this != &arg )
	{
		init( arg.name, arg.typeString, arg.data, arg.callItByValue, arg.addressIdx );
	}

	return *this;
}


ExportDataArgument& ExportDataArgument::operator=( const Matrix& arg )
{
	init( "M", "double", arg );
	return *this;
}



returnValue ExportDataArgument::init(	const char* const _name,
										const char* const _typeString,
										uint _nRows,
										uint _nCols,
										BooleanType _callItByValue,
										const ExportIndex& _addressIdx
										)
{
	clear( );

	data.init( _nRows,_nCols );
	data.setAll( undefinedEntry );

	if ( _name != 0 )
		setName( _name );

	if ( _typeString != 0 )
		setTypeString( _typeString );
	
	callItByValue = _callItByValue;
	addressIdx  = _addressIdx;

	return SUCCESSFUL_RETURN;
}


returnValue ExportDataArgument::init(	const char* const _name,
										const char* const _typeString,
										const Matrix& _data,
										BooleanType _callItByValue,
										const ExportIndex& _addressIdx
										)
{
	clear( );

	data = _data;

	if ( _name != 0 )
		setName( _name );

	if ( _typeString != 0 )
		setTypeString( _typeString );
	
	callItByValue = _callItByValue;
	addressIdx  = _addressIdx;

	return SUCCESSFUL_RETURN;
}


ExportDataArgument ExportDataArgument::getAddress(	uint rowIdx,
													uint colIdx
													) const
{
	ASSERT( rowIdx < getNumRows() );
	ASSERT( colIdx < getNumCols() );

	ExportIndex tmpAddressIdx( "i" );
	tmpAddressIdx == rowIdx*getNumCols()+colIdx;

	ExportDataArgument tmp(	name,typeString,getNumRows(),getNumCols(),
							BT_FALSE,tmpAddressIdx );

	return tmp;
}


ExportDataArgument ExportDataArgument::getAddress(	const ExportIndex& rowIdx,
													uint colIdx
													) const
{
	if ( rowIdx.isGiven( ) )
	{
		ASSERT( rowIdx.getGivenValue() < (int)getNumRows() );
	}
	
	ASSERT( colIdx < getNumCols() );
	
	ExportDataArgument tmp(	name,typeString,getNumRows(),getNumCols(),
							BT_FALSE,rowIdx*getNumCols()+colIdx );

	return tmp;
}



const char* ExportDataArgument::getAddressString( ) const
{
	if ( addressIdx.isGiven() == BT_TRUE )
	{
		if ( addressIdx.getGivenValue() == 0 )
			sprintf( exportDataString,"%s",name );
		else
			sprintf( exportDataString,"&(%s[%d])",name,addressIdx.getGivenValue() );
	}
	else
	{
		sprintf( exportDataString,"&(%s[%s])",name,addressIdx.get() );
	}

	return exportDataString;
}



uint ExportDataArgument::getNumRows( ) const
{
	return data.getNumRows( );
}


uint ExportDataArgument::getNumCols( ) const
{
	return data.getNumCols( );
}


uint ExportDataArgument::getDim( ) const
{
	return data.getDim( );
}



BooleanType ExportDataArgument::isGiven( ) const
{
	for ( uint i=0; i<getNumRows(); ++i )
		for ( uint j=0; j<getNumCols(); ++j )
			if ( acadoIsEqual( data(i,j),undefinedEntry ) == BT_TRUE )
				return BT_FALSE;
			
	return BT_TRUE;
}


BooleanType ExportDataArgument::isCalledByValue( ) const
{
	return callItByValue;
}


returnValue ExportDataArgument::callByValue( )
{
	callItByValue = BT_TRUE;
	return SUCCESSFUL_RETURN;
}



returnValue	ExportDataArgument::setName(	const char* const _name
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


returnValue	ExportDataArgument::setTypeString(	const char* const _typeString
												)
{
	if ( _typeString == 0 )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( typeString == 0 )
		typeString = new char[MAX_LENGTH_NAME+1];

	strncpy( typeString,_typeString,MAX_LENGTH_NAME );
	typeString[MAX_LENGTH_NAME] = '\0';
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportDataArgument::exportDeclaration(	FILE *file,
													const char* _realString,
													const char* _intString,
													int _precision
													) const
{
	if ( ( isCalledByValue() == BT_TRUE ) && ( getDim() == 1 ) )
		acadoFPrintf( file,"%s %s;\n", typeString,name );
	else
		acadoFPrintf( file,"%s %s[%d];\n", typeString,name,getDim() );
	
	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportDataArgument::clear( )
{
	if ( name != 0 )
	{
		delete[] name;
		name = 0;
	}
	
	if ( typeString != 0 )
	{
		delete[] typeString;
		typeString = 0;
	}

	return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO

// end of file.
