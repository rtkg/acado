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
 *    \file src/code_generation/export_data.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010
 */


#include <acado/code_generation/export_data.hpp>
#include <acado/code_generation/export_arithmetic_statement.hpp>


BEGIN_NAMESPACE_ACADO


static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5
static char exportDataString[1024];


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportData::ExportData( ) : ExportDataArgument( )
{
	rowOffset.init( "run1" );
	colOffset.init( "run2" );
	
	setSubmatrixOffsets( 0,0,0,0,0 );
}


ExportData::ExportData(	const char* const _name,
						const char* const _typeString,
						uint _nRows,
						uint _nCols,
						BooleanType _callItByValue
						) : ExportDataArgument( _name,_typeString,_nRows,_nCols,_callItByValue )
{
	rowOffset.init( "run1" );
	colOffset.init( "run2" );
	
	setSubmatrixOffsets( 0,0,_nCols,0,0 );
}


ExportData::ExportData(	const char* const _name,
						const char* const _typeString,
						const Matrix& _data,
						BooleanType _callItByValue
						) : ExportDataArgument( _name,_typeString,_data,_callItByValue )
{
	rowOffset.init( "run1" );
	colOffset.init( "run2" );
	
	setSubmatrixOffsets( 0,0,_data.getNumCols(),0,0 );
}


ExportData::ExportData( const ExportData& arg ) : ExportDataArgument( arg )
{
	setSubmatrixOffsets( arg.rowOffset, arg.colOffset, arg.colDim, arg.nRows, arg.nCols );
}


ExportData::~ExportData( )
{
}


ExportData& ExportData::operator=( const ExportData& arg )
{
	if( this != &arg )
	{
		ExportDataArgument::operator=( arg );
		setSubmatrixOffsets( arg.rowOffset, arg.colOffset, arg.colDim, arg.nRows, arg.nCols );
	}

	return *this;
}


ExportData& ExportData::operator=(	const Matrix& arg
									)
{
	ExportDataArgument::operator=( arg );
	setSubmatrixOffsets( 0,0,0,0,0 );
	
	return *this;
}


double& ExportData::operator()(	uint rowIdx,
								uint colIdx
								)
{
	return data( rowIdx,colIdx );
}


double ExportData::operator()(	uint rowIdx,
								uint colIdx
								) const
{
	return data( rowIdx,colIdx );
}



BooleanType ExportData::isZero(	const ExportIndex& rowIdx,
								const ExportIndex& colIdx
								) const
{
	if ( ( rowIdx.isGiven() == BT_TRUE ) && ( colIdx.isGiven() == BT_TRUE ) )
		return acadoIsEqual( operator()( rowIdx.getGivenValue(),colIdx.getGivenValue() ),0.0 );
	else
		return BT_FALSE;
}


BooleanType ExportData::isZero(	const ExportIndex& rowIdx,
								uint colIdx
								) const
{
	if ( rowIdx.isGiven() == BT_TRUE )
		return acadoIsEqual( operator()(rowIdx.getGivenValue(),colIdx),0.0 );
	else
		return BT_FALSE;
}


BooleanType ExportData::isZero(	uint rowIdx,
								const ExportIndex& colIdx
								) const
{
	if ( colIdx.isGiven() == BT_TRUE )
		return acadoIsEqual( operator()(rowIdx,colIdx.getGivenValue()),0.0 );
	else
		return BT_FALSE;
}


BooleanType ExportData::isZero(	uint rowIdx,
								uint colIdx
								) const
{
	return acadoIsEqual( operator()(rowIdx,colIdx),0.0 );
}



BooleanType ExportData::isOne(	const ExportIndex& rowIdx,
								const ExportIndex& colIdx
								) const
{
	if ( ( rowIdx.isGiven() == BT_TRUE ) && ( colIdx.isGiven() == BT_TRUE ) )
		return acadoIsEqual( operator()(rowIdx.getGivenValue(),colIdx.getGivenValue()),1.0 );
	else
		return BT_FALSE;
}


BooleanType ExportData::isOne(	const ExportIndex& rowIdx,
								uint colIdx
								) const
{
	if ( rowIdx.isGiven() == BT_TRUE )
		return acadoIsEqual( operator()(rowIdx.getGivenValue(),colIdx),1.0 );
	else
		return BT_FALSE;
}


BooleanType ExportData::isOne(	uint rowIdx,
								const ExportIndex& colIdx
								) const
{
	if ( colIdx.isGiven() == BT_TRUE )
		return acadoIsEqual( operator()(rowIdx,colIdx.getGivenValue()),1.0 );
	else
		return BT_FALSE;
}


BooleanType ExportData::isOne(	uint rowIdx,
								uint colIdx
								) const
{
	return acadoIsEqual( operator()(rowIdx,colIdx),1.0 );
}



const char* ExportData::get(	const ExportIndex& rowIdx,
								const ExportIndex& colIdx
								) const
{
	ExportIndex totalIdx = (rowIdx+rowOffset)*colDim + (colIdx+colOffset);
	
	if ( ( totalIdx.isGiven() == BT_TRUE ) && ( rowIdx.isGiven() == BT_TRUE ) && ( colIdx.isGiven() == BT_TRUE ) )
	{
		if ( acadoIsEqual( operator()( rowIdx.getGivenValue(),colIdx.getGivenValue() ),undefinedEntry ) == BT_TRUE )
		{
			if ( ( isCalledByValue() == BT_TRUE ) && ( totalIdx.getGivenValue() == 0 ) )
				sprintf( exportDataString,"%s",name );
			else
				sprintf( exportDataString,"%s[%d]",name, totalIdx.getGivenValue() );
		}
		else
			sprintf( exportDataString,"%.16e",operator()( rowIdx.getGivenValue(),colIdx.getGivenValue() ) );
	}
	else
	{
		sprintf( exportDataString,"%s[%s]", name,totalIdx.get( ) );
	}

	return exportDataString;
}


const char* ExportData::get(	const ExportIndex& rowIdx,
								uint colIdx
								) const
{
	ExportIndex tmpColIdx( "run2" );
	tmpColIdx == colIdx;
	
	return get( rowIdx,tmpColIdx );
}


const char* ExportData::get(	uint rowIdx,
								const ExportIndex& colIdx
								) const
{
	ExportIndex tmpRowIdx( "run1" );
	tmpRowIdx == rowIdx;

	return get( tmpRowIdx,colIdx );
}


const char* ExportData::get(	uint rowIdx,
								uint colIdx
								) const
{
	if ( ( rowOffset.isGiven( ) == BT_TRUE ) && ( colOffset.isGiven( ) == BT_TRUE ) )
	{
		int totalIdx = (rowIdx+rowOffset.getGivenValue())*colDim + (colIdx+colOffset.getGivenValue());
		
		if ( acadoIsEqual( operator()(rowIdx,colIdx),undefinedEntry ) == BT_TRUE )
		{
			if ( ( isCalledByValue() == BT_TRUE ) && ( totalIdx == 0 ) )
				sprintf( exportDataString,"%s",name );
			else
				sprintf( exportDataString,"%s[%d]",name,totalIdx );
		}
		else
			sprintf( exportDataString,"%.16e",operator()(rowIdx,colIdx) );

		return exportDataString;
	}
	else
	{
// 		ASSERT( 1==0 );
		ExportIndex tmpRowIdx( "run1" );
		tmpRowIdx == rowIdx;
		
		ExportIndex tmpColIdx( "run2" );
		tmpColIdx == colIdx;

		return get( tmpRowIdx,tmpColIdx );
	}
}



uint ExportData::getNumRows( ) const
{
	if ( nRows > 0 )
		return nRows;
	else
		return data.getNumRows( );
}


uint ExportData::getNumCols( ) const
{
	if ( nCols > 0 )
		return nCols;
	else
		return data.getNumCols( );
}


uint ExportData::getDim( ) const
{
	return ( getNumRows()*getNumCols() );
}




ExportArithmeticStatement ExportData::operator+(	const ExportData& arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( 0,ESO_ASSIGN,this,ESO_ADD,&arg );
}


ExportArithmeticStatement ExportData::operator-(	const ExportData& arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( 0,ESO_ASSIGN,this,ESO_SUBTRACT,&arg );
}


ExportArithmeticStatement ExportData::operator+=(	const ExportData& arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( this,ESO_ASSIGN,&arg,ESO_ADD_ASSIGN,0 );
}


ExportArithmeticStatement ExportData::operator-=(	const ExportData& arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( this,ESO_ASSIGN,&arg,ESO_SUBTRACT_ASSIGN,0 );
}


ExportArithmeticStatement ExportData::operator*(	const ExportData& arg
													) const
{
	ASSERT( getNumCols() == arg.getNumRows() );
	return ExportArithmeticStatement( 0,ESO_ASSIGN,this,ESO_MULTIPLY,&arg );
}


ExportArithmeticStatement ExportData::operator^(	const ExportData& arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	return ExportArithmeticStatement( 0,ESO_ASSIGN,this,ESO_MULTIPLY_TRANSPOSE,&arg );
}


ExportArithmeticStatement ExportData::operator==(	const ExportData& arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( this,ESO_ASSIGN,&arg,ESO_ASSIGN,0 );
}


ExportArithmeticStatement ExportData::operator==(	ExportArithmeticStatement arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( this,ESO_ASSIGN,arg.rhs1,arg.op1,arg.rhs2,arg.op2,arg.rhs3 );
}


ExportArithmeticStatement ExportData::operator+(	ExportArithmeticStatement arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || ( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) );

	return ExportArithmeticStatement( 0,ESO_UNDEFINED,arg.rhs1,arg.op1,arg.rhs2,ESO_ADD,this );
}


ExportArithmeticStatement ExportData::operator-(	ExportArithmeticStatement arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || ( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) );

	return ExportArithmeticStatement( 0,ESO_UNDEFINED,arg.rhs1,arg.op1,arg.rhs2,ESO_SUBTRACT,this );
}


ExportArithmeticStatement ExportData::operator+=(	ExportArithmeticStatement arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || ( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) );

	return ExportArithmeticStatement( this,ESO_ADD_ASSIGN,arg.rhs1,arg.op1,arg.rhs2,arg.op2,arg.rhs3 );
}


ExportArithmeticStatement ExportData::operator-=(	ExportArithmeticStatement arg
													) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || ( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) );

	return ExportArithmeticStatement( this,ESO_SUBTRACT_ASSIGN,arg.rhs1,arg.op1,arg.rhs2,arg.op2,arg.rhs3 );
}



ExportArithmeticStatement ExportData::operator+(	const Matrix& arg
												) const
{
	ExportData tmp( "tmp","double",arg );
	return operator+( tmp );
}


ExportArithmeticStatement ExportData::operator-(	const Matrix& arg
												) const
{
	ExportData tmp( "tmp","double",arg );
	return operator-( tmp );
}


ExportArithmeticStatement ExportData::operator+=(	const Matrix& arg
												) const
{
	ExportData tmp( "tmp","double",arg );
	return operator+=( tmp );
}


ExportArithmeticStatement ExportData::operator-=(	const Matrix& arg
												) const
{
	ExportData tmp( "tmp","double",arg );
	return operator-=( tmp );
}


ExportArithmeticStatement ExportData::operator*(	const Matrix& arg
												) const
{
	ExportData tmp( "tmp","double",arg );
	return operator*( tmp );
}


ExportArithmeticStatement ExportData::operator^(	const Matrix& arg
												) const
{
	ExportData tmp( "tmp","double",arg );
	return operator^( tmp );
}


ExportArithmeticStatement ExportData::operator==(	const Matrix& arg
												) const
{
	ExportData tmp( "tmp","double",arg );
	return operator==( tmp );
}



ExportData ExportData::getTranspose( ) const
{
	ExportData transposed( name,typeString,getNumCols(),getNumRows() );
	
	for( uint i=0; i<getNumCols(); ++i )
		for( uint j=0; j<getNumRows(); ++j )
			transposed( i,j ) = operator()( j,i );
		
	return transposed;
}



ExportData ExportData::getRow(	uint idx
								) const
{
	ExportData tmp( name,typeString,1,getNumCols( ) );
	
	for( uint i=0; i<getNumCols(); ++i )
		tmp( 0,i ) = operator()( idx,i );
	
	tmp.setSubmatrixOffsets( idx,0,getNumCols( ) );

	return tmp;
}


ExportData ExportData::getRow(	const ExportIndex& idx
								) const
{
	ExportData tmp( *this );
	ExportIndex tmpColOffset( idx.name );
	tmpColOffset == 0;
	
	tmp.setSubmatrixOffsets( idx,tmpColOffset,getNumCols( ), 1,getNumCols( ) );

	return tmp;
}


ExportData ExportData::getCol(	uint idx
								) const
{
	ExportData tmp( name,typeString,getNumRows( ),1 );
	
	for( uint i=0; i<getNumRows(); ++i )
		tmp( i,0 ) = operator()( i,idx );

	tmp.setSubmatrixOffsets( 0,idx,getNumCols( ) );

	return tmp;
}


ExportData ExportData::getCol(	const ExportIndex& idx
								) const
{
	ExportData tmp = *this;
	ExportIndex tmpRowOffset( idx.name );
	tmpRowOffset == 0;

	tmp.setSubmatrixOffsets( tmpRowOffset,idx,getNumCols( ), getNumRows(),1 );

	return tmp;
}



ExportData ExportData::getRows(	uint idx1,
								uint idx2
								) const
{
	ASSERT( 0 <= idx1 );
	ASSERT( idx1 <= idx2 );
	ASSERT( idx1 <= getNumRows( ) );

	ExportData tmp( name,typeString,idx2-idx1,getNumCols( ) ); // idx2-idx1+1
	
	for( uint i=idx1; i<idx2; ++i ) // <idx2
		for( uint j=0; j<getNumCols(); ++j )
			tmp( i-idx1,j ) = operator()( i,j );

	tmp.setSubmatrixOffsets( idx1,0,getNumCols( ) );

	return tmp;
}


ExportData ExportData::getRows(	const ExportIndex& idx1,
								const ExportIndex& idx2
								) const
{
	ASSERT( (idx2-idx1).isGiven() == BT_TRUE );
	
	if ( idx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= idx1.getGivenValue() );
		ASSERT( idx1.getGivenValue() <= idx2.getGivenValue() );
		ASSERT( idx1.getGivenValue() <= (int)getNumRows( ) );
	}

	ExportData tmp( *this );
	ExportIndex tmpColOffset( idx1.name );
	tmpColOffset == 0;

	tmp.setSubmatrixOffsets( idx1,tmpColOffset,getNumCols( ), (idx2-idx1).getGivenValue(),getNumCols( ) );
	return tmp;
}



ExportData ExportData::getCols(	uint idx1,
								uint idx2
								) const
{
	ASSERT( 0 <= idx1 );
	ASSERT( idx1 <= idx2 );
	ASSERT( idx1 <= getNumCols( ) );

	ExportData tmp( name,typeString,getNumRows( ),idx2-idx1 );// idx2-idx1+1
	
	for( uint i=0; i<getNumRows(); ++i )
		for( uint j=idx1; j<idx2; ++j )// <idx2
			tmp( i,j-idx1 ) = operator()( i,j );

	tmp.setSubmatrixOffsets( 0,idx1,getNumCols( ) );

	return tmp;
}


ExportData ExportData::getCols(	const ExportIndex& idx1,
								const ExportIndex& idx2
								) const
{
	ASSERT( (idx2-idx1).isGiven() == BT_TRUE );
	
	if ( idx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= idx1.getGivenValue() );
		ASSERT( idx1.getGivenValue() <= idx2.getGivenValue() );
		ASSERT( idx1.getGivenValue() <= (int)getNumCols( ) );
	}

	ExportData tmp = *this;
	ExportIndex tmpRowOffset( idx1.name );
	tmpRowOffset == 0;

	tmp.setSubmatrixOffsets( tmpRowOffset,idx1,getNumCols( ), getNumRows( ),(idx2-idx1).getGivenValue() );
	return tmp;
}


ExportData ExportData::getSubMatrix(	uint rowIdx1,
										uint rowIdx2,
										uint colIdx1,
										uint colIdx2
										) const
{
	ASSERT( 0 <= rowIdx1 );
	ASSERT( rowIdx1 <= rowIdx2 );
	ASSERT( rowIdx1 <= getNumRows( ) );
	ASSERT( 0 <= colIdx1 );
	ASSERT( colIdx1 <= colIdx2 );
	ASSERT( colIdx1 <= getNumCols( ) );

	ExportData tmp( name,typeString,rowIdx2-rowIdx1,colIdx2-colIdx1 );
	
	for( uint i=rowIdx1; i<rowIdx2; ++i )
		for( uint j=colIdx1; j<colIdx2; ++j )
			tmp( i-rowIdx1,j-colIdx1 ) = operator()( i,j );

	tmp.setSubmatrixOffsets( rowIdx1,colIdx1,getNumCols( ) );

	return tmp;
}


ExportData ExportData::getSubMatrix(	const ExportIndex& rowIdx1,
										const ExportIndex& rowIdx2,
										uint colIdx1,
										uint colIdx2
										) const
{
	ASSERT( (rowIdx2-rowIdx1).isGiven() == BT_TRUE );
	
	if ( rowIdx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= rowIdx1.getGivenValue() );
		ASSERT( rowIdx1.getGivenValue() <= rowIdx2.getGivenValue() );
		ASSERT( rowIdx1.getGivenValue() <= (int)getNumRows( ) );
	}

	ASSERT( 0 <= colIdx1 );
	ASSERT( colIdx1 <= colIdx2 );
	ASSERT( colIdx1 <= getNumCols( ) );

	ExportData tmp( *this );
	ExportIndex tmpColOffset( rowIdx1.name );
	tmpColOffset == colIdx1;

	tmp.setSubmatrixOffsets( rowIdx1,tmpColOffset,getNumCols( ), (rowIdx2-rowIdx1).getGivenValue(),colIdx2-colIdx1 );

	return tmp;
}


ExportData ExportData::getSubMatrix(	uint rowIdx1,
										uint rowIdx2,
										const ExportIndex& colIdx1,
										const ExportIndex& colIdx2
										) const
{
	ASSERT( (colIdx2-colIdx1).isGiven() == BT_TRUE );
	
	ASSERT( 0 <= rowIdx1 );
	ASSERT( rowIdx1 <= rowIdx2 );
	ASSERT( rowIdx1 <= getNumRows( ) );
	
	if ( colIdx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= colIdx1.getGivenValue() );
		ASSERT( colIdx1.getGivenValue() <= colIdx2.getGivenValue() );
		ASSERT( colIdx1.getGivenValue() <= (int)getNumCols( ) );
	}

	ExportData tmp = *this;
	ExportIndex tmpRowOffset( colIdx1.name );
	tmpRowOffset == rowIdx1;

	tmp.setSubmatrixOffsets( tmpRowOffset,colIdx1,getNumCols( ), rowIdx2-rowIdx1,(colIdx2-colIdx1).getGivenValue() );

	return tmp;
}


ExportData ExportData::getSubMatrix(	const ExportIndex& rowIdx1,
										const ExportIndex& rowIdx2,
										const ExportIndex& colIdx1,
										const ExportIndex& colIdx2
										) const
{
	ASSERT( (rowIdx2-rowIdx1).isGiven() == BT_TRUE );
	ASSERT( (colIdx2-colIdx1).isGiven() == BT_TRUE );
	
	if ( rowIdx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= rowIdx1.getGivenValue() );
		ASSERT( rowIdx1.getGivenValue() <= rowIdx2.getGivenValue() );
		ASSERT( rowIdx1.getGivenValue() <= (int)getNumRows( ) );
	}

	if ( colIdx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= colIdx1.getGivenValue() );
		ASSERT( colIdx1.getGivenValue() <= colIdx2.getGivenValue() );
		ASSERT( colIdx1.getGivenValue() <= (int)getNumCols( ) );
	}

	ExportData tmp( *this );

	tmp.setSubmatrixOffsets( rowIdx1,colIdx1,getNumCols( ), (rowIdx2-rowIdx1).getGivenValue(),(colIdx2-colIdx1).getGivenValue() );

	return tmp;
}



ExportData ExportData::makeRowVector( ) const
{
	ASSERT( ( nRows == 0 ) && ( nCols == 0 ) );
	
	ExportData tmp( name,typeString,1,getDim() );
	
	for ( uint i=0; i<getNumRows(); ++i )
		for ( uint j=0; j<getNumCols(); ++j )
			tmp( 0,i*getNumCols()+j ) = operator()( i,j );
	
	return tmp;
}


ExportData ExportData::makeColVector( ) const
{
	ASSERT( ( nRows == 0 ) && ( nCols == 0 ) );
	
	ExportData tmp( name,typeString,getDim(),1 );
	
	for ( uint i=0; i<getNumRows(); ++i )
		for ( uint j=0; j<getNumCols(); ++j )
			tmp( i*getNumCols()+j,0 ) = operator()( i,j );
	
	return tmp;
}



BooleanType ExportData::isVector( ) const
{
	if ( ( getNumRows( ) == 1 ) || ( getNumCols( ) == 1 ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}



returnValue ExportData::print( ) const
{
	return data.print( name );
}

//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportData::setSubmatrixOffsets(	uint _rowOffset,
												uint _colOffset,
												uint _colDim,
												uint _nRows,
												uint _nCols
												)
{
	if ( ( _rowOffset < 0 ) || ( _colOffset < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	
	if ( _colOffset > _colDim )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	rowOffset == _rowOffset;
	colOffset == _colOffset;
	colDim    = _colDim;
	
	nRows = _nRows;
	nCols = _nCols;
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportData::setSubmatrixOffsets(	const ExportIndex& _rowOffset,
												const ExportIndex& _colOffset,
												uint _colDim,
												uint _nRows,
												uint _nCols
												)
{
	if ( ( _rowOffset.isGiven() == BT_TRUE ) && ( _rowOffset.getGivenValue() < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	
	if ( ( _colOffset.isGiven() == BT_TRUE ) && ( _colOffset.getGivenValue() < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	
	if ( ( _colOffset.isGiven() == BT_TRUE ) && ( _colOffset.getGivenValue() > (int)_colDim ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	rowOffset = _rowOffset;
	colOffset = _colOffset;
	colDim    = _colDim;
	
	nRows = _nRows;
	nCols = _nCols;
	
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
