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
 *    \file src/code_generation/export_arithmetic_statement.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2011
 */

#include <acado/code_generation/export_arithmetic_statement.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportArithmeticStatement::ExportArithmeticStatement( )
{
	lhs  = 0;
	rhs1 = 0;
	rhs2 = 0;
	rhs3 = 0;

	op0 = ESO_UNDEFINED;
	op1 = ESO_UNDEFINED;
	op2 = ESO_UNDEFINED;
	
	outerLoopVariable == -1;
}


ExportArithmeticStatement::ExportArithmeticStatement(	const ExportData* const _lhs,
														ExportStatementOperator _op0,
														const ExportData* const _rhs1,
														ExportStatementOperator _op1,
														const ExportData* const _rhs2,
														ExportStatementOperator _op2,
														const ExportData* const _rhs3
														) : ExportStatement( )
{
	ASSERT( ( _op0 == ESO_UNDEFINED ) || ( _op0 == ESO_ASSIGN ) || ( _op0 == ESO_ADD_ASSIGN ) || ( _op0 == ESO_SUBTRACT_ASSIGN ) );
	ASSERT( ( _op2 == ESO_UNDEFINED ) || ( _op2 == ESO_ADD ) || ( _op2 == ESO_SUBTRACT ) );

	if ( _lhs != 0 )
		lhs  = new ExportData( *_lhs );
	else
		lhs = 0;
	
	if ( _rhs1 != 0 )
		rhs1  = new ExportData( *_rhs1 );
	else
		rhs1 = 0;
	
	if ( _rhs2 != 0 )
		rhs2  = new ExportData( *_rhs2 );
	else
		rhs2 = 0;

	if ( _rhs3 != 0 )
		rhs3  = new ExportData( *_rhs3 );
	else
		rhs3 = 0;

	op0  = _op0;
	op1  = _op1;
	op2  = _op2;
	
	outerLoopVariable == -1;
}


ExportArithmeticStatement::ExportArithmeticStatement( const ExportArithmeticStatement& arg ) : ExportStatement( arg )
{
	if ( arg.lhs != 0 )
		lhs  = new ExportData( *(arg.lhs) );
	else
		lhs = 0;
	
	if ( arg.rhs1 != 0 )
		rhs1  = new ExportData( *(arg.rhs1) );
	else
		rhs1 = 0;
	
	if ( arg.rhs2 != 0 )
		rhs2  = new ExportData( *(arg.rhs2) );
	else
		rhs2 = 0;
	
	if ( arg.rhs3 != 0 )
		rhs3  = new ExportData( *(arg.rhs3) );
	else
		rhs3 = 0;

	op0  = arg.op0;
	op1  = arg.op1;
	op2  = arg.op2;
	
	outerLoopVariable = arg.outerLoopVariable;
}


ExportArithmeticStatement::~ExportArithmeticStatement( )
{
	if ( lhs != 0 )
		delete lhs;

	if ( rhs1 != 0 )
		delete rhs1;

	if ( rhs2 != 0 )
		delete rhs2;
	
	if ( rhs3 != 0 )
		delete rhs3;
}


ExportArithmeticStatement& ExportArithmeticStatement::operator=( const ExportArithmeticStatement& arg )
{
	if( this != &arg )
	{
		if ( lhs != 0 )
			delete lhs;

		if ( rhs1 != 0 )
			delete rhs1;

		if ( rhs2 != 0 )
			delete rhs2;

		if ( rhs3 != 0 )
			delete rhs3;

		ExportStatement::operator=( arg );
		
		if ( arg.lhs != 0 )
			lhs  = new ExportData( *(arg.lhs) );
		else
			lhs = 0;
		
		if ( arg.rhs1 != 0 )
			rhs1  = new ExportData( *(arg.rhs1) );
		else
			rhs1 = 0;
		
		if ( arg.rhs2 != 0 )
			rhs2  = new ExportData( *(arg.rhs2) );
		else
			rhs2 = 0;

		if ( arg.rhs3 != 0 )
			rhs3  = new ExportData( *(arg.rhs3) );
		else
			rhs3 = 0;

		op0  = arg.op0;
		op1  = arg.op1;
		op2  = arg.op2;
		
		outerLoopVariable = arg.outerLoopVariable;
	}

	return *this;
}


ExportStatement* ExportArithmeticStatement::clone( ) const
{
	return new ExportArithmeticStatement(*this);
}


uint ExportArithmeticStatement::getNumRows( ) const
{
	if ( rhs1 == 0 )
		return 0;
	else
	{
		if ( op1 != ESO_MULTIPLY_TRANSPOSE )
			return rhs1->getNumRows( );
		else
			return rhs1->getNumCols( );
	}
}


uint ExportArithmeticStatement::getNumCols( ) const
{
	if ( rhs1 == 0 )
		return 0;
	else
	{
		if ( rhs2 == 0 )
			return rhs1->getNumCols( );
		else
			return rhs2->getNumCols( );
	}
}


returnValue ExportArithmeticStatement::exportDataDeclaration(	FILE *file,
																const char* _realString,
																const char* _intString,
																int _precision
																) const
{
	if ( outerLoopVariable.isGiven() == BT_FALSE )
		acadoFPrintf( file,"%s %s;\n", _intString,outerLoopVariable.name );
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportArithmeticStatement::exportCode(	FILE* file,
													const char* _realString,
													const char* _intString,
													int _precision
													) const
{
// 	if ( lhs == 0 )
// 		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );
	
	switch( op1 )
	{
		case ESO_ADD:
			return exportCodeAddSubtract( file,"+",_realString,_intString,_precision );

		case ESO_SUBTRACT:
			return exportCodeAddSubtract( file,"-",_realString,_intString,_precision );
			
		case ESO_ADD_ASSIGN:
			return exportCodeAssign( file,"+=",_realString,_intString,_precision );

		case ESO_SUBTRACT_ASSIGN:
			return exportCodeAssign( file,"-=",_realString,_intString,_precision );

		case ESO_MULTIPLY:
			return exportCodeMultiply( file,BT_FALSE,_realString,_intString,_precision );
			
		case ESO_MULTIPLY_TRANSPOSE:
			return exportCodeMultiply( file,BT_TRUE,_realString,_intString,_precision );

		case ESO_ASSIGN:
			return exportCodeAssign( file,"=",_realString,_intString,_precision );

		default:
			return ACADOERROR( RET_UNKNOWN_BUG );
	}
	
	return ACADOERROR( RET_UNKNOWN_BUG );
}


ExportArithmeticStatement& ExportArithmeticStatement::unrollOuterLoop( )
{
	outerLoopVariable.init( "run1" );
	return *this;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportArithmeticStatement::exportCodeAddSubtract(	FILE* file,
																const char* const _sign,
																const char* _realString,
																const char* _intString,
																int _precision
																) const
{
	if ( ( rhs1 == 0 ) || ( rhs2 == 0 ) )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	if ( ( rhs1->getNumRows() != rhs2->getNumRows() ) || 
		 ( rhs1->getNumCols() != rhs2->getNumCols() ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
	
	if ( lhs != 0 )
	{
		if ( ( rhs1->getNumRows() != lhs->getNumRows() ) || 
		     ( rhs1->getNumCols() != lhs->getNumCols() ) )
			return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
	}
	
	for( uint i=0; i<getNumRows( ); ++i )
		for( uint j=0; j<getNumCols( ); ++j )
		{
			if ( lhs != 0 )
				acadoFPrintf( file,"%s = ", lhs->get(i,j) );

			if ( rhs1->isZero(i,j) == BT_FALSE )
			{
				acadoFPrintf( file,"%s", rhs1->get(i,j) );
				if ( rhs2->isZero(i,j) == BT_FALSE )
					acadoFPrintf( file," %s %s;\n", _sign,rhs2->get(i,j) );
				else
					acadoFPrintf( file,";\n" );
			}
			else
			{
				if ( rhs2->isZero(i,j) == BT_FALSE )
					acadoFPrintf( file,"%s %s;\n", _sign,rhs2->get(i,j) );
				else
					acadoFPrintf( file,"0.0;\n" );
			}
		}

	return SUCCESSFUL_RETURN;
}


returnValue ExportArithmeticStatement::exportCodeMultiply(	FILE* file,
															BooleanType transposeRhs1,
															const char* _realString,
															const char* _intString,
															int _precision
															) const
{
	if ( ( lhs == 0 ) || ( rhs1 == 0 ) || ( rhs2 == 0 ) )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	char assignString[3];
	switch ( op0 )
	{
		case ESO_ASSIGN:
			strcpy( assignString,"=" );
			break;
		
		case ESO_ADD_ASSIGN:
			strcpy( assignString,"+=" );
			break;
			
		case ESO_SUBTRACT_ASSIGN:
			strcpy( assignString,"-=" );
			break;
			
		default:
			return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );
	}

	uint nRowsRhs1;
	uint nColsRhs1;
	
	if ( transposeRhs1 == BT_FALSE )
	{
		nRowsRhs1 = rhs1->getNumRows( );
		nColsRhs1 = rhs1->getNumCols( );
	}
	else
	{
		nRowsRhs1 = rhs1->getNumCols( );
		nColsRhs1 = rhs1->getNumRows( );
	}
	
	if ( ( nColsRhs1 != rhs2->getNumRows( ) ) || 
		 ( nRowsRhs1 != lhs->getNumRows( ) ) || 
		 ( rhs2->getNumCols( ) != lhs->getNumCols( ) ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );


	char sign[2] = "+";

	if ( op2 != ESO_UNDEFINED )
	{
		if ( ( rhs3->getNumRows( ) != lhs->getNumRows( ) ) || 
			 ( rhs3->getNumCols( ) != lhs->getNumCols( ) ) )
			return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
		
		if ( op2 == ESO_SUBTRACT )
			sign[0] = '-';
	}


	BooleanType allZero;
// 	uint ii, kk;
// 	uint iiRhs1, kkRhs1;

	ExportIndex ii( "run1" ), iiRhs1( "run1" );
	ExportIndex kk( "run3" ), kkRhs1( "run3" );

	uint iMax = getNumRows( );
	if ( outerLoopVariable.isGiven( ) == BT_FALSE )
		iMax = 1;
	
	for( uint i=0; i<iMax; ++i )
	{
		if ( outerLoopVariable.isGiven( ) == BT_FALSE )
			acadoFPrintf( file,"for ( %s=0; %s<%d; ++%s ){\n", ii.name,ii.name,getNumRows(),ii.name );
		else
			ii == i;
	
		for( uint j=0; j<getNumCols( ); ++j )
		{
			allZero = BT_TRUE;
			acadoFPrintf( file,"%s %s", lhs->get(ii,j),assignString );

			for( uint k=0; k<nColsRhs1; ++k )
			{
				kk == k;
				if ( transposeRhs1 == BT_FALSE )
				{
					iiRhs1 = ii;
					kkRhs1 = kk;
				}
				else
				{
					iiRhs1 = kk;
					kkRhs1 = ii;
				}
				
				if ( ( rhs1->isZero(iiRhs1,kkRhs1) == BT_FALSE ) && 
					 ( rhs2->isZero(kk,j) == BT_FALSE ) )
				{
					allZero = BT_FALSE;
					
					if ( rhs1->isOne(iiRhs1,kkRhs1) == BT_FALSE )
					{
						acadoFPrintf( file," %s %s", sign,rhs1->get(iiRhs1,kkRhs1) );
						if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file,"*%s", rhs2->get(kk,j) );
					}
					else
					{
						if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file," %s %s", sign,rhs2->get(kk,j) );
						else
							acadoFPrintf( file," %s 1.0", sign );
					}
				}
			}
			
			if ( ( op2 == ESO_ADD ) || ( op2 == ESO_SUBTRACT ) )
				acadoFPrintf( file," + %s;\n", rhs3->get(ii,j) );

			if ( op2 == ESO_UNDEFINED )
			{
				if ( allZero == BT_TRUE )
					acadoFPrintf( file," 0.0;\n" );
				else
					acadoFPrintf( file,";\n" );
			}
		}
	}
	
	if ( outerLoopVariable.isGiven( ) == BT_FALSE )
		acadoFPrintf( file,"}\n" );
	

	return SUCCESSFUL_RETURN;
}


returnValue ExportArithmeticStatement::exportCodeAssign(	FILE* file,
															const char* const _op,
															const char* _realString,
															const char* _intString,
															int _precision
															) const
{
	if ( ( lhs == 0 ) || ( rhs1 == 0 ) || ( rhs2 != 0 ) )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	if ( ( rhs1->getNumRows( ) != lhs->getNumRows( ) ) || 
		 ( rhs1->getNumCols( ) != lhs->getNumCols( ) ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	
	for( uint i=0; i<getNumRows( ); ++i )
		for( uint j=0; j<getNumCols( ); ++j )
		{
			if ( ( _op[0] == '=' ) || ( rhs1->isZero(i,j) == BT_FALSE ) )
			{
				acadoFPrintf( file,"%s %s ", lhs->get(i,j),_op );
				acadoFPrintf( file,"%s;\n", rhs1->get(i,j) );
			}
		}
	
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
