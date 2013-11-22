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
 *    \file src/code_generation/export_for_loop.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/code_generation/export_for_loop.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportForLoop::ExportForLoop(	const char* _loopVariable,
								int _startValue,
								int _finalValue,
								int _increment,
								BooleanType _doLoopUnrolling
								) : ExportStatementBlock( )
{
	ExportIndex loopVariableTmp( _loopVariable );
	init( loopVariableTmp,_startValue,_finalValue,_increment,_doLoopUnrolling );
}


ExportForLoop::ExportForLoop(	const ExportIndex& _loopVariable,
								int _startValue,
								int _finalValue,
								int _increment,
								BooleanType _doLoopUnrolling
								) : ExportStatementBlock( )
{
	init( _loopVariable,_startValue,_finalValue,_increment,_doLoopUnrolling );
}


ExportForLoop::ExportForLoop( const ExportForLoop& arg ) : ExportStatementBlock( arg )
{
	init( arg.loopVariable,arg.startValue,arg.finalValue,arg.increment,arg.doLoopUnrolling );
}


ExportForLoop::~ExportForLoop( )
{
	clear( );
}


ExportForLoop& ExportForLoop::operator=( const ExportForLoop& arg )
{
	if ( this != &arg )
	{
		ExportStatementBlock::operator=( arg );
		init( arg.loopVariable,arg.startValue,arg.finalValue,arg.increment,arg.doLoopUnrolling );
	}

	return *this;
}


ExportStatement* ExportForLoop::clone( ) const
{
	return new ExportForLoop(*this);
}



returnValue ExportForLoop::init(	const char* _loopVariable,
									int _startValue,
									int _finalValue,
									int _increment,
									BooleanType _doLoopUnrolling
									)
{
	ExportIndex loopVariableTmp;
	
	if ( _loopVariable != 0 )
		loopVariableTmp.init( _loopVariable );
	else
		loopVariableTmp.init( "run1" );
	
	return init( loopVariableTmp,_startValue,_finalValue,_increment,_doLoopUnrolling );
}


returnValue ExportForLoop::init(	const ExportIndex& _loopVariable,
									int _startValue,
									int _finalValue,
									int _increment,
									BooleanType _doLoopUnrolling
									)
{
	ASSERT( _loopVariable.isGiven( ) == BT_FALSE );

	clear( );
	
	if ( ( _startValue > _finalValue ) && ( _increment >= 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	
	if ( ( _startValue < _finalValue ) && ( _increment <= 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	loopVariable = _loopVariable;
	startValue = _startValue;
	finalValue = _finalValue;
	increment  = _increment;
	doLoopUnrolling = _doLoopUnrolling;

	return SUCCESSFUL_RETURN;
}




returnValue ExportForLoop::exportDataDeclaration(	FILE *file,
													const char* _realString,
													const char* _intString,
													int _precision
													) const
{
	if ( doLoopUnrolling == BT_FALSE )
		acadoFPrintf( file,"%s %s;\n", _intString,loopVariable.name );
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportForLoop::exportCode(	FILE* file,
										const char* _realString,
										const char* _intString,
										int _precision
										) const
{
	if ( doLoopUnrolling == BT_FALSE )
	{
		acadoFPrintf( file,"for( %s=%d; %s<%d; ", loopVariable.name,startValue,loopVariable.name,finalValue );
		
		switch( increment )
		{
			case 1:
				acadoFPrintf( file,"++%s ){\n", loopVariable.name );
				break;
				
			case -1:
				acadoFPrintf( file,"--%s ){\n", loopVariable.name );
				break;
				
			default:
				if ( increment > 0 )
					acadoFPrintf( file," %s=%s+%d ){\n", loopVariable.name,loopVariable.name,increment );
				else
					acadoFPrintf( file," %s=%s%d ){\n", loopVariable.name,loopVariable.name,increment );
				break;
		}
	}
	
	ExportStatementBlock::exportCode( file,_realString,_intString,_precision );
	
	if ( doLoopUnrolling == BT_FALSE )
		acadoFPrintf( file,"}\n");

	return SUCCESSFUL_RETURN;
}



ExportForLoop& ExportForLoop::unrollLoop( )
{
	doLoopUnrolling = BT_TRUE;
	return *this;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportForLoop::clear( )
{
	return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO

// end of file.
