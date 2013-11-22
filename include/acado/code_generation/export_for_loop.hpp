/*
 *	This file is part of ACADO Toolkit.
 *
 *	ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *	Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *	Developed within the Optimization in Engineering Center (OPTEC) under
 *	supervision of Moritz Diehl. All rights reserved.
 *
 *	ACADO Toolkit is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 3 of the License, or (at your option) any later version.
 *
 *	ACADO Toolkit is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *	Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with ACADO Toolkit; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
*	\file include/acado/code_generation/export_for_loop.hpp
*	\author Hans Joachim Ferreau, Boris Houska
*/



#ifndef ACADO_TOOLKIT_EXPORT_FOR_LOOP_HPP
#define ACADO_TOOLKIT_EXPORT_FOR_LOOP_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_index.hpp>
#include <acado/code_generation/export_data.hpp>
#include <acado/code_generation/export_statement_block.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief ...
 *
 *	\ingroup UserDataStructures
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportForLoop : public ExportStatementBlock
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:

		/**< Default Constructor. 
		 */
		ExportForLoop(	const char* _loopVariable = "run1",
						int _startValue = 0,
						int _finalValue = 0,
						int _increment = 1,
						BooleanType _doLoopUnrolling = BT_FALSE
						);

		ExportForLoop(	const ExportIndex& _loopVariable,
						int _startValue = 0,
						int _finalValue = 0,
						int _increment = 1,
						BooleanType _doLoopUnrolling = BT_FALSE
						);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg	Right-hand side object.
		 */
		ExportForLoop(	const ExportForLoop& arg
						);

		/** Destructor.
		 */
		virtual ~ExportForLoop( );

		/**< Assignment Operator (deep copy).
		 *
		 *	@param[in] arg	Right-hand side object.
		 */
		ExportForLoop& operator=(	const ExportForLoop& rhs
									);

		/** Clone constructor (deep copy). */
		virtual ExportStatement* clone( ) const;


		returnValue init(	const char* _loopVariable = "run1",
							int _startValue = 0,
							int _finalValue = 0,
							int _increment = 1,
							BooleanType _doLoopUnrolling = BT_FALSE
							);

		returnValue init(	const ExportIndex& _loopVariable,
							int _startValue = 0,
							int _finalValue = 0,
							int _increment = 1,
							BooleanType _doLoopUnrolling = BT_FALSE
							);


		virtual returnValue exportDataDeclaration(	FILE *file,
													const char* _realString = "double",
													const char* _intString = "int",
													int _precision = 16
													) const;

		virtual returnValue exportCode(	FILE* file,
										const char* _realString = "double",
										const char* _intString = "int",
										int _precision = 16
										) const;


		ExportForLoop& unrollLoop( );


	//
	// PROTECTED MEMBER FUNCTIONS:
	//
	protected:
		returnValue clear( );


	//
	// DATA MEMBERS:
	//
	protected:

		ExportIndex loopVariable;
		
		int startValue;
		int finalValue;
		int increment;
		
		BooleanType doLoopUnrolling;
};


CLOSE_NAMESPACE_ACADO



#endif	// ACADO_TOOLKIT_EXPORT_FOR_LOOP_HPP


/*
 *	end of file
 */
