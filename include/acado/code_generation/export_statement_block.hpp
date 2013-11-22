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
*	\file include/acado/code_generation/export_statement_block.hpp
*	\author Hans Joachim Ferreau, Boris Houska
*/



#ifndef ACADO_TOOLKIT_EXPORT_STATEMENT_BLOCK_HPP
#define ACADO_TOOLKIT_EXPORT_STATEMENT_BLOCK_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_statement.hpp>
#include <acado/code_generation/export_data_argument.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief ...
 *
 *	\ingroup UserDataStructures
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportStatementBlock : public ExportStatement
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:

		/**< Default Constructor. 
		 */
		ExportStatementBlock( );

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg	Right-hand side object.
		 */
		ExportStatementBlock(	const ExportStatementBlock& arg
								);

		/** Destructor.
		 */
		virtual ~ExportStatementBlock( );

		/**< Assignment Operator (deep copy).
		 *
		 *	@param[in] arg	Right-hand side object.
		 */
		ExportStatementBlock& operator=(	const ExportStatementBlock& rhs
											);

		/** Clone constructor (deep copy). */
		virtual ExportStatement* clone( ) const;


		/** ...
		 *
		 *  \return ...
		 */
		returnValue addStatement(	const ExportStatement& _statement
									);

		returnValue addStatement(	const char* const _statementString
									);

		returnValue addFunctionCall(	const char* _fName = "acadoFcn",
										const ExportDataArgument& _argument1 = emptyConstExportDataArgument,
										const ExportDataArgument& _argument2 = emptyConstExportDataArgument,
										const ExportDataArgument& _argument3 = emptyConstExportDataArgument,
										const ExportDataArgument& _argument4 = emptyConstExportDataArgument,
										const ExportDataArgument& _argument5 = emptyConstExportDataArgument,
										const ExportDataArgument& _argument6 = emptyConstExportDataArgument,
										const ExportDataArgument& _argument7 = emptyConstExportDataArgument,
										const ExportDataArgument& _argument8 = emptyConstExportDataArgument,
										const ExportDataArgument& _argument9 = emptyConstExportDataArgument
										);

		returnValue addLinebreak( );

		returnValue addComment(	const char* const _statementString
								);


		/** ...
		 *
		 *  \return ...
		 */
		uint getNumStatements( ) const;


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



	//
	// PROTECTED MEMBER FUNCTIONS:
	//
	protected:
		returnValue clear( );


	//
	// DATA MEMBERS:
	//
	protected:

		uint nStatements;
		ExportStatement** statements;
};


CLOSE_NAMESPACE_ACADO



#endif	// ACADO_TOOLKIT_EXPORT_STATEMENT_BLOCK_HPP


/*
 *	end of file
 */
