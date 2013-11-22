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
 *    \file include/acado/code_generation/export_function.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_FUNCTION_HPP
#define ACADO_TOOLKIT_EXPORT_FUNCTION_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_data.hpp>
#include <acado/code_generation/export_argument_list.hpp>
#include <acado/code_generation/export_statement_block.hpp>
#include <acado/code_generation/export_statement_string.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief ....
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class ....
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */

class ExportFunction
{
	friend class ExportFunctionCall;
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
		ExportFunction(	const char* _name = "acadoFcn",
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

        /** Copy constructor (deep copy). */
        ExportFunction(	const ExportFunction& arg
						);

        /** Destructor. */
        virtual ~ExportFunction( );

        /** Assignment operator (deep copy). */
        ExportFunction& operator=(	const ExportFunction& arg
									);


		returnValue init(	const char* _name = "acadoFcn",
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


		/** ...
		 *
		 *  \return ...
		 */
		returnValue addArgument(	const ExportDataArgument& _argument1,
									const ExportDataArgument& _argument2 = emptyConstExportDataArgument,
									const ExportDataArgument& _argument3 = emptyConstExportDataArgument,
									const ExportDataArgument& _argument4 = emptyConstExportDataArgument,
									const ExportDataArgument& _argument5 = emptyConstExportDataArgument,
									const ExportDataArgument& _argument6 = emptyConstExportDataArgument,
									const ExportDataArgument& _argument7 = emptyConstExportDataArgument,
									const ExportDataArgument& _argument8 = emptyConstExportDataArgument,
									const ExportDataArgument& _argument9 = emptyConstExportDataArgument
									);

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

// 		returnValue addLocalDataDeclaration(	const ExportDataArgument& _argument1
// 												);


		virtual returnValue exportDataDeclaration(	FILE *file,
													const char* _realString = "double",
													const char* _intString = "int",
													int _precision = 16
													) const;
		
		virtual returnValue exportForwardDeclaration(	FILE *file,
														const char* _realString = "double",
														const char* _intString = "int",
														int _precision = 16
														) const;

		virtual returnValue exportCode(	FILE *file,
										const char* _realString = "double",
										const char* _intString = "int",
										int _precision = 16
										) const;


		returnValue setReturnValue(	const ExportData& _functionReturnValue
									);



	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		returnValue clear( );


		returnValue	setName(	const char* const _name
								);
								


    protected:

		char* name;
		
		ExportStatementBlock functionBody;
		ExportArgumentList   functionArguments;
		
		ExportData* functionReturnValue;
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_FUNCTION_HPP

// end of file.
