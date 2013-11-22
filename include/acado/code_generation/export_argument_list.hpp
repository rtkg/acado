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
*	\file include/acado/code_generation/export_argument_list.hpp
*	\author Hans Joachim Ferreau, Boris Houska
*/



#ifndef ACADO_TOOLKIT_EXPORT_ARGUMENT_LIST_HPP
#define ACADO_TOOLKIT_EXPORT_ARGUMENT_LIST_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_data_argument.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief ...
 *
 *	\ingroup UserDataStructures
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportArgumentList
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:

		/**< Default Constructor. 
		 */
		ExportArgumentList( );

		ExportArgumentList(	const ExportDataArgument& _argument1,
							const ExportDataArgument& _argument2 = emptyConstExportDataArgument,
							const ExportDataArgument& _argument3 = emptyConstExportDataArgument,
							const ExportDataArgument& _argument4 = emptyConstExportDataArgument,
							const ExportDataArgument& _argument5 = emptyConstExportDataArgument,
							const ExportDataArgument& _argument6 = emptyConstExportDataArgument,
							const ExportDataArgument& _argument7 = emptyConstExportDataArgument,
							const ExportDataArgument& _argument8 = emptyConstExportDataArgument,
							const ExportDataArgument& _argument9 = emptyConstExportDataArgument
							);
		
		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg	Right-hand side object.
		 */
		ExportArgumentList(	const ExportArgumentList& arg
							);

		/** Destructor.
		 */
		~ExportArgumentList( );

		/**< Assignment Operator (deep copy).
		 *
		 *	@param[in] arg	Right-hand side object.
		 */
		ExportArgumentList& operator=(	const ExportArgumentList& rhs
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
		uint getNumArguments( ) const;


		virtual returnValue exportCode(	FILE* file,
										const char* _realString = "double",
										const char* _intString = "int",
										BooleanType includeTypes = BT_TRUE
										) const;


		returnValue clear( );


	//
	// PROTECTED MEMBER FUNCTIONS:
	//
	protected:
		/** ...
		 *
		 *  \return ...
		 */
		returnValue addSingleArgument(	const ExportDataArgument& _argument
										);


	//
	// DATA MEMBERS:
	//
	protected:

		uint nArguments;
		ExportDataArgument** arguments;
};


CLOSE_NAMESPACE_ACADO



#endif	// ACADO_TOOLKIT_EXPORT_ARGUMENT_LIST_HPP


/*
 *	end of file
 */
