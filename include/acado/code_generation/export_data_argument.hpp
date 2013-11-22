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
 *    \file include/acado/code_generation/export_data_argument.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_DATA_ARGUMENT_HPP
#define ACADO_TOOLKIT_EXPORT_DATA_ARGUMENT_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/code_generation/export_index.hpp>


BEGIN_NAMESPACE_ACADO


class ExportArithmeticStatement;
class ExportIndex;


/** 
 *	\brief ....
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class ....
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */

class ExportDataArgument
{
	friend class ExportArgumentList;

    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        ExportDataArgument( );

        /** Default constructor. */
		ExportDataArgument(	const char* const _name,
							const char* const _typeString = "int",
							uint _nRows = 1,
							uint _nCols = 1,
							BooleanType _callByValue = BT_FALSE,
							const ExportIndex& _addressIdx = emptyConstExportIndex
							);

		ExportDataArgument(	const char* const _name,
							const char* const _typeString,
							const Matrix& _data,
							BooleanType _callByValue = BT_FALSE,
							const ExportIndex& _addressIdx = emptyConstExportIndex
							);

// 		ExportDataArgument(	const ExportIndex& _arg
// 							);

        /** Copy constructor (deep copy). */
        ExportDataArgument(	const ExportDataArgument& arg
							);

        /** Destructor. */
        ~ExportDataArgument( );

        /** Assignment operator (deep copy). */
        ExportDataArgument& operator=(	const ExportDataArgument& arg
										);

		ExportDataArgument& operator=(	const Matrix& arg
								);


		returnValue init(	const char* const _name,
							const char* const _typeString = "int",
							uint _nRows = 1,
							uint _nCols = 1,
							BooleanType _callByValue = BT_FALSE,
							const ExportIndex& _addressIdx = emptyConstExportIndex
							);
							
		returnValue init(	const char* const _name,
							const char* const _typeString,
							const Matrix& _data,
							BooleanType _callByValue = BT_FALSE,
							const ExportIndex& _addressIdx = emptyConstExportIndex
							);


		ExportDataArgument getAddress(	uint rowIdx,
										uint colIdx
										) const;

		ExportDataArgument getAddress(	const ExportIndex& rowIdx,
										uint colIdx
										) const;

		const char* getAddressString( ) const;


		virtual uint getNumRows( ) const;
		
		virtual uint getNumCols( ) const;

		virtual uint getDim( ) const;
		

		BooleanType isGiven( ) const;

		BooleanType isCalledByValue( ) const;
		
		returnValue callByValue( );


		returnValue	setName(	const char* const _name
								);

		returnValue	setTypeString(	const char* const _typeString
									);
									

		returnValue exportDeclaration(	FILE *file,
										const char* _realString = "double",
										const char* _intString = "int",
										int _precision = 16
										) const;

	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:
		returnValue clear( );


    protected:

		Matrix data;

		char* name;
		char* typeString;
		
		ExportIndex addressIdx;
		
		BooleanType callItByValue;
};


static const ExportDataArgument emptyConstExportDataArgument;


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_DATA_ARGUMENT_HPP

// end of file.
