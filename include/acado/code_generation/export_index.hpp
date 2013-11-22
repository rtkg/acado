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
 *    \file include/acado/code_generation/export_index.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_INDEX_HPP
#define ACADO_TOOLKIT_EXPORT_INDEX_HPP

#include <acado/utils/acado_utils.hpp>


BEGIN_NAMESPACE_ACADO


class ExportDataArgument;


/** 
 *	\brief ....
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class ....
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */

class ExportIndex
{
	friend class ExportForLoop;
	friend class ExportData;
	friend class ExportArithmeticStatement;

    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        ExportIndex( );

		ExportIndex(	const char* const _name,
						const char* const _typeString = "int",
						const int* const _value  = 0,
						const int* const _factor = 0,
						const int* const _offset = 0
						);

        /** Copy constructor (deep copy). */
        ExportIndex(	const ExportIndex& arg
						);

        /** Destructor. */
        ~ExportIndex( );

        /** Assignment operator (deep copy). */
        ExportIndex& operator=(	const ExportIndex& arg
								);

						
		returnValue init(	const char* const _name,
							const char* const _typeString = "int",
							const int* const _value  = 0,
							const int* const _factor = 0,
							const int* const _offset = 0
							);


		const char* get( ) const;
		
		const int getGivenValue( ) const;


		ExportIndex operator+(	const ExportIndex& arg
								) const;

		ExportIndex operator-(	const ExportIndex& arg
								) const;


		ExportIndex operator+(	int _offset
								) const;

		ExportIndex operator-(	int _offset
								) const;

		ExportIndex operator*(	int _factor
								) const;

		ExportIndex& operator==(	int _value
									);

		BooleanType isGiven( ) const;


		returnValue	setName(	const char* const _name
								);

		returnValue	setTypeString(	const char* const _typeString
									);


		ExportDataArgument makeArgument( ) const;



	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:
		
		returnValue clear( );


    protected:

		int* value;
		int* factor;
		int* offset;

		char* name;
		char* typeString;
};

static const int emptyConstExportIndexValue = 0;
static const ExportIndex emptyConstExportIndex( "i","int",&emptyConstExportIndexValue );


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_INDEX_HPP

// end of file.
