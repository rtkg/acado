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
 *    \file include/acado/code_generation/export_data.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_DATA_HPP
#define ACADO_TOOLKIT_EXPORT_DATA_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>



BEGIN_NAMESPACE_ACADO


static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5
static char exportDataString[1024];


/** 
 *	\brief ....
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class ....
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */

class ExportData
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        ExportData( );

        /** Default constructor. */
        ExportData(	uint nRows,
					uint nCols,
					const char* const name
					);

        /** Copy constructor (deep copy). */
        ExportData( const ExportData& arg );

        /** Destructor. */
        ~ExportData( );

        /** Assignment operator (deep copy). */
        ExportData& operator=( const ExportData& arg );


		double& operator()(	uint rowIdx,
							uint colIdx
							);

		const char* get(	uint rowIdx,
							uint colIdx
							) const;

		const char* operator*(	const Vector& arg
								) const;
							
							

		returnValue print( ) const;


	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		returnValue	setName(	const char* const _name
								);


    protected:

		Matrix data;
		char* name;
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_DATA_HPP

// end of file.
