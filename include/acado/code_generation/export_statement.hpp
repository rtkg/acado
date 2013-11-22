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
 *    \file include/acado/code_generation/export_statement.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_STATEMENT_HPP
#define ACADO_TOOLKIT_EXPORT_STATEMENT_HPP

#include <acado/utils/acado_utils.hpp>



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

class ExportStatement
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        ExportStatement( );

        /** Copy constructor (deep copy). */
        ExportStatement(	const ExportStatement& arg
							);

        /** Destructor. */
        virtual ~ExportStatement( );

        /** Assignment operator (deep copy). */
        ExportStatement& operator=(	const ExportStatement& arg
									);

		/** Clone constructor (deep copy). */
		virtual ExportStatement* clone( ) const = 0;


		virtual returnValue exportDataDeclaration(	FILE *file,
													const char* _realString = "double",
													const char* _intString = "int",
													int _precision = 16
													) const;

		virtual returnValue exportCode(	FILE* file,
										const char* _realString = "double",
										const char* _intString = "int",
										int _precision = 16
										) const = 0;


	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:



    protected:

};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_STATEMENT_HPP

// end of file.
