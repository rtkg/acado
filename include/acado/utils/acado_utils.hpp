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
 *    \file   include/utils/acado_utils.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date   2008-2010
 *
 *    This file declares several global utility functions.
 */


#ifndef ACADO_TOOLKIT_ACADO_UTILS_HPP
#define ACADO_TOOLKIT_ACADO_UTILS_HPP


#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
#include <math.h>


#include <acado/utils/acado_types.hpp>
#include <acado/utils/acado_constants.hpp>
#include <acado/utils/acado_default_options.hpp>
#include <acado/utils/acado_message_handling.hpp>
#include <acado/utils/acado_debugging.hpp>
#include <acado/utils/acado_io_utils.hpp>
#include <acado/utils/acado_mat_file.hpp>
#include <acado/utils/acado_string.hpp>
#include <acado/utils/acado_stream.hpp>


BEGIN_NAMESPACE_ACADO


/** Prints ACADO Toolkit's copyright notice.
 * \return SUCCESSFUL_RETURN */
returnValue acadoPrintCopyrightNotice( );


/** Prints ACADO Toolkit's copyright notice for subpackages.
 * \return SUCCESSFUL_RETURN */
returnValue acadoPrintCopyrightNotice(	const char* subpackage 
									);


/** Prints ACADO Toolkit's copyright notice for auto generated code.
 * \return SUCCESSFUL_RETURN */
returnValue acadoPrintAutoGenerationNotice( FILE  *file );


/** Returns the current system time.
 * \return current system time */
double acadoGetTime( );


/** Returns if x is integer-valued.
 */
BooleanType acadoIsInteger( double x );


double acadoDiv( double nom, double den );


double acadoMod( double nom, double den );


/** Returns the maximum of x and y
 */
int acadoMax( const int x, const int y );

/** Returns the maximum of x and y
 */
double acadoMax( const double x, const double y );


/** Returns the minimum of x and y
 */
int acadoMin( const int x, const int y );

/** Returns the minimum of x and y
 */
double acadoMin( const double x, const double y );


/** Returns whether str1 and str2 are identical
 */
BooleanType acadoIsEqual( const char* str1, const char* str2 );


/** Returns whether x and y are numerically equal
 */
BooleanType acadoIsEqual( double x, double y, double TOL = EQUALITY_EPS );


/** Returns whether x is numerically greater or equal than y
 */
BooleanType acadoIsGreater( double x, double y, double TOL = EQUALITY_EPS );


/** Returns whether x is numerically smaller or equal than y
 */
BooleanType acadoIsSmaller( double x, double y, double TOL = EQUALITY_EPS );


/** Returns whether x is numerically strictly greater than y
 */
BooleanType acadoIsStrictlyGreater( double x, double y, double TOL = EQUALITY_EPS );


/** Returns whether x is numerically strictly smaller than y
 */
BooleanType acadoIsStrictlySmaller( double x, double y, double TOL = EQUALITY_EPS );


/** Returns whether x is numerically greater than 0
 */
BooleanType acadoIsPositive( double x, double TOL = EQUALITY_EPS );


/** Returns whether x is numerically smaller than 0
 */
BooleanType acadoIsNegative( double x, double TOL = EQUALITY_EPS );


/** Returns whether x is numerically 0
 */
BooleanType acadoIsZero( double x, double TOL = EQUALITY_EPS );


/** Returns whether x is greater/smaller than +/-INFTY
 */
BooleanType acadoIsInfty( double x, double TOL = 0.1 );


/** Returns whether x lies within [-INFTY,INFTY]
 */
BooleanType acadoIsFinite( double x, double TOL = 0.1 );


/** Specific rounding implemenation for compiler who don't support the round command. Does a round to nearest.
 */
int acadoRound (double x);


/** Assigns one string to the other.
 *
 *	@param[out] toString		Pointer to string to be assigned.
 *	@param[in]  fromString		String to be copied.
 *	@param[in]  defaultString	Default string to be copied in case fromString is empty.
 *
 *  \return SUCCESSFUL_RETURN, \n
 *          RET_UNKNOWN_BUG 
 */
returnValue acadoAssignString(	char** toString,
								const char* const fromString,
								const char* const defaultString
								);



CLOSE_NAMESPACE_ACADO



#include <acado/utils/acado_utils.ipp>


#endif	// ACADO_TOOLKIT_ACADO_UTILS_HPP


/*
 *	end of file
 */
