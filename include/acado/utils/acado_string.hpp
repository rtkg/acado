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
 *    \file include/acado/utils/acado_string.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 *
 */


#ifndef ACADO_TOOLKIT_ACADO_STRING_HPP
#define ACADO_TOOLKIT_ACADO_STRING_HPP


#include <acado/utils/acado_namespace_macros.hpp>
#include <acado/utils/acado_types.hpp>
#include <acado/utils/acado_message_handling.hpp>
#include <acado/utils/acado_constants.hpp>
#include <acado/utils/acado_debugging.hpp>
#include <acado/utils/acado_io_utils.hpp>

BEGIN_NAMESPACE_ACADO


class Stream;


/**
 *  \brief Provides a very rudimentary class to handle strings.
 *
 *  The class String provides a very rudimentary class to handle strings.
 *
 *  \author Boris Houska, Hans Joachim Ferreau
 */

class String{

    public:

        /** Default constructor */
        String();

        /** Constructor taking a char-pointer. */
        String( const char *name_ );

        /** Constructor taking a double value. */
        String( const double &val_ );

        /** Constructor taking a double value. */
        String( const int &val_ );

        /** Copy constructor */
        String( const String& arg );

        /** Assignment operator */
        String& operator=( const String& arg );

        /** Assignment operator */
        String& operator=( const char *name_ );

        /** Access operator */
        char operator()( uint idx ) const;

        /** Destructor */
        ~String();

        /** Prints the String */
        returnValue print() const;

        /** Prints the String */
        returnValue print( FILE *file ) const;

        /** add operator */
        String& operator+=( const String& arg );

        /** add operator */
        String& operator+=( const char* arg );


        /** get length */
        uint getLength() const;


        /** Returns the name of the string. */
        const char* getName() const;


        /** test whether the string is empty */
        BooleanType isEmpty() const;


        /** copy routine (protected, for internal use only) */
        returnValue copy( const String& arg );


    protected:

        uint  length; /**< the length of the char-pointer */
        char *name  ; /**< the char pointer               */

};


CLOSE_NAMESPACE_ACADO

#endif  // ACADO_TOOLKIT_ACADO_STRING_HPP

//end of file
