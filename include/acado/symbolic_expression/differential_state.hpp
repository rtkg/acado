/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic DifferentialState and Dynamic Optimization.
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
*    \file include/acado/symbolic_expression/differential_state.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#ifndef ACADO_TOOLKIT_DIFFERENTIAL_STATE_HPP
#define ACADO_TOOLKIT_DIFFERENTIAL_STATE_HPP


#include <acado/symbolic_expression/expression.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements a differential state.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class DifferentialState implements a differential state variable within the 
 *	family of Expressions.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class DifferentialState : public Expression{

public:

    /** Default constructor */
    DifferentialState();

    /** Default constructor */
    DifferentialState( uint nRows_, uint nCols_ = 1, String name_ = "" );

    /** vector constructor */
    DifferentialState( uint nRows_, String name_ );

    /** scalar constructor */
    DifferentialState( String name_ );

    /** Copy constructor (deep copy). */
    DifferentialState( const DifferentialState &arg );

    /** Default destructor. */
    virtual ~DifferentialState();

    /** Assignment Operator (deep copy). */
    DifferentialState& operator=( const DifferentialState &arg );


     /** Provides a deep copy of the expression. \n
      *  \return a clone of the expression.      \n
      */
     virtual Expression* clone() const;


     /** This function clears all static counters. Although this \n
      *  function is public it should never be used in C-code.   \n
      *  It is necessary for some Matlab-specific interfaces.    \n
      *  Please have a look into the header file                 \n
      *  include/global/utils/matlab_acado_utils.hpp             \n
      *  for more details.                                       \n
      */
     returnValue clearStaticCounters();


//
//  PROTECTED MEMBERS:
//

protected:

     static int count;

};


CLOSE_NAMESPACE_ACADO



#endif
