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
 *    \file include/acado/code_generation/gauss_newton_export.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_GAUSS_NEWTON_EXPORT_HPP
#define ACADO_TOOLKIT_GAUSS_NEWTON_EXPORT_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/function/function.hpp>
#include <acado/code_generation/condensing_export.hpp>

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

class GaussNewtonExport{

    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

        /** Default constructor. */
        GaussNewtonExport( );

        /** Copy constructor (deep copy). */
        GaussNewtonExport( const GaussNewtonExport& arg );

        /** Destructor. */
        ~GaussNewtonExport( );

        /** Assignment operator (deep copy). */
        GaussNewtonExport& operator=( const GaussNewtonExport& arg );

        returnValue setCondensingExport( const CondensingExport &condenser_ );

		returnValue setControlBounds(	const VariablesGrid &xBounds );



        returnValue exportHeader             (	FILE* file
												) const;

        returnValue exportForwardDeclarations(	FILE* file
												) const;

        returnValue exportCode               (	const String& dirName
												) const;


        uint getNX() const;
        uint getNU() const;
        uint getN () const;

		uint getNumStateBounds( ) const;


    protected:

        CondensingExport condenser;

		VariablesGrid uBounds;
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_GAUSS_NEWTON_EXPORT_HPP

// end of file.
