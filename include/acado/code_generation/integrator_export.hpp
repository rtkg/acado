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
 *    \file include/acado/integrator/integrator_export.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_INTEGRATOR_EXPORT_HPP
#define ACADO_TOOLKIT_INTEGRATOR_EXPORT_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>

#include <acado/code_generation/export_ode_function.hpp>
#include <acado/code_generation/export_data.hpp>
#include <acado/code_generation/export_function.hpp>
#include <acado/code_generation/export_arithmetic_statement.hpp>
#include <acado/code_generation/export_function_call.hpp>
#include <acado/code_generation/export_for_loop.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Code Generator which is able to export Runge Kutta Integration Algorithms.
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class IntegratorExport ....
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */

class IntegratorExport{

    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

        /** Default constructor. */
        IntegratorExport( );

        /** Copy constructor (deep copy). */
        IntegratorExport( const IntegratorExport& arg );

        /** Destructor. */
        ~IntegratorExport( );

        /** Assignment operator (deep copy). */
        IntegratorExport& operator=( const IntegratorExport& arg );



        returnValue setODE( const Expression        &rhs,
                            const DifferentialState &x  ,
                            const Control           &u    );


        returnValue setGrid( const Grid &grid_ );


        returnValue exportHeader             (	FILE* file
												) const;

        returnValue exportForwardDeclarations(	FILE* file
												) const;

        returnValue exportCode               (	const String& dirName
												) const;


        uint getNX() const;
        uint getNU() const;


    protected:

        void generateHeader( FILE *file, uint rhsDim, uint xDim ) const;
        void generateCode  ( FILE *file, uint rhsDim, uint xDim ) const;


    protected:

        uint NX;
        uint NU;

        ExportODEfunction ODE;
        Grid grid;
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_INTEGRATOR_EXPORT_HPP

// end of file.
