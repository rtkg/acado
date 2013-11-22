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
 *    \file include/acado/code_generation/mpc_export.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_MPC_EXPORT_HPP
#define ACADO_TOOLKIT_MPC_EXPORT_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/function/function.hpp>
#include <acado/code_generation/gauss_newton_export.hpp>
#include <acado/ocp/ocp.hpp>

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

class MPCexport : public UserInteraction
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        MPCexport( );

        /** Default constructor. */
        MPCexport( const OCP& _ocp );

        /** Copy constructor (deep copy). */
        MPCexport( const MPCexport& arg );

        /** Destructor. */
        ~MPCexport( );

        /** Assignment operator (deep copy). */
        MPCexport& operator=( const MPCexport& arg );

        returnValue setOCP(	const OCP& _ocp
							);

        returnValue exportCode(	const char* const dirName
								);


		returnValue printDimensionsQP( );



    protected:

        returnValue setup( );
		
		returnValue checkConsistency( ) const;

        returnValue exportTemplateMain(	const String& dirName
										) const;

        returnValue exportMakefile(	const String& dirName
									) const;

		returnValue exportQPsolverInterface(	const String& dirName
												) const;

        returnValue exportAuxiliaryFunctions(	const String& dirName
												) const;

        returnValue setupOptions( );


    protected:

		uint NX;
		uint NU;
		uint N;

		GaussNewtonExport gnExport;
		OCP ocp;
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_MPC_EXPORT_HPP

// end of file.
