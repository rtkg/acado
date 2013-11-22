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
 *    \file   examples/code_generation/getting_started.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date   2011
 */


#include <acado_code_generation.hpp>


int main( )
{
	USING_NAMESPACE_ACADO

	// DEFINE THE VARIABLES:
	// ----------------------------------------------------------
	DifferentialState   p    ;  // the trolley position
	DifferentialState   v    ;  // the trolley velocity 
	DifferentialState   phi  ;  // the excitation angle
	DifferentialState   omega;  // the angular velocity
	Control             a    ;  // the acc. of the trolley

	const double     g = 9.81;  // the gravitational constant 
	const double     b = 0.20;  // the friction coefficient
	// ----------------------------------------------------------


	// DEFINE THE MODEL EQUATIONS:
	// ----------------------------------------------------------
	DifferentialEquation f; 

	f << dot( p     )  ==  v                                ;
	f << dot( v     )  ==  a                                ;
	f << dot( phi   )  ==  omega                            ;
	f << dot( omega )  == -g*sin(phi) - a*cos(phi) - b*omega;
	// ----------------------------------------------------------


	// DEFINE THE WEIGHTING MATRICES:
	// ----------------------------------------------------------
	Matrix Q  = eye(4);
	Matrix R  = eye(1);
	
	Matrix P  = eye(4);
	P *= 5.0;
	// ----------------------------------------------------------


	// SET UP THE MPC - OPTIMAL CONTROL PROBLEM:
	// ----------------------------------------------------------
	OCP ocp( 0.0,3.0, 10 );

	ocp.minimizeLSQ       ( Q,R );
	ocp.minimizeLSQEndTerm( P   );

	ocp.subjectTo( f );
	ocp.subjectTo( -1.0 <= a <= 1.0 );
// 	ocp.subjectTo( -0.5 <= v <= 1.5 );
	// ----------------------------------------------------------


	// DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	MPCexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,   SINGLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,       INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,  30              );
	mpc.set( QP_SOLVER,             QP_QPOASES      );
	mpc.set( HOTSTART_QP,           NO              );
	mpc.set( GENERATE_TEST_FILE,    YES             );
	mpc.set( GENERATE_MAKE_FILE,    YES             );

	mpc.exportCode( "getting_started_export" );
	// ----------------------------------------------------------

	return 0;
}
