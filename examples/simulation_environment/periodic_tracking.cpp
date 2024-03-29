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
 *    \file   examples/simulation_environment/periodic_tracking.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_toolkit.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // ------------------------
    DifferentialState x;
    Control           u;
    Disturbance       w;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f, f2;

    f  << dot(x) == -2.0*x + u;
    f2 << dot(x) == -2.0*x + u + 0.1*w; //- 0.000000000001*x*x


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;

    h << x;
    h << u;

    Matrix Q(2,2);
    Q.setIdentity();

    Vector r(2);
    r.setAll( 0.0 );


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;
    const double t_end   = 7.0;

    OCP ocp        ( t_start, t_end, 14 );
    ocp.minimizeLSQ( Q, h, r );
    ocp.subjectTo  ( f );

    ocp.subjectTo( -1.0 <= u <= 2.0 );
    //ocp.subjectTo(  w == 0.0 );


    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
    OutputFcn identity;
    DynamicSystem dynamicSystem( f2,identity );
    Process process( dynamicSystem,INT_RK45 );


	VariablesGrid disturbance = fopen( "my_disturbance.txt", "r" );

// 	GnuplotWindow window2;
// 		window2.addSubplot( disturbance, "my disturbance"   );
// 	window2.plot();

	process.setProcessDisturbance( disturbance );



    // SETUP OF THE ALGORITHM AND THE TUNING OPTIONS:
    // ----------------------------------------------
	double samplingTime = 0.5;
    RealTimeAlgorithm  algorithm( ocp,samplingTime );

// //  algorithm.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE );
     algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
// 
// //     algorithm.set( ABSOLUTE_TOLERANCE  , 1e-7 );
// //     algorithm.set( INTEGRATOR_TOLERANCE, 1e-9 );
// 
//     algorithm.set( KKT_TOLERANCE, 1e-4 );

	algorithm.set( MAX_NUM_ITERATIONS,2 );
// 	algorithm.set( TERMINATE_AT_CONVERGENCE,YES );


    Vector x0(1);
    x0(0)  = 1.0;

// // 	algorithm.solve( x0 );
// 
//     GnuplotWindow window1;
//         window1.addSubplot( x, "DIFFERENTIAL STATE: x" );
//         window1.addSubplot( u, "CONTROL: u" );
//     window1.plot();
// 
// 	return 0;


    // SETTING UP THE NMPC CONTROLLER:
    // -------------------------------

    VariablesGrid myReference = fopen( "my_reference.txt", "r" );
    PeriodicReferenceTrajectory reference( myReference );

	GnuplotWindow window3;
		window3.addSubplot( myReference(1), "my reference"   );
	window3.plot();
	
    Controller controller( algorithm,reference );



    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
    double simulationStart =  0.0;
    double simulationEnd   =  15.0;

    SimulationEnvironment sim( simulationStart, simulationEnd, process, controller );

    sim.init( x0 );
    sim.run( );


    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
    VariablesGrid sampledProcessOutput;
    sim.getSampledProcessOutput( sampledProcessOutput );

    VariablesGrid feedbackControl;
    sim.getFeedbackControl( feedbackControl );

    GnuplotWindow window;
        window.addSubplot( sampledProcessOutput(0), "DIFFERENTIAL STATE: x" );
        window.addSubplot( feedbackControl(0),      "CONTROL: u" );
    window.plot();


    return 0;
}

