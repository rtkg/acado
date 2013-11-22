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
 *    \file examples/modeling_tools/hamilton_formalism.cpp
 *    \author Boris Houska, Joris Gillis, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>


/** In this example we simulate a simple spring with spring constant D  \n
 *  and mass m. The equations of motion are automatically derived with  \n
 *  the Hamiltonian formalism.                                          \n
 */



/* >>> start tutorial code >>> */
int main( ){


    USING_NAMESPACE_ACADO

    // DEFINE VALRIABLES:
    // ---------------------------
    DifferentialStateVector  q(1);        // the position of the spring
    DifferentialStateVector  p(1);        // the generalized moment

    const double m  = 1.00;               // the mass of the spring
    const double D  = 2.00;               // the spring constant
    const double g  = 9.81;               // the gravitational constant

    IntermediateState           T;        // the kinetic energy     T
    IntermediateState           V;        // the potential energy   V
    IntermediateStateVector  Q(1);        // the generalized force  Q


    // DEFINE THE KINETIC AND POTENTIAL ENERGY:
    // ----------------------------------------

    T = 0.5*m*p(0)*p(0)           ;
    V = m*g*q(0) + 0.5*D*q(0)*q(0);

    Q.setComponent( 0, 0.0 );  // no external force


    // AUTOMATICALLY DERIVE THE EQUATIONS OF MOTION:
    // ---------------------------------------------
    DifferentialEquation  f;
    HamiltonianFormalism( f, T+V, Q, p, q );


    // Define an integrator:
    // ---------------------
    IntegratorBDF integrator( f );


    // Define an initial value:
    // ------------------------
    double x_start[2] = {-3.0, 0.0 };   // start the spring at q(0) = -3, p(0) = 0

    double t_start    =   0.0;
    double t_end      =  10.0;


    // START THE INTEGRATION
    // ----------------------
    integrator.set( INTEGRATOR_PRINTLEVEL, MEDIUM );

    integrator.freezeAll();
    integrator.integrate( x_start, t_start, t_end );


    // PLOT THE RESULTS:
    // -----------------------

    VariablesGrid xres;
    integrator.getTrajectory(&xres,NULL,NULL,NULL,NULL,NULL);

    GnuplotWindow window;
        window.addSubplot( xres(0), "The position q" );
        window.addSubplot( xres(1), "The generalized moment p" );
    window.plot();

    return 0;
}
/* <<< end tutorial code <<< */
