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
 *    \file examples/modeling_tools/lagrange_formalism.cpp
 *    \author Boris Houska, Joris Gillis, Hans Joachim Ferreau
 *    \date 2008
 */


#include <modeling_tools/kinetics_tools.hpp>
#include <acado_integrators.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>


/** In this example we simulate a simple double pendulum.   \n
 *  The equations of motion are automatically derived with  \n
 *  the Lagrangian formalism.                               \n
 */


/* >>> start tutorial code >>> */
int main( ){


    USING_NAMESPACE_ACADO


    // DEFINE VALRIABLES:
    // ---------------------------
    DifferentialStateVector  q(2);   // the generalized coordinates of the pendulum
    DifferentialStateVector dq(2);   // the associated velocities


    const double L1    = 1.00;       // length of the first pendulum
    const double L2    = 1.00;       // length of the second pendulum
    const double m1    = 1.00;       // mass of the first pendulum
    const double m2    = 1.00;       // mass of the second pendulum
    const double g     = 9.81;       // gravitational constant
    const double alpha = 0.10;       // a friction constant

    const double J_11 = (m1+m2)*L1*L1;   // auxiliary variable (inertia comp.)
    const double J_22 =  m2    *L2*L2;   // auxiliary variable (inertia comp.)
    const double J_12 =  m2    *L1*L2;   // auxiliary variable (inertia comp.)

    const double E1   = -(m1+m2)*g*L1;   // auxiliary variable (pot energy 1)
    const double E2   = - m2    *g*L2;   // auxiliary variable (pot energy 2)

    IntermediateState  c1;
    IntermediateState  c2;
    IntermediateState  c3;

    IntermediateState   T;
    IntermediateState   V;
    IntermediateStateVector Q;


    // COMPUTE THE KINETIC ENERGY T AND THE POTENTIAL V:
    // -------------------------------------------------

    c1 = cos(q(0));
    c2 = cos(q(1));
    c3 = cos(q(0)+q(1));

    T  = 0.5*J_11*dq(0)*dq(0) + 0.5*J_22*dq(1)*dq(1) + J_12*c3*dq(0)*dq(1);
    V  = E1*c1 + E2*c2;
    Q  = (-alpha*dq);


    // AUTOMATICALLY DERIVE THE EQUATIONS OF MOTION BASED ON THE LAGRANGIAN FORMALISM:
    // -------------------------------------------------------------------------------
    DifferentialEquation  f;
    LagrangianFormalism( f, T - V, Q, q, dq );


    // Define an integrator:
    // ---------------------
    IntegratorBDF integrator( f );


    // Define an initial value:
    // ------------------------

    double x_start[4] = { 0.0, 0.5, 0.0, 0.1 };

    double t_start    =   0.0;
    double t_end      =   3.0;

    // START THE INTEGRATION
    // ----------------------
    integrator.set( INTEGRATOR_PRINTLEVEL, MEDIUM );
    integrator.set( INTEGRATOR_TOLERANCE, 1e-12 );

    integrator.freezeAll();
    integrator.integrate( x_start, t_start, t_end );


    VariablesGrid xres;
    integrator.getTrajectory(&xres,NULL,NULL,NULL,NULL,NULL);

    GnuplotWindow window;
        window.addSubplot( xres(0), "The excitation angle of pendulum 1" );
        window.addSubplot( xres(1), "The excitation angle of pendulum 2" );
        window.addSubplot( xres(2), "The angular velocity of pendulum 1" );
        window.addSubplot( xres(3), "The angular velocity of pendulum 2" );
    window.plot();


    return 0;
}
/* <<< end tutorial code <<< */
