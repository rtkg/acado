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
 *    \file examples/modeling_tools/newton_euler_formalism.cpp
 *    \author Boris Houska, Joris Gillis, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>


/** In this example we simulate a simple pendulum.          \n
 *  The equations of motion are automatically derived with  \n
 *  the Newton-Euler formalism leading to an DAE with       \n
 *  index 3. The DAE is automaically reduced to an index 1  \n
 *  equation.                                               \n
 */


/* >>> start tutorial code >>> */
int main( ){


    USING_NAMESPACE_ACADO

    // DEFINE VALRIABLES:
    // ---------------------------
    DifferentialStateVector       x(3);  // the position of the pendulum  (x,y,alpha)
    DifferentialStateVector       v(3);  // the associated velocities
    AlgebraicStateVector          a(3);  // the associated accelerations
    AlgebraicStateVector     lambda(2);  // the constraint forces


    const double L = 1.00;               // the length of the pendulum
    const double m = 1.00;               // the mass of the pendulum
    const double g = 9.81;               // the gravitational constant

    const double J = m*L*L;              // the inertial of the pendulum

    IntermediateStateVector R(3);
    IntermediateStateVector G(2);

    R.setComponent( 0, m*a(0)       );       // ----------------------------------------
    R.setComponent( 1, m*a(1) + m*g );       // the definition of the force residuum:
    R.setComponent( 2, J*a(2)       );       //       R := m*a - F

    G.setComponent( 0, x(0)-L*sin(x(2)) );   // definition of the constraint manifold G
    G.setComponent( 1, x(1)+L*cos(x(2)) );   // ---------------------------------------



    // AUTOMATIC GENERATION OF AN INDEX 1 DAE SYSTEM BASES ON THE
    // NEWTON EULER FORMALISM:
    // -----------------------------------------------------------
    DifferentialEquation  f;
    NewtonEulerFormalism( f, R, G, x, v, a, lambda );


    // Define an integrator:
    // ---------------------
    IntegratorBDF integrator( f );

    // Define an initial value:
    // ------------------------
    double x_start[6];
    double z_start[5];

    x_start[0] =  1.9866932270683099e-01;
    x_start[1] = -9.8006654611577315e-01;
    x_start[2] =  2.0000003107582773e-01;
    x_start[3] = -1.4519963562050693e-04;
    x_start[4] =  4.7104175041346282e-04;
    x_start[5] =  4.4177521668741377e-04;

    z_start[0] = -9.5504866367984165e-01;
    z_start[1] = -1.9359778029074531e-01;
    z_start[2] = -9.7447321693831934e-01;
    z_start[3] = -9.5504866367984165e-01;
    z_start[4] =  9.6164022197092560e+00;


    double t_start    =   0.0;
    double t_end      =  10.0;

    // START THE INTEGRATION
    // ----------------------
    integrator.set( INTEGRATOR_PRINTLEVEL, MEDIUM );
//    integrator.set( INTEGRATOR_TOLERANCE, 1e-12 );

    integrator.freezeAll();
    integrator.integrate( x_start, z_start, t_start, t_end );


    VariablesGrid xres,zres;
    integrator.getTrajectory(&xres,&zres,NULL,NULL,NULL,NULL);

    GnuplotWindow window;
        window.addSubplot( xres(0), "The x-position of the mass m" );
        window.addSubplot( xres(1), "The y-position of the mass m" );
        window.addSubplot( xres(2), "The excitation angle of the pendulum" );
        window.addSubplot( xres(3), "The velocity in x-direction" );
        window.addSubplot( xres(4), "The velocity in y-direction" );
        window.addSubplot( xres(5), "The angular velocity" );
//         window.addSubplot( zres(0), "The acceleration in x-direction" );
//         window.addSubplot( zres(1), "The acceleration in y-direction" );
//         window.addSubplot( zres(2), "The angular acceleration" );
        window.addSubplot( zres(3), "The constraint force in x-direction" );
        window.addSubplot( zres(4), "The constraint force in y-direction" );
    window.plot();


    return 0;
}
/* <<< end tutorial code <<< */
