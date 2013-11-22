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
 *    \file   examples/ocp/lsq_term_c.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_optimal_control.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>


USING_NAMESPACE_ACADO


void ffcn_model( double *x, double *f, void *user_data ){

    f[0] =  x[0];
    f[1] =  2.0*x[1];
}


void loadLSQfunction( Function *f ){

    const int  dim = 2;  // the dimension of the right hand side
    const int  nt  = 0;  // the explicit dependence on time
    const int  nx  = 1;  // the number of differential states
    const int  nxa = 0;  // the number of algebraic states
    const int  nu  = 1;  // the number of controls
    const int  nv  = 0;  // the number of integer controls
    const int  np  = 0;  // the number of parameters
    const int  nq  = 0;  // the number of integer parameters
    const int  nw  = 0;  // the number of disturbances
    const int  ndx = 0;  // the number of differential state derivatives

    f->setCFunction( dim, nt, nx, nxa, nu, nv, np, nq, nw, ndx, ffcn_model );
}


int main( ){


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState         x;
    Control                   u;
    DifferentialEquation      f;

    const double t_start =  0.0;
    const double t_end   = 10.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x) == -x + 0.5*x*x + u;

    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------

    Function h;
    loadLSQfunction( &h );

    Matrix S(2,2);
    Vector r(2);

    S.setIdentity();
    r.setAll( 0.1 );


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 30 );
    ocp.minimizeLSQ( S, h, r );

    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x == 1.0 );


    // Additionally, flush a plotting object
    GnuplotWindow window;
        window.addSubplot( x,"DifferentialState x" );
        window.addSubplot( u,"Control u" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);
    algorithm << window;

 //  algorithm.set( HESSIAN_APPROXIMATION, CONSTANT_HESSIAN );
 //  algorithm.set( HESSIAN_APPROXIMATION, FULL_BFGS_UPDATE );
 //  algorithm.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE );
     algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
 //  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON_WITH_BLOCK_BFGS );
 //  algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );

    algorithm.solve();


    return 0;
}



