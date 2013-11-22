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
 *    \file   examples/ocp/test.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_optimal_control.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState       x,L;
    Control                   u;
    DifferentialEquation    f,g;

    const double t_start =  0.0;
    const double t_end   = 10.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x) == -x + 0.01*x*x + u;
    f << dot(L) == -x + 0.01*x*x + u;

    g << dot(x) ==  x - 2.00*x*x + u;
    g << dot(L) ==  x*x + u*u;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 60 );

    ocp.minimizeMayerTerm( L );

    ocp.subjectTo( f, 30 );
    ocp.subjectTo( g, 30 );

    ocp.subjectTo( AT_START, x == 1.0 );
    ocp.subjectTo( AT_START, L == 0.0 );


    // Additionally, flush a plotting object
    GnuplotWindow window1;
	window1.addSubplot( x,"DifferentialState x" );
	window1.addSubplot( u,"Control u" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);
    algorithm << window1;
    algorithm.solve();


    return 0;
}



