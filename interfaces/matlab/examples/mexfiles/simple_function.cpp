/**
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
 *    Author: David Ariens  --  http://www.acadotoolkit.org/matlab 
 *    Date: 2010
 *    
 *    SIMPLE FUNCTION EXAMPLE
 *
 *    Compilation:
 *     - Go to the folder <ACADOtoolkit-inst-dir>/interfaces/matlab/
 *     - Run: makemex('examples/mexfiles/simple_function.cpp', 'simple_function', 'examples/mexfiles/');
 *     - Run: cd ('examples/mexfiles/');
 *     - Run: simple_function();
 */
 
#include <acado_toolkit.hpp>                    // Include the ACADO toolkit
#include <acado/utils/matlab_acado_utils.hpp>   // Include specific Matlab utils

USING_NAMESPACE_ACADO                           // Open the namespace

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )  // Start the MEX function. Do NOT change the header of this function.
 { 
    clearAllStaticCounters( );                  // Clear software counters
 
    
    
    DifferentialState x;
    IntermediateState z;
    TIME              t;
    Function          f;

    z = 0.5*x + 1.0    ;

    f << exp(x) + t    ;
    f << exp(z+exp(z)) ;

    printf("the dimension of f is %d   \n",  f.getDim() );
    printf("f depends on  %d  states   \n",  f.getNX () );
    printf("f depends on  %d  controls \n",  f.getNU () );

    if( f.isConvex() == BT_TRUE )
        printf("all components of function f are convex. \n");
    
    

    clearAllStaticCounters( );                  // Clear software counters
} 

