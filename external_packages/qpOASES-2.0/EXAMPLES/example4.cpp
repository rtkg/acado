/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file EXAMPLES/example4.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2009
 *
 *	Very simple example for testing qpOASES (using the possibility to specify 
 *	user-defined constraint product function).
 */



#include <stdlib.h>

#include <QProblem.hpp>
#include "example4CP.cpp"


/**	Example for qpOASES main function using the possibility to specify 
 *	user-defined constraint product function. */
int main( )
{
	using namespace qpOASES;

	int i,j;

	/* Setup data of first QP... */
	double H[7*7];
	double A[50*7];
	double g[7];
	double lbA[50];

	/*	    ( 1.0 0.5 |                    )
	 *	    ( 0.5 2.0 |                    )
	 *	    ( --------+------------------- )
	 *	H = (         | 1e-6               )
	 *	    (         |      1e-6          )
	 *	    (         |           ...      )
	 *	    (         |               1e-6 ) */
	for( i=0; i<7*7; ++i )
		H[i] = 0.0;
	for( i=2; i<7; ++i )
		H[i*7+i] = 1.0e-6;
	H[0] = 1.0;
	H[1] = 0.5;
	H[7] = 0.5;
	H[8] = 2.0;

	/*	    ( x.x x.x | 1.0             )
	 *	    ( x.x x.x | ...             )
	 *	    ( x.x x.x | 1.0             )
	 *	    ( x.x x.x |     1.0         )
	 *	A = ( x.x x.x |     ...         )
	 *	    ( x.x x.x |     1.0         )
	 *	    ( x.x x.x |         ...     )
	 *	    ( x.x x.x |             1.0 )
	 *	    ( x.x x.x |             ... )
	 *	    ( x.x x.x |             1.0 ) */
	for( i=0; i<50*7; ++i )
		A[i] = 0.0;
	for( i=0; i<50; ++i )
	{
		for( j=0; j<2; ++j )
			A[i*7+j] = (double)rand() / (double)RAND_MAX;

		A[i*7 + (i/10)+2] = 1.0;
	}

	/*	    ( -1.0 )
	 *	    ( -0.5 )
	 *	    ( ---- )
	 *	g = (      )
	 *	    (      )
	 *	    (      )
	 *	    (      ) */
	for( i=0; i<7; ++i )
		g[i] = 0.0;
	g[0] = -1.0;
	g[1] = -0.5;

	for( i=0; i<50; ++i )
		lbA[i] = 1.0;

	/* ... and setting up user-defined constraint product function. */
	MyConstraintProduct myCP( 7,50,A );


	/* Setting up QProblem object and set construct product function. */
	QProblem example( 7,50 );
	example.setConstraintProduct( &myCP );


	/* Solve first QP. */
	double cputime = 1.0;
	int nWSR = 100;
	example.init( H,g,A,0,0,lbA,0, nWSR,&cputime );


	/* Solve second QP using a modified gradient. */
	g[0] = -2.0;
	g[1] =  0.5;

	cputime = 1.0;
	nWSR = 100;
	example.hotstart( g,0,0,lbA,0, nWSR,&cputime );

	return 0;
}


/*
 *	end of file
 */
