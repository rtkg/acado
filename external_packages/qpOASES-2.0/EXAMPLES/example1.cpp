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
 *	\file EXAMPLES/example1.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Very simple example for testing qpOASES (using QProblem class).
 */



#include <QProblem.hpp>


/** Example for qpOASES main function using the QProblem class. */
int main( )
{
	using namespace qpOASES;

	/* Setup data of first QP. */
	double H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	double A[1*2] = { 1.0, 1.0 };
	double g[2] = { 1.5, 1.0 };
	double lb[2] = { 0.5, -2.0 };
	double ub[2] = { 5.0, 2.0 };
	double lbA[1] = { -1.0 };
	double ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	double g_new[2] = { 1.0, 1.5 };
	double lb_new[2] = { 0.0, -1.0 };
	double ub_new[2] = { 5.0, -0.5 };
	double lbA_new[1] = { -2.0 };
	double ubA_new[1] = { 1.0 };


	/* Setting up QProblem object. */
	QProblem example( 2,1 );

	/* Solve first QP. */
	int nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );

	/* Solve second QP. */
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0 );

	return 0;
}


/*
 *	end of file
 */
