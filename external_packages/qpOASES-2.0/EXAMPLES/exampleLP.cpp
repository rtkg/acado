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
 *	\file EXAMPLES/exampleLP.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2008-2009
 *
 *	Very simple example for solving a LP sequence using qpOASES.
 */



#include <QProblem.hpp>


/** Example for qpOASES main function solving LPs. */
int main( )
{
	using namespace qpOASES;

	/* Setup data of first LP. */
	double A[1*2] = { 1.0, 1.0 };
	double g[2] = { 1.5, 1.0 };
	double lb[2] = { 0.5, -2.0 };
	double ub[2] = { 5.0, 2.0 };
	double lbA[1] = { -1.0 };
	double ubA[1] = { 2.0 };

	/* Setup data of second LP. */
	double g_new[2] = { 1.0, 1.5 };
	double lb_new[2] = { 0.0, -1.0 };
	double ub_new[2] = { 5.0, -0.5 };
	double lbA_new[1] = { -2.0 };
	double ubA_new[1] = { 1.0 };


	/* Setting up QProblem object with zero Hessian matrix. */
	QProblem example( 2,1,HST_ZERO );

	/* Solve first LP. */
	int nWSR = 10;
	example.init( 0,g,A,lb,ub,lbA,ubA, nWSR,0 );

	/* Solve second LP. */
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0 );

	return 0;
}


/*
 *	end of file
 */
