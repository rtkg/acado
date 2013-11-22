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
 *	\file INTERFACES/SCILAB/qpOASESroutines.cpp
 *	\author Holger Diedam, Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Interface that enables to call qpOASES from scilab
 *  (C++ file to provide an interface between the files that
 *  have to be compiled with gcc and the qpOASES library).
 *
 */


#include <iostream>

#include "SQProblem.hpp"


using namespace qpOASES;

/* global pointers to qpOASES objects */
QProblem*  qp  = 0;
QProblemB* qpb = 0;
SQProblem* sqp = 0;


extern "C"
{
	void qpoases(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
					int *nV, int* nC, int* nWSR,
					double* obj, double* x, double* y, int* status, int* nWSRout
					);

	void init(		double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
					int* nV, int* nC, int* nWSR,
					double* obj, double* x, double* y, int* status, int* nWSRout
					);
	void initSB(	double* H, double* g, double* lb, double* ub,
					int* nV, int* nWSR,
					double* obj, double* x, double* y, int* status, int* nWSRout
					);
	void initVM(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
					int* nV, int* nC, int* nWSR,
					double* obj, double* x, double* y, int* status, int* nWSRout
					);

	void hotstart(		double* g, double* lb, double* ub, double* lbA, double* ubA,
						int* nWSR,
						double* obj, double* x, double* y, int* status, int* nWSRout
						);
	void hotstartSB(	double* g, double* lb, double* ub,
						int* nWSR,
						double* obj, double* x, double* y, int* status, int* nWSRout
						);
	void hotstartVM(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
						int* nWSR,
						double* obj, double* x, double* y, int* status, int* nWSRout
						);

	void cleanupp( );
	void cleanupSB( );
	void cleanupVM( );
}



/*
 *	t r a n s f o r m A
 */
void transformA( double* A, int nV, int nC )
{
	int i, j;

	double* A_tmp = new double[nC*nV];

	for( i=0; i<nV*nC; ++i )
		A_tmp[i] = A[i];

	for( i=0; i<nC; ++i )
		for( j=0; j<nV; ++j )
			A[i*nV + j] = A_tmp[j*nC + i];

	delete[] A_tmp;

	return;
}


/*
 *	g e t S t a t u s
 */
int getStatus( returnValue returnvalue )
{
	switch ( returnvalue )
	{
		case SUCCESSFUL_RETURN:
			return 0;

		case RET_MAX_NWSR_REACHED:
			return 1;

		default:
			return -1;
	}
}


/*
 *	q p o a s e s
 */
void qpoases(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
				int *nV, int* nC, int* nWSR,
				double* obj, double* x, double* y, int* status, int* nWSRout
				)
{
	/* transform A into C style matrix */
	transformA( A, *nV,*nC );

	/* setup and solve initial QP */
	QProblem single_qp( *nV,*nC );
	single_qp.setPrintLevel( PL_LOW );
	returnValue returnvalue = single_qp.init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	*obj = single_qp.getObjVal( );
	single_qp.getPrimalSolution( x );
	single_qp.getDualSolution( y );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;

	return;
}


/*
 *	i n i t
 */
void init(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
			int* nV, int* nC, int* nWSR,
			double* obj, double* x, double* y, int* status, int* nWSRout
			)
{
	cleanupp( );

	/* transform A into C style matrix */
	transformA( A, *nV,*nC );

	/* setup and solve initial QP */
	qp = new QProblem( *nV,*nC );
	qp->setPrintLevel( PL_LOW );
	returnValue returnvalue = qp->init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	*obj = qp->getObjVal( );
	qp->getPrimalSolution( x );
	qp->getDualSolution( y );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;

	return;
}


/*
 *	i n i t S B
 */
void initSB(	double* H, double* g, double* lb, double* ub,
				int* nV, int* nWSR,
				double* obj, double* x, double* y, int* status, int* nWSRout
				)
{
	cleanupSB( );

	/* setup and solve initial QP */
	qpb = new QProblemB( *nV );
	qpb->setPrintLevel( PL_LOW );
	returnValue returnvalue = qpb->init( H,g,lb,ub, *nWSR,0 );

	/* assign lhs arguments */
	*obj = qpb->getObjVal( );
	qpb->getPrimalSolution( x );
	qpb->getDualSolution( y );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;

	return;
}


/*
 *	i n i t V M
 */
void initVM(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
				int* nV, int* nC, int* nWSR,
				double* obj, double* x, double* y, int* status, int* nWSRout
				)
{
	cleanupVM( );

	/* transform A into C style matrix */
	transformA( A, *nV,*nC );

	/* setup and solve initial QP */
	sqp = new SQProblem( *nV,*nC );
	sqp->setPrintLevel( PL_LOW );
	returnValue returnvalue = sqp->init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	*obj = sqp->getObjVal( );
	sqp->getPrimalSolution( x );
	sqp->getDualSolution( y );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;

	return;
}


/*
 *	h o t s t a r t
 */
void hotstart(	double* g, double* lb, double* ub, double* lbA, double* ubA,
				int* nWSR,
				double* obj, double* x, double* y, int* status, int* nWSRout
				)
{
	/* has QP been initialised? */
	if ( qp == 0 )
	{
		*status = -1;
		return;
	}

	/* solve QP */
	returnValue returnvalue = qp->hotstart( g,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	*obj = qp->getObjVal( );
	qp->getPrimalSolution( x );
	qp->getDualSolution( y );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;

	return;
}


/*
 *	h o t s t a r t S B
 */
void hotstartSB(	double* g, double* lb, double* ub,
					int* nWSR,
					double* obj, double* x, double* y, int* status, int* nWSRout
					)
{
	/* has QP been initialised? */
	if ( qpb == 0 )
	{
		*status = -1;
		return;
	}

	/* solve QP */
	returnValue returnvalue = qpb->hotstart( g,lb,ub, *nWSR,0 );

	/* assign lhs arguments */
	*obj = qpb->getObjVal( );
	qpb->getPrimalSolution( x );
	qpb->getDualSolution( y );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;

	return;
}


/*
 *	h o t s t a r t V M
 */
void hotstartVM(	double* H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA,
					int* nWSR,
					double* obj, double* x, double* y, int* status, int* nWSRout
					)
{
	/* has QP been initialised? */
	if ( sqp == 0 )
	{
		*status = -1;
		return;
	}

	/* transform A into C style matrix */
	transformA( A, sqp->getNV( ),sqp->getNC( ) );

	/* solve QP */
	returnValue returnvalue = sqp->hotstart( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	*obj = sqp->getObjVal( );
	sqp->getPrimalSolution( x );
	sqp->getDualSolution( y );
	*status = getStatus( returnvalue );
	*nWSRout = *nWSR;

	return;
}


/*
 *	c l e a n u p p
 */
void cleanupp( )
{
	/* Remark: A function cleanup already exists! */
	if ( qp != 0 )
	{
		delete qp;
		qp = 0;
	}

	return;
}


/*
 *	c l e a n u p S B
 */
void cleanupSB( )
{
	if ( qpb != 0 )
	{
		delete qpb;
		qpb = 0;
	}

	return;
}


/*
 *	c l e a n u p V M
 */
void cleanupVM( )
{
	if ( sqp != 0 )
	{
		delete sqp;
		sqp = 0;
	}

	return;
}


/*
 *	end of file
 */
