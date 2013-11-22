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
 *	\file SRC/QProblem.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Implementation of the QProblem class which is able to use the newly
 *	developed online active set strategy for parametric quadratic programming.
 */


#include <stdio.h>

#include <QProblem.hpp>


//#define __MANY_CONSTRAINTS__


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	Q P r o b l e m
 */
QProblem::QProblem( ) : QProblemB( )
{
	A   = 0;
	lbA = 0;
	ubA = 0;

	constraints = 0;

	sizeT = 0;
	T = 0;
	Q = 0;

	Ax_l = 0;
	Ax_u = 0;

	cyclingManager = 0;

	constraintProduct = 0;
}


/*
 *	Q P r o b l e m
 */
QProblem::QProblem( int _nV, int _nC ) : QProblemB( _nV )
{
	/* consistency checks */
	if ( _nV <= 0 )
		_nV = 1;

	if ( _nC < 0 )
	{
		_nC = 0;
		THROWERROR( RET_INVALID_ARGUMENTS );
	}

	A = new double[_nC*_nV];
	lbA = new double[_nC];
	ubA = new double[_nC];

	constraints = new Constraints( _nC );

	delete[] y; /* y of no constraints version too short! */
	y = new double[_nV+_nC];

	sizeT = _nC;
	if ( _nC > _nV )
		sizeT = _nV;
	T = new double[sizeT*sizeT];
	Q = new double[_nV*_nV];

	Ax_l = new double[_nC];
	Ax_u = new double[_nC];

	cyclingManager = new CyclingManager( _nV,_nC );

	constraintProduct = 0;
}


/*
 *	Q P r o b l e m
 */
QProblem::QProblem( int _nV, int _nC, HessianType _hessianType ) : QProblemB( _nV,_hessianType )
{
	/* consistency checks */
	if ( _nV <= 0 )
		_nV = 1;

	if ( _nC < 0 )
	{
		_nC = 0;
		THROWERROR( RET_INVALID_ARGUMENTS );
	}

	A = new double[_nC*_nV];
	lbA = new double[_nC];
	ubA = new double[_nC];

	constraints = new Constraints( _nC );

	delete[] y; /* y of no constraints version too short! */
	y = new double[_nV+_nC];

	sizeT = _nC;
	if ( _nC > _nV )
		sizeT = _nV;
	T = new double[sizeT*sizeT];
	Q = new double[_nV*_nV];

	Ax_l = new double[_nC];
	Ax_u = new double[_nC];

	cyclingManager = new CyclingManager( _nV,_nC );

	constraintProduct = 0;
}


/*
 *	Q P r o b l e m
 */
QProblem::QProblem( const QProblem& rhs ) :	QProblemB( rhs )
{
	int i;

	int _nV = rhs.getNV( );
	int _nC = rhs.getNC( );

	if ( rhs.constraints != 0 )
		constraints = new Constraints( *(rhs.constraints) );
	else
		constraints = 0;

	if ( rhs.A != 0 )
	{
		A = new double[_nC*_nV];
		for( i=0; i<_nC*_nV; ++i )
			A[i] = rhs.A[i];
	}
	else
		A = 0;

	if ( rhs.lbA != 0 )
	{
		lbA = new double[_nC];
		setLBA( rhs.lbA );
	}
	else
		lbA = 0;

	if ( rhs.ubA != 0 )
	{
		ubA = new double[_nC];
		setUBA( rhs.ubA );
	}
	else
		ubA = 0;

	if ( rhs.y != 0 )
	{
		delete[] y; /* y of no constraints version too short! */
		y = new double[_nV+_nC];
		for( i=0; i<(_nV+_nC); ++i )
			y[i] = rhs.y[i];
	}
	else
		y = 0;

	sizeT = rhs.sizeT;

	if ( rhs.T != 0 )
	{
		T = new double[sizeT*sizeT];
		for( i=0; i<sizeT*sizeT; ++i )
			T[i] = rhs.T[i];
	}
	else
		T = 0;

	if ( rhs.Q != 0 )
	{
		Q = new double[_nV*_nV];
		for( i=0; i<_nV*_nV; ++i )
			Q[i] = rhs.Q[i];
	}
	else
		Q = 0;

	if ( rhs.Ax_l != 0 )
	{
		Ax_l = new double[_nC];
		for( i=0; i<_nC; ++i )
			Ax_l[i] = rhs.Ax_l[i];
	}
	else
		Ax_l = 0;

	if ( rhs.Ax_u != 0 )
	{
		Ax_u = new double[_nC];
		for( i=0; i<_nC; ++i )
			Ax_u[i] = rhs.Ax_u[i];
	}
	else
		Ax_u = 0;

	if ( rhs.cyclingManager != 0 )
	{
		cyclingManager = new CyclingManager( _nV,_nC );
		*cyclingManager = *(rhs.cyclingManager);
	}
	else
		cyclingManager = 0;

	if ( rhs.constraintProduct != 0 )
		constraintProduct = rhs.constraintProduct;
	else
		constraintProduct = 0;
}


/*
 *	~ Q P r o b l e m
 */
QProblem::~QProblem( )
{
	if ( A != 0 )
		delete[] A;

	if ( lbA != 0 )
		delete[] lbA;

	if ( ubA != 0 )
		delete[] ubA;

	if ( constraints != 0 )
		delete constraints;

	if ( T != 0 )
		delete[] T;

	if ( Q != 0 )
		delete[] Q;

	if ( Ax_l != 0 )
		delete[] Ax_l;

	if ( Ax_u != 0 )
		delete[] Ax_u;

	if ( cyclingManager != 0 )
		delete cyclingManager;
}


/*
 *	o p e r a t o r =
 */
QProblem& QProblem::operator=( const QProblem& rhs )
{
	int i;

	if ( this != &rhs )
	{
		QProblemB::operator=( rhs );

		if ( A != 0 )
			delete[] A;

		if ( lbA != 0 )
			delete[] lbA;

		if ( ubA != 0 )
			delete[] ubA;

		if ( constraints != 0 )
			delete constraints;

		if ( T != 0 )
			delete[] T;

		if ( Q != 0 )
			delete[] Q;

		if ( Ax_l != 0 )
			delete[] Ax_l;

		if ( Ax_u != 0 )
			delete[] Ax_u;

		if ( cyclingManager != 0 )
			delete cyclingManager;


		int _nV = rhs.getNV( );
		int _nC = rhs.getNC( );

		if ( rhs.constraints != 0 )
			constraints = new Constraints( *(rhs.constraints) );
		else
			constraints = 0;

		if ( rhs.A != 0 )
		{
			A = new double[_nC*_nV];
			for( i=0; i<_nC*_nV; ++i )
				A[i] = rhs.A[i];
		}
		else
			A = 0;

		if ( rhs.lbA != 0 )
		{
			lbA = new double[_nC];
			setLBA( rhs.lbA );
		}
		else
			lbA = 0;

		if ( rhs.ubA != 0 )
		{
			ubA = new double[_nC];
			setUBA( rhs.ubA );
		}
		else
			ubA = 0;

		if ( rhs.y != 0 )
		{
			delete[] y; /* y of no constraints version too short! */
			y = new double[_nV+_nC];
			for( i=0; i<(_nV+_nC); ++i )
				y[i] = rhs.y[i];
		}
		else
			y = 0;

		sizeT = rhs.sizeT;

		if ( rhs.T != 0 )
		{
			T = new double[sizeT*sizeT];
			for( i=0; i<sizeT*sizeT; ++i )
				T[i] = rhs.T[i];
		}
		else
			T = 0;

		if ( rhs.Q != 0 )
		{
			Q = new double[_nV*_nV];
			for( i=0; i<_nV*_nV; ++i )
				Q[i] = rhs.Q[i];
		}
		else
			Q = 0;

		if ( rhs.Ax_l != 0 )
		{
			Ax_l = new double[_nC];
			for( i=0; i<_nC; ++i )
				Ax_l[i] = rhs.Ax_l[i];
		}
		else
			Ax_l = 0;

		if ( rhs.Ax_u != 0 )
		{
			Ax_u = new double[_nC];
			for( i=0; i<_nC; ++i )
				Ax_u[i] = rhs.Ax_u[i];
		}
		else
			Ax_u = 0;

		if ( rhs.cyclingManager != 0 )
		{
			cyclingManager = new CyclingManager( _nV,_nC );
			*cyclingManager = *(rhs.cyclingManager);
		}
		else
			cyclingManager = 0;
		}

		if ( rhs.constraintProduct != 0 )
				constraintProduct = rhs.constraintProduct;
			else
				constraintProduct = 0;

	return *this;
}


/*
 *	r e s e t
 */
returnValue QProblem::reset( )
{
	int i;
	int nV = getNV( );
	int nC = getNC( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );


	/* 1) Reset bounds, Cholesky decomposition and status flags. */
	if ( QProblemB::reset( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_RESET_FAILED );

	/* 2) Reset constraints. */
	if ( constraints != 0 )
		delete constraints;
	constraints = new Constraints( nC );

	/* 3) Reset TQ factorisation. */
	for( i=0; i<sizeT*sizeT; ++i )
		T[i] = 0.0;

	for( i=0; i<nV*nV; ++i )
		Q[i] = 0.0;

	/* 4) Reset cycling manager. */
	if ( cyclingManager != 0 )
		if ( cyclingManager->clearCyclingData( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_RESET_FAILED );

	/* 5) Reset constraint product pointer. */
	constraintProduct = 0;

	return SUCCESSFUL_RETURN;
}


/*
 *	i n i t
 */
returnValue QProblem::init(	const double* const _H, const double* const _g, const double* const _A,
							const double* const _lb, const double* const _ub,
							const double* const _lbA, const double* const _ubA,
							int& nWSR, double* const cputime
							)
{
	if ( getNV( ) == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* 1) Consistency check. */
	if ( isInitialised( ) == BT_TRUE )
	{
		THROWWARNING( RET_QP_ALREADY_INITIALISED );
		reset( );
	}

	/* 2) Setup QP data. */
	if ( setupQPdata( _H,_g,_A,_lb,_ub,_lbA,_ubA ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 3) Call to main initialisation routine (without any additional information). */
	return solveInitialQP( 0,0,0,0, nWSR,cputime );
}


/*
 *	i n i t
 */
returnValue QProblem::init(	const char* const H_file, const char* const g_file, const char* const A_file,
							const char* const lb_file, const char* const ub_file,
							const char* const lbA_file, const char* const ubA_file,
							int& nWSR, double* const cputime
							)
{
	if ( getNV( ) == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* 1) Consistency check. */
	if ( isInitialised( ) == BT_TRUE )
	{
		THROWWARNING( RET_QP_ALREADY_INITIALISED );
		reset( );
	}

	/* 2) Setup QP data from files. */
	if ( setupQPdataFromFile( H_file,g_file,A_file,lb_file,ub_file,lbA_file,ubA_file ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_UNABLE_TO_READ_FILE );

	/* 3) Call to main initialisation routine (without any additional information). */
	return solveInitialQP( 0,0,0,0, nWSR,cputime );
}


/*
 *	i n i t
 */
returnValue QProblem::init( const double* const _H, const double* const _g, const double* const _A,
							const double* const _lb, const double* const _ub,
							const double* const _lbA, const double* const _ubA,
							int& nWSR, double* const cputime,
							const double* const xOpt, const double* const yOpt,
							const Bounds* const guessedBounds, const Constraints* const guessedConstraints
							)
{
	int i;
	int nV = getNV( );
	int nC = getNC( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* 1) Consistency checks. */
	if ( isInitialised( ) == BT_TRUE )
	{
		THROWWARNING( RET_QP_ALREADY_INITIALISED );
		reset( );
	}

	if ( guessedBounds != 0 )
	{
		for( i=0; i<nV; ++i )
		{
			if ( ( guessedBounds->getStatus( i ) == ST_UNDEFINED ) ||
				 ( guessedBounds->getStatus( i ) == ST_DISABLED )  ||
				 ( guessedBounds->getStatus( i ) == ST_DISABLING ) )
				return THROWERROR( RET_INVALID_ARGUMENTS );
		}
	}

	if ( guessedConstraints != 0 )
	{
		for( i=0; i<nC; ++i )
			if ( guessedConstraints->getStatus( i ) == ST_UNDEFINED )
				return THROWERROR( RET_INVALID_ARGUMENTS );
	}

	/* exclude these possibilities in order to avoid inconsistencies */
	if ( ( xOpt == 0 ) && ( yOpt != 0 ) && ( ( guessedBounds != 0 ) || ( guessedConstraints != 0 ) ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 2) Setup QP data. */
	if ( setupQPdata( _H,_g,_A,_lb,_ub,_lbA,_ubA ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 3) Call to main initialisation routine. */
	return solveInitialQP( xOpt,yOpt,guessedBounds,guessedConstraints, nWSR,cputime );
}


/*
 *	i n i t
 */
returnValue QProblem::init( const char* const H_file, const char* const g_file, const char* const A_file,
							const char* const lb_file, const char* const ub_file,
							const char* const lbA_file, const char* const ubA_file,
							int& nWSR, double* const cputime,
							const double* const xOpt, const double* const yOpt,
							const Bounds* const guessedBounds, const Constraints* const guessedConstraints
							)
{
	int i;
	int nV = getNV( );
	int nC = getNC( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* 1) Consistency checks. */
	if ( isInitialised( ) == BT_TRUE )
	{
		THROWWARNING( RET_QP_ALREADY_INITIALISED );
		reset( );
	}

	for( i=0; i<nV; ++i )
	{
		if ( ( guessedBounds->getStatus( i ) == ST_UNDEFINED ) ||
			 ( guessedBounds->getStatus( i ) == ST_DISABLED )  ||
			 ( guessedBounds->getStatus( i ) == ST_DISABLING ) )
			return THROWERROR( RET_INVALID_ARGUMENTS );
	}

	for( i=0; i<nC; ++i )
		if ( guessedConstraints->getStatus( i ) == ST_UNDEFINED )
			return THROWERROR( RET_INVALID_ARGUMENTS );

	/* exclude these possibilities in order to avoid inconsistencies */
	if ( ( xOpt == 0 ) && ( yOpt != 0 ) && ( ( guessedBounds != 0 ) || ( guessedConstraints != 0 ) ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 2) Setup QP data from files. */
	if ( setupQPdataFromFile( H_file,g_file,A_file,lb_file,ub_file,lbA_file,ubA_file ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_UNABLE_TO_READ_FILE );

	/* 3) Call to main initialisation routine. */
	return solveInitialQP( xOpt,yOpt,guessedBounds,guessedConstraints, nWSR,cputime );
}


/*
 *	h o t s t a r t
 */
returnValue QProblem::hotstart(	const double* const g_new,
								const double* const lb_new, const double* const ub_new,
								const double* const lbA_new, const double* const ubA_new,
								int& nWSR, double* const cputime
								)
{
	if ( getNV( ) == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	++count;

	if ( usingRegularisation( ) == BT_TRUE )
		return solveRegularisedQP( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime );
	else
		return solveQP(            g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime,0 );
}


/*
 *	h o t s t a r t
 */
returnValue QProblem::hotstart(	const char* const g_file,
								const char* const lb_file, const char* const ub_file,
								const char* const lbA_file, const char* const ubA_file,
								int& nWSR, double* const cputime
								)
{
	int nV = getNV( );
	int nC = getNC( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* consistency check */
	if ( g_file == 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	/* 1) Allocate memory (if bounds exist). */
	double* g_new  = new double[nV];
	double* lb_new = 0;
	double* ub_new = 0;
	double* lbA_new = 0;
	double* ubA_new = 0;

	if ( lb_file != 0 )
		lb_new = new double[nV];
	if ( ub_file != 0 )
		ub_new = new double[nV];
	if ( lbA_file != 0 )
		lbA_new = new double[nC];
	if ( ubA_file != 0 )
		ubA_new = new double[nC];

	/* 2) Load new QP vectors from file. */
	returnValue returnvalue;
	returnvalue = loadQPvectorsFromFile(	g_file,lb_file,ub_file,lbA_file,ubA_file,
											g_new,lb_new,ub_new,lbA_new,ubA_new
											);
	if ( returnvalue != SUCCESSFUL_RETURN )
	{
		if ( ubA_file != 0 )
			delete[] ubA_new;
		if ( lbA_file != 0 )
			delete[] lbA_new;
		if ( ub_file != 0 )
			delete[] ub_new;
		if ( lb_file != 0 )
			delete[] lb_new;
		delete[] g_new;

		return THROWERROR( RET_UNABLE_TO_READ_FILE );
	}

	/* 3) Actually perform hotstart. */
	++count;

	if ( usingRegularisation( ) == BT_TRUE )
		returnvalue = solveRegularisedQP( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime );
	else
		returnvalue = solveQP(	          g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime,0 );

	/* 4) Free memory. */
	if ( ubA_file != 0 )
		delete[] ubA_new;
	if ( lbA_file != 0 )
		delete[] lbA_new;
	if ( ub_file != 0 )
		delete[] ub_new;
	if ( lb_file != 0 )
		delete[] lb_new;
	delete[] g_new;

	return returnvalue;
}


/*
 *	h o t s t a r t
 */
returnValue QProblem::hotstart(	const double* const g_new,
								const double* const lb_new, const double* const ub_new,
								const double* const lbA_new, const double* const ubA_new,
								int& nWSR, double* const cputime,
								const Bounds* const guessedBounds, const Constraints* const guessedConstraints
								)
{
	int nV = getNV( );
	int nC = getNC( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );


	/* start runtime measurement */
	double starttime = 0.0;
	if ( cputime != 0 )
		starttime = getCPUtime( );


	/* 1) Update working sets according to guesses for working sets of bounds and constraints. */
	if ( ( guessedBounds != 0 ) && ( guessedConstraints != 0 ) )
	{
		if ( setupAuxiliaryQP( guessedBounds,guessedConstraints ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	if ( ( guessedBounds == 0 ) && ( guessedConstraints != 0 ) )
	{
		/* create empty bounds for setting up auxiliary QP */
		Bounds emptyBounds( nV );
		if ( emptyBounds.setupAllFree( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		if ( setupAuxiliaryQP( &emptyBounds,guessedConstraints ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	if ( ( guessedBounds != 0 ) && ( guessedConstraints == 0 ) )
	{
		/* create empty constraints for setting up auxiliary QP */
		Constraints emptyConstraints( nC );
		if ( emptyConstraints.setupAllInactive( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		if ( setupAuxiliaryQP( guessedBounds,&emptyConstraints ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	if ( ( guessedBounds == 0 ) && ( guessedConstraints == 0 ) )
	{
		/* create empty bounds and constraints for setting up auxiliary QP */
		Bounds emptyBounds( nV );
		Constraints emptyConstraints( nC );
		if ( emptyBounds.setupAllFree( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
		if ( emptyConstraints.setupAllInactive( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		if ( setupAuxiliaryQP( &emptyBounds,&emptyConstraints ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	status = QPS_AUXILIARYQPSOLVED;


	/* 2) Perform usual homotopy. */

	/* Allow only remaining CPU time for usual hotstart. */
	if ( cputime != 0 )
		*cputime -= getCPUtime( ) - starttime;


	returnValue returnvalue;

	++count;

	if ( usingRegularisation( ) == BT_TRUE )
		returnvalue = solveRegularisedQP( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime );
	else
		returnvalue = solveQP(	          g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime,0 );


	/* stop runtime measurement */
	if ( cputime != 0 )
		*cputime = getCPUtime( ) - starttime;

	return returnvalue;
}


/*
 *	h o t s t a r t
 */
returnValue QProblem::hotstart(	const char* const g_file,
								const char* const lb_file, const char* const ub_file,
								const char* const lbA_file, const char* const ubA_file,
								int& nWSR, double* const cputime,
								const Bounds* const guessedBounds, const Constraints* const guessedConstraints
								)
{
	int nV = getNV( );
	int nC = getNC( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* consistency check */
	if ( g_file == 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	/* 1) Allocate memory (if bounds exist). */
	double* g_new  = new double[nV];
	double* lb_new = 0;
	double* ub_new = 0;
	double* lbA_new = 0;
	double* ubA_new = 0;

	if ( lb_file != 0 )
		lb_new = new double[nV];
	if ( ub_file != 0 )
		ub_new = new double[nV];
	if ( lbA_file != 0 )
		lbA_new = new double[nC];
	if ( ubA_file != 0 )
		ubA_new = new double[nC];

	/* 2) Load new QP vectors from file. */
	returnValue returnvalue;
	returnvalue = loadQPvectorsFromFile(	g_file,lb_file,ub_file,lbA_file,ubA_file,
											g_new,lb_new,ub_new,lbA_new,ubA_new
											);
	if ( returnvalue != SUCCESSFUL_RETURN )
	{
		if ( ubA_file != 0 )
			delete[] ubA_new;
		if ( lbA_file != 0 )
			delete[] lbA_new;
		if ( ub_file != 0 )
			delete[] ub_new;
		if ( lb_file != 0 )
			delete[] lb_new;
		delete[] g_new;

		return THROWERROR( RET_UNABLE_TO_READ_FILE );
	}

	/* 3) Actually perform hotstart using initialised homotopy. */
	returnvalue = hotstart(	g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime,
							guessedBounds,guessedConstraints
							);

	/* 4) Free memory. */
	if ( ubA_file != 0 )
		delete[] ubA_new;
	if ( lbA_file != 0 )
		delete[] lbA_new;
	if ( ub_file != 0 )
		delete[] ub_new;
	if ( lb_file != 0 )
		delete[] lb_new;
	delete[] g_new;

	return returnvalue;
}


/*
 *	g e t N Z
 */
int QProblem::getNZ( ) const
{
	/* nZ = nFR - nAC */
	return getNFR( ) - getNAC( );
}


/*
 *	g e t D u a l S o l u t i o n
 */
returnValue QProblem::getDualSolution( double* const yOpt ) const
{
	int i;

	/* return optimal dual solution vector
	 * only if current QP has been solved */
	if ( ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
		 ( getStatus( ) == QPS_SOLVED ) )
	{
		for( i=0; i<getNV( )+getNC( ); ++i )
			yOpt[i] = y[i];

		return SUCCESSFUL_RETURN;
	}
	else
	{
		return RET_QP_NOT_SOLVED;
	}
}



/*
 *	s e t C o n s t r a i n t P r o d u c t
 */
returnValue QProblem::setConstraintProduct( ConstraintProduct* const _constraintProduct )
{
	constraintProduct = _constraintProduct;

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t P r o p e r t i e s
 */
returnValue QProblem::printProperties( )
{
	#ifndef __XPCTARGET__
	/* Do not print properties if print level is set to none! */
	if ( printLevel == PL_NONE )
		return SUCCESSFUL_RETURN;

	char myPrintfString[80];

	myPrintf( "\n#################   qpOASES  --  QP PROPERTIES   #################\n" );
	myPrintf( "\n" );

	/* 1) Variables properties. */
	snprintf( myPrintfString,80,  "Number of Variables: %4.1d\n",getNV( ) );
	myPrintf( myPrintfString );

	if ( bounds->isNoLower( ) == BT_TRUE )
			myPrintf( "Variables are not bounded from below.\n" );
		else
			myPrintf( "Variables are bounded from below.\n" );

	if ( bounds->isNoUpper( ) == BT_TRUE )
			myPrintf( "Variables are not bounded from above.\n" );
		else
			myPrintf( "Variables are bounded from above.\n" );

	myPrintf( "\n" );


	/* 2) Constraints properties. */
	snprintf( myPrintfString,80,  "Total number of Constraints:      %4.1d\n",getNC( ) );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,  "Number of Equality Constraints:   %4.1d\n",getNEC( ) );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,  "Number of Inequality Constraints: %4.1d\n",getNC( )-getNEC( ) );
	myPrintf( myPrintfString );

	if ( getNC( ) > 0 )
	{
		if ( constraints->isNoLower( ) == BT_TRUE )
				myPrintf( "Constraints are not bounded from below.\n" );
			else
				myPrintf( "Constraints are bounded from below.\n" );

		if ( constraints->isNoUpper( ) == BT_TRUE )
				myPrintf( "Constraints are not bounded from above.\n" );
			else
				myPrintf( "Constraints are bounded from above.\n" );
	}

	myPrintf( "\n" );


	/* 3) Further properties. */
	switch ( hessianType )
	{
		case HST_ZERO:
			myPrintf( "Hessian is zero matrix (i.e. actually an LP is solved).\n" );
			break;

		case HST_IDENTITY:
			myPrintf( "Hessian is identity matrix.\n" );
			break;

		case HST_POSDEF:
			myPrintf( "Hessian matrix is (strictly) positive definite.\n" );
			break;

		case HST_POSDEF_NULLSPACE:
			myPrintf( "Hessian matrix is positive definite on null space of active constraints.\n" );
			break;

		case HST_SEMIDEF:
			myPrintf( "Hessian matrix is positive semi-definite.\n" );
			break;

		default:
			myPrintf( "Hessian matrix has unknown type.\n" );
			break;
	}

	if ( infeasible == BT_TRUE )
		myPrintf( "QP was found to be infeasible.\n" );
	else
		myPrintf( "QP seems to be feasible.\n" );

	if ( unbounded == BT_TRUE )
		myPrintf( "QP was found to be unbounded from below.\n" );
	else
		myPrintf( "QP seems to be bounded from below.\n" );

	myPrintf( "\n" );


	/* 4) QP object properties. */
	switch ( status )
	{
		case QPS_NOTINITIALISED:
			myPrintf( "Status of QP object: freshly instantiated or reset.\n" );
			break;

		case QPS_PREPARINGAUXILIARYQP:
			myPrintf( "Status of QP object: an auxiliary QP is currently setup.\n" );
			break;

		case QPS_AUXILIARYQPSOLVED:
			myPrintf( "Status of QP object: an auxilary QP was solved.\n" );
			break;

		case QPS_PERFORMINGHOMOTOPY:
			myPrintf( "Status of QP object: a homotopy step is performed.\n" );
			break;

		case QPS_HOMOTOPYQPSOLVED:
			myPrintf( "Status of QP object: an intermediate QP along the homotopy path was solved.\n" );
			break;

		case QPS_SOLVED:
			myPrintf( "Status of QP object: solution of the actual QP was found.\n" );
			break;
	}

	switch ( printLevel	)
	{
		case PL_LOW:
			myPrintf( "Print level of QP object is low, i.e. only error are printed.\n" );
			break;

		case PL_MEDIUM:
			myPrintf( "Print level of QP object is medium, i.e. error and warnings are printed.\n" );
			break;

		case PL_HIGH:
			myPrintf( "Print level of QP object is high, i.e. all available output is printed.\n" );
			break;

		default:
			break;
	}

	myPrintf( "\n" );
	#endif

	return SUCCESSFUL_RETURN;
}



/*****************************************************************************
 *  P R O T E C T E D                                                        *
 *****************************************************************************/

/*
 *	s o l v e I n i t i a l Q P
 */
returnValue QProblem::solveInitialQP(	const double* const xOpt, const double* const yOpt,
										const Bounds* const guessedBounds, const Constraints* const guessedConstraints,
										int& nWSR, double* const cputime
										)
{
	int i;

	/* some definitions */
	int nV = getNV( );
	int nC = getNC( );


	/* start runtime measurement */
	double starttime = 0.0;
	if ( cputime != 0 )
		starttime = getCPUtime( );

	status = QPS_NOTINITIALISED;


	/* I) ANALYSE QP DATA: */
	#ifdef __MANY_CONSTRAINTS__
	/* 0) Checks if l1-norm of each constraint is not greater than 1. */
	int j;
	double l1;
	for( i=0; i<nC; ++i )
	{
		l1 = 0.0;
		for( j=0; j<nV; ++j )
			l1 += fabs( A[i*nC + j] );

		if ( l1 > 1.0 + 10.0*EPS )
			return THROWERROR( RET_CONSTRAINTS_ARE_NOT_SCALED );
	}
	#endif

	/* 1) Check if Hessian happens to be the identity matrix. */
	if ( determineHessianType( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	/* 2) Setup type of bounds and constraints (i.e. unbounded, implicitly fixed etc.). */
	if ( setupSubjectToType( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	/* 3) Initialise cycling manager. */
	cyclingManager->clearCyclingData( );

	status = QPS_PREPARINGAUXILIARYQP;


	/* II) SETUP AUXILIARY QP WITH GIVEN OPTIMAL SOLUTION: */
	/* 1) Setup bounds and constraints data structure. */
	if ( bounds->setupAllFree( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	if ( constraints->setupAllInactive( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	/* 2) Setup optimal primal/dual solution for auxiliary QP. */
	if ( setupAuxiliaryQPsolution( xOpt,yOpt ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	/* 3) Obtain linear independent working set for auxiliary QP. */
	Bounds* auxiliaryBounds = new Bounds( nV );
	Constraints* auxiliaryConstraints = new Constraints( nC );

	if ( obtainAuxiliaryWorkingSet(	xOpt,yOpt,guessedBounds,guessedConstraints,
									auxiliaryBounds,auxiliaryConstraints ) != SUCCESSFUL_RETURN )
	{
		delete auxiliaryConstraints; delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED );
	}

	/* 4) Setup working set of auxiliary QP and setup matrix factorisations. */
	/* a) Regularise Hessian if necessary. */
	if ( ( hessianType == HST_ZERO ) || ( hessianType == HST_SEMIDEF ) )
	{
		if ( regulariseHessian( ) != SUCCESSFUL_RETURN )
		{
			delete auxiliaryConstraints; delete auxiliaryBounds;
			return THROWERROR( RET_INIT_FAILED );
		}
	}

	/* b) TQ factorisation. */
	if ( setupTQfactorisation( ) != SUCCESSFUL_RETURN )
	{
		delete auxiliaryConstraints; delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED_TQ );
	}

	/* c) Working set of auxiliary QP. */
	if ( setupAuxiliaryWorkingSet( auxiliaryBounds,auxiliaryConstraints,BT_TRUE ) != SUCCESSFUL_RETURN )
	{
		delete auxiliaryConstraints; delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED );
	}

	/* d) Cholesky decomposition. */
	returnValue returnvalueCholesky;

	if ( ( getNAC( ) + getNFX( ) ) == 0 )
	{
		/* Factorise full Hessian if no bounds/constraints are active. */
		returnvalueCholesky = setupCholeskyDecomposition( );

		/* If Hessian is not positive definite, regularise and try again. */
		if ( returnvalueCholesky == RET_HESSIAN_NOT_SPD )
		{
			if ( regulariseHessian( ) != SUCCESSFUL_RETURN )
			{
				delete auxiliaryConstraints; delete auxiliaryBounds;
				return THROWERROR( RET_INIT_FAILED );
			}

			if ( setupCholeskyDecomposition( ) != SUCCESSFUL_RETURN )
			{
				delete auxiliaryConstraints; delete auxiliaryBounds;
				return THROWERROR( RET_INIT_FAILED_CHOLESKY );
			}
		}

		if ( returnvalueCholesky == RET_INDEXLIST_CORRUPTED )
		{
			delete auxiliaryConstraints; delete auxiliaryBounds;
			return THROWERROR( RET_INIT_FAILED_CHOLESKY );
		}
	}
	else
	{
		/* Factorise projected Hessian if there active bounds/constraints. */
		returnvalueCholesky = setupCholeskyDecompositionProjected( );

		/* If Hessian is not positive definite, regularise and try again. */
		if ( returnvalueCholesky == RET_HESSIAN_NOT_SPD )
		{
			if ( regulariseHessian( ) != SUCCESSFUL_RETURN )
			{
				delete auxiliaryConstraints; delete auxiliaryBounds;
				return THROWERROR( RET_INIT_FAILED );
			}

			if ( setupCholeskyDecompositionProjected( ) != SUCCESSFUL_RETURN )
			{
				delete auxiliaryConstraints; delete auxiliaryBounds;
				return THROWERROR( RET_INIT_FAILED_CHOLESKY );
			}
		}

		if ( returnvalueCholesky == RET_INDEXLIST_CORRUPTED )
		{
			delete auxiliaryConstraints; delete auxiliaryBounds;
			return THROWERROR( RET_INIT_FAILED_CHOLESKY );
		}
	}

	/* 5) Store original QP formulation... */
	double* g_original = new double[nV];
	double* lb_original = new double[nV];
	double* ub_original = new double[nV];
	double* lbA_original = new double[nC];
	double* ubA_original = new double[nC];

	for( i=0; i<nV; ++i )
	{
		g_original[i] = g[i];
		lb_original[i] = lb[i];
		ub_original[i] = ub[i];
	}

	for( i=0; i<nC; ++i )
	{
		lbA_original[i] = lbA[i];
		ubA_original[i] = ubA[i];
	}

	/* ... and setup QP data of an auxiliary QP having an optimal solution
	 * as specified by the user (or xOpt = yOpt = 0, by default). */
	if ( setupAuxiliaryQPgradient( ) != SUCCESSFUL_RETURN )
	{
		delete[] ubA_original; delete[] lbA_original; delete[] ub_original; delete[] lb_original; delete[] g_original;
		delete auxiliaryConstraints; delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED );
	}

	if ( setupAuxiliaryQPbounds( auxiliaryBounds,auxiliaryConstraints,BT_TRUE ) != SUCCESSFUL_RETURN )
	{
		delete[] ubA_original; delete[] lbA_original; delete[] ub_original; delete[] lb_original; delete[] g_original;
		delete auxiliaryConstraints; delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED );
	}

	delete auxiliaryConstraints; delete auxiliaryBounds;
	status = QPS_AUXILIARYQPSOLVED;


	/* III) SOLVE ACTUAL INITIAL QP: */
	/* Allow only remaining CPU time for usual hotstart. */
	if ( cputime != 0 )
		*cputime -= getCPUtime( ) - starttime;

	/* Use hotstart method to find the solution of the original initial QP,... */
	returnValue returnvalue = hotstart( g_original,lb_original,ub_original,lbA_original,ubA_original, nWSR,cputime );

	/* ... deallocate memory,... */
	delete[] ubA_original; delete[] lbA_original; delete[] ub_original; delete[] lb_original; delete[] g_original;

	/* ... check for infeasibility and unboundedness... */
	if ( isInfeasible( ) == BT_TRUE )
		return THROWERROR( RET_INIT_FAILED_INFEASIBILITY );

	if ( isUnbounded( ) == BT_TRUE )
		return THROWERROR( RET_INIT_FAILED_UNBOUNDEDNESS );

	/* ... and internal errors. */
	if ( ( returnvalue != SUCCESSFUL_RETURN ) && ( returnvalue != RET_MAX_NWSR_REACHED ) )
		return THROWERROR( RET_INIT_FAILED_HOTSTART );


	/* stop runtime measurement */
	if ( cputime != 0 )
		*cputime = getCPUtime( ) - starttime;

	THROWINFO( RET_INIT_SUCCESSFUL );

	return returnvalue;
}


/*
 *	s o l v e Q P
 */
returnValue QProblem::solveQP(	const double* const g_new,
								const double* const lb_new, const double* const ub_new,
								const double* const lbA_new, const double* const ubA_new,
								int& nWSR, double* const cputime, int nWSRperformed
								)
{
	int iter;
	int nV  = getNV( );
	int nC  = getNC( );

	/* consistency check */
	if ( ( getStatus( ) == QPS_NOTINITIALISED )       ||
		 ( getStatus( ) == QPS_PREPARINGAUXILIARYQP ) ||
		 ( getStatus( ) == QPS_PERFORMINGHOMOTOPY )   )
	{
		return THROWERROR( RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED );
	}

	/* start runtime measurement */
	double starttime = 0.0;
	if ( cputime != 0 )
		starttime = getCPUtime( );


	/* I) PREPARATIONS */
	/* 1) Allocate delta vectors of gradient and (constraints') bounds,
	 *    index arrays and step direction arrays. */
	int nFR, nFX, nAC, nIAC;

	int *FR_idx  = new int[nV];
	int *FX_idx  = new int[nV];
	int *AC_idx  = new int[nC];
	int *IAC_idx = new int[nC];

	double* delta_xFR = new double[nV];
	double* delta_xFX = new double[nV];
	double* delta_yAC = new double[nC];
	double* delta_yFX = new double[nV];
	double* delta_Ax_l = new double[nC];
	double* delta_Ax_u = new double[nC];

	double* delta_g   = new double[nV];
	double* delta_lb  = new double[nV];
	double* delta_ub  = new double[nV];
	double* delta_lbA = new double[nC];
	double* delta_ubA = new double[nC];

	returnValue returnvalue;
	BooleanType Delta_bC_isZero, Delta_bB_isZero;

	int BC_idx;
	SubjectToStatus BC_status;
	BooleanType BC_isBound;

	char messageString[80];

	/* 2) Update type of bounds and constraints, e.g.
	 *    a former equality constraint might have become a normal one etc. */
	if ( setupSubjectToType( lb_new,ub_new,lbA_new,ubA_new ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_HOTSTART_FAILED );

	/* 3) Reset cycling manager and status flags. */
	cyclingManager->clearCyclingData( );

	infeasible = BT_FALSE;
	unbounded  = BT_FALSE;


	/* II) MAIN HOMOTOPY LOOP */
	for( iter=0; iter<nWSR; ++iter )
	{
		if ( isCPUtimeLimitExceeded( cputime,starttime,iter ) == BT_TRUE )
		{
			/* If CPU time limit is exceeded, stop homotopy loop immediately!
			 * Assign number of working set recalculations (runtime measurement is stopped later). */
			nWSR = iter;
			break;
		}

		status = QPS_PERFORMINGHOMOTOPY;

		#ifndef __XPCTARGET__
		snprintf( messageString,80,"%d ...",iter );
		getGlobalMessageHandler( )->throwInfo( RET_ITERATION_STARTED,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		#endif

		/* some more definitions */
		nFR  = getNFR( );
		nFX  = getNFX( );
		nAC  = getNAC( );
		nIAC = getNIAC( );

		/* 1) Determine index arrays. */
		if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
		{
			delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
			return THROWERROR( RET_HOTSTART_FAILED );
		}

		if ( bounds->getFixed( )->getNumberArray( FX_idx ) != SUCCESSFUL_RETURN )
		{
			delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
			return THROWERROR( RET_HOTSTART_FAILED );
		}

		if ( constraints->getActive( )->getNumberArray( AC_idx ) != SUCCESSFUL_RETURN )
		{
			delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
			return THROWERROR( RET_HOTSTART_FAILED );
		}

		if ( constraints->getInactive( )->getNumberArray( IAC_idx ) != SUCCESSFUL_RETURN )
		{
			delete[] IAC_idx; delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
			return THROWERROR( RET_HOTSTART_FAILED );
		}

		/* 2) Detemination of shift direction of the gradient and the (constraints') bounds. */
		returnvalue = determineDataShift(	FX_idx, AC_idx,
											g_new,lbA_new,ubA_new,lb_new,ub_new,
											delta_g,delta_lbA,delta_ubA,delta_lb,delta_ub,
											Delta_bC_isZero, Delta_bB_isZero
											);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_Ax_u; delete[] delta_Ax_l; delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] IAC_idx; delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_SHIFT_DETERMINATION_FAILED );
			return returnvalue;
		}

		/* 3) Determination of step direction of X and Y. */
		returnvalue = determineStepDirection(	FR_idx,FX_idx,AC_idx,
												delta_g,delta_lbA,delta_ubA,delta_lb,delta_ub,
												Delta_bC_isZero, Delta_bB_isZero,
												delta_xFX,delta_xFR,delta_yAC,delta_yFX
												);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_Ax_u; delete[] delta_Ax_l; delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] IAC_idx; delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_STEPDIRECTION_DETERMINATION_FAILED );
			return returnvalue;
		}

		/* 4) Determination of step length TAU. */
		returnvalue = determineStepLength(	FR_idx,FX_idx,AC_idx,IAC_idx,
											delta_lbA,delta_ubA,delta_lb,delta_ub,
											delta_xFX,delta_xFR,delta_yAC,delta_yFX,delta_Ax_l,delta_Ax_u,
											BC_idx,BC_status,BC_isBound
											);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_Ax_u; delete[] delta_Ax_l; delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] IAC_idx; delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_STEPLENGTH_DETERMINATION_FAILED );
			return returnvalue;
		}

		/* 5) Realisation of the homotopy step. */
		returnvalue = performStep(	FR_idx,FX_idx,AC_idx,IAC_idx,
									delta_g,delta_lbA,delta_ubA,delta_lb,delta_ub,
									delta_xFX,delta_xFR,delta_yAC,delta_yFX,delta_Ax_l,delta_Ax_u,
									BC_idx,BC_status,BC_isBound
									);

		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_Ax_u; delete[] delta_Ax_l; delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] IAC_idx; delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			/* Optimal solution found? */
			if ( returnvalue == RET_OPTIMAL_SOLUTION_FOUND )
			{
				status = QPS_SOLVED;

				THROWINFO( RET_OPTIMAL_SOLUTION_FOUND );

				if ( printIteration( nWSRperformed+iter,BC_idx,BC_status,BC_isBound ) != SUCCESSFUL_RETURN )
					THROWERROR( RET_PRINT_ITERATION_FAILED ); /* do not pass this as return value! */

				return SUCCESSFUL_RETURN;
			}
			else
			{
				/* Checks for infeasibility... */
				if ( isInfeasible( ) == BT_TRUE )
				{
					status = QPS_HOMOTOPYQPSOLVED;
					return THROWERROR( RET_HOTSTART_STOPPED_INFEASIBILITY );
				}

				/* ...unboundedness... */
				if ( unbounded == BT_TRUE ) /* not necessary since objective function convex! */
					return THROWERROR( RET_HOTSTART_STOPPED_UNBOUNDEDNESS );

				/* ... and throw unspecific error otherwise */
				THROWERROR( RET_HOMOTOPY_STEP_FAILED );
				return returnvalue;
			}
		}

		/* 6) Output information of successful QP iteration. */
		status = QPS_HOMOTOPYQPSOLVED;

		if ( printIteration( nWSRperformed+iter,BC_idx,BC_status,BC_isBound ) != SUCCESSFUL_RETURN )
			THROWERROR( RET_PRINT_ITERATION_FAILED ); /* do not pass this as return value! */
	}

	delete[] delta_Ax_u; delete[] delta_Ax_l; delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
	delete[] IAC_idx; delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
	delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

	/* stop runtime measurement */
	if ( cputime != 0 )
		*cputime = getCPUtime( ) - starttime;


	/* if programm gets to here, output information that QP could not be solved
	 * within the given maximum numbers of working set changes */
	if ( printLevel == PL_HIGH )
	{
		#ifndef __XPCTARGET__
		snprintf( messageString,80,"(nWSR = %d)",nWSR );
		return getGlobalMessageHandler( )->throwWarning( RET_MAX_NWSR_REACHED,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		#else
		return RET_MAX_NWSR_REACHED;
		#endif
	}
	else
	{
		return RET_MAX_NWSR_REACHED;
	}
}


/*
 *	s o l v e R e g u l a r i s e d Q P
 */
returnValue QProblem::solveRegularisedQP(	const double* const g_new,
											const double* const lb_new, const double* const ub_new,
											const double* const lbA_new, const double* const ubA_new,
											int& nWSR, double* const cputime
											)
{
	int i, step;
	int nV = getNV( );


	/* Stop here if QP has not been regularised (i.e. normal QP solution). */
	if ( usingRegularisation( ) == BT_FALSE )
		return solveQP( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime,0 );


	/* I) SOLVE USUAL REGULARISED QP */
	returnValue returnvalue;

	int totalNWSR = 0;
	int curNWSR   = 0;

	double totalCputime = 0.0;
	double curCputime   = 0.0;

	if ( cputime == 0 )
	{
		curNWSR = nWSR;
		returnvalue = solveQP( g_new,lb_new,ub_new,lbA_new,ubA_new, curNWSR,0,0 );
	}
	else
	{
		curNWSR = nWSR;
		curCputime = *cputime;
		returnvalue = solveQP( g_new,lb_new,ub_new,lbA_new,ubA_new, curNWSR,&curCputime,0 );
	}
	totalNWSR    += curNWSR;
	totalCputime += curCputime;

	/* Only continue if QP solution has been successful. */
	if ( returnvalue != SUCCESSFUL_RETURN )
	{
		nWSR = totalNWSR;
		if ( cputime != 0 )
			*cputime = totalCputime;

		if ( returnvalue == RET_MAX_NWSR_REACHED )
			THROWWARNING( RET_NO_REGSTEP_NWSR );

		return returnvalue;
	}


	/* II) PERFORM SUCCESSIVE REGULARISATION STEPS */
	double* gMod = new double[nV];

	for( step=0; step<nRegSteps; ++step )
	{
		/* 1) Modify gradient: gMod = g - eps*xOpt
		 *    (assuming regularisation matrix to be eps*Id). */
		for( i=0; i<nV; ++i )
			gMod[i] = g_new[i] - eps*x[i];

		/* 2) Solve regularised QP with modified gradient allowing
		 *    only as many working set recalculations and CPU time
		 *    as have been left from previous QP solutions. */
		if ( cputime == 0 )
		{
			curNWSR = nWSR - totalNWSR;
			returnvalue = solveQP( gMod,lb_new,ub_new,lbA_new,ubA_new, curNWSR,0,totalNWSR );
		}
		else
		{
			curNWSR = nWSR - totalNWSR;
			curCputime = *cputime - totalCputime;
			returnvalue = solveQP( gMod,lb_new,ub_new,lbA_new,ubA_new, curNWSR,&curCputime,totalNWSR );
		}

		totalNWSR    += curNWSR;
		totalCputime += curCputime;

		/* Only continue if QP solution has been successful. */
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] gMod;

			nWSR = totalNWSR;
			if ( cputime != 0 )
				*cputime = totalCputime;

			if ( returnvalue == RET_MAX_NWSR_REACHED )
				THROWWARNING( RET_FEWER_REGSTEPS_NWSR );

			return returnvalue;
		}
	}

	delete[] gMod;

	nWSR = totalNWSR;
	if ( cputime != 0 )
		*cputime = totalCputime;

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p S u b j e c t T o T y p e
 */
returnValue QProblem::setupSubjectToType( )
{
	return setupSubjectToType( lb,ub,lbA,ubA );
}


/*
 *	s e t u p S u b j e c t T o T y p e
 */
returnValue QProblem::setupSubjectToType(	const double* const lb_new, const double* const ub_new,
											const double* const lbA_new, const double* const ubA_new
											)
{
	int i;
	int nC = getNC( );


	/* I) SETUP SUBJECTTOTYPE FOR BOUNDS */
	if ( QProblemB::setupSubjectToType( lb_new,ub_new ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_SETUPSUBJECTTOTYPE_FAILED );


	/* II) SETUP SUBJECTTOTYPE FOR CONSTRAINTS */
	/* 1) Check if lower constraints' bounds are present. */
	constraints->setNoLower( BT_TRUE );
	if ( lbA_new != 0 )
	{
		for( i=0; i<nC; ++i )
		{
			if ( lbA_new[i] > -INFTY )
			{
				constraints->setNoLower( BT_FALSE );
				break;
			}
		}
	}

	/* 2) Check if upper constraints' bounds are present. */
	constraints->setNoUpper( BT_TRUE );
	if ( ubA_new != 0 )
	{
		for( i=0; i<nC; ++i )
		{
			if ( ubA_new[i] < INFTY )
			{
				constraints->setNoUpper( BT_FALSE );
				break;
			}
		}
	}

	/* 3) Determine implicit equality constraints and unbounded constraints. */
	if ( ( lbA_new != 0 ) && ( ubA_new != 0 ) )
	{
		for( i=0; i<nC; ++i )
		{
			if ( ( lbA_new[i] < -INFTY + BOUNDTOL ) && ( ubA_new[i] > INFTY - BOUNDTOL ) )
			{
				constraints->setType( i,ST_UNBOUNDED );
			}
			else
			{
				if ( lbA_new[i] > ubA_new[i] - BOUNDTOL )
					constraints->setType( i,ST_EQUALITY );
				else
					constraints->setType( i,ST_BOUNDED );
			}
		}
	}
	else
	{
		if ( ( lbA_new == 0 ) && ( ubA_new == 0 ) )
		{
			for( i=0; i<nC; ++i )
				constraints->setType( i,ST_UNBOUNDED );
		}
		else
		{
			for( i=0; i<nC; ++i )
				constraints->setType( i,ST_BOUNDED );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	c h o l e s k y D e c o m p o s i t i o n P r o j e c t e d
 */
returnValue QProblem::setupCholeskyDecompositionProjected( )
{
	int i, j, k, ii, kk;
	int nV  = getNV( );
	int nFR = getNFR( );
	int nAC = getNAC( );
	int nZ  = getNZ( );

	/* 1) Initialises R with all zeros. */
	for( i=0; i<nV*nV; ++i )
		R[i] = 0.0;

	/* 2) Calculate Cholesky decomposition of projected Hessian Z'*H*Z. */
	if ( ( hessianType == HST_ZERO ) || ( hessianType == HST_IDENTITY ) )
	{
		if ( hessianType == HST_ZERO )
		{
			/* if Hessian is zero matrix, it is assumed that it has been
			 * regularised and thus its Cholesky factor is the identity
			 * matrix scaled by sqrt(eps). */
			if ( usingRegularisation( ) == BT_TRUE )
			{
				for( i=0; i<nV; ++i )
					R[i*nV + i] = sqrt( eps );
			}
			else
				return THROWERROR( RET_CHOLESKY_OF_ZERO_HESSIAN );
		}
		else
		{
			/* if Hessian is identity, so is its Cholesky factor. */
			for( i=0; i<nV; ++i )
				R[i*nV + i] = 1.0;
		}
	}
	else
	{
		if ( nZ > 0 )
		{
			int* FR_idx = new int[nFR];
			if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
			{
				delete[] FR_idx;
				return THROWERROR( RET_INDEXLIST_CORRUPTED );
			}

			int* AC_idx = new int[nAC];
			if ( constraints->getActive( )->getNumberArray( AC_idx ) != SUCCESSFUL_RETURN )
			{
				delete[] AC_idx; delete[] FR_idx;
				return THROWERROR( RET_INDEXLIST_CORRUPTED );
			}


			double* HZ  = new double[nFR*nZ];
			double* ZHZ = new double[nZ*nZ];

			/* calculate H*Z */
			for ( i=0; i<nFR; ++i )
			{
				ii = FR_idx[i];

				for ( j=0; j<nZ; ++j )
				{
					HZ[i*nZ + j] = 0.0;
					for ( k=0; k<nFR; ++k )
					{
						kk = FR_idx[k];
						HZ[i*nZ + j] += H[ii*nV + kk] * Q[kk*nV + j];
					}
				}
			}

			/* calculate Z'*H*Z */
			for ( i=0; i<nZ; ++i )
				for ( j=0; j<nZ; ++j )
				{
					ZHZ[i*nZ + j] = 0.0;
					for ( k=0; k<nFR; ++k )
					{
						kk = FR_idx[k];
						ZHZ[i*nZ + j] += Q[kk*nV + i] * HZ[k*nZ + j];
					}
				}

			/* R'*R = Z'*H*Z */
			double sum;

			for( i=0; i<nZ; ++i )
			{
				/* j == i */
				sum = ZHZ[i*nZ + i];

				for( k=(i-1); k>=0; --k )
					sum -= R[k*nV + i] * R[k*nV + i];

				if ( sum > 0.0 )
					R[i*nV + i] = sqrt( sum );
				else
				{
					delete[] ZHZ; delete[] HZ; delete[] AC_idx; delete[] FR_idx;

					hessianType = HST_SEMIDEF;
					return RET_HESSIAN_NOT_SPD;
				}

				for( j=(i+1); j<nZ; ++j )
				{
					sum = ZHZ[j*nZ + i];

					for( k=(i-1); k>=0; --k )
						sum -= R[k*nV + i] * R[k*nV + j];

					R[i*nV + j] = sum / R[i*nV + i];
				}
			}

			delete[] ZHZ; delete[] HZ; delete[] AC_idx; delete[] FR_idx;
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p T Q f a c t o r i s a t i o n
 */
returnValue QProblem::setupTQfactorisation( )
{
	int i, ii;
	int nV  = getNV( );
	int nFR = getNFR( );

	int* FR_idx = new int[nFR];
	if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FR_idx;
		return THROWERROR( RET_INDEXLIST_CORRUPTED );
	}

	/* 1) Set Q to unity matrix. */
	for( i=0; i<nV*nV; ++i )
		Q[i] = 0.0;

	for( i=0; i<nFR; ++i )
	{
		ii = FR_idx[i];
		Q[ii*nV + i] = 1.0;
	}

 	/* 2) Set T to zero matrix. */
	for( i=0; i<sizeT*sizeT; ++i )
		T[i] = 0.0;

	delete[] FR_idx;

	return SUCCESSFUL_RETURN;
}


/*
 *	o b t a i n A u x i l i a r y W o r k i n g S e t
 */
returnValue QProblem::obtainAuxiliaryWorkingSet(	const double* const xOpt, const double* const yOpt,
													const Bounds* const guessedBounds, const Constraints* const guessedConstraints,
													Bounds* auxiliaryBounds, Constraints* auxiliaryConstraints
													) const
{
	int i = 0;
	int nV = getNV( );
	int nC = getNC( );


	/* 1) Ensure that desiredBounds is allocated (and different from guessedBounds). */
	if ( ( auxiliaryBounds == 0 ) || ( auxiliaryBounds == guessedBounds ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	if ( ( auxiliaryConstraints == 0 ) || ( auxiliaryConstraints == guessedConstraints ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	SubjectToStatus guessedStatus;

	/* 2) Setup working set of bounds for auxiliary initial QP. */
	if ( QProblemB::obtainAuxiliaryWorkingSet( xOpt,yOpt,guessedBounds, auxiliaryBounds ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );

	/* 3) Setup working set of constraints for auxiliary initial QP. */
	if ( guessedConstraints != 0 )
	{
		/* If an initial working set is specific, use it!
		 * Moreover, add all equality constraints if specified. */
		for( i=0; i<nC; ++i )
		{
			/* Add constraint only if it is not (goint to be) disabled! */
			guessedStatus = guessedConstraints->getStatus( i );

			if ( ( guessedStatus == ST_DISABLED ) || ( guessedStatus == ST_DISABLING ) )
			{
				if ( auxiliaryConstraints->setupConstraint( i,ST_DISABLED ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
			}
			else
			{
				#ifdef __ALWAYS_INITIALISE_WITH_ALL_EQUALITIES__
				if ( constraints->getType( i ) == ST_EQUALITY )
				{

					if ( auxiliaryConstraints->setupConstraint( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
				else
				#endif
				{
					if ( auxiliaryConstraints->setupConstraint( i,guessedStatus ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
			}
		}
	}
	else	/* No initial working set specified. */
	{
		/* Obtain initial working set by "clipping". */
		if ( ( xOpt != 0 ) && ( yOpt == 0 ) )
		{
			for( i=0; i<nC; ++i )
			{
				if ( Ax_l[i] - lbA[i] <= BOUNDTOL )
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
					continue;
				}

				if ( ubA[i] - Ax_u[i] <= BOUNDTOL )
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_UPPER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
					continue;
				}

				/* Moreover, add all equality constraints if specified. */
				#ifdef __ALWAYS_INITIALISE_WITH_ALL_EQUALITIES__
				if ( constraints->getType( i ) == ST_EQUALITY )
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
				else
				#endif
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_INACTIVE ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
			}
		}

		/* Obtain initial working set in accordance to sign of dual solution vector. */
		if ( ( xOpt == 0 ) && ( yOpt != 0 ) )
		{
			for( i=0; i<nC; ++i )
			{
				if ( yOpt[nV+i] > ZERO )
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
					continue;
				}

				if ( yOpt[nV+i] < -ZERO )
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_UPPER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
					continue;
				}

				/* Moreover, add all equality constraints if specified. */
				#ifdef __ALWAYS_INITIALISE_WITH_ALL_EQUALITIES__
				if ( constraints->getType( i ) == ST_EQUALITY )
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
				else
				#endif
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_INACTIVE ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
			}
		}

		/* If xOpt and yOpt are null pointer and no initial working is specified,
		 * start with empty working set (or implicitly fixed bounds and equality constraints only)
		 * for auxiliary QP. */
		if ( ( xOpt == 0 ) && ( yOpt == 0 ) )
		{
			for( i=0; i<nC; ++i )
			{
				/* Only add all equality constraints if specified. */
				#ifdef __ALWAYS_INITIALISE_WITH_ALL_EQUALITIES__
				if ( constraints->getType( i ) == ST_EQUALITY )
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
				else
				#endif
				{
					if ( auxiliaryConstraints->setupConstraint( i,ST_INACTIVE ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
			}
		}
	}

	return SUCCESSFUL_RETURN;
}



/*
 *	s e t u p A u x i l i a r y W o r k i n g S e t
 */
returnValue QProblem::setupAuxiliaryWorkingSet(	const Bounds* const auxiliaryBounds,
												const Constraints* const auxiliaryConstraints,
												BooleanType setupAfresh
												)
{
	int i;
	int nV = getNV( );
	int nC = getNC( );

	/* consistency checks */
	if ( auxiliaryBounds != 0 )
	{
		for( i=0; i<nV; ++i )
			if ( ( bounds->getStatus( i ) == ST_UNDEFINED ) || ( auxiliaryBounds->getStatus( i ) == ST_UNDEFINED ) )
				return THROWERROR( RET_UNKNOWN_BUG );
	}
	else
	{
		return THROWERROR( RET_INVALID_ARGUMENTS );
	}

	if ( auxiliaryConstraints != 0 )
	{
		for( i=0; i<nC; ++i )
			if ( ( constraints->getStatus( i ) == ST_UNDEFINED ) || ( auxiliaryConstraints->getStatus( i ) == ST_UNDEFINED ) )
				return THROWERROR( RET_UNKNOWN_BUG );
	}
	else
	{
		return THROWERROR( RET_INVALID_ARGUMENTS );
	}


	/* I) SETUP CHOLESKY FLAG:
	 *    Cholesky decomposition shall only be updated if working set
	 *    shall be updated (i.e. NOT setup afresh!) */
	BooleanType updateCholesky;
	if ( setupAfresh == BT_TRUE )
		updateCholesky = BT_FALSE;
	else
		updateCholesky = BT_TRUE;


	/* II) REMOVE FORMERLY ACTIVE (CONSTRAINTS') BOUNDS (IF NECESSARY): */
	if ( setupAfresh == BT_FALSE )
	{
		/* 1) Remove all active constraints that shall be inactive or disabled AND
		*    all active constraints that are active at the wrong bound. */
		for( i=0; i<nC; ++i )
		{
			if ( ( constraints->getStatus( i ) == ST_LOWER ) && ( auxiliaryConstraints->getStatus( i ) != ST_LOWER ) )
				if ( removeConstraint( i,updateCholesky ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_SETUP_WORKINGSET_FAILED );

			if ( ( constraints->getStatus( i ) == ST_UPPER ) && ( auxiliaryConstraints->getStatus( i ) != ST_UPPER ) )
				if ( removeConstraint( i,updateCholesky ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_SETUP_WORKINGSET_FAILED );

			if ( constraints->getStatus( i ) == ST_DISABLING )
			{
				/* (removed constraint will become DISABLED!) */
				if ( removeConstraint( i,updateCholesky ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_SETUP_WORKINGSET_FAILED );
			}
		}

		/* 2) Remove all active bounds that shall be inactive AND
		*    all active bounds that are active at the wrong bound. */
		for( i=0; i<nV; ++i )
		{
			if ( ( bounds->getStatus( i ) == ST_LOWER ) && ( auxiliaryBounds->getStatus( i ) != ST_LOWER ) )
				if ( removeBound( i,updateCholesky ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_SETUP_WORKINGSET_FAILED );

			if ( ( bounds->getStatus( i ) == ST_UPPER ) && ( auxiliaryBounds->getStatus( i ) != ST_UPPER ) )
				if ( removeBound( i,updateCholesky ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_SETUP_WORKINGSET_FAILED );
		}
	}


	/* III) ADD NEWLY ACTIVE (CONSTRAINTS') BOUNDS: */
	/* 1) Add all inactive bounds that shall be active AND
	 *    all formerly active bounds that have been active at the wrong bound. */
	for( i=0; i<nV; ++i )
	{
		if ( ( bounds->getStatus( i ) == ST_INACTIVE ) && ( auxiliaryBounds->getStatus( i ) != ST_INACTIVE ) )
		{
			/* Add bound only if it is linearly independent from the current working set. */
			if ( addBound_checkLI( i ) == RET_LINEARLY_INDEPENDENT )
			{
				if ( addBound( i,auxiliaryBounds->getStatus( i ),updateCholesky ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_SETUP_WORKINGSET_FAILED );
			}
		}
	}

	/* 2) Add all inactive or disabled constraints that shall be active AND
	 *    all formerly active constraints that have been active at the wrong bound. */
	for( i=0; i<nC; ++i )
	{
		if ( ( auxiliaryConstraints->getStatus( i ) == ST_LOWER ) || ( auxiliaryConstraints->getStatus( i ) == ST_UPPER ) )
		{
			/* formerly inactive */
			if ( constraints->getStatus( i ) == ST_INACTIVE )
			{
				/* Add constraint only if it is linearly independent from the current working set. */
				if ( addConstraint_checkLI( i ) == RET_LINEARLY_INDEPENDENT )
				{
					if ( addConstraint( i,auxiliaryConstraints->getStatus( i ),updateCholesky ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_SETUP_WORKINGSET_FAILED );
				}
			}

			/* formerly disabled (disabling implicitly included!) */
			if ( constraints->getStatus( i ) == ST_DISABLED )
			{
				/* Add constraint only if it is linearly independent from the current working set. */
				if ( addConstraint_checkLI( i ) == RET_LINEARLY_INDEPENDENT )
				{
					if ( addConstraint( i,auxiliaryConstraints->getStatus( i ),updateCholesky ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_SETUP_WORKINGSET_FAILED );
				}
				else
				{
					/* If a disabled constraint cannot become active,
					 * change its status to inactive. */
					if ( constraints->moveDisabledToInactive( i ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_REMOVECONSTRAINT_FAILED );
				}
			}
		}
	}

	/* 3) Make sure that constraints get same inactive/disabled structure
	 *    as desired by the auxiliary constraints. */
	for( i=0; i<nC; ++i )
	{
		if ( ( auxiliaryConstraints->getStatus( i ) == ST_DISABLING ) && ( constraints->getStatus( i ) == ST_INACTIVE ) )
		{
			if ( constraints->moveInactiveToDisabled( i ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_REMOVECONSTRAINT_FAILED );
		}

		if ( ( auxiliaryConstraints->getStatus( i ) == ST_DISABLED ) && ( constraints->getStatus( i ) == ST_INACTIVE ) )
		{
			if ( constraints->moveInactiveToDisabled( i ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_REMOVECONSTRAINT_FAILED );
		}

		if ( ( auxiliaryConstraints->getStatus( i ) == ST_INACTIVE ) && ( constraints->getStatus( i ) == ST_DISABLED ) )
		{
			if ( constraints->moveDisabledToInactive( i ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_REMOVECONSTRAINT_FAILED );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A u x i l i a r y Q P s o l u t i o n
 */
returnValue QProblem::setupAuxiliaryQPsolution(	const double* const xOpt, const double* const yOpt
												)
{
	int i, j;
	int nV = getNV( );
	int nC = getNC( );


	/* Setup primal/dual solution vector for auxiliary initial QP:
	 * if a null pointer is passed, a zero vector is assigned;
	 *  old solution vector is kept if pointer to internal solution vevtor is passed. */
	if ( xOpt != 0 )
	{
		if ( xOpt != x )
			for( i=0; i<nV; ++i )
				x[i] = xOpt[i];

		for ( j=0; j<nC; ++j )
		{
			Ax_l[j] = 0.0;

			for( i=0; i<nV; ++i )
				Ax_l[j] += A[j*nV + i] * x[i];

			Ax_u[j] = Ax_l[j];
		}
	}
	else
	{
		for( i=0; i<nV; ++i )
			x[i] = 0.0;

		for ( j=0; j<nC; ++j )
		{
			Ax_l[j] = 0.0;
			Ax_u[j] = 0.0;
		}
	}

	if ( yOpt != 0 )
	{
		if ( yOpt != y )
			for( i=0; i<nV+nC; ++i )
				y[i] = yOpt[i];
	}
	else
	{
		for( i=0; i<nV+nC; ++i )
			y[i] = 0.0;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A u x i l i a r y Q P g r a d i e n t
 */
returnValue QProblem::setupAuxiliaryQPgradient( )
{
	int i, j;
	int nV = getNV( );
	int nC = getNC( );


	/* Setup gradient vector: g = -H*x + [Id A]'*[yB yC]
	 *                          = yB - H*x + A'*yC. */
	switch ( hessianType )
	{
		case HST_ZERO:
			for ( i=0; i<nV; ++i )
				g[i] = y[i] - eps*x[i];
			break;

		case HST_IDENTITY:
			for ( i=0; i<nV; ++i )
				g[i] = y[i] - x[i];
			break;

		default:
			for ( i=0; i<nV; ++i )
			{
				/* y'*Id */
				g[i] = y[i];

				/* - H*x */
				for ( j=0; j<nV; ++j )
					g[i] -= H[i*nV + j] * x[j];
			}
			break;
	}

	for ( i=0; i<nV; ++i )
	{
		/* + A'*yC */
		for ( j=0; j<nC; ++j )
			g[i] += A[j*nV + i] * y[nV+j];
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A u x i l i a r y Q P b o u n d s
 */
returnValue QProblem::setupAuxiliaryQPbounds(	const Bounds* const auxiliaryBounds,
												const Constraints* const auxiliaryConstraints,
												BooleanType useRelaxation
												)
{
	int i;
	int nV = getNV( );
	int nC = getNC( );


	/* 1) Setup bound vectors. */
	for ( i=0; i<nV; ++i )
	{
		switch ( bounds->getStatus( i ) )
		{
			case ST_INACTIVE:
				if ( useRelaxation == BT_TRUE )
				{
					if ( bounds->getType( i ) == ST_EQUALITY )
					{
						lb[i] = x[i];
						ub[i] = x[i];
					}
					else
					{
						/* If a bound is inactive although it was supposed to be
						* active by the auxiliaryBounds, it could not be added
						* due to linear dependence. Thus set it "strongly inactive". */
						if ( auxiliaryBounds->getStatus( i ) == ST_LOWER )
							lb[i] = x[i];
						else
							lb[i] = x[i] - BOUNDRELAXATION;

						if ( auxiliaryBounds->getStatus( i ) == ST_UPPER )
							ub[i] = x[i];
						else
							ub[i] = x[i] + BOUNDRELAXATION;
					}
				}
				break;

			case ST_LOWER:
				lb[i] = x[i];
				if ( bounds->getType( i ) == ST_EQUALITY )
				{
					ub[i] = x[i];
				}
				else
				{
					if ( useRelaxation == BT_TRUE )
						ub[i] = x[i] + BOUNDRELAXATION;
				}
				break;

			case ST_UPPER:
				ub[i] = x[i];
				if ( bounds->getType( i ) == ST_EQUALITY )
				{
					lb[i] = x[i];
				}
				else
				{
					if ( useRelaxation == BT_TRUE )
						lb[i] = x[i] - BOUNDRELAXATION;
				}
				break;

			default:
				return THROWERROR( RET_UNKNOWN_BUG );
		}
	}

	/* 2) Setup constraints vectors. */
	for ( i=0; i<nC; ++i )
	{
		switch ( constraints->getStatus( i ) )
		{
			case ST_INACTIVE:
				if ( useRelaxation == BT_TRUE )
				{
					if ( constraints->getType( i ) == ST_EQUALITY )
					{
						lbA[i] = Ax_l[i];
						ubA[i] = Ax_u[i];
					}
					else
					{
						/* If a constraint is inactive although it was supposed to be
						* active by the auxiliaryConstraints, it could not be added
						* due to linear dependence. Thus set it "strongly inactive". */
						if ( auxiliaryConstraints->getStatus( i ) == ST_LOWER )
							lbA[i] = Ax_l[i];
						else
							lbA[i] = Ax_l[i] - BOUNDRELAXATION;

						if ( auxiliaryConstraints->getStatus( i ) == ST_UPPER )
							ubA[i] = Ax_u[i];
						else
							ubA[i] = Ax_u[i] + BOUNDRELAXATION;
					}
				}
				break;

			case ST_LOWER:
				lbA[i] = Ax_l[i];
				if ( constraints->getType( i ) == ST_EQUALITY )
				{
					ubA[i] = Ax_l[i];
				}
				else
				{
					if ( useRelaxation == BT_TRUE )
						ubA[i] = Ax_l[i] + BOUNDRELAXATION;
				}
				break;

			case ST_UPPER:
				ubA[i] = Ax_u[i];
				if ( constraints->getType( i ) == ST_EQUALITY )
				{
					lbA[i] = Ax_u[i];
				}
				else
				{
					if ( useRelaxation == BT_TRUE )
						lbA[i] = Ax_u[i] - BOUNDRELAXATION;
				}
				break;

			default:
				return THROWERROR( RET_UNKNOWN_BUG );
		}

		Ax_l[i] = Ax_l[i] - lbA[i];
		Ax_u[i] = ubA[i] - Ax_u[i];
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	a d d C o n s t r a i n t
 */
returnValue QProblem::addConstraint(	int number, SubjectToStatus C_status,
										BooleanType updateCholesky
										)
{
	int i, j, ii;

	/* consistency checks */
	if ( constraints->getStatus( number ) != ST_INACTIVE )
		return THROWERROR( RET_CONSTRAINT_ALREADY_ACTIVE );

	if ( ( constraints->getNC( ) - getNAC( ) ) == constraints->getNUC( ) )
		return THROWERROR( RET_ALL_CONSTRAINTS_ACTIVE );

	if ( ( getStatus( ) == QPS_NOTINITIALISED )    ||
		 ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
		 ( getStatus( ) == QPS_SOLVED )            )
	{
		return THROWERROR( RET_UNKNOWN_BUG );
	}


	/* I) ENSURE LINEAR INDEPENDENCE OF THE WORKING SET,
	 *    i.e. remove a constraint or bound if linear dependence occurs. */
	/* check for LI only if Cholesky decomposition shall be updated! */
	if ( updateCholesky == BT_TRUE )
	{
		returnValue ensureLIreturnvalue = addConstraint_ensureLI( number,C_status );

		switch ( ensureLIreturnvalue )
		{
			case SUCCESSFUL_RETURN:
				break;

			case RET_LI_RESOLVED:
				break;

			case RET_ENSURELI_FAILED_NOINDEX:
				return THROWERROR( RET_ADDCONSTRAINT_FAILED_INFEASIBILITY );

			case RET_ENSURELI_FAILED_CYCLING:
				return THROWERROR( RET_ADDCONSTRAINT_FAILED_INFEASIBILITY );

			default:
				return THROWERROR( RET_ENSURELI_FAILED );
		}
	}

	/* some definitions */
	int nV  = getNV( );
	int nFR = getNFR( );
	int nAC = getNAC( );
	int nZ  = getNZ( );

	int tcol = sizeT - nAC;


	int* FR_idx = new int[nFR];
	if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FR_idx;
		return THROWERROR( RET_ADDCONSTRAINT_FAILED );
	}

	double* aFR = new double[nFR];
	double* wZ = new double[nZ];
	for( i=0; i<nZ; ++i )
		wZ[i] = 0.0;


	/* II) ADD NEW ACTIVE CONSTRAINT TO MATRIX T: */
	/* 1) Add row [wZ wY] = aFR'*[Z Y] to the end of T: assign aFR. */
	for( i=0; i<nFR; ++i )
	{
		ii = FR_idx[i];
		aFR[i] = A[number*nV + ii];
	}

	/* calculate wZ */
	for( i=0; i<nFR; ++i )
	{
		ii = FR_idx[i];
		for( j=0; j<nZ; ++j )
			wZ[j] += aFR[i] * Q[ii*nV + j];
	}

	/* 2) Calculate wY and store it directly into T. */
	if ( nAC > 0 )
	{
		for( j=0; j<nAC; ++j )
			T[nAC*sizeT + tcol+j] = 0.0;
		for( i=0; i<nFR; ++i )
		{
			ii = FR_idx[i];
			for( j=0; j<nAC; ++j )
				T[nAC*sizeT + tcol+j] += aFR[i] * Q[ii*nV + nZ+j];
		}
	}

	delete[] aFR;


	double c, s, nu;

	if ( nZ > 0 )
	{
		/* II) RESTORE TRIANGULAR FORM OF T: */
		/*     Use column-wise Givens rotations to restore reverse triangular form
		*      of T, simultanenous change of Q (i.e. Z) and R. */
		for( j=0; j<nZ-1; ++j )
		{
			computeGivens( wZ[j+1],wZ[j], wZ[j+1],wZ[j],c,s );
			nu = s/(1.0+c);

			for( i=0; i<nFR; ++i )
			{
				ii = FR_idx[i];
				applyGivens( c,s,nu,Q[ii*nV + 1+j],Q[ii*nV + j], Q[ii*nV + 1+j],Q[ii*nV + j] );
			}

			if ( ( updateCholesky == BT_TRUE ) &&
				 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
			{
				for( i=0; i<=j+1; ++i )
					applyGivens( c,s,nu,R[i*nV + 1+j],R[i*nV + j], R[i*nV + 1+j],R[i*nV + j] );
			}
		}

		T[nAC*sizeT + tcol-1] = wZ[nZ-1];


		if ( ( updateCholesky == BT_TRUE ) &&
			 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
		{
			/* III) RESTORE TRIANGULAR FORM OF R:
			 *      Use row-wise Givens rotations to restore upper triangular form of R. */
			for( i=0; i<nZ-1; ++i )
			{
				computeGivens( R[i*nV + i],R[(1+i)*nV + i], R[i*nV + i],R[(1+i)*nV + i],c,s );
				nu = s/(1.0+c);

				for( j=(1+i); j<(nZ-1); ++j ) /* last column of R is thrown away */
					applyGivens( c,s,nu,R[i*nV + j],R[(1+i)*nV + j], R[i*nV + j],R[(1+i)*nV + j] );
			}
			/* last column of R is thrown away */
			for( i=0; i<nZ; ++i )
				R[i*nV + nZ-1] = 0.0;
		}
	}

	delete[] wZ;
	delete[] FR_idx;


	/* IV) UPDATE INDICES */
	if ( constraints->moveInactiveToActive( number,C_status ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_ADDCONSTRAINT_FAILED );


	return SUCCESSFUL_RETURN;
}



/*
 *	a d d C o n s t r a i n t _ c h e c k L I
 */
returnValue QProblem::addConstraint_checkLI( int number ) const
{
	int i, j, jj;
	int nV  = getNV( );
	int nFR = getNFR( );
	int nZ  = getNZ( );

	int* FR_idx = new int[nFR];
	if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FR_idx;
		return THROWERROR( RET_INDEXLIST_CORRUPTED );
	}


	/* Check if constraint <number> is linearly independent from the
	   the active ones (<=> is element of null space of Afr). */
	double sum;

	for( i=0; i<nZ; ++i )
	{
		sum = 0.0;
		for( j=0; j<nFR; ++j )
		{
			jj = FR_idx[j];
			sum += Q[jj*nV + i] * A[number*nV + jj];
		}

		if ( fabs( sum ) > 10.0*EPS )
		{
			delete[] FR_idx;
			return THROWINFO( RET_LINEARLY_INDEPENDENT );
		}
	}

	delete[] FR_idx;

	return THROWINFO( RET_LINEARLY_DEPENDENT );
}


/*
 *	a d d C o n s t r a i n t _ e n s u r e L I
 */
returnValue QProblem::addConstraint_ensureLI( int number, SubjectToStatus C_status )
{
	int i, j, ii, jj;
	int nV  = getNV( );
	int nFR = getNFR( );
	int nFX = getNFX( );
	int nAC = getNAC( );
	int nZ  = getNZ( );


	/* I) Check if new constraint is linearly independent from the active ones. */
	returnValue returnvalueCheckLI = addConstraint_checkLI( number );

	if ( returnvalueCheckLI == RET_INDEXLIST_CORRUPTED )
		return THROWERROR( RET_ENSURELI_FAILED );

	if ( returnvalueCheckLI == RET_LINEARLY_INDEPENDENT )
		return SUCCESSFUL_RETURN;


 	/* II) NEW CONSTRAINT IS LINEARLY DEPENDENT: */
	/* 1) Determine coefficients of linear combination,
	 *    cf. M.J. Best. Applied Mathematics and Parallel Computing, chapter:
	 *    An Algorithm for the Solution of the Parametric Quadratic Programming
	 *    Problem, pages 57-76. Physica-Verlag, Heidelberg, 1996. */
	int* FR_idx = new int[nFR];
	if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FR_idx;
		return THROWERROR( RET_ENSURELI_FAILED );
	}

	int* FX_idx = new int[nFX];
	if ( bounds->getFixed( )->getNumberArray( FX_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FR_idx;
		return THROWERROR( RET_ENSURELI_FAILED );
	}

	double* xiC = new double[nAC];
	double* xiC_TMP = new double[nAC];
	double* xiB = new double[nFX];

	/* 2) Calculate xiC */
	if ( nAC > 0 )
	{
		if ( C_status == ST_LOWER )
		{
			for( i=0; i<nAC; ++i )
			{
				xiC_TMP[i] = 0.0;
				for( j=0; j<nFR; ++j )
				{
					jj = FR_idx[j];
					xiC_TMP[i] += Q[jj*nV + nZ+i] * A[number*nV + jj];
				}
			}
		}
		else
		{
			for( i=0; i<nAC; ++i )
			{
				xiC_TMP[i] = 0.0;
				for( j=0; j<nFR; ++j )
				{
					jj = FR_idx[j];
					xiC_TMP[i] -= Q[jj*nV + nZ+i] * A[number*nV + jj];
				}
			}
		}

		if ( backsolveT( xiC_TMP, BT_TRUE, xiC ) != SUCCESSFUL_RETURN )
		{
			delete[] FX_idx; delete[] FR_idx;
			delete[] xiB; delete[] xiC_TMP; delete[] xiC;
			return THROWERROR( RET_ENSURELI_FAILED_TQ );
		}
	}

	/* 3) Calculate xiB. */
	int* AC_idx = new int[nAC];
	if ( constraints->getActive( )->getNumberArray( AC_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
		delete[] xiB; delete[] xiC_TMP; delete[] xiC;
		return THROWERROR( RET_ENSURELI_FAILED );
	}

	if ( C_status == ST_LOWER )
	{
		for( i=0; i<nFX; ++i )
		{
			ii = FX_idx[i];
			xiB[i] = A[number*nV + ii];

			for( j=0; j<nAC; ++j )
			{
				jj = AC_idx[j];
				xiB[i] -= A[jj*nV + ii] * xiC[j];
			}
		}
	}
	else
	{
		for( i=0; i<nFX; ++i )
		{
			ii = FX_idx[i];
			xiB[i] = -A[number*nV + ii];

			for( j=0; j<nAC; ++j )
			{
				jj = AC_idx[j];
				xiB[i] -= A[jj*nV + ii] * xiC[j];
			}
		}
	}


	/* III) DETERMINE CONSTRAINT/BOUND TO BE REMOVED. */
	double y_min = INFTY * INFTY;
	int y_min_number = -1;
	BooleanType y_min_isBound = BT_FALSE;

	/* 1) Constraints. */
	for( i=0; i<nAC; ++i )
	{
		ii = AC_idx[i];

		if ( constraints->getStatus( ii ) == ST_LOWER )
		{
			if ( ( xiC[i] > ZERO ) && ( y[nV+ii] >= 0.0 ) )
			{
				if ( y[nV+ii]/xiC[i] < y_min )
				{
					y_min = y[nV+ii]/xiC[i];
					y_min_number = ii;
				}
			}
		}
		else
		{
			if ( ( xiC[i] < -ZERO ) && ( y[nV+ii] <= 0.0 ) )
			{
				if ( y[nV+ii]/xiC[i] < y_min )
				{
					y_min = y[nV+ii]/xiC[i];
					y_min_number = ii;
				}
			}
		}
	}

	/* 2) Bounds. */
	for( i=0; i<nFX; ++i )
	{
		ii = FX_idx[i];

		if ( bounds->getStatus( ii ) == ST_LOWER )
		{
			if ( ( xiB[i] > ZERO ) && ( y[ii] >= 0.0 ) )
			{
				if ( y[ii]/xiB[i] < y_min )
				{
					y_min = y[ii]/xiB[i];
					y_min_number = ii;
					y_min_isBound = BT_TRUE;
				}
			}
		}
		else
		{
			if ( ( xiB[i] < -ZERO ) && ( y[ii] <= 0.0 ) )
			{
				if ( y[ii]/xiB[i] < y_min )
				{
					y_min = y[ii]/xiB[i];
					y_min_number = ii;
					y_min_isBound = BT_TRUE;
				}
			}
		}
	}

	/* setup output preferences */
	char messageString[80];

	/* IV) REMOVE CONSTRAINT/BOUND FOR RESOLVING LINEAR DEPENDENCE: */
	if ( y_min_number >= 0 )
	{
		/* 1) Check for cycling due to infeasibility. */
		if ( ( cyclingManager->getCyclingStatus( number,BT_FALSE ) == CYC_PREV_REMOVED ) &&
			 ( cyclingManager->getCyclingStatus( y_min_number,y_min_isBound ) == CYC_PREV_ADDED ) )
		{
			infeasible = BT_TRUE;

			delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] xiB; delete[] xiC_TMP; delete[] xiC;

			return THROWERROR( RET_ENSURELI_FAILED_CYCLING );
		}
		else
		{
			/* set cycling data */
			cyclingManager->clearCyclingData( );
			cyclingManager->setCyclingStatus( number,BT_FALSE, CYC_PREV_ADDED );
			cyclingManager->setCyclingStatus( y_min_number,y_min_isBound, CYC_PREV_REMOVED );
		}

		/* 2) Update Lagrange multiplier... */
		for( i=0; i<nAC; ++i )
		{
			ii = AC_idx[i];
			y[nV+ii] -= y_min * xiC[i];
		}
		for( i=0; i<nFX; ++i )
		{
			ii = FX_idx[i];
			y[ii] -= y_min * xiB[i];
		}

		/* ... also for newly active constraint... */
		if ( C_status == ST_LOWER )
			y[nV+number] = y_min;
		else
			y[nV+number] = -y_min;

		/* ... and for constraint to be removed. */
		if ( y_min_isBound == BT_TRUE )
		{
			#ifndef __XPCTARGET__
			snprintf( messageString,80,"bound no. %d.",y_min_number );
			getGlobalMessageHandler( )->throwInfo( RET_REMOVE_FROM_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
			#endif

			if ( removeBound( y_min_number,BT_TRUE ) != SUCCESSFUL_RETURN )
			{
				delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
				delete[] xiB; delete[] xiC_TMP; delete[] xiC;

				return THROWERROR( RET_REMOVE_FROM_ACTIVESET_FAILED );
			}
			y[y_min_number] = 0.0;
		}
		else
		{
			#ifndef __XPCTARGET__
			snprintf( messageString,80,"constraint no. %d.",y_min_number );
			getGlobalMessageHandler( )->throwInfo( RET_REMOVE_FROM_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
			#endif

			if ( removeConstraint( y_min_number,BT_TRUE ) != SUCCESSFUL_RETURN )
			{
				delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
				delete[] xiB; delete[] xiC_TMP; delete[] xiC;

				return THROWERROR( RET_REMOVE_FROM_ACTIVESET_FAILED );
			}

			y[nV+y_min_number] = 0.0;
		}
	}
	else
	{
		/* no constraint/bound can be removed => QP is infeasible! */
		infeasible = BT_TRUE;

		delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
		delete[] xiB; delete[] xiC_TMP; delete[] xiC;

		return THROWERROR( RET_ENSURELI_FAILED_NOINDEX );
	}

	delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
	delete[] xiB; delete[] xiC_TMP; delete[] xiC;

	return getGlobalMessageHandler( )->throwInfo( RET_LI_RESOLVED,0,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
}



/*
 *	a d d B o u n d
 */
returnValue QProblem::addBound(	int number, SubjectToStatus B_status,
								BooleanType updateCholesky
								)
{
	int i, j, ii;

	/* consistency checks */
	if ( bounds->getStatus( number ) != ST_INACTIVE )
		return THROWERROR( RET_BOUND_ALREADY_ACTIVE );

	if ( getNFR( ) == bounds->getNUV( ) )
		return THROWERROR( RET_ALL_BOUNDS_ACTIVE );

	if ( ( getStatus( ) == QPS_NOTINITIALISED )    ||
		 ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
 		 ( getStatus( ) == QPS_SOLVED )            )
	{
		return THROWERROR( RET_UNKNOWN_BUG );
	}


	/* I) ENSURE LINEAR INDEPENDENCE OF THE WORKING SET,
	 *    i.e. remove a constraint or bound if linear dependence occurs. */
	/* check for LI only if Cholesky decomposition shall be updated! */
	if ( updateCholesky == BT_TRUE )
	{
		returnValue ensureLIreturnvalue = addBound_ensureLI( number,B_status );

		switch ( ensureLIreturnvalue )
		{
			case SUCCESSFUL_RETURN:
				break;

			case RET_LI_RESOLVED:
				break;

			case RET_ENSURELI_FAILED_NOINDEX:
				return THROWERROR( RET_ADDBOUND_FAILED_INFEASIBILITY );

			case RET_ENSURELI_FAILED_CYCLING:
				return THROWERROR( RET_ADDBOUND_FAILED_INFEASIBILITY );

			default:
				return THROWERROR( RET_ENSURELI_FAILED );
		}
	}


	/* some definitions */
	int nV  = getNV( );
	int nFR = getNFR( );
	int nAC = getNAC( );
	int nZ  = getNZ( );

	int tcol = sizeT - nAC;


	/* II) SWAP INDEXLIST OF FREE VARIABLES:
	 *     move the variable to be fixed to the end of the list of free variables. */
	int lastfreenumber = bounds->getFree( )->getLastNumber( );
	if ( lastfreenumber != number )
		if ( bounds->swapFree( number,lastfreenumber ) != SUCCESSFUL_RETURN )
			THROWERROR( RET_ADDBOUND_FAILED );


	int* FR_idx = new int[nFR];
	if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_ADDBOUND_FAILED );

	double* w = new double[nFR];


	/* III) ADD NEW ACTIVE BOUND TO TOP OF MATRIX T: */
	/* 1) add row [wZ wY] = [Z Y](number) at the top of T: assign w */
	for( i=0; i<nFR; ++i )
		w[i] = Q[FR_idx[nFR-1]*nV + i];


	/* 2) Use column-wise Givens rotations to restore reverse triangular form
	 *    of the first row of T, simultanenous change of Q (i.e. Z) and R. */
	double c, s, nu;

	for( j=0; j<nZ-1; ++j )
	{
		computeGivens( w[j+1],w[j], w[j+1],w[j],c,s );
		nu = s/(1.0+c);

		for( i=0; i<nFR; ++i )
		{
			ii = FR_idx[i];
			applyGivens( c,s,nu,Q[ii*nV + 1+j],Q[ii*nV + j], Q[ii*nV + 1+j],Q[ii*nV + j] );
		}

		if ( ( updateCholesky == BT_TRUE ) &&
			 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
		{
			for( i=0; i<=j+1; ++i )
				applyGivens( c,s,nu,R[i*nV + 1+j],R[i*nV + j], R[i*nV + 1+j],R[i*nV + j] );
		}
	}


	if ( nAC > 0 )	  /* ( nAC == 0 ) <=> ( nZ == nFR ) <=> Y and T are empty => nothing to do */
	{
		/* store new column a in a temporary vector instead of shifting T one column to the left */
		double* tmp = new double[nAC];
		for( i=0; i<nAC; ++i )
			tmp[i] = 0.0;

		{
			j = nZ-1;

			computeGivens( w[j+1],w[j], w[j+1],w[j],c,s );
			nu = s/(1.0+c);

			for( i=0; i<nFR; ++i )
			{
				ii = FR_idx[i];
				applyGivens( c,s,nu,Q[ii*nV + 1+j],Q[ii*nV + j], Q[ii*nV + 1+j],Q[ii*nV + j] );
			}

			applyGivens( c,s,nu,T[(nAC-1)*sizeT + tcol],tmp[nAC-1], tmp[nAC-1],T[(nAC-1)*sizeT + tcol] );
		}

		for( j=nZ; j<nFR-1; ++j )
		{
			computeGivens( w[j+1],w[j], w[j+1],w[j],c,s );
			nu = s/(1.0+c);

			for( i=0; i<nFR; ++i )
			{
				ii = FR_idx[i];
				applyGivens( c,s,nu,Q[ii*nV + 1+j],Q[ii*nV + j], Q[ii*nV + 1+j],Q[ii*nV + j] );
			}

			for( i=(nFR-2-j); i<nAC; ++i )
				applyGivens( c,s,nu,T[i*sizeT + 1+tcol-nZ+j],tmp[i], tmp[i],T[i*sizeT + 1+tcol-nZ+j] );
		}

		delete[] tmp;
	}

	delete[] w;	delete[] FR_idx;


	if ( ( updateCholesky == BT_TRUE ) &&
		 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
	{
		/* IV) RESTORE TRIANGULAR FORM OF R:
		 *     use row-wise Givens rotations to restore upper triangular form of R */
		for( i=0; i<nZ-1; ++i )
		{
			computeGivens( R[i*nV + i],R[(1+i)*nV + i], R[i*nV + i],R[(1+i)*nV + i],c,s );
			nu = s/(1.0+c);

			for( j=(1+i); j<nZ-1; ++j ) /* last column of R is thrown away */
				applyGivens( c,s,nu,R[i*nV + j],R[(1+i)*nV + j], R[i*nV + j],R[(1+i)*nV + j] );
		}
		/* last column of R is thrown away */
		for( i=0; i<nZ; ++i )
			R[i*nV + nZ-1] = 0.0;
	}


	/* V) UPDATE INDICES */
	if ( bounds->moveFreeToFixed( number,B_status ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_ADDBOUND_FAILED );

	return SUCCESSFUL_RETURN;
}


/*
 *	a d d B o u n d _ c h e c k L I
 */
returnValue QProblem::addBound_checkLI( int number ) const
{
	int i;

	/* some definitions */
	int nV  = getNV( );
	int nZ  = getNZ( );

	/* Check if constraint <number> is linearly independent from the
	   the active ones (<=> is element of null space of Afr). */
	for( i=0; i<nZ; ++i )
	{
		if ( fabs( Q[number*nV + i] ) > EPS )
			return THROWINFO( RET_LINEARLY_INDEPENDENT );
	}

	return THROWINFO( RET_LINEARLY_DEPENDENT );
}


/*
 *	a d d B o u n d _ e n s u r e L I
 */
returnValue QProblem::addBound_ensureLI( int number, SubjectToStatus B_status )
{
	int i, j, ii, jj;
	int nV  = getNV( );
	int nFR = getNFR( );
	int nFX = getNFX( );
	int nAC = getNAC( );
	int nZ  = getNZ( );


	/* I) Check if new constraint is linearly independent from the active ones. */
	returnValue returnvalueCheckLI = addBound_checkLI( number );

	if ( returnvalueCheckLI == RET_INDEXLIST_CORRUPTED )
		return THROWERROR( RET_ENSURELI_FAILED );

	if ( returnvalueCheckLI == RET_LINEARLY_INDEPENDENT )
		return SUCCESSFUL_RETURN;


 	/* II) NEW BOUND IS LINEARLY DEPENDENT: */
	/* 1) Determine coefficients of linear combination,
	 *    cf. M.J. Best. Applied Mathematics and Parallel Computing, chapter:
	 *    An Algorithm for the Solution of the Parametric Quadratic Programming
	 *    Problem, pages 57-76. Physica-Verlag, Heidelberg, 1996. */
	int* FR_idx = new int[nFR];
	if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FR_idx;
		return THROWERROR( RET_ENSURELI_FAILED );
	}

	int* FX_idx = new int[nFX];
	if ( bounds->getFixed( )->getNumberArray( FX_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FX_idx; delete[] FR_idx;
		return THROWERROR( RET_ENSURELI_FAILED );
	}

	int* AC_idx = new int[nAC];
	if ( constraints->getActive( )->getNumberArray( AC_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
		return THROWERROR( RET_ENSURELI_FAILED );
	}

	double* xiC = new double[nAC];
	double* xiC_TMP = new double[nAC];
	double* xiB = new double[nFX];

	/* 2) Calculate xiC. */
	if ( nAC > 0 )
	{
		if ( B_status == ST_LOWER )
		{
			for( i=0; i<nAC; ++i )
				xiC_TMP[i] = Q[number*nV + nZ+i];
		}
		else
		{
			for( i=0; i<nAC; ++i )
				xiC_TMP[i] = -Q[number*nV + nZ+i];
		}

		if ( backsolveT( xiC_TMP, BT_TRUE, xiC ) != SUCCESSFUL_RETURN )
		{
			delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] xiB; delete[] xiC_TMP; delete[] xiC;
			return THROWERROR( RET_ENSURELI_FAILED_TQ );
		}
	}

	/* 3) Calculate xiB. */
	for( i=0; i<nFX; ++i )
	{
		ii = FX_idx[i];

		xiB[i] = 0.0;
		for( j=0; j<nAC; ++j )
		{
			jj = AC_idx[j];
			xiB[i] -= A[jj*nV + ii] * xiC[j];
		}
	}


	/* III) DETERMINE CONSTRAINT/BOUND TO BE REMOVED. */
	double y_min = INFTY * INFTY;
	int y_min_number = -1;
	BooleanType y_min_isBound = BT_FALSE;

	/* 1) Constraints. */
	for( i=0; i<nAC; ++i )
	{
		ii = AC_idx[i];

		if ( constraints->getStatus( ii ) == ST_LOWER )
		{
			if ( ( xiC[i] > ZERO ) && ( y[nV+ii] >= 0.0 ) )
			{
				if ( y[nV+ii]/xiC[i] < y_min )
				{
					y_min = y[nV+ii]/xiC[i];
					y_min_number = ii;
				}
			}
		}
		else
		{
			if ( ( xiC[i] < -ZERO ) && ( y[nV+ii] <= 0.0 ) )
			{
				if ( y[nV+ii]/xiC[i] < y_min )
				{
					y_min = y[nV+ii]/xiC[i];
					y_min_number = ii;
				}
			}
		}
	}

	/* 2) Bounds. */
	for( i=0; i<nFX; ++i )
	{
		ii = FX_idx[i];

		if ( bounds->getStatus( ii ) == ST_LOWER )
		{
			if ( ( xiB[i] > ZERO ) && ( y[ii] >= 0.0 ) )
			{
				if ( y[ii]/xiB[i] < y_min )
				{
					y_min = y[ii]/xiB[i];
					y_min_number = ii;
					y_min_isBound = BT_TRUE;
				}
			}
		}
		else
		{
			if ( ( xiB[i] < -ZERO ) && ( y[ii] <= 0.0 ) )
			{
				if ( y[ii]/xiB[i] < y_min )
				{
					y_min = y[ii]/xiB[i];
					y_min_number = ii;
					y_min_isBound = BT_TRUE;
				}
			}
		}
	}


	/* IV) REMOVE CONSTRAINT/BOUND FOR RESOLVING LINEAR DEPENDENCE: */
	char messageString[80];

	if ( y_min_number >= 0 )
	{
		/* 1) Check for cycling due to infeasibility. */
		if ( ( cyclingManager->getCyclingStatus( number,BT_TRUE ) == CYC_PREV_REMOVED ) &&
			 ( cyclingManager->getCyclingStatus( y_min_number,y_min_isBound ) == CYC_PREV_ADDED ) )
		{
			infeasible = BT_TRUE;

			delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
			delete[] xiB; delete[] xiC_TMP; delete[] xiC;

			return THROWERROR( RET_ENSURELI_FAILED_CYCLING );
		}
		else
		{
			/* set cycling data */
			cyclingManager->clearCyclingData( );
			cyclingManager->setCyclingStatus( number,BT_TRUE, CYC_PREV_ADDED );
			cyclingManager->setCyclingStatus( y_min_number,y_min_isBound, CYC_PREV_REMOVED );
		}


		/* 2) Update Lagrange multiplier... */
		for( i=0; i<nAC; ++i )
		{
			ii = AC_idx[i];
			y[nV+ii] -= y_min * xiC[i];
		}
		for( i=0; i<nFX; ++i )
		{
			ii = FX_idx[i];
			y[ii] -= y_min * xiB[i];
		}

		/* ... also for newly active bound ... */
		if ( B_status == ST_LOWER )
			y[number] = y_min;
		else
			y[number] = -y_min;

		/* ... and for bound to be removed. */
		if ( y_min_isBound == BT_TRUE )
		{
			#ifndef __XPCTARGET__
			snprintf( messageString,80,"bound no. %d.",y_min_number );
			getGlobalMessageHandler( )->throwInfo( RET_REMOVE_FROM_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
			#endif

			if ( removeBound( y_min_number,BT_TRUE ) != SUCCESSFUL_RETURN )
			{
				delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
				delete[] xiB; delete[] xiC_TMP; delete[] xiC;

				return THROWERROR( RET_REMOVE_FROM_ACTIVESET_FAILED );
			}
			y[y_min_number] = 0.0;
		}
		else
		{
			#ifndef __XPCTARGET__
			snprintf( messageString,80,"constraint no. %d.",y_min_number );
			getGlobalMessageHandler( )->throwInfo( RET_REMOVE_FROM_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
			#endif

			if ( removeConstraint( y_min_number,BT_TRUE ) != SUCCESSFUL_RETURN )
			{
				delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
				delete[] xiB; delete[] xiC_TMP; delete[] xiC;

				return THROWERROR( RET_REMOVE_FROM_ACTIVESET_FAILED );
			}

			y[nV+y_min_number] = 0.0;
		}
	}
	else
	{
		/* no constraint/bound can be removed => QP is infeasible! */
		infeasible = BT_TRUE;
		//myPrintf( "nono!!\n" );

		delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
		delete[] xiB; delete[] xiC_TMP; delete[] xiC;

		return THROWERROR( RET_ENSURELI_FAILED_NOINDEX );
	}

	delete[] AC_idx; delete[] FX_idx; delete[] FR_idx;
	delete[] xiB; delete[] xiC_TMP; delete[] xiC;

	return getGlobalMessageHandler( )->throwInfo( RET_LI_RESOLVED,0,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
}



/*
 *	r e m o v e C o n s t r a i n t
 */
returnValue QProblem::removeConstraint(	int number,
										BooleanType updateCholesky
										)
{
	int i, j, ii, jj;

	/* consistency check */
	if ( ( getStatus( ) == QPS_NOTINITIALISED )    ||
		 ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
 		 ( getStatus( ) == QPS_SOLVED )            )
	{
		return THROWERROR( RET_UNKNOWN_BUG );
	}

	/* some definitions */
	int nV  = getNV( );
	int nFR = getNFR( );
	int nAC = getNAC( );
	int nZ  = getNZ( );

	int tcol = sizeT - nAC;
	int number_idx = constraints->getActive( )->getIndex( number );


	/* consistency checks */
	if ( ( constraints->getStatus( number ) == ST_INACTIVE ) || ( constraints->getStatus( number ) == ST_DISABLED ) )
		return THROWERROR( RET_CONSTRAINT_NOT_ACTIVE );

	if ( ( number_idx < 0 ) || ( number_idx >= nAC ) )
		return THROWERROR( RET_CONSTRAINT_NOT_ACTIVE );


	int* FR_idx = new int[nFR];
	if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FR_idx;
		return THROWERROR( RET_REMOVECONSTRAINT_FAILED );
	}


	/* I) REMOVE <number>th ROW FROM T,
	 *    i.e. shift rows number+1 through nAC  upwards (instead of the actual
	 *    constraint number its corresponding index within matrix A is used). */
	if ( number_idx < nAC-1 )
	{
		for( i=(number_idx+1); i<nAC; ++i )
			for( j=(nAC-i-1); j<nAC; ++j )
				T[(i-1)*sizeT + tcol+j] = T[i*sizeT + tcol+j];
		/* gimmick: write zeros into the last row of T */
		for( j=0; j<nAC; ++j )
			T[(nAC-1)*sizeT + tcol+j] = 0.0;


		/* II) RESTORE TRIANGULAR FORM OF T,
		 *     use column-wise Givens rotations to restore reverse triangular form
		 *     of T simultanenous change of Q (i.e. Y). */
		double c, s, nu;

		for( j=(nAC-2-number_idx); j>=0; --j )
		{
			computeGivens( T[(nAC-2-j)*sizeT + tcol+1+j],T[(nAC-2-j)*sizeT + tcol+j], T[(nAC-2-j)*sizeT + tcol+1+j],T[(nAC-2-j)*sizeT + tcol+j],c,s );
			nu = s/(1.0+c);

			for( i=(nAC-j-1); i<(nAC-1); ++i )
				applyGivens( c,s,nu,T[i*sizeT + tcol+1+j],T[i*sizeT + tcol+j], T[i*sizeT + tcol+1+j],T[i*sizeT + tcol+j] );

			for( i=0; i<nFR; ++i )
			{
				ii = FR_idx[i];
				applyGivens( c,s,nu,Q[ii*nV + nZ+1+j],Q[ii*nV + nZ+j], Q[ii*nV + nZ+1+j],Q[ii*nV + nZ+j] );
			}
		}
	}
	else
	{
		/* gimmick: write zeros into the last row of T */
		for( j=0; j<nAC; ++j )
			T[(nAC-1)*sizeT + tcol+j] = 0.0;
	}


	if ( ( updateCholesky == BT_TRUE ) &&
		 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
	{
		/* III) UPDATE CHOLESKY DECOMPOSITION,
		 *      calculate new additional column (i.e. [r sqrt(rho2)]')
		 *      of the Cholesky factor R. */
		double* Hz = new double[nFR];
		for ( i=0; i<nFR; ++i )
			Hz[i] = 0.0;
		double rho2 = 0.0;

		/* 1) Calculate Hz = H*z, where z is the new rightmost column of Z
		 *    (i.e. the old leftmost column of Y).  */
		for( j=0; j<nFR; ++j )
		{
			jj = FR_idx[j];
			for( i=0; i<nFR; ++i )
				Hz[i] += H[jj*nV + FR_idx[i]] * Q[jj*nV + nZ];
		}

		if ( nZ > 0 )
		{
			double* ZHz = new double[nZ];
			for ( i=0; i<nZ; ++i )
				ZHz[i] = 0.0;
			double* r = new double[nZ];

			/* 2) Calculate ZHz = Z'*Hz (old Z). */
			for( j=0; j<nFR; ++j )
			{
				jj = FR_idx[j];

				for( i=0; i<nZ; ++i )
					ZHz[i] += Q[jj*nV + i] * Hz[j];
			}

			/* 3) Calculate r = R^-T * ZHz. */
			if ( backsolveR( ZHz,BT_TRUE,r ) != SUCCESSFUL_RETURN )
			{
				delete[] FR_idx; delete[] Hz; delete[] r; delete[] ZHz;
				return THROWERROR( RET_REMOVECONSTRAINT_FAILED );
			}

			/* 4) Calculate rho2 = rho^2 = z'*Hz - r'*r
			 *    and store r into R. */
			for( i=0; i<nZ; ++i )
			{
				rho2 -= r[i]*r[i];
				R[i*nV + nZ] = r[i];
			}

			delete[] r;	delete[] ZHz;
		}

		for( j=0; j<nFR; ++j )
			rho2 += Q[FR_idx[j]*nV + nZ] * Hz[j];

		delete[] Hz;

		/* 5) Store rho into R. */
		if ( rho2 > 0.0 )
			R[nZ*nV + nZ] = sqrt( rho2 );
		else
		{
			delete[] FR_idx;

			hessianType = HST_SEMIDEF;
			return THROWERROR( RET_HESSIAN_NOT_SPD );
		}
	}

 	delete[] FR_idx;


	/* IV) UPDATE INDICES */
	if ( constraints->getStatus( number ) == ST_DISABLING )
	{
		if ( constraints->moveActiveToDisabled( number ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_REMOVECONSTRAINT_FAILED );
	}
	else
	{
		if ( constraints->moveActiveToInactive( number ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_REMOVECONSTRAINT_FAILED );
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	r e m o v e B o u n d
 */
returnValue QProblem::removeBound(	int number,
									BooleanType updateCholesky
									)
{
	int i, j, ii, jj;

	/* consistency checks */
	if ( bounds->getStatus( number ) == ST_INACTIVE )
		return THROWERROR( RET_BOUND_NOT_ACTIVE );

	if ( ( getStatus( ) == QPS_NOTINITIALISED )    ||
		 ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
 		 ( getStatus( ) == QPS_SOLVED )            )
	{
		return THROWERROR( RET_UNKNOWN_BUG );
	}

	/* some definitions */
	int nV  = getNV( );
	int nFR = getNFR( );
	int nAC = getNAC( );
	int nZ  = getNZ( );

	int tcol = sizeT - nAC;


	/* I) UPDATE INDICES */
	if ( bounds->moveFixedToFree( number ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_REMOVEBOUND_FAILED );


	int* FR_idx = new int[nFR+1];
	if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] FR_idx;
		return THROWERROR( RET_REMOVEBOUND_FAILED );
	}

	/* I) APPEND <nFR+1>th UNITY VECOTR TO Q. */
	int nnFRp1 = FR_idx[nFR];
	for( i=0; i<nFR; ++i )
	{
		ii = FR_idx[i];
		Q[ii*nV + nFR] = 0.0;
		Q[nnFRp1*nV + i] = 0.0;
	}
	Q[nnFRp1*nV + nFR] = 1.0;

	if ( nAC > 0 )
	{
		/* store new column a in a temporary vector instead of shifting T one column to the left and appending a */
		int* AC_idx = new int[nAC];
		if ( constraints->getActive( )->getNumberArray( AC_idx ) != SUCCESSFUL_RETURN )
		{
			delete[] AC_idx; delete[] FR_idx;
			return THROWERROR( RET_REMOVEBOUND_FAILED );
		}

		double* tmp = new double[nAC];
		for( i=0; i<nAC; ++i )
		{
			ii = AC_idx[i];
			tmp[i] =  A[ii*nV + number];
		}


		/* II) RESTORE TRIANGULAR FORM OF T,
		 *     use column-wise Givens rotations to restore reverse triangular form
		 *     of T = [T A(:,number)], simultanenous change of Q (i.e. Y and Z). */
		double c, s, nu;

		for( j=(nAC-1); j>=0; --j )
		{
			computeGivens( tmp[nAC-1-j],T[(nAC-1-j)*sizeT + tcol+j],T[(nAC-1-j)*sizeT + tcol+j],tmp[nAC-1-j],c,s );
			nu = s/(1.0+c);

			for( i=(nAC-j); i<nAC; ++i )
				applyGivens( c,s,nu,tmp[i],T[i*sizeT + tcol+j],T[i*sizeT + tcol+j],tmp[i] );

			for( i=0; i<=nFR; ++i )
			{
				ii = FR_idx[i];
				/* nZ+1+nAC = nFR+1  /  nZ+(1) = nZ+1 */
				applyGivens( c,s,nu,Q[ii*nV + nZ+1+j],Q[ii*nV + nZ+j],Q[ii*nV + nZ+1+j],Q[ii*nV + nZ+j] );
			}
		}

		delete[] tmp; delete[] AC_idx;
	}


	if ( ( updateCholesky == BT_TRUE ) &&
		 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
	{
		/* III) UPDATE CHOLESKY DECOMPOSITION,
		 *      calculate new additional column (i.e. [r sqrt(rho2)]')
		 *      of the Cholesky factor R: */
		double z2 = Q[nnFRp1*nV + nZ];
		double rho2 = H[nnFRp1*nV + nnFRp1]*z2*z2; /* rho2 = h2*z2*z2 */

		if ( nFR > 0 )
		{
			double* Hz = new double[nFR];
			for( i=0; i<nFR; ++i )
				Hz[i] = 0.0;
			/* 1) Calculate R'*r = Zfr'*Hfr*z1 + z2*Zfr'*h1 =: Zfr'*Hz + z2*Zfr'*h1 =: rhs and
			 *    rho2 = z1'*Hfr*z1 + 2*z2*h1'*z1 + h2*z2^2 - r'*r =: z1'*Hz + 2*z2*h1'*z1 + h2*z2^2 - r'r */
			for( j=0; j<nFR; ++j )
			{
				jj = FR_idx[j];
				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					/*			   H * z1 */
					Hz[i] += H[jj*nV + ii] * Q[jj*nV + nZ];
				}
			}

			if ( nZ > 0 )
			{
				double* r = new double[nZ];
				double* rhs = new double[nZ];
				for( i=0; i<nZ; ++i )
					rhs[i] = 0.0;

				/* 2) Calculate rhs. */
				for( j=0; j<nFR; ++j )
				{
					jj = FR_idx[j];
					for( i=0; i<nZ; ++i )
										/* Zfr' * ( Hz + z2*h1 ) */
						rhs[i] += Q[jj*nV + i] * ( Hz[j] + z2 * H[nnFRp1*nV + jj] );
				}

				/* 3) Calculate r = R^-T * rhs. */
				if ( backsolveR( rhs,BT_TRUE,BT_TRUE,r ) != SUCCESSFUL_RETURN )
				{
					delete[] FR_idx; delete[] Hz; delete[] r; delete[] rhs;
					return THROWERROR( RET_REMOVEBOUND_FAILED );
				}


				/* 4) Calculate rho2 = rho^2 = z'*Hz - r'*r
				 *    and store r into R. */
				for( i=0; i<nZ; ++i )
				{
					rho2 -= r[i]*r[i];
					R[i*nV + nZ] = r[i];
				}

				delete[] rhs; delete[] r;
			}

			for( j=0; j<nFR; ++j )
			{
				jj = FR_idx[j];
							/* z1' * ( Hz + 2*z2*h1 ) */
				rho2 += Q[jj*nV + nZ] * ( Hz[j] + 2.0*z2*H[nnFRp1*nV + jj] );
			}

			delete[] Hz;
		}

		/* 5) Store rho into R. */
		if ( rho2 > 0.0 )
			R[nZ*nV + nZ] = sqrt( rho2 );
		else
		{
			delete[] FR_idx;

			hessianType = HST_SEMIDEF;
			return THROWERROR( RET_HESSIAN_NOT_SPD );
		}
	}

	delete[] FR_idx;

	return SUCCESSFUL_RETURN;
}


/*
 *	d i s a b l e C o n s t r a i n t s
 */
returnValue QProblem::disableConstraints( const Indexlist* const numbers )
{
	int i;
	int number;

	for ( i=0; i<numbers->getLength( ); ++i )
	{
		number = numbers->getNumber( i );

		switch ( constraints->getStatus( number ) )
		{
			case ST_INACTIVE:
				/* switch constraint immediately to disabled index set or ... */
				if ( constraints->moveInactiveToDisabled( number ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_DISABLECONSTRAINTS_FAILED );

			case ST_LOWER:
				/* ... wait until constraint becomes inactive */
				constraints->setStatus( number,ST_DISABLING );

			case ST_UPPER:
				/* ... wait until constraint becomes inactive */
				constraints->setStatus( number,ST_DISABLING );

			case ST_DISABLED:
				THROWWARNING( RET_ALREADY_DISABLED );

			case ST_DISABLING:
				THROWWARNING( RET_ALREADY_DISABLED );

			default:
				return THROWERROR( RET_DISABLECONSTRAINTS_FAILED );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	e n a b l e C o n s t r a i n t s
 */
returnValue QProblem::enableConstraints( const Indexlist* const numbers )
{
	int i, j;
	int number;

	int nV = getNV( );

	for ( i=0; i<numbers->getLength( ); ++i )
	{
		number = numbers->getNumber( i );

		switch ( constraints->getStatus( number ) )
		{
			case ST_INACTIVE:
				THROWWARNING( RET_ALREADY_ENABLED );

			case ST_LOWER:
				THROWWARNING( RET_ALREADY_ENABLED );

			case ST_UPPER:
 				THROWWARNING( RET_ALREADY_ENABLED );

			case ST_DISABLED:
				/* switch constraint to inactive index set and ... */
				if ( constraints->moveDisabledToInactive( number ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_ENABLECONSTRAINTS_FAILED );

				/* ... update Ax vector (updates have been skipped while disabled) and ... */
				Ax_l[number] = 0.0;
				for ( j=0; j<nV; ++j )
					Ax_l[number] += A[number*nV + j] * x[j];
				Ax_u[number] = Ax_l[number];

				/* ... shift its bounds as closely to its current value as possible */
				if ( Ax_l[number] <= lbA[number] )
					lbA[number] = ( 1.0 - getSign( Ax_l[number] ) * ENABLINGFACTOR ) * Ax_l[number] - ENABLINGOFFSET;

				if ( Ax_u[number] >= ubA[number] )
					ubA[number] = ( 1.0 + getSign( Ax_u[number] ) * ENABLINGFACTOR ) * Ax_u[number] + ENABLINGOFFSET;

				Ax_l[number] = Ax_l[number] - lbA[number];
				Ax_u[number] = ubA[number] - Ax_u[number];


			case ST_DISABLING:
				/* do nothing because constraint is still active (despite former disabling process) */

			default:
				return THROWERROR( RET_ENABLECONSTRAINTS_FAILED );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	b a c k s o l v e T
 */
returnValue QProblem::backsolveT( const double* const b, BooleanType transposed, double* const a ) const
{
	int i, j;
	int nT = getNAC( );
	int tcol = sizeT - nT;

	double sum;

	/* nothing to do */
	if ( nT <= 0 )
		return SUCCESSFUL_RETURN;


	/* Solve Ta = b, where T might be transposed. */
	if ( transposed == BT_FALSE )
	{
		/* solve Ta = b */
		for( i=0; i<nT; ++i )
		{
			sum = b[i];
			for( j=0; j<i; ++j )
				sum -= T[i*sizeT + sizeT-1-j] * a[nT-1-j];

			if ( fabs( T[i*sizeT + sizeT-1-i] ) >= ZERO*fabs( sum ) )
				a[nT-1-i] = sum / T[i*sizeT + sizeT-1-i];
			else
				return THROWERROR( RET_DIV_BY_ZERO );
		}
	}
	else
	{
		/* solve T^T*a = b */
		for( i=0; i<nT; ++i )
		{
			sum = b[i];
			for( j=0; j<i; ++j )
				sum -= T[(nT-1-j)*sizeT + tcol+i] * a[nT-1-j];

			if ( fabs( T[(nT-1-i)*sizeT + tcol+i] ) >= ZERO*fabs( sum ) )
				a[nT-1-i] = sum / T[(nT-1-i)*sizeT + tcol+i];
			else
				return THROWERROR( RET_DIV_BY_ZERO );
		}
	}


	return SUCCESSFUL_RETURN;
}


/*
 *	d e t e r m i n e D a t a S h i f t
 */
returnValue QProblem::determineDataShift(	const int* const FX_idx, const int* const AC_idx,
											const double* const g_new, const double* const lbA_new, const double* const ubA_new,
											const double* const lb_new, const double* const ub_new,
											double* const delta_g, double* const delta_lbA, double* const delta_ubA,
											double* const delta_lb, double* const delta_ub,
											BooleanType& Delta_bC_isZero, BooleanType& Delta_bB_isZero
											) const
{
	int i, ii;
	int nC  = getNC( );
	int nAC = getNAC( );


	/* I) DETERMINE DATA SHIFT FOR BOUNDS */
	QProblemB::determineDataShift(	FX_idx,g_new,lb_new,ub_new,
									delta_g,delta_lb,delta_ub,
									Delta_bB_isZero );


	/* II) DETERMINE DATA SHIFT FOR CONSTRAINTS */
	/* 1) Calculate shift directions. */
	for( i=0; i<nC; ++i )
	{
		/* if lower constraints' bounds are to be disabled or do not exist, shift them to -infinity */
		if ( ( constraints->isEnabled( i ) == BT_TRUE ) && ( lbA_new != 0 ) )
			delta_lbA[i] = lbA_new[i] - lbA[i];
		else
			delta_lbA[i] = -INFTY - lbA[i];
	}

	for( i=0; i<nC; ++i )
	{
		/* if upper constraints' bounds are to be disabled or do not exist, shift them to infinity */
		if ( ( constraints->isEnabled( i ) == BT_TRUE ) && ( ubA_new != 0 ) )
			delta_ubA[i] = ubA_new[i] - ubA[i];
		else
			delta_ubA[i] = INFTY - ubA[i];
	}

	/* 2) Determine if active constraints' bounds are to be shifted. */
	Delta_bC_isZero = BT_TRUE;

	for ( i=0; i<nAC; ++i )
	{
		ii = AC_idx[i];

		if ( ( fabs( delta_lbA[ii] ) > EPS ) || ( fabs( delta_ubA[ii] ) > EPS ) )
		{
			Delta_bC_isZero = BT_FALSE;
			break;
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	d e t e r m i n e S t e p D i r e c t i o n
 */
returnValue QProblem::determineStepDirection(	const int* const FR_idx, const int* const FX_idx, const int* const AC_idx,
												const double* const delta_g, const double* const delta_lbA, const double* const delta_ubA,
												const double* const delta_lb, const double* const delta_ub,
												BooleanType Delta_bC_isZero, BooleanType Delta_bB_isZero,
												double* const delta_xFX, double* const delta_xFR,
												double* const delta_yAC, double* const delta_yFX
												) const
{
	int i, j, ii, jj;
	int nV  = getNV( );
	int nFR = getNFR( );
	int nFX = getNFX( );
	int nAC = getNAC( );
	int nZ  = getNZ( );

	/* initialise auxiliary vectors */
	double* HMX_delta_xFX = new double[nFR];
	double* YFR_delta_xFRy = new double[nFR];
	double* ZFR_delta_xFRz = new double[nFR];
	double* HFR_YFR_delta_xFRy = new double[nFR];
	for( i=0; i<nFR; ++i )
	{
		delta_xFR[i] = 0.0;
		HMX_delta_xFX[i] = 0.0;
		YFR_delta_xFRy[i] = 0.0;
		ZFR_delta_xFRz[i] = 0.0;
		HFR_YFR_delta_xFRy[i] = 0.0;
	}

	double* delta_xFRy = new double[nAC];
	double* delta_xFRz = new double[nZ];
	for( i=0; i<nZ; ++i )
		delta_xFRz[i] = 0.0;



	/* I) DETERMINE delta_xFX */
	if ( nFX > 0 )
	{
		for( i=0; i<nFX; ++i )
		{
			ii = FX_idx[i];

			if ( bounds->getStatus( ii ) == ST_LOWER )
				delta_xFX[i] = delta_lb[ii];
			else
				delta_xFX[i] = delta_ub[ii];
		}
	}

	/* II) DETERMINE delta_xFR */
	if ( nFR > 0 )
	{
		/* 1) Determine delta_xFRy. */
		if ( nAC > 0 )
		{
			if ( ( Delta_bC_isZero == BT_TRUE ) && ( Delta_bB_isZero == BT_TRUE ) )
			{
				for( i=0; i<nAC; ++i )
					delta_xFRy[i] = 0.0;

				for( i=0; i<nFR; ++i )
					delta_xFR[i] = 0.0;
			}
			else
			{
				/* auxillary variable */
				double* delta_xFRy_TMP = new double[nAC];

				for( i=0; i<nAC; ++i )
				{
					ii = AC_idx[i];

					if ( constraints->getStatus( ii ) == ST_LOWER )
						delta_xFRy_TMP[i] = delta_lbA[ii];
					else
						delta_xFRy_TMP[i] = delta_ubA[ii];

					if ( Delta_bB_isZero == BT_FALSE )
					{
						for( j=0; j<nFX; ++j )
						{
							jj = FX_idx[j];
							delta_xFRy_TMP[i] -= A[ii*nV + jj] * delta_xFX[j];
						}
					}
				}

				if ( backsolveT( delta_xFRy_TMP, BT_FALSE, delta_xFRy ) != SUCCESSFUL_RETURN )
				{
					delete[] delta_xFRy_TMP;
					delete[] delta_xFRz; delete[] delta_xFRy;
					delete[] HFR_YFR_delta_xFRy; delete[] ZFR_delta_xFRz; delete[] YFR_delta_xFRy; delete[] HMX_delta_xFX;

					return THROWERROR( RET_STEPDIRECTION_FAILED_TQ );
				}
				delete[] delta_xFRy_TMP;

				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					for( j=0; j<nAC; ++j )
						YFR_delta_xFRy[i] += Q[ii*nV + nZ+j] * delta_xFRy[j];

					/* delta_xFR = YFR*delta_xFRy (+ ZFR*delta_xFRz) */
					delta_xFR[i] = YFR_delta_xFRy[i];
				}
			}
		}

		/* 2) Determine delta_xFRz. */
		if ( ( hessianType == HST_ZERO ) || ( hessianType == HST_IDENTITY ) )
		{
			for( j=0; j<nFR; ++j )
			{
				jj = FR_idx[j];
				for( i=0; i<nZ; ++i )
					delta_xFRz[i] -= Q[jj*nV + i] * delta_g[jj];
			}

			if ( hessianType == HST_ZERO )
			{
				for( i=0; i<nZ; ++i )
					delta_xFRz[i] /= eps;
			}

			if ( nZ > 0 )
			{
				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					for( j=0; j<nZ; ++j )
						ZFR_delta_xFRz[i] += Q[ii*nV + j] * delta_xFRz[j];

					delta_xFR[i] += ZFR_delta_xFRz[i];
				}
			}
		}
		else
		{
			if ( Delta_bB_isZero == BT_FALSE )
			{
				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					for( j=0; j<nFX; ++j )
					{
						jj = FX_idx[j];
						HMX_delta_xFX[i] += H[ii*nV + jj] * delta_xFX[j];
					}
				}
			}

			if ( nAC > 0 )
			{
				if ( ( Delta_bC_isZero == BT_FALSE ) || ( Delta_bB_isZero == BT_FALSE ) )
				{
					for( i=0; i<nFR; ++i )
					{
						ii = FR_idx[i];
						for( j=0; j<nFR; ++j )
						{
							jj = FR_idx[j];
							HFR_YFR_delta_xFRy[i] += H[ii*nV + jj] * YFR_delta_xFRy[j];
						}
					}
				}
			}


			if ( nZ > 0 )
			{
				/* auxiliary variable */
				double* delta_xFRz_TMP = new double[nZ];
				double* delta_xFRz_RHS = new double[nFR];


				if ( ( nAC > 0 ) && ( nFX > 0 ) && ( Delta_bB_isZero == BT_FALSE ) )
				{
					for( j=0; j<nFR; ++j )
					{
						jj = FR_idx[j];
						delta_xFRz_RHS[j] = delta_g[jj] + HFR_YFR_delta_xFRy[j] + HMX_delta_xFX[j];
					}
				}
				else
				{
					if ( ( nAC == 0 ) && ( Delta_bB_isZero == BT_TRUE ) )
					{
						for( j=0; j<nFR; ++j )
						{
							jj = FR_idx[j];
							delta_xFRz_RHS[j] = delta_g[jj];
						}
					}
					else
					{
						if ( nAC > 0 ) /* => Delta_bB_isZero == BT_TRUE, as BT_FALSE would imply nFX>0 */
						{
							for( j=0; j<nFR; ++j )
							{
								jj = FR_idx[j];
								delta_xFRz_RHS[j] = delta_g[jj] + HFR_YFR_delta_xFRy[j];
							}
						}
						else /* Delta_bB_isZero == BT_FALSE, as nAC==0 */
						{
							for( j=0; j<nFR; ++j )
							{
								jj = FR_idx[j];
								delta_xFRz_RHS[j] = delta_g[jj] + HMX_delta_xFX[j];
							}
						}
					}
				}

				for( j=0; j<nFR; ++j )
				{
					jj = FR_idx[j];
					for( i=0; i<nZ; ++i )
						delta_xFRz[i] -= Q[jj*nV + i] * delta_xFRz_RHS[j];
				}


				if ( backsolveR( delta_xFRz,BT_TRUE,delta_xFRz_TMP ) != SUCCESSFUL_RETURN )
				{
					delete[] delta_xFRz_RHS; delete[] delta_xFRz_TMP;
					delete[] delta_xFRz; delete[] delta_xFRy;
					delete[] HFR_YFR_delta_xFRy; delete[] ZFR_delta_xFRz; delete[] YFR_delta_xFRy; delete[] HMX_delta_xFX;

					return THROWERROR( RET_STEPDIRECTION_FAILED_CHOLESKY );
				}


				if ( backsolveR( delta_xFRz_TMP,BT_FALSE,delta_xFRz ) != SUCCESSFUL_RETURN )
				{
					delete[] delta_xFRz_RHS; delete[] delta_xFRz_TMP;
					delete[] delta_xFRz; delete[] delta_xFRy;
					delete[] HFR_YFR_delta_xFRy; delete[] ZFR_delta_xFRz; delete[] YFR_delta_xFRy; delete[] HMX_delta_xFX;

					return THROWERROR( RET_STEPDIRECTION_FAILED_CHOLESKY );
				}
				delete[] delta_xFRz_RHS; delete[] delta_xFRz_TMP;


				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					for( j=0; j<nZ; ++j )
						ZFR_delta_xFRz[i] += Q[ii*nV + j] * delta_xFRz[j];

					delta_xFR[i] += ZFR_delta_xFRz[i];
				}
			}
		}
	}

	/* III) DETERMINE delta_yAC */
	if ( nAC > 0 ) /* => ( nFR = nZ + nAC > 0 ) */
	{
		/* auxiliary variables */
		double* delta_yAC_TMP = new double[nAC];
		for( i=0; i<nAC; ++i )
			delta_yAC_TMP[i] = 0.0;
		double* delta_yAC_RHS = new double[nFR];
		for( i=0; i<nFR; ++i )
			delta_yAC_RHS[i] = 0.0;

		if ( ( hessianType == HST_ZERO ) || ( hessianType == HST_IDENTITY ) )
		{
			/* if zero:     delta_yAC = (T')^-1 * ( Yfr*delta_gFR + eps*delta_xFRy ),
			 * if identity: delta_yAC = (T')^-1 * ( Yfr*delta_gFR +     delta_xFRy ) */
			for( j=0; j<nAC; ++j )
			{
				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					delta_yAC_TMP[j] += Q[ii*nV + nZ+j] * delta_g[ii];
				}
			}

			if ( hessianType == HST_ZERO )
			{
				for( j=0; j<nAC; ++j )
					delta_yAC_TMP[j] += eps*delta_xFRy[j];
			}
			else
			{
				for( j=0; j<nAC; ++j )
					delta_yAC_TMP[j] += delta_xFRy[j];
			}
		}
		else
		{
			if ( ( Delta_bC_isZero == BT_TRUE ) && ( Delta_bB_isZero == BT_TRUE ) )
			{
				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					delta_yAC_RHS[i] = delta_g[ii];
				}
			}
			else
			{
				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					delta_yAC_RHS[i] = HFR_YFR_delta_xFRy[i] + delta_g[ii];
				}
			}

			if ( nZ > 0 )
			{
				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					for( j=0; j<nFR; ++j )
					{
						jj = FR_idx[j];
						delta_yAC_RHS[i] += H[ii*nV + jj] * ZFR_delta_xFRz[j];
					}
				}
			}

			if ( nFX > 0 )
			{
				if ( Delta_bB_isZero == BT_FALSE )
				{
					for( i=0; i<nFR; ++i )
						delta_yAC_RHS[i] += HMX_delta_xFX[i];
				}
			}

			for( i=0; i<nAC; ++i)
			{
				for( j=0; j<nFR; ++j )
				{
					jj = FR_idx[j];
					delta_yAC_TMP[i] += Q[jj*nV + nZ+i] * delta_yAC_RHS[j];
				}
			}
		}

		if ( backsolveT( delta_yAC_TMP,BT_TRUE,delta_yAC ) != SUCCESSFUL_RETURN )
		{
			delete[] delta_yAC_RHS;	delete[] delta_yAC_TMP;
			delete[] delta_xFRz; delete[] delta_xFRy;
			delete[] HFR_YFR_delta_xFRy; delete[] ZFR_delta_xFRz; delete[] YFR_delta_xFRy; delete[] HMX_delta_xFX;

			return THROWERROR( RET_STEPDIRECTION_FAILED_TQ );
		}
		delete[] delta_yAC_RHS;	delete[] delta_yAC_TMP;
	}


	/* IV) DETERMINE delta_yFX */
	if ( nFX > 0 )
	{
		for( i=0; i<nFX; ++i )
		{
			ii = FX_idx[i];

			delta_yFX[i] = delta_g[ii];

			for( j=0; j<nAC; ++j )
			{
				jj = AC_idx[j];
				delta_yFX[i] -= A[jj*nV + ii] * delta_yAC[j];
			}

			if ( ( hessianType == HST_ZERO ) || ( hessianType == HST_IDENTITY ) )
			{
				if ( hessianType == HST_ZERO )
					delta_yFX[i] += eps*delta_xFX[i];
				else
					delta_yFX[i] += delta_xFX[i];
			}
			else
			{
				for( j=0; j<nFR; ++j )
				{
					jj = FR_idx[j];
					delta_yFX[i] += H[ii*nV + jj] * delta_xFR[j];
				}

				if ( Delta_bB_isZero == BT_FALSE )
				{
					for( j=0; j<nFX; ++j )
					{
						jj = FX_idx[j];
						delta_yFX[i] += H[ii*nV + jj] * delta_xFX[j];
					}
				}
			}
		}
	}

	delete[] delta_xFRz; delete[] delta_xFRy;
	delete[] HFR_YFR_delta_xFRy; delete[] ZFR_delta_xFRz; delete[] YFR_delta_xFRy; delete[] HMX_delta_xFX;

	return SUCCESSFUL_RETURN;
}


/*
 *	d e t e r m i n e S t e p L e n g t h
 */
returnValue QProblem::determineStepLength(	const int* const FR_idx, const int* const FX_idx, const int* const AC_idx, const int* const IAC_idx,
											const double* const delta_lbA, const double* const delta_ubA,
											const double* const delta_lb, const double* const delta_ub,
											const double* const delta_xFX, const double* const delta_xFR,
											const double* const delta_yAC, const double* const delta_yFX,
											double* const delta_Ax_l, double* const delta_Ax_u, int& BC_idx, SubjectToStatus& BC_status, BooleanType& BC_isBound
											)
{
	int i, j, ii, jj;
	int nV  = getNV( );
	int nC  = getNC( );
	int nFR = getNFR( );
	int nFX = getNFX( );
	int nAC = getNAC( );
	int nIAC = getNIAC( );

	/* initialise maximum steplength array */
	double* maxStepLength = new double[nV+nC+nV+nC];
	for ( i=0; i<2*(nV+nC); ++i )
		maxStepLength[i] = 1.0;


	/* I) DETERMINE MAXIMUM DUAL STEPLENGTH: */
	/* 1) Ensure that active dual constraints' bounds remain valid
	 *    (ignoring inequality constraints).  */
	for( i=0; i<nAC; ++i )
	{
		ii = AC_idx[i];

		if ( constraints->getType( ii ) != ST_EQUALITY )
		{
			if ( constraints->getStatus( ii ) == ST_LOWER )
			{
				/* active lower constraints' bounds */
				if ( delta_yAC[i] < -ZERO )
				{
					if ( y[nV+ii] > 0.0 )
						maxStepLength[nV+ii] = y[nV+ii] / ( -delta_yAC[i] );
					else
						maxStepLength[nV+ii] = 0.0;
				}
			}
			else
			{
				/* active upper constraints' bounds */
				if ( delta_yAC[i] > ZERO )
				{
					if ( y[nV+ii] < 0.0 )
						maxStepLength[nV+ii] = y[nV+ii] / ( -delta_yAC[i] );
					else
						maxStepLength[nV+ii] = 0.0;
				}
			}
		}
	}

	/* 2) Ensure that active dual bounds remain valid
	 *    (ignoring implicitly fixed variables). */
	for( i=0; i<nFX; ++i )
	{
		ii = FX_idx[i];

		if ( bounds->getType( ii ) != ST_EQUALITY )
		{
			if ( bounds->getStatus( ii ) == ST_LOWER )
			{
				/* active lower bounds */
				if ( delta_yFX[i] < -ZERO )
				{
					if ( y[ii] > 0.0 )
						maxStepLength[ii] = y[ii] / ( -delta_yFX[i] );
					else
						maxStepLength[ii] = 0.0;
				}
			}
			else
			{
				/* active upper bounds */
				if ( delta_yFX[i] > ZERO )
				{
					if ( y[ii] < 0.0 )
						maxStepLength[ii] = y[ii] / ( -delta_yFX[i] );
					else
						maxStepLength[ii] = 0.0;
				}
			}
		}
	}


	/* II) DETERMINE MAXIMUM PRIMAL STEPLENGTH */
	/* 1) Ensure that inactive constraints' bounds remain valid
	 *    (ignoring unbounded constraints). */
	double* delta_x = new double[nV];
	for( j=0; j<nFR; ++j )
	{
		jj = FR_idx[j];
		delta_x[jj] = delta_xFR[j];
	}
	for( j=0; j<nFX; ++j )
	{
		jj = FX_idx[j];
		delta_x[jj] = delta_xFX[j];
	}

	#ifdef __MANY_CONSTRAINTS__
	double delta_x_max = 0.0;
	for( i=0; i<nV; ++i )
	{
		if ( fabs( delta_x[i] ) > delta_x_max )
			delta_x_max = fabs( delta_x[i] );
	}
	#endif

	for( i=0; i<nIAC; ++i )
	{
		ii = IAC_idx[i];

		if ( constraints->getType( ii ) != ST_UNBOUNDED )
		{
			#ifdef __MANY_CONSTRAINTS__
			delta_Ax_l[ii] = -delta_x_max;
			if ( delta_lbA[ii] > 0.0 )
				delta_Ax_l[ii] -= delta_lbA[ii];

			delta_Ax_u[ii] = -delta_x_max;
			if ( delta_ubA[ii] < 0.0 )
				delta_Ax_l[ii] += delta_ubA[ii];

			if ( ( -delta_Ax_l[ii] >= Ax_l[ii] ) || ( -delta_Ax_u[ii] >= Ax_u[ii] ) )
			{
				/* calculate product A*x */
				if ( constraintProduct == 0 )
				{
					Ax_l[ii] = 0.0;
					delta_Ax_l[ii] = 0.0;
					for( j=0; j<nV; ++j )
					{
						Ax_l[ii] += A[ii*nV + j] * x[j];
						delta_Ax_l[ii] += A[ii*nV + j] * delta_x[j];
					}
				}
				else
				{
					if ( (*constraintProduct)( ii,x, &(Ax_l[ii]) ) != 0 )
					{
						delete[] delta_x; delete[] maxStepLength;
						return THROWERROR( RET_ERROR_IN_CONSTRAINTPRODUCT );
					}

					if ( (*constraintProduct)( ii,delta_x, &(delta_Ax_l[ii]) ) != 0 )
					{
						delete[] delta_x; delete[] maxStepLength;
						return THROWERROR( RET_ERROR_IN_CONSTRAINTPRODUCT );
					}
				}

				Ax_u[ii] = ubA[ii] - Ax_l[ii];
				Ax_l[ii] = Ax_l[ii] - lbA[ii];
				delta_Ax_u[ii] = delta_Ax_l[ii];

				/* inactive lower constraints' bounds */
				if ( constraints->isNoLower( ) == BT_FALSE )
				{
					if ( delta_lbA[ii] > delta_Ax_l[ii] )
					{
						if ( Ax_l[ii] > 0.0 )
							maxStepLength[nV+ii] = Ax_l[ii] / ( delta_lbA[ii] - delta_Ax_l[ii] );
						else
							maxStepLength[nV+ii] = 0.0;
					}
				}
				delta_Ax_l[ii] = delta_Ax_l[ii] - delta_lbA[ii];
	
				/* inactive upper constraints' bounds */
				if ( constraints->isNoUpper( ) == BT_FALSE )
				{
					if ( delta_ubA[ii] < delta_Ax_u[ii] )
					{
						if ( Ax_u[ii] > 0.0 )
							maxStepLength[nV+nC+nV+ii] = ( -Ax_u[ii] ) / ( delta_ubA[ii] - delta_Ax_u[ii] );
						else
							maxStepLength[nV+nC+nV+ii] = 0.0;
					}
				}
				delta_Ax_u[ii] = delta_ubA[ii] - delta_Ax_u[ii];
			}
			#else
			/* calculate product A*x */
			if ( constraintProduct == 0 )
			{
				delta_Ax_l[ii] = 0.0;
 				for( j=0; j<nV; ++j )
 					delta_Ax_l[ii] += A[ii*nV + j] * delta_x[j];
			}
			else
			{
				if ( (*constraintProduct)( ii,delta_x, &(delta_Ax_l[ii]) ) != 0 )
				{
					delete[] delta_x; delete[] maxStepLength;
					return THROWERROR( RET_ERROR_IN_CONSTRAINTPRODUCT );
				}
			}
			delta_Ax_u[ii] = delta_Ax_l[ii];

			/* inactive lower constraints' bounds */
			if ( constraints->isNoLower( ) == BT_FALSE )
			{
				if ( delta_lbA[ii] > delta_Ax_l[ii] )
				{
					if ( Ax_l[ii] > 0.0 )
						maxStepLength[nV+ii] = Ax_l[ii] / ( delta_lbA[ii] - delta_Ax_l[ii] );
					else
						maxStepLength[nV+ii] = 0.0;
				}
			}
			delta_Ax_l[ii] = delta_Ax_l[ii] - delta_lbA[ii];

			/* inactive upper constraints' bounds */
			if ( constraints->isNoUpper( ) == BT_FALSE )
			{
				if ( delta_ubA[ii] < delta_Ax_u[ii] )
				{
					if ( Ax_u[ii] > 0.0 )
						maxStepLength[nV+nC+nV+ii] = ( -Ax_u[ii] ) / ( delta_ubA[ii] - delta_Ax_u[ii] );
					else
						maxStepLength[nV+nC+nV+ii] = 0.0;
				}
			}
			delta_Ax_u[ii] = delta_ubA[ii] - delta_Ax_u[ii];
			#endif
		}
	}

	delete[] delta_x;


	/* 2) Ensure that inactive bounds remain valid
	 *    (ignoring unbounded variables). */
	/* inactive lower bounds */
	if ( bounds->isNoLower( ) == BT_FALSE )
	{
		for( i=0; i<nFR; ++i )
		{
			ii = FR_idx[i];
			if ( bounds->getType( ii ) != ST_UNBOUNDED )
				if ( delta_lb[ii] > delta_xFR[i] )
				{
					if ( x[ii] > lb[ii] )
						maxStepLength[ii] = ( x[ii] - lb[ii] ) / ( delta_lb[ii] - delta_xFR[i] );
					else
						maxStepLength[ii] = 0.0;
				}
		}
	}

	/* inactive upper bounds */
	if ( bounds->isNoUpper( ) == BT_FALSE )
	{
		for( i=0; i<nFR; ++i )
		{
			ii = FR_idx[i];
			if ( bounds->getType( ii ) != ST_UNBOUNDED )
				if ( delta_ub[ii] < delta_xFR[i] )
				{
					if ( x[ii] < ub[ii] )
						maxStepLength[nV+nC+ii] = ( x[ii] - ub[ii] ) / ( delta_ub[ii] - delta_xFR[i] );
					else
						maxStepLength[nV+nC+ii] = 0.0;
				}
		}
	}


	/* III) DETERMINE MAXIMUM HOMOTOPY STEPLENGTH */
	double tau_new = 1.0;

	BC_idx = -1;
	BC_status = ST_UNDEFINED;
	BC_isBound = BT_FALSE;

	for ( i=0; i<nV; ++i )
	{
		/* 1) Consider lower/dual blocking bounds. */
		if ( maxStepLength[i] < tau_new )
		{
			tau_new = maxStepLength[i];
			BC_idx = i;
			BC_isBound = BT_TRUE;
			if ( bounds->getStatus( i ) == ST_INACTIVE ) /* inactive? */
				BC_status = ST_LOWER;
			else
				BC_status = ST_INACTIVE;
		}

		/* 2) Consider upper blocking bounds. */
		if ( maxStepLength[nV+nC+i] < tau_new )
		{
			tau_new = maxStepLength[nV+nC+i];
			BC_idx = i;
			BC_isBound = BT_TRUE;
			BC_status = ST_UPPER;
		}
	}

	for ( i=nV; i<nV+nC; ++i )
	{
		/* 3) Consider lower/dual blocking constraints. */
		if ( maxStepLength[i] < tau_new )
		{
			tau_new = maxStepLength[i];
			BC_idx = i-nV;
			BC_isBound = BT_FALSE;
			if ( constraints->getStatus( i-nV ) == ST_INACTIVE ) /* inactive? */
				BC_status = ST_LOWER;
			else
				BC_status = ST_INACTIVE;
		}

		/* 4) Consider upper blocking constraints. */
		if ( maxStepLength[nV+nC+i] < tau_new )
		{
			tau_new = maxStepLength[nV+nC+i];
			BC_idx = i-nV;
			BC_isBound = BT_FALSE;
			BC_status = ST_UPPER;
		}
	}

	delete[] maxStepLength;


	/* IV) CLEAR CYCLING DATA
	 *     if a positive step can be taken */
	if ( tau_new > EPS )
		cyclingManager->clearCyclingData( );


	/* V) SET MAXIMUM HOMOTOPY STEPLENGTH */
	tau = tau_new;

	/* Optimal solution found as stepsize is numerically one! */
	if ( tau > 1.0 - 100.0*EPS )
	{
		tau = 1.0;
		BC_idx = -1;
		BC_status = ST_UNDEFINED;
		BC_isBound = BT_FALSE;
	}

	#ifndef __XPCTARGET__
	char messageString[80];

	if ( BC_status == ST_UNDEFINED )
		snprintf( messageString,80,"Stepsize is %.16e!",tau );
	else
		snprintf( messageString,80,"Stepsize is %.16e! (BC_idx = %d, BC_isBound = %d, BC_status = %d)",tau,BC_idx,BC_isBound,BC_status );

	getGlobalMessageHandler( )->throwInfo( RET_STEPSIZE_NONPOSITIVE,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	p e r f o r m S t e p
 */
returnValue QProblem::performStep(	const int* const FR_idx, const int* const FX_idx, const int* const AC_idx, const int* const IAC_idx,
									const double* const delta_g, const double* const delta_lbA, const double* const delta_ubA,
									const double* const delta_lb, const double* const delta_ub,
									const double* const delta_xFX, const double* const delta_xFR,
									const double* const delta_yAC, const double* const delta_yFX,
									const double* const delta_Ax_l, const double* const delta_Ax_u, int BC_idx, SubjectToStatus BC_status, BooleanType BC_isBound
									)
{
	int i, j, ii;
	int nV  = getNV( );
	int nC  = getNC( );
	int nFR = getNFR( );
	int nFX = getNFX( );
	int nAC = getNAC( );
	int nIAC = getNIAC( );


	/* I) CHECK (CONSTRAINTS') BOUNDS' CONSISTENCY */
	if ( areBoundsConsistent( delta_lb,delta_ub,delta_lbA,delta_ubA ) == BT_FALSE )
	{
		infeasible = BT_TRUE;
		tau = 0.0;

		return THROWERROR( RET_QP_INFEASIBLE );
	}

    /*print( &tau,1,"tau" );
    print( delta_xFR,nFR,"delta_uFR" );
    print( delta_xFX,nFX,"delta_uFX" );*/

	/* II) MOVE ALONG HOMOTOPY PATH */
	if ( tau > ZERO )
	{
		/* 1) Perform step in primal und dual space... */
		for( i=0; i<nFR; ++i )
		{
			ii = FR_idx[i];
			x[ii] += tau*delta_xFR[i];
		}
       
		for( i=0; i<nFX; ++i )
		{
			ii = FX_idx[i];
			x[ii] += tau*delta_xFX[i];
			y[ii] += tau*delta_yFX[i];
		}

		for( i=0; i<nAC; ++i )
		{
			ii = AC_idx[i];
			y[nV+ii] += tau*delta_yAC[i];
		}

		/* 2) Shift QP data. */
		for( i=0; i<nV; ++i )
		{
			g[i]  += tau*delta_g[i];
			lb[i] += tau*delta_lb[i];
			ub[i] += tau*delta_ub[i];
		}

		for( i=0; i<nC; ++i )
		{
			lbA[i] += tau*delta_lbA[i];
			ubA[i] += tau*delta_ubA[i];
		}

		/* ... also for Ax. */
		for( i=0; i<nIAC; ++i )
		{
			ii = IAC_idx[i];
			if ( constraints->getType( ii ) != ST_UNBOUNDED )
			{
				Ax_l[ii] += tau*delta_Ax_l[ii];
				Ax_u[ii] += tau*delta_Ax_u[ii];
			}
		}
		for( i=0; i<nAC; ++i )
		{
			ii = AC_idx[i];

			Ax_l[ii] = 0.0;
			for( j=0; j<nV; ++j )
				Ax_l[ii] += A[ii*nV + j] * x[j];

			Ax_u[ii] = ubA[ii] - Ax_l[ii];
			Ax_l[ii] = Ax_l[ii] - lbA[ii];
		}
	}
	else
	{
		/* print a stepsize warning if stepsize is zero */
		#ifndef __XPCTARGET__
		char messageString[80];
		snprintf( messageString,80,"Stepsize is %.6e",tau );
		getGlobalMessageHandler( )->throwWarning( RET_STEPSIZE,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		#endif
	}


	/* III) UPDATE ACTIVE SET */
	char messageString[80];

	switch ( BC_status )
	{
		/* Optimal solution found as no working set change detected. */
		case ST_UNDEFINED:
			return RET_OPTIMAL_SOLUTION_FOUND;


		/* Remove one variable from active set. */
		case ST_INACTIVE:
			if ( BC_isBound == BT_TRUE )
			{
				#ifndef __XPCTARGET__
				snprintf( messageString,80,"bound no. %d.", BC_idx );
				getGlobalMessageHandler( )->throwInfo( RET_REMOVE_FROM_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
				#endif

				if ( removeBound( BC_idx,BT_TRUE ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_REMOVE_FROM_ACTIVESET_FAILED );

				y[BC_idx] = 0.0;
			}
			else
			{
				#ifndef __XPCTARGET__
				snprintf( messageString,80,"constraint no. %d.", BC_idx );
				getGlobalMessageHandler( )->throwInfo( RET_REMOVE_FROM_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
				#endif

				if ( removeConstraint( BC_idx,BT_TRUE ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_REMOVE_FROM_ACTIVESET_FAILED );

				y[nV+BC_idx] = 0.0;
			}
			break;


		/* Add one variable to active set. */
		default:
			if ( BC_isBound == BT_TRUE )
			{
				#ifndef __XPCTARGET__
				if ( BC_status == ST_LOWER )
					snprintf( messageString,80,"lower bound no. %d.", BC_idx );
				else
					snprintf( messageString,80,"upper bound no. %d.", BC_idx );
				getGlobalMessageHandler( )->throwInfo( RET_ADD_TO_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
				#endif

				if ( addBound( BC_idx,BC_status,BT_TRUE ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_ADD_TO_ACTIVESET_FAILED );
			}
			else
			{
				#ifndef __XPCTARGET__
				if ( BC_status == ST_LOWER )
					snprintf( messageString,80,"lower constraint's bound no. %d.", BC_idx );
				else
					snprintf( messageString,80,"upper constraint's bound no. %d.", BC_idx );
				getGlobalMessageHandler( )->throwInfo( RET_ADD_TO_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
				#endif

				if ( addConstraint( BC_idx,BC_status,BT_TRUE ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_ADD_TO_ACTIVESET_FAILED );
			}
			break;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A u x i l i a r y Q P
 */
returnValue QProblem::setupAuxiliaryQP( const Bounds* const guessedBounds, const Constraints* const guessedConstraints )
{
	int i, j;
	int nV = getNV( );
	int nC = getNC( );

	/* consistency check */
	if ( ( guessedBounds == 0 ) || ( guessedConstraints == 0 ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* nothing to do */
	if ( ( guessedBounds == bounds ) && ( guessedConstraints == constraints ) )
		return SUCCESSFUL_RETURN;

	status = QPS_PREPARINGAUXILIARYQP;


	/* I) SETUP WORKING SET ... */
	if ( shallRefactorise( guessedBounds,guessedConstraints ) == BT_TRUE )
	{
		/* ... WITH REFACTORISATION: */
		/* 1) Reset bounds/constraints ... */
		if ( bounds != 0 )
			delete bounds;
		if ( constraints != 0 )
			delete constraints;
		bounds = new Bounds( nV );
		constraints = new Constraints( nC );

		/*    ... and set them up afresh. */
		if ( setupSubjectToType( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		if ( bounds->setupAllFree( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		if ( constraints->setupAllInactive( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		/* 2) Setup TQ factorisation. */
		if ( setupTQfactorisation( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		/* 3) Setup guessed working sets afresh. */
		if ( setupAuxiliaryWorkingSet( guessedBounds,guessedConstraints,BT_TRUE ) != SUCCESSFUL_RETURN )
			THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		/* 4) Calculate Cholesky decomposition. */
		if ( ( getNAC( ) + getNFX( ) ) == 0 )
		{
			/* Factorise full Hessian if no bounds/constraints are active. */
			if ( setupCholeskyDecomposition( ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
		}
		else
		{
			/* Factorise projected Hessian if there active bounds/constraints. */
			if ( setupCholeskyDecompositionProjected( ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
		}
	}
	else
	{
		/* ... WITHOUT REFACTORISATION: */
		if ( setupAuxiliaryWorkingSet( guessedBounds,guessedConstraints,BT_FALSE ) != SUCCESSFUL_RETURN )
			THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}


	/* II) SETUP AUXILIARY QP DATA: */
	/* 1) Ensure that dual variable is zero for fixed bounds and active constraints. */
	for ( i=0; i<nV; ++i )
		if ( bounds->getStatus( i ) != ST_INACTIVE )
			y[i] = 0.0;

	for ( i=0; i<nC; ++i )
		if ( constraints->getStatus( i ) != ST_INACTIVE )
			y[nV+i] = 0.0;

	/* 2) Setup gradient and (constraints') bound vectors. */
	if ( setupAuxiliaryQPgradient( ) != SUCCESSFUL_RETURN )
		THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

	for ( j=0; j<nC; ++j )
	{
		Ax_l[j] = 0.0;

		for( i=0; i<nV; ++i )
			Ax_l[j] += A[j*nV + i] * x[i];

		Ax_u[j] = Ax_l[j];
	}

	/* (also sets Ax_l and Ax_u) */
	if ( setupAuxiliaryQPbounds( 0,0,BT_FALSE ) != SUCCESSFUL_RETURN )
		THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

	return SUCCESSFUL_RETURN;
}


/*
 *	s h a l l R e f a c t o r i s e
 */

BooleanType QProblem::shallRefactorise(	const Bounds* const guessedBounds,
										const Constraints* const guessedConstraints
										) const
{
	int i;
	int nV = getNV( );
	int nC = getNC( );

	/* always refactorise if Hessian is not known to be positive definite */
	if ( getHessianType( ) == HST_SEMIDEF )
		return BT_TRUE;

	/* 1) Determine number of bounds that have same status
	 *    in guessed AND current bounds.*/
	int differenceNumberBounds = 0;

	for( i=0; i<nV; ++i )
		if ( guessedBounds->getStatus( i ) != bounds->getStatus( i ) )
			++differenceNumberBounds;

	/* 2) Determine number of constraints that have same status
	 *    in guessed AND current constraints.*/
	int differenceNumberConstraints = 0;

	for( i=0; i<nC; ++i )
		if ( guessedConstraints->getStatus( i ) != constraints->getStatus( i ) )
			++differenceNumberConstraints;

	/* 3) Decide wheter to refactorise or not. */
	if ( 2*(differenceNumberBounds+differenceNumberConstraints) > guessedConstraints->getNAC( )+guessedBounds->getNFX( ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}


/*
 *	a r e B o u n d s C o n s i s t e n t
 */
BooleanType QProblem::areBoundsConsistent(	const double* const delta_lb, const double* const delta_ub,
											const double* const delta_lbA, const double* const delta_ubA
											) const
{
	int i;

	/* 1) Check bounds' consistency. */
	if ( QProblemB::areBoundsConsistent( delta_lb,delta_ub ) == BT_FALSE )
		return BT_FALSE;

	/* 2) Check constraints' consistency, i.e.
	 *    check if delta_lb[i] is greater than delta_ub[i]
	 *    for a component i whose bounds are already (numerically) equal. */
	for( i=0; i<getNC( ); ++i )
		if ( ( lbA[i] > ubA[i] - BOUNDTOL ) && ( delta_lbA[i] > delta_ubA[i] + EPS ) )
			return BT_FALSE;

	return BT_TRUE;
}


/*
 *	s e t u p Q P d a t a
 */
returnValue QProblem::setupQPdata(	const double* const _H, const double* const _g, const double* const _A,
									const double* const _lb, const double* const _ub,
									const double* const _lbA, const double* const _ubA
									)
{
	int i;
	int nC = getNC( );


	/* 1) Load Hessian matrix as well as lower and upper bounds vectors. */
	if ( QProblemB::setupQPdata( _H,_g,_lb,_ub ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 2) Load constraint matrix. */
	if ( ( nC > 0 ) && ( _A == 0 ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	if ( nC > 0 )
	{
		setA( _A );

		/* 3) Setup lower constraints' bounds vector. */
		if ( _lbA != 0 )
		{
			setLBA( _lbA );
		}
		else
		{
			/* if no lower constraints' bounds are specified, set them to -infinity */
			for( i=0; i<nC; ++i )
				lbA[i] = -INFTY;
		}

		/* 4) Setup upper constraints' bounds vector. */
		if ( _ubA != 0 )
		{
			setUBA( _ubA );
		}
		else
		{
			/* if no upper constraints' bounds are specified, set them to infinity */
			for( i=0; i<nC; ++i )
				ubA[i] = INFTY;
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p Q P d a t a F r o m F i l e
 */
returnValue QProblem::setupQPdataFromFile(	const char* const H_file, const char* const g_file, const char* const A_file,
											const char* const lb_file, const char* const ub_file,
											const char* const lbA_file, const char* const ubA_file
											)
{
	int i;
	int nV = getNV( );
	int nC = getNC( );

	returnValue returnvalue;


	/* 1) Load Hessian matrix as well as lower and upper bounds vectors from files. */
	returnvalue = QProblemB::setupQPdataFromFile( H_file,g_file,lb_file,ub_file );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return THROWERROR( returnvalue );

	/* 2) Load constraint matrix from file. */
	if ( ( nC > 0 ) && ( A_file == 0 ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	if ( nC > 0 )
	{
		returnvalue = readFromFile( A, nC,nV, A_file );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return THROWERROR( returnvalue );

		/* 3) Load lower constraints' bounds vector from file. */
		if ( lbA_file != 0 )
		{
			returnvalue = readFromFile( lbA, nC, lbA_file );
			if ( returnvalue != SUCCESSFUL_RETURN )
				return THROWERROR( returnvalue );
		}
		else
		{
			/* if no lower constraints' bounds are specified, set them to -infinity */
			for( i=0; i<nC; ++i )
				lbA[i] = -INFTY;
		}

		/* 4) Load upper constraints' bounds vector from file. */
		if ( ubA_file != 0 )
		{
			returnvalue = readFromFile( ubA, nC, ubA_file );
			if ( returnvalue != SUCCESSFUL_RETURN )
				return THROWERROR( returnvalue );
		}
		else
		{
			/* if no upper constraints' bounds are specified, set them to infinity */
			for( i=0; i<nC; ++i )
				ubA[i] = INFTY;
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	l o a d Q P v e c t o r s F r o m F i l e
 */
returnValue QProblem::loadQPvectorsFromFile(	const char* const g_file, const char* const lb_file, const char* const ub_file,
												const char* const lbA_file, const char* const ubA_file,
												double* const g_new, double* const lb_new, double* const ub_new,
												double* const lbA_new, double* const ubA_new
												) const
{
	int nC = getNC( );

	returnValue returnvalue;


	/* 1) Load gradient vector as well as lower and upper bounds vectors from files. */
	returnvalue = QProblemB::loadQPvectorsFromFile( g_file,lb_file,ub_file, g_new,lb_new,ub_new );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return THROWERROR( returnvalue );

	if ( nC > 0 )
	{
		/* 2) Load lower constraints' bounds vector from file. */
		if ( lbA_file != 0 )
		{
			if ( lbA_new != 0 )
			{
				returnvalue = readFromFile( lbA_new, nC, lbA_file );
				if ( returnvalue != SUCCESSFUL_RETURN )
					return THROWERROR( returnvalue );
			}
			else
			{
				/* If filename is given, storage must be provided! */
				return THROWERROR( RET_INVALID_ARGUMENTS );
			}
		}

		/* 3) Load upper constraints' bounds vector from file. */
		if ( ubA_file != 0 )
		{
			if ( ubA_new != 0 )
			{
				returnvalue = readFromFile( ubA_new, nC, ubA_file );
				if ( returnvalue != SUCCESSFUL_RETURN )
					return THROWERROR( returnvalue );
			}
			else
			{
				/* If filename is given, storage must be provided! */
				return THROWERROR( RET_INVALID_ARGUMENTS );
			}
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t I t e r a t i o n
 */
returnValue QProblem::printIteration( 	int iteration,
										int BC_idx,	SubjectToStatus BC_status, BooleanType BC_isBound
		  								)
{
	#ifndef __XPCTARGET__
	char myPrintfString[80];

	/* consistency check */
	if ( iteration < 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* nothing to do */
	if ( printLevel != PL_MEDIUM )
		return SUCCESSFUL_RETURN;


	/* 1) Print header at first iteration. */
 	if ( iteration == 0 )
	{
		snprintf( myPrintfString,80,"\n\n####################   qpOASES  --  QP NO. %3.0d   #####################\n\n", count );
		myPrintf( myPrintfString );

		myPrintf( "    Iter   |    StepLength    |       Info       |   nFX   |   nAC    \n" );
		myPrintf( " ----------+------------------+------------------+---------+--------- \n" );
	}

	/* 2) Print iteration line. */
	if ( BC_status == ST_UNDEFINED )
	{
		if ( hessianType == HST_ZERO )
			snprintf( myPrintfString,80,"   %5.1d   |   %1.6e   |    LP SOLVED     |  %4.1d   |  %4.1d   \n", iteration,tau,getNFX( ),getNAC( ) );
		else
			snprintf( myPrintfString,80,"   %5.1d   |   %1.6e   |    QP SOLVED     |  %4.1d   |  %4.1d   \n", iteration,tau,getNFX( ),getNAC( ) );
		myPrintf( myPrintfString );
	}
	else
	{
		char info[8];

		if ( BC_status == ST_INACTIVE )
			snprintf( info,5,"REM " );
		else
			snprintf( info,5,"ADD " );

		if ( BC_isBound == BT_TRUE )
			snprintf( &(info[4]),4,"BND" );
		else
			snprintf( &(info[4]),4,"CON" );

		snprintf( myPrintfString,80,"   %5.1d   |   %1.6e   |   %s %4.1d   |  %4.1d   |  %4.1d   \n", iteration,tau,info,BC_idx,getNFX( ),getNAC( ) );
		myPrintf( myPrintfString );
	}
	#endif

	return SUCCESSFUL_RETURN;
}


#ifndef __DSPACE__
} /* qpOASES */
#endif


/*
 *	end of file
 */
