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
 *	\file SRC/QProblemB.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Implementation of the QProblemB class which is able to use the newly
 *	developed online active set strategy for parametric quadratic programming.
 */


#include <stdio.h>

#include <QProblemB.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	Q P r o b l e m B
 */
QProblemB::QProblemB( )
{
	/* print copyright notice */
	#ifndef __SUPPRESSANYOUTPUT__
		#ifndef __XPCTARGET__
		#ifndef __DSPACE__
		printCopyrightNotice( );
		#endif
		#endif
	#endif

	/* reset global message handler */
	getGlobalMessageHandler( )->reset( );

	H = 0;
	g = 0;
	lb = 0;
	ub = 0;

	bounds = 0;

	R = 0;

	x = 0;
	y = 0;

	tau = 0.0;

	eps = 0.0;
	nRegSteps = MAX_NUMBER_OF_REGULARISATION_STEPS;

	hessianType = HST_UNKNOWN;
	infeasible = BT_FALSE;
	unbounded = BT_FALSE;

	status = QPS_NOTINITIALISED;

	printLevel =   PL_MEDIUM;
	setPrintLevel( PL_MEDIUM );
	#ifdef __DEBUG__
	printLevel =   PL_HIGH;
	setPrintLevel( PL_HIGH );
	#endif
	#ifdef __XPCTARGET__
	printLevel =   PL_NONE;
	setPrintLevel( PL_NONE );
	#endif

	count = 0;
}


/*
 *	Q P r o b l e m B
 */
QProblemB::QProblemB( int _nV )
{
	/* print copyright notice */
	#ifndef __SUPPRESSANYOUTPUT__
		#ifndef __XPCTARGET__
		#ifndef __DSPACE__
		printCopyrightNotice( );
		#endif
		#endif
	#endif

	/* consistency check */
	if ( _nV <= 0 )
	{
		_nV = 1;
		THROWERROR( RET_INVALID_ARGUMENTS );
	}

	/* reset global message handler */
	getGlobalMessageHandler( )->reset( );

	H = new double[_nV*_nV];
	g = new double[_nV];
	lb = new double[_nV];
	ub = new double[_nV];

	bounds = new Bounds( _nV );

	R = new double[_nV*_nV];

	x = new double[_nV];
	y = new double[_nV];

	tau = 0.0;

	eps = 0.0;
	nRegSteps = MAX_NUMBER_OF_REGULARISATION_STEPS;

	hessianType = HST_UNKNOWN;
	infeasible = BT_FALSE;
	unbounded = BT_FALSE;

	status = QPS_NOTINITIALISED;

	printLevel =   PL_MEDIUM;
	setPrintLevel( PL_MEDIUM );
	#ifdef __DEBUG__
	printLevel =   PL_HIGH;
	setPrintLevel( PL_HIGH );
	#endif
	#ifdef __XPCTARGET__
	printLevel =   PL_NONE;
	setPrintLevel( PL_NONE );
	#endif

	count = 0;
}


/*
 *	Q P r o b l e m B
 */
QProblemB::QProblemB( int _nV, HessianType _hessianType )
{
	/* print copyright notice */
	#ifndef __SUPPRESSANYOUTPUT__
		#ifndef __XPCTARGET__
		#ifndef __DSPACE__
		printCopyrightNotice( );
		#endif
		#endif
	#endif

	/* consistency check */
	if ( _nV <= 0 )
	{
		_nV = 1;
		THROWERROR( RET_INVALID_ARGUMENTS );
	}

	/* reset global message handler */
	getGlobalMessageHandler( )->reset( );

	if ( ( _hessianType == HST_ZERO ) || ( _hessianType == HST_IDENTITY ) )
		H = 0; /* do NOT allocate memory for trivial Hessians! */
	else
		H = new double[_nV*_nV];

	g = new double[_nV];
	lb = new double[_nV];
	ub = new double[_nV];

	bounds = new Bounds( _nV );

	R = new double[_nV*_nV];

	x = new double[_nV];
	y = new double[_nV];

	tau = 0.0;

	eps = 0.0;
	nRegSteps = MAX_NUMBER_OF_REGULARISATION_STEPS;

	hessianType = _hessianType;
	infeasible = BT_FALSE;
	unbounded = BT_FALSE;

	status = QPS_NOTINITIALISED;

	printLevel =   PL_MEDIUM;
	setPrintLevel( PL_MEDIUM );
	#ifdef __DEBUG__
	printLevel =   PL_HIGH;
	setPrintLevel( PL_HIGH );
	#endif
	#ifdef __XPCTARGET__
	printLevel =   PL_NONE;
	setPrintLevel( PL_NONE );
	#endif

	count = 0;
}


/*
 *	Q P r o b l e m B
 */
QProblemB::QProblemB( const QProblemB& rhs )
{
	int i;

	int _nV = rhs.getNV( );

	if ( rhs.bounds != 0 )
		bounds = new Bounds( *(rhs.bounds) );
	else
		bounds = 0;

	if ( rhs.H != 0 )
	{
		H = new double[_nV*_nV];
		setH( rhs.H );
	}
	else
		H = 0;

	if ( rhs.g != 0 )
	{
		g = new double[_nV];
		setG( rhs.g );
	}
	else
		g = 0;

	if ( rhs.lb != 0 )
	{
		lb = new double[_nV];
		setLB( rhs.lb );
	}
	else
		lb = 0;

	if ( rhs.ub != 0 )
	{
		ub = new double[_nV];
		setUB( rhs.ub );
	}
	else
		ub = 0;

	if ( rhs.R != 0 )
	{
		R = new double[_nV*_nV];
		for( i=0; i<_nV*_nV; ++i )
			R[i] = rhs.R[i];
	}
	else
		R = 0;

	if ( rhs.x != 0 )
	{
		x = new double[_nV];
		for( i=0; i<_nV; ++i )
			x[i] = rhs.x[i];
	}
	else
		x = 0;

	if ( rhs.y != 0 )
	{
		y = new double[_nV];
		for( i=0; i<_nV; ++i )
			y[i] = rhs.y[i];
	}
	else
		y = 0;

	tau = rhs.tau;

	eps = rhs.eps;
	nRegSteps = rhs.nRegSteps;

	hessianType = rhs.hessianType;
	infeasible = rhs.infeasible;
	unbounded = rhs.unbounded;

	status = rhs.status;

	printLevel = rhs.printLevel;
	setPrintLevel( rhs.printLevel );

	count = rhs.count;
}


/*
 *	~ Q P r o b l e m B
 */
QProblemB::~QProblemB( )
{
	if ( H != 0 )
		delete[] H;

	if ( g != 0 )
		delete[] g;

	if ( lb != 0 )
		delete[] lb;

	if ( ub != 0 )
		delete[] ub;

	if ( bounds != 0 )
		delete bounds;

	if ( R != 0 )
		delete[] R;

	if ( x != 0 )
		delete[] x;

	if ( y != 0 )
		delete[] y;

	/* reset global message handler */
	getGlobalMessageHandler( )->reset( );
}


/*
 *	o p e r a t o r =
 */
QProblemB& QProblemB::operator=( const QProblemB& rhs )
{
	int i;

	if ( this != &rhs )
	{
		if ( H != 0 )
			delete[] H;

		if ( g != 0 )
			delete[] g;

		if ( lb != 0 )
			delete[] lb;

		if ( ub != 0 )
			delete[] ub;

		if ( bounds != 0 )
			delete bounds;

		if ( R != 0 )
			delete[] R;

		if ( x != 0 )
			delete[] x;

		if ( y != 0 )
			delete[] y;


		int _nV = rhs.getNV( );

		if ( rhs.bounds != 0 )
			bounds = new Bounds( *(rhs.bounds) );
		else
			bounds = 0;

		if ( rhs.H != 0 )
		{
			H = new double[_nV*_nV];
			setH( rhs.H );
		}
		else
			H = 0;

		if ( rhs.g != 0 )
		{
			g = new double[_nV];
			setG( rhs.g );
		}
		else
			g = 0;

		if ( rhs.lb != 0 )
		{
			lb = new double[_nV];
			setLB( rhs.lb );
		}
		else
			lb = 0;

		if ( rhs.ub != 0 )
		{
			ub = new double[_nV];
			setUB( rhs.ub );
		}
		else
			ub = 0;

		if ( rhs.R != 0 )
		{
			R = new double[_nV*_nV];
			for( i=0; i<_nV*_nV; ++i )
				R[i] = rhs.R[i];
		}
		else
			R = 0;

		if ( rhs.x != 0 )
		{
			x = new double[_nV];
			for( i=0; i<_nV; ++i )
				x[i] = rhs.x[i];
		}
		else
			x = 0;

		if ( rhs.y != 0 )
		{
			y = new double[_nV];
			for( i=0; i<_nV; ++i )
				y[i] = rhs.y[i];
		}
		else
			y = 0;

		tau = rhs.tau;

		eps = rhs.eps;
		nRegSteps = rhs.nRegSteps;

		hessianType = rhs.hessianType;
		infeasible = rhs.infeasible;
		unbounded = rhs.unbounded;

		status = rhs.status;

		printLevel = rhs.printLevel;
		setPrintLevel( rhs.printLevel );

		count = rhs.count;
	}

	return *this;
}


/*
 *	r e s e t
 */
returnValue QProblemB::reset( )
{
	int i;
	int nV = getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* 1) Reset bounds. */
	if ( bounds != 0 )
		delete bounds;
	bounds = new Bounds( nV );

	/* 2) Reset Cholesky decomposition. */
	for( i=0; i<nV*nV; ++i )
		R[i] = 0.0;

	/* 3) Reset steplength and status flags. */
	tau = 0.0;

	eps = 0.0;
	nRegSteps = MAX_NUMBER_OF_REGULARISATION_STEPS;

	hessianType = HST_UNKNOWN;
	infeasible = BT_FALSE;
	unbounded = BT_FALSE;

	status = QPS_NOTINITIALISED;

	return SUCCESSFUL_RETURN;
}


/*
 *	i n i t
 */
returnValue QProblemB::init(	const double* const _H, const double* const _g,
								const double* const _lb, const double* const _ub,
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
	if ( setupQPdata( _H,_g,_lb,_ub ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 3) Call to main initialisation routine (without any additional information). */
	return solveInitialQP( 0,0,0, nWSR,cputime );
}


/*
 *	i n i t
 */
returnValue QProblemB::init(	const char* const H_file, const char* const g_file,
								const char* const lb_file, const char* const ub_file,
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
	if ( setupQPdataFromFile( H_file,g_file,lb_file,ub_file ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_UNABLE_TO_READ_FILE );

	/* 3) Call to main initialisation routine (without any additional information). */
	return solveInitialQP( 0,0,0, nWSR,cputime );
}


/*
 *	i n i t
 */
returnValue QProblemB::init( 	const double* const _H, const double* const _g,
								const double* const _lb, const double* const _ub,
								int& nWSR, double* const cputime,
								const double* const xOpt, const double* const yOpt,
								const Bounds* const guessedBounds
								)
{
	int i;
	int nV = getNV( );

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

	/* exclude this possibility in order to avoid inconsistencies */
	if ( ( xOpt == 0 ) && ( yOpt != 0 ) && ( guessedBounds != 0 ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 2) Setup QP data. */
	if ( setupQPdata( _H,_g,_lb,_ub ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 3) Call to main initialisation routine. */
	return solveInitialQP( xOpt,yOpt,guessedBounds, nWSR,cputime );
}


/*
 *	i n i t
 */
returnValue QProblemB::init( 	const char* const H_file, const char* const g_file,
								const char* const lb_file, const char* const ub_file,
								int& nWSR, double* const cputime,
								const double* const xOpt, const double* const yOpt,
								const Bounds* const guessedBounds
								)
{
	int i;
	int nV = getNV( );

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

	/* exclude this possibility in order to avoid inconsistencies */
	if ( ( xOpt == 0 ) && ( yOpt != 0 ) && ( guessedBounds != 0 ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* 2) Setup QP data from files. */
	if ( setupQPdataFromFile( H_file,g_file,lb_file,ub_file ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_UNABLE_TO_READ_FILE );

	/* 3) Call to main initialisation routine. */
	return solveInitialQP( xOpt,yOpt,guessedBounds, nWSR,cputime );
}


/*
 *	h o t s t a r t
 */
returnValue QProblemB::hotstart(	const double* const g_new,
									const double* const lb_new, const double* const ub_new,
									int& nWSR, double* const cputime
									)
{
	if ( getNV( ) == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	++count;

	if ( usingRegularisation( ) == BT_TRUE )
		return solveRegularisedQP( g_new,lb_new,ub_new, nWSR,cputime );
	else
		return solveQP(            g_new,lb_new,ub_new, nWSR,cputime,0 );
}


/*
 *	h o t s t a r t
 */
returnValue QProblemB::hotstart(	const char* const g_file,
									const char* const lb_file, const char* const ub_file,
									int& nWSR, double* const cputime
									)
{
	int nV  = getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* consistency check */
	if ( g_file == 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	/* 1) Allocate memory (if bounds exist). */
	double* g_new  = new double[nV];
	double* lb_new = 0;
	double* ub_new = 0;

	if ( lb_file != 0 )
		lb_new = new double[nV];
	if ( ub_file != 0 )
		ub_new = new double[nV];

	/* 2) Load new QP vectors from file. */
	returnValue returnvalue;
	returnvalue = loadQPvectorsFromFile(	g_file,lb_file,ub_file,
											g_new,lb_new,ub_new
											);
	if ( returnvalue != SUCCESSFUL_RETURN )
	{
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
		returnvalue = solveRegularisedQP( g_new,lb_new,ub_new, nWSR,cputime );
	else
		returnvalue = solveQP(            g_new,lb_new,ub_new, nWSR,cputime,0 );

	/* 4) Free memory. */
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
returnValue QProblemB::hotstart(	const double* const g_new,
									const double* const lb_new, const double* const ub_new,
									int& nWSR, double* const cputime,
									const Bounds* const guessedBounds
									)
{
	int nV = getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );


	/* start runtime measurement */
	double starttime = 0.0;
	if ( cputime != 0 )
		starttime = getCPUtime( );


	/* 1) Update working set according to guess for working set of bounds. */
	if ( guessedBounds != 0 )
	{
		if ( setupAuxiliaryQP( guessedBounds ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}
	else
	{
		/* create empty bounds for setting up auxiliary QP */
		Bounds emptyBounds( nV );
		if ( emptyBounds.setupAllFree( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		if ( setupAuxiliaryQP( &emptyBounds ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	/* 2) Perform usual homotopy. */

	/* Allow only remaining CPU time for usual hotstart. */
	if ( cputime != 0 )
		*cputime -= getCPUtime( ) - starttime;


	returnValue returnvalue;

	++count;

	if ( usingRegularisation( ) == BT_TRUE )
		returnvalue = solveRegularisedQP( g_new,lb_new,ub_new, nWSR,cputime );
	else
		returnvalue = solveQP(            g_new,lb_new,ub_new, nWSR,cputime,0 );


	/* stop runtime measurement */
	if ( cputime != 0 )
		*cputime = getCPUtime( ) - starttime;

	return returnvalue;
}


/*
 *	h o t s t a r t
 */
returnValue QProblemB::hotstart(	const char* const g_file,
									const char* const lb_file, const char* const ub_file,
									int& nWSR, double* const cputime,
									const Bounds* const guessedBounds
									)
{
	int nV = getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	/* consistency check */
	if ( g_file == 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	/* 1) Allocate memory (if bounds exist). */
	double* g_new  = new double[nV];
	double* lb_new = 0;
	double* ub_new = 0;

	if ( lb_file != 0 )
		lb_new = new double[nV];
	if ( ub_file != 0 )
		ub_new = new double[nV];

	/* 2) Load new QP vectors from file. */
	returnValue returnvalue;
	returnvalue = loadQPvectorsFromFile(	g_file,lb_file,ub_file,
											g_new,lb_new,ub_new
											);
	if ( returnvalue != SUCCESSFUL_RETURN )
	{
		if ( ub_file != 0 )
			delete[] ub_new;
		if ( lb_file != 0 )
			delete[] lb_new;
		delete[] g_new;

		return THROWERROR( RET_UNABLE_TO_READ_FILE );
	}

	/* 3) Actually perform hotstart using initialised homotopy. */
	returnvalue = hotstart(	g_new,lb_new,ub_new, nWSR,cputime,
							guessedBounds
							);

	/* 4) Free memory. */
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
int QProblemB::getNZ( ) const
{
	/* if no constraints are present: nZ=nFR */
	return getNFR( );
}


/*
 *	g e t O b j V a l
 */
double QProblemB::getObjVal( ) const
{
	double objVal;

	/* calculated optimal objective function value
	 * only if current QP has been solved */
	if ( ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
		 ( getStatus( ) == QPS_SOLVED ) )
	{
		objVal = getObjVal( x );
	}
	else
	{
		objVal = INFTY;
	}

	return objVal;
}


/*
 *	g e t O b j V a l
 */
double QProblemB::getObjVal( const double* const _x ) const
{
	int i, j;
	int nV = getNV( );

	if ( nV == 0 )
		return 0.0;

	double objVal = 0.0;

	for( i=0; i<nV; ++i )
		objVal += _x[i]*g[i];

	switch ( hessianType )
	{
		case HST_ZERO:
			for( i=0; i<nV; ++i )
				objVal += 0.5*_x[i]*eps*_x[i];
			break;

		case HST_IDENTITY:
			for( i=0; i<nV; ++i )
				objVal += 0.5*_x[i]*_x[i];
			break;

		default:
			for( i=0; i<nV; ++i )
				for( j=0; j<nV; ++j )
					objVal += 0.5*_x[i]*H[i*nV + j]*_x[j];
			break;
	}

	/* When using regularisation, the objective function value
	 * needs to be modified as follows:
	 * objVal = objVal - 0.5*_x*(Hmod-H)*_x - _x'*(gMod-g)
	 *        = objVal - 0.5*_x*eps*_x * - _x'*(-eps*_x)
	 *        = objVal + 0.5*_x*eps*_x */
	if ( usingRegularisation( ) == BT_TRUE )
	{
		for( i=0; i<nV; ++i )
			objVal += 0.5*_x[i]*eps*_x[i];
	}

	return objVal;
}


/*
 *	g e t P r i m a l S o l u t i o n
 */
returnValue QProblemB::getPrimalSolution( double* const xOpt ) const
{
	int i;

	/* return optimal primal solution vector
	 * only if current QP has been solved */
	if ( ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
		 ( getStatus( ) == QPS_SOLVED ) )
	{
		for( i=0; i<getNV( ); ++i )
			xOpt[i] = x[i];

		return SUCCESSFUL_RETURN;
	}
	else
	{
		return RET_QP_NOT_SOLVED;
	}
}


/*
 *	g e t D u a l S o l u t i o n
 */
returnValue QProblemB::getDualSolution( double* const yOpt ) const
{
	int i;

	/* return optimal dual solution vector
	 * only if current QP has been solved */
	if ( ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
		 ( getStatus( ) == QPS_SOLVED ) )
	{
		for( i=0; i<getNV( ); ++i )
			yOpt[i] = y[i];

		return SUCCESSFUL_RETURN;
	}
	else
	{
		return RET_QP_NOT_SOLVED;
	}
}


/*
 *	s e t P r i n t L e v e l
 */
returnValue QProblemB::setPrintLevel( PrintLevel _printLevel )
{
	#ifndef __MATLAB__
	if ( ( printLevel == PL_HIGH ) && ( printLevel != _printLevel ) )
		THROWINFO( RET_PRINTLEVEL_CHANGED );
	#endif

	printLevel = _printLevel;

	/* update message handler preferences */
 	switch ( printLevel )
 	{
 		case PL_NONE:
 			getGlobalMessageHandler( )->setErrorVisibilityStatus( VS_HIDDEN );
			getGlobalMessageHandler( )->setWarningVisibilityStatus( VS_HIDDEN );
			getGlobalMessageHandler( )->setInfoVisibilityStatus( VS_HIDDEN );
			break;

		case PL_LOW:
			#ifndef __XPCTARGET__
			getGlobalMessageHandler( )->setErrorVisibilityStatus( VS_VISIBLE );
			getGlobalMessageHandler( )->setWarningVisibilityStatus( VS_HIDDEN );
			getGlobalMessageHandler( )->setInfoVisibilityStatus( VS_HIDDEN );
			#endif
			break;

		case PL_MEDIUM:
			#ifndef __XPCTARGET__
			getGlobalMessageHandler( )->setErrorVisibilityStatus( VS_VISIBLE );
			getGlobalMessageHandler( )->setWarningVisibilityStatus( VS_VISIBLE );
			getGlobalMessageHandler( )->setInfoVisibilityStatus( VS_HIDDEN );
			#endif
			break;

		default: /* PL_HIGH */
			#ifndef __XPCTARGET__
			getGlobalMessageHandler( )->setErrorVisibilityStatus( VS_VISIBLE );
			getGlobalMessageHandler( )->setWarningVisibilityStatus( VS_VISIBLE );
			getGlobalMessageHandler( )->setInfoVisibilityStatus( VS_VISIBLE );
			#endif
			break;
 	}

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t P r o p e r t i e s
 */
returnValue QProblemB::printProperties( )
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


	/* 2) Further properties. */
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


	/* 3) QP object properties. */
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
 *	d e t e r m i n e H e s s i a n T y p e
 */
returnValue QProblemB::determineHessianType( )
{
	int i, j;
	int nV = getNV( );

	/* if Hessian type has been set by user, do NOT change it! */
	if ( hessianType != HST_UNKNOWN )
		return SUCCESSFUL_RETURN;

	/* if Hessian has not been allocated, assume it to be all zeros! */
	if ( H == 0 )
	{
		hessianType = HST_ZERO;
		return SUCCESSFUL_RETURN;
	}


	/* 1) If Hessian has outer-diagonal elements,
	 *    Hessian is assumed to be positive definite. */
	hessianType = HST_POSDEF;
	for ( i=0; i<nV; ++i )
		for ( j=0; j<i; ++j )
			if ( ( fabs( H[i*nV + j] ) > EPS ) || ( fabs( H[j*nV + i] ) > EPS ) )
				return SUCCESSFUL_RETURN;

	/* 2) Otherwise it is diagonal and test for identity or zero matrix is performed. */
	//hessianType = HST_DIAGONAL;

	BooleanType isIdentity = BT_TRUE;
	BooleanType isZero = BT_TRUE;

	for ( i=0; i<nV; ++i )
	{
		if ( H[i*nV + i] < -ZERO )
			return THROWERROR( RET_HESSIAN_INDEFINITE );

		if ( fabs( H[i*nV + i] - 1.0 ) > EPS )
			isIdentity = BT_FALSE;

		if ( fabs( H[i*nV + i] ) > EPS )
			isZero = BT_FALSE;
	}

	if ( isIdentity == BT_TRUE )
		hessianType = HST_IDENTITY;

	if ( isZero == BT_TRUE )
		hessianType = HST_ZERO;

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p S u b j e c t T o T y p e
 */
returnValue QProblemB::setupSubjectToType( )
{
	return setupSubjectToType( lb,ub );
}


/*
 *	s e t u p S u b j e c t T o T y p e
 */
returnValue QProblemB::setupSubjectToType( const double* const lb_new, const double* const ub_new )
{
	int i;
	int nV = getNV( );


	/* 1) Check if lower bounds are present. */
	bounds->setNoLower( BT_TRUE );
	if ( lb_new != 0 )
	{
		for( i=0; i<nV; ++i )
		{
			if ( lb_new[i] > -INFTY )
			{
				bounds->setNoLower( BT_FALSE );
				break;
			}
		}
	}

	/* 2) Check if upper bounds are present. */
	bounds->setNoUpper( BT_TRUE );
	if ( ub_new != 0 )
	{
		for( i=0; i<nV; ++i )
		{
			if ( ub_new[i] < INFTY )
			{
				bounds->setNoUpper( BT_FALSE );
				break;
			}
		}
	}

	/* 3) Determine implicitly fixed and unbounded variables. */
	if ( ( lb_new != 0 ) && ( ub_new != 0 ) )
	{
		for( i=0; i<nV; ++i )
		{
			if ( ( lb_new[i] < -INFTY + BOUNDTOL ) && ( ub_new[i] > INFTY - BOUNDTOL ) )
			{
				bounds->setType( i,ST_UNBOUNDED );
			}
			else
			{
				if ( lb_new[i] > ub_new[i] - BOUNDTOL )
					bounds->setType( i,ST_EQUALITY );
				else
					bounds->setType( i,ST_BOUNDED );
			}
		}
	}
	else
	{
		if ( ( lb_new == 0 ) && ( ub_new == 0 ) )
		{
			for( i=0; i<nV; ++i )
				bounds->setType( i,ST_UNBOUNDED );
		}
		else
		{
			for( i=0; i<nV; ++i )
				bounds->setType( i,ST_BOUNDED );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p C h o l e s k y D e c o m p o s i t i o n
 */
returnValue QProblemB::setupCholeskyDecomposition( )
{
	int i, j, k, ii, jj;
	int nV  = getNV( );
	int nFR = getNFR( );

	/* 1) Initialises R with all zeros. */
	for( i=0; i<nV*nV; ++i )
		R[i] = 0.0;

	/* 2) Calculate Cholesky decomposition of H (projected to free variables). */
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
		if ( nFR > 0 )
		{
			int* FR_idx = new int[nFR];
			if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
			{
				delete[] FR_idx;
				return THROWERROR( RET_INDEXLIST_CORRUPTED );
			}

			/* R'*R = H */
			double sum;

			for( i=0; i<nFR; ++i )
			{
				/* j == i */
				ii = FR_idx[i];
				sum = H[ii*nV + ii];

				for( k=(i-1); k>=0; --k )
					sum -= R[k*nV + i] * R[k*nV + i];

				if ( sum > 0.0 )
					R[i*nV + i] = sqrt( sum );
				else
				{
					delete[] FR_idx;

					hessianType = HST_SEMIDEF;
					return RET_HESSIAN_NOT_SPD;
				}

				/* j > i */
				for( j=(i+1); j<nFR; ++j )
				{
					jj = FR_idx[j];
					sum = H[jj*nV + ii];

					for( k=(i-1); k>=0; --k )
						sum -= R[k*nV + i] * R[k*nV + j];

					R[i*nV + j] = sum / R[i*nV + i];
				}
			}

			delete[] FR_idx;
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	o b t a i n A u x i l i a r y W o r k i n g S e t
 */
returnValue QProblemB::obtainAuxiliaryWorkingSet(	const double* const xOpt, const double* const yOpt,
													const Bounds* const guessedBounds, Bounds* auxiliaryBounds
													) const
{
	int i = 0;
	int nV = getNV( );


	/* 1) Ensure that desiredBounds is allocated (and different from guessedBounds). */
	if ( ( auxiliaryBounds == 0 ) || ( auxiliaryBounds == guessedBounds ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	/* 2) Setup working set for auxiliary initial QP. */
	if ( guessedBounds != 0 )
	{
		/* If an initial working set is specific, use it!
		 * Moreover, add all implictly fixed variables if specified. */
		for( i=0; i<nV; ++i )
		{
			#ifdef __ALWAYS_INITIALISE_WITH_ALL_EQUALITIES__
			if ( bounds->getType( i ) == ST_EQUALITY )
			{
				if ( auxiliaryBounds->setupBound( i,ST_LOWER ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
			}
			else
			#endif
			{
				if ( auxiliaryBounds->setupBound( i,guessedBounds->getStatus( i ) ) != SUCCESSFUL_RETURN )
					return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
			}
		}
	}
	else	/* No initial working set specified. */
	{
		if ( ( xOpt != 0 ) && ( yOpt == 0 ) )
		{
			/* Obtain initial working set by "clipping". */
			for( i=0; i<nV; ++i )
			{
				if ( xOpt[i] <= lb[i] + BOUNDTOL )
				{
					if ( auxiliaryBounds->setupBound( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
					continue;
				}

				if ( xOpt[i] >= ub[i] - BOUNDTOL )
				{
					if ( auxiliaryBounds->setupBound( i,ST_UPPER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
					continue;
				}

				/* Moreover, add all implictly fixed variables if specified. */
				#ifdef __ALWAYS_INITIALISE_WITH_ALL_EQUALITIES__
				if ( bounds->getType( i ) == ST_EQUALITY )
				{
					if ( auxiliaryBounds->setupBound( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
				else
				#endif
				{
					if ( auxiliaryBounds->setupBound( i,ST_INACTIVE ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
			}
		}

		if ( ( xOpt == 0 ) && ( yOpt != 0 ) )
		{
			/* Obtain initial working set in accordance to sign of dual solution vector. */
			for( i=0; i<nV; ++i )
			{
				if ( yOpt[i] > ZERO )
				{
					if ( auxiliaryBounds->setupBound( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
					continue;
				}

				if ( yOpt[i] < -ZERO )
				{
					if ( auxiliaryBounds->setupBound( i,ST_UPPER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
					continue;
				}

				/* Moreover, add all implictly fixed variables if specified. */
				#ifdef __ALWAYS_INITIALISE_WITH_ALL_EQUALITIES__
				if ( bounds->getType( i ) == ST_EQUALITY )
				{
					if ( auxiliaryBounds->setupBound( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
				else
				#endif
				{
					if ( auxiliaryBounds->setupBound( i,ST_INACTIVE ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
			}
		}

		/* If xOpt and yOpt are null pointer and no initial working is specified,
		 * start with empty working set (or implicitly fixed bounds only)
		 * for auxiliary QP. */
		if ( ( xOpt == 0 ) && ( yOpt == 0 ) )
		{
			for( i=0; i<nV; ++i )
			{
				/* Only add all implictly fixed variables if specified. */
				#ifdef __ALWAYS_INITIALISE_WITH_ALL_EQUALITIES__
				if ( bounds->getType( i ) == ST_EQUALITY )
				{
					if ( auxiliaryBounds->setupBound( i,ST_LOWER ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
				else
				#endif
				{
					if ( auxiliaryBounds->setupBound( i,ST_INACTIVE ) != SUCCESSFUL_RETURN )
						return THROWERROR( RET_OBTAINING_WORKINGSET_FAILED );
				}
			}
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	b a c k s o l v e R
 */
returnValue QProblemB::backsolveR(	const double* const b, BooleanType transposed,
									double* const a
									) const
{
	/* Call standard backsolve procedure (i.e. removingBound == BT_FALSE). */
	return backsolveR( b,transposed,BT_FALSE,a );
}


/*
 *	b a c k s o l v e R
 */
returnValue QProblemB::backsolveR(	const double* const b, BooleanType transposed,
									BooleanType removingBound,
									double* const a
									) const
{
	int i, j;
	int nV = getNV( );
	int nR = getNZ( );

	double sum;

	/* if backsolve is called while removing a bound, reduce nZ by one. */
	if ( removingBound == BT_TRUE )
		--nR;

	/* nothing to do */
	if ( nR <= 0 )
		return SUCCESSFUL_RETURN;


	/* Solve Ra = b, where R might be transposed. */
	if ( transposed == BT_FALSE )
	{
		/* solve Ra = b */
		for( i=(nR-1); i>=0; --i )
		{
			sum = b[i];
			for( j=(i+1); j<nR; ++j )
				sum -= R[i*nV + j] * a[j];

			if ( fabs( R[i*nV + i] ) >= ZERO*fabs( sum ) )
				a[i] = sum / R[i*nV + i];
			else
				return THROWERROR( RET_DIV_BY_ZERO );
		}
	}
	else
	{
		/* solve R^T*a = b */
		for( i=0; i<nR; ++i )
		{
			sum = b[i];

			for( j=0; j<i; ++j )
				sum -= R[j*nV + i] * a[j];

			if ( fabs( R[i*nV + i] ) >= ZERO*fabs( sum ) )
				a[i] = sum / R[i*nV + i];
			else
				return THROWERROR( RET_DIV_BY_ZERO );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	d e t e r m i n e D a t a S h i f t
 */
returnValue QProblemB::determineDataShift(	const int* const FX_idx,
											const double* const g_new, const double* const lb_new, const double* const ub_new,
											double* const delta_g, double* const delta_lb, double* const delta_ub,
											BooleanType& Delta_bB_isZero
											) const
{
	int i, ii;
	int nV  = getNV( );
	int nFX = getNFX( );


	/* 1) Calculate shift directions. */
	for( i=0; i<nV; ++i )
		delta_g[i]  = g_new[i]  - g[i];

	if ( lb_new != 0 )
	{
		for( i=0; i<nV; ++i )
			delta_lb[i] = lb_new[i] - lb[i];
	}
	else
	{
		/* if no lower bounds exist, assume the new lower bounds to be -infinity */
		for( i=0; i<nV; ++i )
			delta_lb[i] = -INFTY - lb[i];
	}

	if ( ub_new != 0 )
	{
		for( i=0; i<nV; ++i )
			delta_ub[i] = ub_new[i] - ub[i];
	}
	else
	{
		/* if no upper bounds exist, assume the new upper bounds to be infinity */
		for( i=0; i<nV; ++i )
			delta_ub[i] = INFTY - ub[i];
	}

	/* 2) Determine if active bounds are to be shifted. */
	Delta_bB_isZero = BT_TRUE;

	for ( i=0; i<nFX; ++i )
	{
		ii = FX_idx[i];

		if ( ( fabs( delta_lb[ii] ) > EPS ) || ( fabs( delta_ub[ii] ) > EPS ) )
		{
			Delta_bB_isZero = BT_FALSE;
			break;
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	a r e B o u n d s C o n s i s t e n t
 */
BooleanType QProblemB::areBoundsConsistent(	const double* const delta_lb, const double* const delta_ub
											) const
{
	int i;

	/* Check if delta_lb[i] is greater than delta_ub[i]
	 * for a component i whose bounds are already (numerically) equal. */
	for( i=0; i<getNV( ); ++i )
		if ( ( lb[i] > ub[i] - BOUNDTOL ) && ( delta_lb[i] > delta_ub[i] + EPS ) )
			return BT_FALSE;

	return BT_TRUE;
}


/*
 *	s e t u p Q P d a t a
 */
returnValue QProblemB::setupQPdata(	const double* const _H, const double* const _g,
									const double* const _lb, const double* const _ub
									)
{
	int i;
	int nV = getNV( );

	/* 1) Setup Hessian matrix. */
	if ( _H != 0 )
	{
		if ( H == 0 )
		{
			// former trivial Hessian is replaced by given one
			H = new double[nV*nV];
		}

		setH( _H );
	}
	else
	{
		if ( H != 0 )
			return THROWERROR( RET_NO_HESSIAN_SPECIFIED );
	}

	/* 2) Setup gradient vector. */
	if ( _g == 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );
	else
		setG( _g );

	/* 3) Setup lower bounds vector. */
	if ( _lb != 0 )
	{
		setLB( _lb );
	}
	else
	{
		/* if no lower bounds are specified, set them to -infinity */
		for( i=0; i<nV; ++i )
			lb[i] = -INFTY;
	}

	/* 4) Setup upper bounds vector. */
	if ( _ub != 0 )
	{
		setUB( _ub );
	}
	else
	{
		/* if no upper bounds are specified, set them to infinity */
		for( i=0; i<nV; ++i )
			ub[i] = INFTY;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p Q P d a t a F r o m F i l e
 */
returnValue QProblemB::setupQPdataFromFile(	const char* const H_file, const char* const g_file,
											const char* const lb_file, const char* const ub_file
											)
{
	int i;
	int nV = getNV( );

	returnValue returnvalue;


	/* 1) Load Hessian matrix from file. */
	if ( H_file != 0 )
	{
		if ( H == 0 )
		{
			// former trivial Hessian is replaced by given one
			H = new double[nV*nV];
		}

		returnvalue = readFromFile( H, nV,nV, H_file );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return THROWERROR( returnvalue );
	}
	else
	{
		if ( H != 0 )
			return THROWERROR( RET_NO_HESSIAN_SPECIFIED );
	}

	/* 2) Load gradient vector from file. */
	if ( g_file == 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	returnvalue = readFromFile( g, nV, g_file );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return THROWERROR( returnvalue );

	/* 3) Load lower bounds vector from file. */
	if ( lb_file != 0 )
	{
		returnvalue = readFromFile( lb, nV, lb_file );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return THROWERROR( returnvalue );
	}
	else
	{
		/* if no lower bounds are specified, set them to -infinity */
		for( i=0; i<nV; ++i )
			lb[i] = -INFTY;
	}

	/* 4) Load upper bounds vector from file. */
	if ( ub_file != 0 )
	{
		returnvalue = readFromFile( ub, nV, ub_file );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return THROWERROR( returnvalue );
	}
	else
	{
		/* if no upper bounds are specified, set them to infinity */
		for( i=0; i<nV; ++i )
			ub[i] = INFTY;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	l o a d Q P v e c t o r s F r o m F i l e
 */
returnValue QProblemB::loadQPvectorsFromFile(	const char* const g_file, const char* const lb_file, const char* const ub_file,
												double* const g_new, double* const lb_new, double* const ub_new
												) const
{
	int nV = getNV( );

	returnValue returnvalue;


	/* 1) Load gradient vector from file. */
	if ( ( g_file != 0 ) && ( g_new != 0 ) )
	{
		returnvalue = readFromFile( g_new, nV, g_file );
		if ( returnvalue != SUCCESSFUL_RETURN )
			return THROWERROR( returnvalue );
	}
	else
	{
		/* At least gradient vector needs to be specified! */
		return THROWERROR( RET_INVALID_ARGUMENTS );
	}

	/* 2) Load lower bounds vector from file. */
	if ( lb_file != 0 )
	{
		if ( lb_new != 0 )
		{
			returnvalue = readFromFile( lb_new, nV, lb_file );
			if ( returnvalue != SUCCESSFUL_RETURN )
				return THROWERROR( returnvalue );
		}
		else
		{
			/* If filename is given, storage must be provided! */
			return THROWERROR( RET_INVALID_ARGUMENTS );
		}
	}

	/* 3) Load upper bounds vector from file. */
	if ( ub_file != 0 )
	{
		if ( ub_new != 0 )
		{
			returnvalue = readFromFile( ub_new, nV, ub_file );
			if ( returnvalue != SUCCESSFUL_RETURN )
				return THROWERROR( returnvalue );
		}
		else
		{
			/* If filename is given, storage must be provided! */
			return THROWERROR( RET_INVALID_ARGUMENTS );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	i s C P U t i m e L i m i t E x c e e d e d
 */
BooleanType QProblemB::isCPUtimeLimitExceeded(	const double* const cputime,
												double starttime,
												int nWSR
												) const
{
	/* Always perform next QP iteration if no CPU time limit is given. */
	if ( cputime == 0 )
		return BT_FALSE;

	/* Always perform first QP iteration. */
	if ( nWSR <= 0 )
		return BT_FALSE;

	double elapsedTime = getCPUtime( ) - starttime;
	double timePerIteration = elapsedTime / ((double) nWSR);

	/* Determine if next QP iteration exceed CPU time limit
	 * considering the (current) average CPU time per iteration. */
	if ( ( elapsedTime + timePerIteration*1.25 ) <= ( *cputime ) )
		return BT_FALSE;
	else
		return BT_TRUE;
}


/*
 *	r e g u l a r i s e H e s s i a n
 */
returnValue QProblemB::regulariseHessian( )
{
	int i;
	int nV = getNV( );

	/* Regularisation of identity Hessian not possible. */
	if ( hessianType == HST_IDENTITY )
		return THROWERROR( RET_CANNOT_REGULARISE_IDENTITY );

	/* Determine regularisation parameter. */
	if ( usingRegularisation( ) == BT_TRUE )
		return THROWERROR( RET_HESSIAN_ALREADY_REGULARISED );
	else
		eps = determineEpsForRegularisation( );

	/* Regularisation of zero Hessian is done implicitly. */
	if ( hessianType == HST_ZERO )
		return SUCCESSFUL_RETURN;

	for( i=0; i<nV; ++i )
		H[i*nV + i] += eps;

	THROWINFO( RET_USING_REGULARISATION );

	return SUCCESSFUL_RETURN;
}



/*****************************************************************************
 *  P R I V A T E                                                            *
 *****************************************************************************/

/*
 *	s o l v e I n i t i a l Q P
 */
returnValue QProblemB::solveInitialQP(	const double* const xOpt, const double* const yOpt,
										const Bounds* const guessedBounds,
										int& nWSR, double* const cputime
										)
{
	int i;
	int nV = getNV( );


	/* start runtime measurement */
	double starttime = 0.0;
	if ( cputime != 0 )
		starttime = getCPUtime( );


	status = QPS_NOTINITIALISED;

	/* I) ANALYSE QP DATA: */
	/* 1) Check if Hessian happens to be the identity matrix. */
	if ( determineHessianType( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	/* 2) Setup type of bounds (i.e. unbounded, implicitly fixed etc.). */
	if ( setupSubjectToType( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	status = QPS_PREPARINGAUXILIARYQP;


	/* II) SETUP AUXILIARY QP WITH GIVEN OPTIMAL SOLUTION: */
	/* 1) Setup bounds data structure. */
	if ( bounds->setupAllFree( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	/* 2) Setup optimal primal/dual solution for auxiliary QP. */
	if ( setupAuxiliaryQPsolution( xOpt,yOpt ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_INIT_FAILED );

	/* 3) Obtain linear independent working set for auxiliary QP. */
	Bounds* auxiliaryBounds = new Bounds( nV );
	if ( obtainAuxiliaryWorkingSet( xOpt,yOpt,guessedBounds, auxiliaryBounds ) != SUCCESSFUL_RETURN )
	{
		delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED );
	}

	/* 4) Setup working set of auxiliary QP and setup cholesky decomposition. */
	/* a) Working set of auxiliary QP. */
	if ( setupAuxiliaryWorkingSet( auxiliaryBounds,BT_TRUE ) != SUCCESSFUL_RETURN )
	{
		delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED );
	}

	/* b) Regularise Hessian if necessary. */
	if ( ( hessianType == HST_ZERO ) || ( hessianType == HST_SEMIDEF ) )
	{
		if ( regulariseHessian( ) != SUCCESSFUL_RETURN )
		{
			delete auxiliaryBounds;
			return THROWERROR( RET_INIT_FAILED );
		}
	}

	/* c) Cholesky decomposition. */
	returnValue returnvalueCholesky = setupCholeskyDecomposition( );

	/* If Hessian is not positive definite, regularise and try again. */
	if ( returnvalueCholesky == RET_HESSIAN_NOT_SPD )
	{
		if ( regulariseHessian( ) != SUCCESSFUL_RETURN )
		{
			delete auxiliaryBounds;
			return THROWERROR( RET_INIT_FAILED );
		}

		if ( setupCholeskyDecomposition( ) != SUCCESSFUL_RETURN )
		{
			delete auxiliaryBounds;
			return THROWERROR( RET_INIT_FAILED_CHOLESKY );
		}
	}

	if ( returnvalueCholesky == RET_INDEXLIST_CORRUPTED )
	{
		delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED_CHOLESKY );
	}


	/* 5) Store original QP formulation... */
	double* g_original = new double[nV];
	double* lb_original = new double[nV];
	double* ub_original = new double[nV];

	for( i=0; i<nV; ++i )
	{
		g_original[i]  = g[i];
		lb_original[i] = lb[i];
		ub_original[i] = ub[i];
	}

	/* ... and setup QP data of an auxiliary QP having an optimal solution
	 * as specified by the user (or xOpt = yOpt = 0, by default). */
	if ( setupAuxiliaryQPgradient( ) != SUCCESSFUL_RETURN )
	{
		delete[] ub_original; delete[] lb_original; delete[] g_original;
		delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED );
	}

	if ( setupAuxiliaryQPbounds( BT_TRUE ) != SUCCESSFUL_RETURN )
	{
 		delete[] ub_original; delete[] lb_original; delete[] g_original;
		delete auxiliaryBounds;
		return THROWERROR( RET_INIT_FAILED );
	}

	delete auxiliaryBounds;
	status = QPS_AUXILIARYQPSOLVED;


	/* III) SOLVE ACTUAL INITIAL QP: */

	/* Allow only remaining CPU time for usual hotstart. */
	if ( cputime != 0 )
		*cputime -= getCPUtime( ) - starttime;

	/* Use hotstart method to find the solution of the original initial QP,... */
	returnValue returnvalue = hotstart( g_original,lb_original,ub_original, nWSR,cputime );

	/* ... deallocate memory,... */
	delete[] ub_original; delete[] lb_original; delete[] g_original;

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
returnValue QProblemB::solveQP(	const double* const g_new,
								const double* const lb_new, const double* const ub_new,
								int& nWSR, double* const cputime, int nWSRperformed
								)
{
	int iter;
	int nV  = getNV( );

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
	/* 1) Allocate delta vectors of gradient and bounds,
	 *    index arrays and step direction arrays. */
	int nFR, nFX;

	int *FR_idx = new int[nV];
	int *FX_idx = new int[nV];

	double* delta_xFR = new double[nV];
	double* delta_xFX = new double[nV];
	double* delta_yFX = new double[nV];

	double* delta_g  = new double[nV];
	double* delta_lb = new double[nV];
	double* delta_ub = new double[nV];

	returnValue returnvalue;
	BooleanType Delta_bB_isZero;

	int BC_idx;
	SubjectToStatus BC_status;

	char messageString[80];

	/* 2) Update type of bounds, e.g. a formerly implicitly fixed
	 *    variable might have become a normal one etc. */
	if ( setupSubjectToType( lb_new,ub_new ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_HOTSTART_FAILED );

	/* 3) Reset status flags. */
	infeasible = BT_FALSE;
	unbounded  = BT_FALSE;


	/* II) MAIN HOMOTOPY LOOP */
	for( iter=0; iter<nWSR; ++iter )
	{
		if ( isCPUtimeLimitExceeded( cputime,starttime,iter ) == BT_TRUE )
		{
			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			break;
		}

		status = QPS_PERFORMINGHOMOTOPY;

		#ifndef __XPCTARGET__
		snprintf( messageString,80,"%d ...",iter );
		getGlobalMessageHandler( )->throwInfo( RET_ITERATION_STARTED,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		#endif

		/* some more definitions */
		nFR = getNFR( );
		nFX = getNFX( );

		/* 1) Determine index arrays. */
		if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
		{
			delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_g;
			return THROWERROR( RET_HOTSTART_FAILED );
		}

		if ( bounds->getFixed( )->getNumberArray( FX_idx ) != SUCCESSFUL_RETURN )
		{
			delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_g;
			return THROWERROR( RET_HOTSTART_FAILED );
		}

		/* 2) Initialise shift direction of the gradient and the bounds. */
		returnvalue = determineDataShift(	FX_idx,
											g_new,lb_new,ub_new,
											delta_g,delta_lb,delta_ub,
											Delta_bB_isZero
											);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_SHIFT_DETERMINATION_FAILED );
			return returnvalue;
		}

		/* 3) Determination of step direction of X and Y. */
		returnvalue = determineStepDirection(	FR_idx,FX_idx,
												delta_g,delta_lb,delta_ub,
												Delta_bB_isZero,
												delta_xFX,delta_xFR,delta_yFX
												);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_STEPDIRECTION_DETERMINATION_FAILED );
			return returnvalue;
		}


		/* 4) Determination of step length TAU. */
		returnvalue = determineStepLength(	FR_idx,FX_idx,
											delta_lb,delta_ub,
											delta_xFR,delta_yFX,
											BC_idx,BC_status
											);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_STEPLENGTH_DETERMINATION_FAILED );
			return returnvalue;
		}

		/* 5) Realisation of the homotopy step. */
		returnvalue = performStep(	FR_idx,FX_idx,
									delta_g,delta_lb,delta_ub,
									delta_xFX,delta_xFR,delta_yFX,
									BC_idx,BC_status
									);

		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] FX_idx; delete[] FR_idx;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			/* Optimal solution found? */
			if ( returnvalue == RET_OPTIMAL_SOLUTION_FOUND )
			{
				status = QPS_SOLVED;

				THROWINFO( RET_OPTIMAL_SOLUTION_FOUND );

	 			if ( printIteration( nWSRperformed+iter,BC_idx,BC_status ) != SUCCESSFUL_RETURN )
					THROWERROR( RET_PRINT_ITERATION_FAILED ); /* do not pass this as return value! */

				return SUCCESSFUL_RETURN;
			}
			else
			{
				/* checks for infeasibility... */
				if ( infeasible == BT_TRUE )
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

		if ( printIteration( nWSRperformed+iter,BC_idx,BC_status ) != SUCCESSFUL_RETURN )
			THROWERROR( RET_PRINT_ITERATION_FAILED ); /* do not pass this as return value! */
	}

	delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
	delete[] FX_idx; delete[] FR_idx;
	delete[] delta_ub; delete[] delta_lb; delete[] delta_g;

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
returnValue QProblemB::solveRegularisedQP(	const double* const g_new,
											const double* const lb_new, const double* const ub_new,
											int& nWSR, double* const cputime
											)
{
	int i, step;
	int nV = getNV( );


	/* Stop here if QP has not been regularised (i.e. normal QP solution). */
	if ( usingRegularisation( ) == BT_FALSE )
		return solveQP( g_new,lb_new,ub_new, nWSR,cputime,0 );


	/* I) SOLVE USUAL REGULARISED QP */
	returnValue returnvalue;

	int totalNWSR = 0;
	int curNWSR   = 0;

	double totalCputime = 0.0;
	double curCputime   = 0.0;

	if ( cputime == 0 )
	{
		curNWSR = nWSR;
		returnvalue = solveQP( g_new,lb_new,ub_new, curNWSR,0,0 );
	}
	else
	{
		curNWSR = nWSR;
		curCputime = *cputime;
		returnvalue = solveQP( g_new,lb_new,ub_new, curNWSR,&curCputime,0 );
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
			returnvalue = solveQP( gMod,lb_new,ub_new, curNWSR,0,totalNWSR );
		}
		else
		{
			curNWSR = nWSR - totalNWSR;
			curCputime = *cputime - totalCputime;
			returnvalue = solveQP( gMod,lb_new,ub_new, curNWSR,&curCputime,totalNWSR );
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
 *	s e t u p A u x i l i a r y W o r k i n g S e t
 */
returnValue QProblemB::setupAuxiliaryWorkingSet( 	const Bounds* const auxiliaryBounds,
													BooleanType setupAfresh
													)
{
	int i;
	int nV = getNV( );

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


	/* I) SETUP CHOLESKY FLAG:
	 *    Cholesky decomposition shall only be updated if working set
	 *    shall be updated (i.e. NOT setup afresh!) */
	BooleanType updateCholesky;
	if ( setupAfresh == BT_TRUE )
		updateCholesky = BT_FALSE;
	else
		updateCholesky = BT_TRUE;


	/* II) REMOVE FORMERLY ACTIVE BOUNDS (IF NECESSARY): */
	if ( setupAfresh == BT_FALSE )
	{
		/* Remove all active bounds that shall be inactive AND
		*  all active bounds that are active at the wrong bound. */
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


	/* III) ADD NEWLY ACTIVE BOUNDS: */
	/*      Add all inactive bounds that shall be active AND
	 *      all formerly active bounds that have been active at the wrong bound. */
	for( i=0; i<nV; ++i )
	{
		if ( ( bounds->getStatus( i ) == ST_INACTIVE ) && ( auxiliaryBounds->getStatus( i ) != ST_INACTIVE ) )
		{
			if ( addBound( i,auxiliaryBounds->getStatus( i ),updateCholesky ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_WORKINGSET_FAILED );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A u x i l i a r y Q P s o l u t i o n
 */
returnValue QProblemB::setupAuxiliaryQPsolution(	const double* const xOpt, const double* const yOpt
													)
{
	int i;
	int nV = getNV( );


	/* Setup primal/dual solution vectors for auxiliary initial QP:
	 * if a null pointer is passed, a zero vector is assigned;
	 * old solution vector is kept if pointer to internal solution vector is passed. */
	if ( xOpt != 0 )
	{
		if ( xOpt != x )
			for( i=0; i<nV; ++i )
				x[i] = xOpt[i];
	}
	else
	{
		for( i=0; i<nV; ++i )
			x[i] = 0.0;
	}

	if ( yOpt != 0 )
	{
		if ( yOpt != y )
			for( i=0; i<nV; ++i )
				y[i] = yOpt[i];
	}
	else
	{
		for( i=0; i<nV; ++i )
			y[i] = 0.0;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A u x i l i a r y Q P g r a d i e n t
 */
returnValue QProblemB::setupAuxiliaryQPgradient( )
{
	int i, j;
	int nV = getNV( );

	/* Setup gradient vector: g = -H*x + y'*Id. */
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

				/* -H*x */
				for ( j=0; j<nV; ++j )
					g[i] -= H[i*nV + j] * x[j];
			}
			break;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A u x i l i a r y Q P b o u n d s
 */
returnValue QProblemB::setupAuxiliaryQPbounds( BooleanType useRelaxation )
{
	int i;
	int nV = getNV( );


	/* Setup bound vectors. */
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
						lb[i] = x[i] - BOUNDRELAXATION;
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

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A u x i l i a r y Q P
 */
returnValue QProblemB::setupAuxiliaryQP( const Bounds* const guessedBounds )
{
	int i;
	int nV = getNV( );

	/* nothing to do */
	if ( guessedBounds == bounds )
		return SUCCESSFUL_RETURN;

	status = QPS_PREPARINGAUXILIARYQP;


	/* I) SETUP WORKING SET ... */
	if ( shallRefactorise( guessedBounds ) == BT_TRUE )
	{
		/* ... WITH REFACTORISATION: */
		/* 1) Reset bounds ... */
		if ( bounds != 0 )
			delete bounds;
		bounds = new Bounds( nV );

		/*    ... and set them up afresh. */
		if ( setupSubjectToType( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		if ( bounds->setupAllFree( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		/* 2) Setup guessed working set afresh. */
		if ( setupAuxiliaryWorkingSet( guessedBounds,BT_TRUE ) != SUCCESSFUL_RETURN )
			THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		/* 3) Calculate Cholesky decomposition. */
		if ( setupCholeskyDecomposition( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}
	else
	{
		/* ... WITHOUT REFACTORISATION: */
		if ( setupAuxiliaryWorkingSet( guessedBounds,BT_FALSE ) != SUCCESSFUL_RETURN )
			THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}


	/* II) SETUP AUXILIARY QP DATA: */
	/* 1) Ensure that dual variable is zero for fixed bounds. */
	for ( i=0; i<nV; ++i )
		if ( bounds->getStatus( i ) != ST_INACTIVE )
			y[i] = 0.0;

	/* 2) Setup gradient and bound vectors. */
	if ( setupAuxiliaryQPgradient( ) != SUCCESSFUL_RETURN )
		THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

	if ( setupAuxiliaryQPbounds( BT_FALSE ) != SUCCESSFUL_RETURN )
		THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

	return SUCCESSFUL_RETURN;
}


/*
 *	d e t e r m i n e S t e p D i r e c t i o n
 */
returnValue QProblemB::determineStepDirection(	const int* const FR_idx, const int* const FX_idx,
												const double* const delta_g, const double* const delta_lb, const double* const delta_ub,
												BooleanType Delta_bB_isZero,
												double* const delta_xFX, double* const delta_xFR,
												double* const delta_yFX
												) const
{
	int i, j, ii, jj;
	int nV  = getNV( );
	int nFR = getNFR( );
	int nFX = getNFX( );


	/* initialise auxiliary vectors */
	double* HMX_delta_xFX = new double[nFR];
	for( i=0; i<nFR; ++i )
		HMX_delta_xFX[i] = 0.0;


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
		/* auxiliary variables */
		double* delta_xFRz_TMP = new double[nFR];
		double* delta_xFRz_RHS = new double[nFR];

		/* Determine delta_xFRz. */
		if ( ( hessianType == HST_ZERO ) || ( hessianType == HST_IDENTITY ) )
		{
			for( i=0; i<nFR; ++i )
				HMX_delta_xFX[i] = 0.0;
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
		}

		if ( Delta_bB_isZero == BT_TRUE )
		{
			for( j=0; j<nFR; ++j )
			{
				jj = FR_idx[j];
				delta_xFRz_RHS[j] = delta_g[jj];
			}
		}
		else
		{
			for( j=0; j<nFR; ++j )
			{
				jj = FR_idx[j];
				delta_xFRz_RHS[j] = delta_g[jj] + HMX_delta_xFX[j]; /* *ZFR */
			}
		}

		for( i=0; i<nFR; ++i )
			delta_xFR[i] = -delta_xFRz_RHS[i];

		if ( backsolveR( delta_xFR,BT_TRUE,delta_xFRz_TMP ) != SUCCESSFUL_RETURN )
		{
			delete[] delta_xFRz_RHS; delete[] delta_xFRz_TMP;
			delete[] HMX_delta_xFX;

			return THROWERROR( RET_STEPDIRECTION_FAILED_CHOLESKY );
		}

		if ( backsolveR( delta_xFRz_TMP,BT_FALSE,delta_xFR ) != SUCCESSFUL_RETURN )
		{
			delete[] delta_xFRz_RHS; delete[] delta_xFRz_TMP;
			delete[] HMX_delta_xFX;

			return THROWERROR( RET_STEPDIRECTION_FAILED_CHOLESKY );
		}
		delete[] delta_xFRz_RHS; delete[] delta_xFRz_TMP;
	}


	/* III) DETERMINE delta_yFX */
	if ( nFX > 0 )
	{
		for( i=0; i<nFX; ++i )
		{
			ii = FX_idx[i];

			if ( ( hessianType == HST_ZERO ) || ( hessianType == HST_IDENTITY ) )
			{
				if ( hessianType == HST_ZERO )
					delta_yFX[i] = eps*delta_xFX[i];
				else
					delta_yFX[i] = delta_xFX[i];
			}
			else
			{
				delta_yFX[i] = 0.0;

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

			delta_yFX[i] += delta_g[ii];
		}
	}

	delete[] HMX_delta_xFX;

	return SUCCESSFUL_RETURN;
}


/*
 *	d e t e r m i n e S t e p L e n g t h
 */
returnValue QProblemB::determineStepLength(	const int* const FR_idx, const int* const FX_idx,
											const double* const delta_lb, const double* const delta_ub,
											const double* const delta_xFR,
											const double* const delta_yFX,
											int& BC_idx, SubjectToStatus& BC_status
											)
{
	int i, ii;
	int nFR = getNFR( );
	int nFX = getNFX( );

	double tau_tmp;
	double tau_new = 1.0;

	BC_idx = 0;
	BC_status = ST_UNDEFINED;


	/* I) DETERMINE MAXIMUM DUAL STEPLENGTH, i.e. ensure that
	 *    active dual bounds remain valid (ignoring implicitly fixed variables): */
	for( i=0; i<nFX; ++i )
	{
		ii = FX_idx[i];

		if ( bounds->getType( ii ) != ST_EQUALITY )
		{
			if ( bounds->getStatus( ii ) == ST_LOWER )
			{
				/* 1) Active lower bounds. */
				if ( ( delta_yFX[i] < -ZERO ) && ( y[ii] >= 0.0 ) )
				{
					tau_tmp = y[ii] / ( -delta_yFX[i] );
					if ( tau_tmp < tau_new )
					{
						if ( tau_tmp >= 0.0 )
						{
							tau_new = tau_tmp;
							BC_idx = ii;
							BC_status = ST_INACTIVE;
						}
					}
				}
			}
			else
			{
				/* 2) Active upper bounds. */
				if ( ( delta_yFX[i] > ZERO ) && ( y[ii] <= 0.0 ) )
				{
					tau_tmp = y[ii] / ( -delta_yFX[i] );
					if ( tau_tmp < tau_new )
					{
						if ( tau_tmp >= 0.0 )
						{
							tau_new = tau_tmp;
							BC_idx = ii;
							BC_status = ST_INACTIVE;
						}
					}
				}
			}
		}
	}


	/* II) DETERMINE MAXIMUM PRIMAL STEPLENGTH, i.e. ensure that
	 *     inactive bounds remain valid (ignoring unbounded variables). */
	/* 1) Inactive lower bounds. */
	if ( bounds->isNoLower( ) == BT_FALSE )
	{
		for( i=0; i<nFR; ++i )
		{
			ii = FR_idx[i];

			if ( bounds->getType( ii ) != ST_UNBOUNDED )
			{
				if ( delta_lb[ii] > delta_xFR[i] )
				{
					if ( x[ii] > lb[ii] )
						tau_tmp = ( x[ii] - lb[ii] ) / ( delta_lb[ii] - delta_xFR[i] );
					else
						tau_tmp = 0.0;

					if ( tau_tmp < tau_new )
					{
						if ( tau_tmp >= 0.0 )
						{
							tau_new = tau_tmp;
							BC_idx = ii;
							BC_status = ST_LOWER;
						}
					}
				}
			}
		}
	}

	/* 2) Inactive upper bounds. */
	if ( bounds->isNoUpper( ) == BT_FALSE )
	{
		for( i=0; i<nFR; ++i )
		{
			ii = FR_idx[i];

			if ( bounds->getType( ii ) != ST_UNBOUNDED )
			{
				if ( delta_ub[ii] < delta_xFR[i] )
				{
					if ( x[ii] < ub[ii] )
						tau_tmp = ( x[ii] - ub[ii] ) / ( delta_ub[ii] - delta_xFR[i] );
					else
						tau_tmp = 0.0;

					if ( tau_tmp < tau_new )
					{
						if ( tau_tmp >= 0.0 )
						{
							tau_new = tau_tmp;
							BC_idx = ii;
							BC_status = ST_UPPER;
						}
					}
				}
			}
		}
	}


	/* III) SET MAXIMUM HOMOTOPY STEPLENGTH */
	tau = tau_new;

	/* Optimal solution found as stepsize is numerically one! */
	if ( tau > 1.0 - 100.0*EPS )
	{
		tau = 1.0;
		BC_idx = -1;
		BC_status = ST_UNDEFINED;
	}

	#ifndef __XPCTARGET__
	char messageString[80];

	if ( BC_status == ST_UNDEFINED )
		snprintf( messageString,80,"Stepsize is %.10e!",tau );
	else
		snprintf( messageString,80,"Stepsize is %.10e! (BC_idx = %d, BC_status = %d)",tau,BC_idx,BC_status );

	getGlobalMessageHandler( )->throwInfo( RET_STEPSIZE_NONPOSITIVE,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	p e r f o r m S t e p
 */
returnValue QProblemB::performStep(	const int* const FR_idx, const int* const FX_idx,
									const double* const delta_g, const double* const  delta_lb, const double* const delta_ub,
									const double* const delta_xFX, const double* const delta_xFR,
									const double* const delta_yFX,
									int BC_idx, SubjectToStatus BC_status
									)
{
	int i, ii;
	int nV  = getNV( );
	int nFR = getNFR( );
	int nFX = getNFX( );


	/* I) CHECK BOUNDS' CONSISTENCY */
	if ( areBoundsConsistent( delta_lb,delta_ub ) == BT_FALSE )
	{
		infeasible = BT_TRUE;
		tau = 0.0;

		return THROWERROR( RET_QP_INFEASIBLE );
	}


	/* II) GO TO ACTIVE SET CHANGE */
	if ( tau > ZERO )
	{
		/* 1) Perform step in primal und dual space. */
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

		/* 2) Shift QP data. */
		for( i=0; i<nV; ++i )
		{
			g[i]  += tau*delta_g[i];
			lb[i] += tau*delta_lb[i];
			ub[i] += tau*delta_ub[i];
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


	char messageString[80];

	/* III) UPDATE ACTIVE SET */
	switch ( BC_status )
	{
		/* Optimal solution found as no working set change detected. */
		case ST_UNDEFINED:
			return RET_OPTIMAL_SOLUTION_FOUND;


		/* Remove one variable from active set. */
		case ST_INACTIVE:
			#ifndef __XPCTARGET__
			snprintf( messageString,80,"bound no. %d.", BC_idx );
			getGlobalMessageHandler( )->throwInfo( RET_REMOVE_FROM_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
			#endif

			if ( removeBound( BC_idx,BT_TRUE ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_REMOVE_FROM_ACTIVESET_FAILED );

			y[BC_idx] = 0.0;
			break;


		/* Add one variable to active set. */
		default:
			#ifndef __XPCTARGET__
			if ( BC_status == ST_LOWER )
				snprintf( messageString,80,"lower bound no. %d.", BC_idx );
			else
				snprintf( messageString,80,"upper bound no. %d.", BC_idx );
				getGlobalMessageHandler( )->throwInfo( RET_ADD_TO_ACTIVESET,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
			#endif

			if ( addBound( BC_idx,BC_status,BT_TRUE ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_ADD_TO_ACTIVESET_FAILED );
			break;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s h a l l R e f a c t o r i s e
 */
BooleanType QProblemB::shallRefactorise( const Bounds* const guessedBounds ) const
{
	int i;
	int nV = getNV( );

	/* always refactorise if Hessian is not known to be positive definite */
	if ( getHessianType( ) == HST_SEMIDEF )
		return BT_TRUE;

	/* 1) Determine number of bounds that have same status
	 *    in guessed AND current bounds.*/
	int differenceNumber = 0;

	for( i=0; i<nV; ++i )
		if ( guessedBounds->getStatus( i ) != bounds->getStatus( i ) )
			++differenceNumber;

	/* 2) Decide wheter to refactorise or not. */
	if ( 2*differenceNumber > guessedBounds->getNFX( ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}


/*
 *	a d d B o u n d
 */
returnValue QProblemB::addBound(	int number, SubjectToStatus B_status,
									BooleanType updateCholesky
									)
{
	int i, j;
	int nV  = getNV( );
	int nFR = getNFR( );


	/* consistency check */
	if ( ( getStatus( ) == QPS_NOTINITIALISED )    ||
		 ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
		 ( getStatus( ) == QPS_SOLVED )            )
	{
		return THROWERROR( RET_UNKNOWN_BUG );
	}

	/* Perform cholesky updates only if QProblemB has been initialised! */
	if ( getStatus( ) == QPS_PREPARINGAUXILIARYQP )
	{
		/* UPDATE INDICES */
		if ( bounds->moveFreeToFixed( number,B_status ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_ADDBOUND_FAILED );

		return SUCCESSFUL_RETURN;
	}


	/* I) PERFORM CHOLESKY UPDATE: */
	if ( ( updateCholesky == BT_TRUE ) &&
		 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
	{
		/* 1) Index of variable to be added within the list of free variables. */
		int number_idx = bounds->getFree( )->getIndex( number );

		double c, s, nu;

		/* 2) Use row-wise Givens rotations to restore upper triangular form of R. */
		for( i=number_idx+1; i<nFR; ++i )
		{
			computeGivens( R[(i-1)*nV + i],R[i*nV + i], R[(i-1)*nV + i],R[i*nV + i],c,s );
			nu = s/(1.0+c);

			for( j=(1+i); j<nFR; ++j ) /* last column of R is thrown away */
				applyGivens( c,s,nu,R[(i-1)*nV + j],R[i*nV + j], R[(i-1)*nV + j],R[i*nV + j] );
		}

		/* 3) Delete <number_idx>th column and ... */
		for( i=0; i<nFR-1; ++i )
			for( j=number_idx+1; j<nFR; ++j )
				R[i*nV + j-1] = R[i*nV + j];
		/* ... last column of R. */
		for( i=0; i<nFR; ++i )
			R[i*nV + nFR-1] = 0.0;
	}

	/* II) UPDATE INDICES */
	if ( bounds->moveFreeToFixed( number,B_status ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_ADDBOUND_FAILED );


	return SUCCESSFUL_RETURN;
}


/*
 *	r e m o v e B o u n d
 */
returnValue QProblemB::removeBound(	int number,
									BooleanType updateCholesky
									)
{
	int i, ii;
	int nV  = getNV( );
	int nFR = getNFR( );


	/* consistency check */
	if ( ( getStatus( ) == QPS_NOTINITIALISED )    ||
		 ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
		 ( getStatus( ) == QPS_SOLVED )            )
	{
		return THROWERROR( RET_UNKNOWN_BUG );
	}


	/* I) UPDATE INDICES */
	if ( bounds->moveFixedToFree( number ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_REMOVEBOUND_FAILED );

	/* Perform cholesky updates only if QProblemB has been initialised! */
	if ( getStatus( ) == QPS_PREPARINGAUXILIARYQP )
		return SUCCESSFUL_RETURN;


	/* II) PERFORM CHOLESKY UPDATE */
	if ( ( updateCholesky == BT_TRUE ) &&
		 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
	{
		int* FR_idx = new int[nFR+1];
		if ( bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
		{
			delete[] FR_idx;
			return THROWERROR( RET_REMOVEBOUND_FAILED );
		}

		/* 1) Calculate new column of cholesky decomposition. */
		double* rhs = new double[nFR];
		double* r   = new double[nFR];

		double r0;
		switch ( hessianType )
		{
			case HST_ZERO:
				r0 = eps;
				for( i=0; i<nFR; ++i )
					rhs[i] = 0.0;
				break;

			case HST_IDENTITY:
				r0 = 1.0;
				for( i=0; i<nFR; ++i )
					rhs[i] = 0.0;
				break;

			default:
				r0 = H[number*nV + number];
				for( i=0; i<nFR; ++i )
				{
					ii = FR_idx[i];
					rhs[i] = H[number*nV + ii];
				}
				break;
		}

		if ( backsolveR( rhs,BT_TRUE,BT_TRUE,r ) != SUCCESSFUL_RETURN )
		{
			delete[] rhs; delete[] r; delete[] FR_idx;
			return THROWERROR( RET_REMOVEBOUND_FAILED );
		}

		for( i=0; i<nFR; ++i )
			r0 -= r[i]*r[i];

		/* 2) Store new column into R. */
		for( i=0; i<nFR; ++i )
			R[i*nV + nFR] = r[i];

		if ( r0 > 0.0 )
			R[nFR*nV + nFR] = sqrt( r0 );
		else
		{
			delete[] rhs; delete[] r; delete[] FR_idx;

			hessianType = HST_SEMIDEF;
			return THROWERROR( RET_HESSIAN_NOT_SPD );
		}

		delete[] rhs; delete[] r; delete[] FR_idx;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t I t e r a t i o n
 */
returnValue QProblemB::printIteration( 	int iteration,
										int BC_idx,	SubjectToStatus BC_status
		  								)
{
	#ifndef __XPCTARGET__
	char myPrintfString[160];

	/* consistency check */
	if ( iteration < 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	/* nothing to do */
	if ( printLevel != PL_MEDIUM )
		return SUCCESSFUL_RETURN;


	/* 1) Print header at first iteration. */
 	if ( iteration == 0 )
	{
		snprintf( myPrintfString,160,"\n\n#################   qpOASES  --  QP NO. %3.0d   ##################\n\n", count );
		myPrintf( myPrintfString );

		myPrintf( "    Iter   |    StepLength    |       Info       |   nFX    \n" );
		myPrintf( " ----------+------------------+------------------+--------- \n" );
	}

	/* 2) Print iteration line. */
	if ( BC_status == ST_UNDEFINED )
	{
		if ( hessianType == HST_ZERO )
			snprintf( myPrintfString,80,"   %5.1d   |   %1.6e   |    LP SOLVED     |  %4.1d   \n", iteration,tau,getNFX( ) );
		else
			snprintf( myPrintfString,80,"   %5.1d   |   %1.6e   |    QP SOLVED     |  %4.1d   \n", iteration,tau,getNFX( ) );
		myPrintf( myPrintfString );
	}
	else
	{
		char info[8];

		if ( BC_status == ST_INACTIVE )
			snprintf( info,8,"REM BND" );
		else
			snprintf( info,8,"ADD BND" );

		snprintf( myPrintfString,80,"   %5.1d   |   %1.6e   |   %s %4.1d   |  %4.1d   \n", iteration,tau,info,BC_idx,getNFX( ) );
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
