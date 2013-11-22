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
 *	\file SRC/SQProblem.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Implementation of the SQProblem class which is able to use the newly
 *	developed online active set strategy for parametric quadratic programming
 *	with varying matrices.
 */


#include <SQProblem.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	S Q P r o b l e m
 */
SQProblem::SQProblem( ) : QProblem( )
{
}


/*
 *	S Q P r o b l e m
 */
SQProblem::SQProblem( int _nV, int _nC ) : QProblem( _nV,_nC )
{
}


/*
 *	S Q P r o b l e m
 */
SQProblem::SQProblem( int _nV, int _nC, HessianType _hessianType ) : QProblem( _nV,_nC,_hessianType )
{
}


/*
 *	S Q P r o b l e m
 */
SQProblem::SQProblem( const SQProblem& rhs ) :	QProblem( rhs )
{
}


/*
 *	~ S Q P r o b l e m
 */
SQProblem::~SQProblem( )
{
}


/*
 *	o p e r a t o r =
 */
SQProblem& SQProblem::operator=( const SQProblem& rhs )
{
	if ( this != &rhs )
	{
		QProblem::operator=( rhs );
	}

	return *this;
}


/*
 *	h o t s t a r t
 */
returnValue SQProblem::hotstart(	const double* const H_new, const double* const g_new, const double* const A_new,
									const double* const lb_new, const double* const ub_new,
									const double* const lbA_new, const double* const ubA_new,
									int& nWSR, double* const cputime )
{
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


	/* I) UPDATE QP MATRICES AND VECTORS */
	if ( setupAuxiliaryQP( H_new,A_new ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );


	/* II) PERFORM USUAL HOMOTOPY */

	/* Allow only remaining CPU time for usual hotstart. */
	if ( cputime != 0 )
		*cputime -= getCPUtime( ) - starttime;

	returnValue returnvalue = QProblem::hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime );


	/* stop runtime measurement */
	if ( cputime != 0 )
		*cputime = getCPUtime( ) - starttime;

	return returnvalue;
}


/*
 *	h o t s t a r t
 */
returnValue SQProblem::hotstart(	const char* const H_file, const char* const g_file, const char* const A_file,
									const char* const lb_file, const char* const ub_file,
									const char* const lbA_file, const char* const ubA_file,
									int& nWSR, double* const cputime
									)
{
	int nV = getNV( );
	int nC = getNC( );

	returnValue returnvalue;

	/* consistency checks */
	if ( ( H_file == 0 ) || ( g_file == 0 ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	if ( ( nC > 0 ) && ( A_file == 0 ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	/* 1) Load new QP matrices from files. */
	double* H_new  = new double[nV*nV];
	double* A_new  = new double[nC*nV];

	if ( readFromFile( H_new, nV,nV, H_file ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_UNABLE_TO_READ_FILE );

	if ( readFromFile( A_new, nC,nV, A_file ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_UNABLE_TO_READ_FILE );

	/* 2) Load new QP vectors from files. */
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
		delete[] A_new;
		delete[] H_new;

		return THROWERROR( RET_UNABLE_TO_READ_FILE );
	}

	/* 3) Actually perform hotstart. */
	returnvalue = hotstart(	H_new,g_new,A_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime );

	if ( ubA_file != 0 )
		delete[] ubA_new;
	if ( lbA_file != 0 )
		delete[] lbA_new;
	if ( ub_file != 0 )
		delete[] ub_new;
	if ( lb_file != 0 )
		delete[] lb_new;
	delete[] g_new;
	delete[] A_new;
	delete[] H_new;

	return returnvalue;
}


/*
 *	h o t s t a r t
 */
returnValue SQProblem::hotstart(	const double* const g_new,
									const double* const lb_new, const double* const ub_new,
									const double* const lbA_new, const double* const ubA_new,
									int& nWSR, double* const cputime
									)
{
	/* Call to hotstart function for fixed QP matrices. */
	return QProblem::hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime );
}


/*
 *	h o t s t a r t
 */
returnValue SQProblem::hotstart(	const char* const g_file,
									const char* const lb_file, const char* const ub_file,
									const char* const lbA_file, const char* const ubA_file,
									int& nWSR, double* const cputime
									)
{
	/* Call to hotstart function for fixed QP matrices. */
	return QProblem::hotstart( g_file,lb_file,ub_file,lbA_file,ubA_file, nWSR,cputime );
}


/*
 *	h o t s t a r t
 */
returnValue SQProblem::hotstart(	const double* const g_new,
									const double* const lb_new, const double* const ub_new,
									const double* const lbA_new, const double* const ubA_new,
									int& nWSR, double* const cputime,
									const Bounds* const guessedBounds, const Constraints* const guessedConstraints
									)
{
	/* Call to hotstart function for fixed QP matrices. */
	return QProblem::hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime, guessedBounds,guessedConstraints );
}


/*
 *	h o t s t a r t
 */
returnValue SQProblem::hotstart(	const char* const g_file,
									const char* const lb_file, const char* const ub_file,
									const char* const lbA_file, const char* const ubA_file,
									int& nWSR, double* const cputime,
									const Bounds* const guessedBounds, const Constraints* const guessedConstraints
									)
{
	/* Call to hotstart function for fixed QP matrices. */
	return QProblem::hotstart( g_file,lb_file,ub_file,lbA_file,ubA_file, nWSR,cputime, guessedBounds,guessedConstraints );
}



/*****************************************************************************
 *  P R O T E C T E D                                                        *
 *****************************************************************************/

/*
 *	s e t u p A u x i l i a r y Q P
 */
returnValue SQProblem::setupAuxiliaryQP( const double* const H_new, const double* const A_new )
{
	int i, j;
	int nV = getNV( );
	int nC = getNC( );

	if ( ( getStatus( ) == QPS_NOTINITIALISED )       ||
		 ( getStatus( ) == QPS_PREPARINGAUXILIARYQP ) ||
		 ( getStatus( ) == QPS_PERFORMINGHOMOTOPY )   )
	{
		return THROWERROR( RET_UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED );
	}

	status = QPS_PREPARINGAUXILIARYQP;


	/* I) SETUP NEW QP MATRICES AND VECTORS: */
	/* 1) Shift constraints' bounds vectors by (A_new - A)'*x_opt to ensure
	 *    that old optimal solution remains feasible for new QP data. */
	/*    Firstly, shift by -A'*x_opt and ... */
	if ( nC > 0 )
	{
		if ( A_new == 0 )
			return THROWERROR( RET_INVALID_ARGUMENTS );

		for ( i=0; i<nC; ++i )
		{
			lbA[i] = -Ax_l[i];
			ubA[i] =  Ax_u[i];
		}

		/* Set constraint matrix as well as ... */
		for( i=0; i<nC; ++i )
		{
			Ax_l[i] = 0.0;
			for( j=0; j<nV; ++j )
			{
				A[i*nV + j] = A_new[i*nV + j];
				Ax_l[i] += A[i*nV + j] * x[j];
			}
		}

		/* ... secondly, shift by +A_new'*x_opt. */
		for ( i=0; i<nC; ++i )
		{
			lbA[i] += Ax_l[i];
			ubA[i] += Ax_l[i];
		}

		/* update constraint products. */
		for ( i=0; i<nC; ++i )
		{
			Ax_u[i] = ubA[i] - Ax_l[i];
			Ax_l[i] = Ax_l[i] - lbA[i];
		}
	}

	/* 2) Set new Hessian matrix,determine Hessian type and
	 *    regularise new Hessian matrix if necessary. */
	/* a) Setup new Hessian matrix and determine its type. */
	if ( H_new != 0 )
	{
		if ( H == 0 )
		{
			// former trivial Hessian is replaced by given one
			H = new double[nV*nV];
		}

		setH( H_new );

		hessianType = HST_UNKNOWN;
		if ( determineHessianType( ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );

		/* b) Regularise new Hessian if necessary. */
		if ( ( hessianType == HST_ZERO ) ||
			 ( hessianType == HST_SEMIDEF ) ||
			 ( usingRegularisation( ) == BT_TRUE ) )
		{
			eps = 0.0; // reset previous regularisation parameter

			if ( regulariseHessian( ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
		}
	}
	else
	{
		if ( H != 0 )
			return THROWERROR( RET_NO_HESSIAN_SPECIFIED );
	}



	/* 3) Setup QP gradient. */
	if ( setupAuxiliaryQPgradient( ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );


	/* II) SETUP WORKING SETS AND MATRIX FACTORISATIONS: */
	/* 1) Make a copy of current bounds/constraints ... */
	Bounds*      oldBounds      = bounds;
	Constraints* oldConstraints = constraints;

	/*    ... reset them ... */
	bounds = new Bounds( nV );
	constraints = new Constraints( nC );

	/*    ... and set them up afresh. */
	if ( setupSubjectToType( ) != SUCCESSFUL_RETURN )
	{
		delete oldBounds; delete oldConstraints;
		return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	if ( bounds->setupAllFree( ) != SUCCESSFUL_RETURN )
	{
		delete oldBounds; delete oldConstraints;
		return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	if ( constraints->setupAllInactive( ) != SUCCESSFUL_RETURN )
	{
		delete oldBounds; delete oldConstraints;
		return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	/* 2) Setup TQ factorisation. */
	if ( setupTQfactorisation( ) != SUCCESSFUL_RETURN )
	{
		delete oldBounds; delete oldConstraints;
		return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	/* 3) Setup old working sets afresh (updating TQ factorisation). */
	if ( setupAuxiliaryWorkingSet( oldBounds,oldConstraints,BT_TRUE ) != SUCCESSFUL_RETURN )
	{
		delete oldBounds; delete oldConstraints;
		THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
	}

	/* 4) Calculate Cholesky decomposition. */
	if ( ( getNAC( ) + getNFX( ) ) == 0 )
	{
		/* Factorise full Hessian if no bounds/constraints are active. */
		if ( setupCholeskyDecomposition( ) != SUCCESSFUL_RETURN )
		{
			delete oldBounds; delete oldConstraints;
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
		}
	}
	else
	{
		/* Factorise projected Hessian if there active bounds/constraints. */
		if ( setupCholeskyDecompositionProjected( ) != SUCCESSFUL_RETURN )
		{
			delete oldBounds; delete oldConstraints;
			return THROWERROR( RET_SETUP_AUXILIARYQP_FAILED );
		}
	}

	/* 5) Initialise cycling manager. */
	cyclingManager->clearCyclingData( );

	status = QPS_AUXILIARYQPSOLVED;


	delete oldBounds; delete oldConstraints;

	return SUCCESSFUL_RETURN;
}


#ifndef __DSPACE__
} /* qpOASES */
#endif


/*
 *	end of file
 */
