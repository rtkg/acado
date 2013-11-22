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
 *	\file SRC/EXTRAS/SolutionAnalysis.cpp
 *	\author Boris Houska, Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2008-2009
 *
 *	Implementation of the SolutionAnalysis class designed to perform
 *	additional analysis after solving a QP with qpOASES.
 *
 */


#include <stdio.h>

#include <EXTRAS/SolutionAnalysis.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	S o l u t i o n A n a l y s i s
 */
SolutionAnalysis::SolutionAnalysis( )
{

}


/*
 *	S o l u t i o n A n a l y s i s
 */
SolutionAnalysis::SolutionAnalysis( const SolutionAnalysis& rhs )
{

}


/*
 *	~ S o l u t i o n A n a l y s i s
 */
SolutionAnalysis::~SolutionAnalysis( )
{

}


/*
 *	o p e r a t o r =
 */
SolutionAnalysis& SolutionAnalysis::operator=( const SolutionAnalysis& rhs )
{
	if ( this != &rhs )
	{

	}

	return *this;
}



/*
 *	g e t M a x K K T v i o l a t i o n
 */
returnValue SolutionAnalysis::getMaxKKTviolation( QProblemB* qp, double& maxKKTviolation ) const
{
	int i, j;
	int nV = qp->getNV( );

	double tmp;
	maxKKTviolation = 0.0;


	/* 1) Check for Hx + g - y*A' = 0  (here: A = Id). */
	for( i=0; i<nV; ++i )
	{
		tmp = qp->g[i];

		switch ( qp->getHessianType( ) )
		{
			case HST_ZERO:
					//tmp += qp->eps * qp->x[i];
				break;

			case HST_IDENTITY:
					tmp += qp->x[i];
				break;

			default:
				for( j=0; j<nV; ++j )
					tmp += qp->H[i*nV + j] * qp->x[j];
				break;
		}

		tmp -= qp->y[i];

		if ( fabs( tmp ) > maxKKTviolation )
			maxKKTviolation = fabs( tmp );
	}

	/* 2) Check for lb <= x <= ub. */
	for( i=0; i<nV; ++i )
	{
		if ( qp->lb[i] - qp->x[i] > maxKKTviolation )
			maxKKTviolation = qp->lb[i] - qp->x[i];

		if ( qp->x[i] - qp->ub[i] > maxKKTviolation )
			maxKKTviolation = qp->x[i] - qp->ub[i];
	}

	/* 3) Check for correct sign of y and for complementary slackness. */
	for( i=0; i<nV; ++i )
	{
		switch ( qp->bounds->getStatus( i ) )
		{
			case ST_LOWER:
				if ( -qp->y[i] > maxKKTviolation )
					maxKKTviolation = -qp->y[i];
				if ( fabs( ( qp->x[i] - qp->lb[i] ) * qp->y[i] ) > maxKKTviolation )
					maxKKTviolation = fabs( ( qp->x[i] - qp->lb[i] ) * qp->y[i] );
				break;

			case ST_UPPER:
				if ( qp->y[i] > maxKKTviolation )
					maxKKTviolation = qp->y[i];
				if ( fabs( ( qp->ub[i] - qp->x[i] ) * qp->y[i] ) > maxKKTviolation )
					maxKKTviolation = fabs( ( qp->ub[i] - qp->x[i] ) * qp->y[i] );
				break;

			default: /* inactive */
			if ( fabs( qp->y[i] ) > maxKKTviolation )
					maxKKTviolation = fabs( qp->y[i] );
				break;
		}
	}


	return SUCCESSFUL_RETURN;
}


/*
 *	g e t M a x K K T v i o l a t i o n
 */
returnValue SolutionAnalysis::getMaxKKTviolation( QProblem* qp, double& maxKKTviolation ) const
{
	int i, j;
	int nV = qp->getNV( );
	int nC = qp->getNC( );

	double tmp;
	maxKKTviolation = 0.0;


	/* 1) check for Hx + g - [yFX yAC]*[Id A]' = 0. */
	for( i=0; i<nV; ++i )
	{
		tmp = qp->g[i];

		switch ( qp->getHessianType( ) )
		{
			case HST_ZERO:
					//tmp += qp->eps * qp->x[i];
				break;

			case HST_IDENTITY:
					tmp += qp->x[i];
				break;

			default:
				for( j=0; j<nV; ++j )
					tmp += qp->H[i*nV + j] * qp->x[j];
				break;
		}

		tmp -= qp->y[i];

		for( j=0; j<nC; ++j )
			tmp -= qp->y[nV+j] * qp->A[j*nV + i];

		if ( fabs( tmp ) > maxKKTviolation )
			maxKKTviolation = fabs( tmp );
	}

	/* 2) Check for [lb lbA] <= [Id A]*x <= [ub ubA]. */
	/* lbA <= Ax <= ubA */
	double* Ax = new double[nC];
	for( i=0; i<nC; ++i )
	{
		Ax[i] = 0.0;
		for( j=0; j<nV; ++j )
			Ax[i] += qp->A[i*nV + j] * qp->x[j];
	}

	for( i=0; i<nC; ++i )
	{
		if ( qp->lbA[i] - Ax[i] > maxKKTviolation )
			maxKKTviolation = qp->lbA[i] - Ax[i];

		if ( Ax[i] - qp->ubA[i] > maxKKTviolation )
			maxKKTviolation = Ax[i] - qp->ubA[i];
	}

	/* lb <= x <= ub */
	for( i=0; i<nV; ++i )
	{
		if ( qp->lb[i] - qp->x[i] > maxKKTviolation )
			maxKKTviolation = qp->lb[i] - qp->x[i];

		if ( qp->x[i] - qp->ub[i] > maxKKTviolation )
			maxKKTviolation = qp->x[i] - qp->ub[i];
	}

	/* 3) Check for correct sign of y and for complementary slackness. */
	/* bounds */
	for( i=0; i<nV; ++i )
	{
		switch ( qp->bounds->getStatus( i ) )
		{
			case ST_LOWER:
				if ( -qp->y[i] > maxKKTviolation )
					maxKKTviolation = -qp->y[i];
				if ( fabs( ( qp->x[i] - qp->lb[i] ) * qp->y[i] ) > maxKKTviolation )
					maxKKTviolation = fabs( ( qp->x[i] - qp->lb[i] ) * qp->y[i] );
				break;

			case ST_UPPER:
				if ( qp->y[i] > maxKKTviolation )
					maxKKTviolation = qp->y[i];
				if ( fabs( ( qp->ub[i] - qp->x[i] ) * qp->y[i] ) > maxKKTviolation )
					maxKKTviolation = fabs( ( qp->ub[i] - qp->x[i] ) * qp->y[i] );
				break;

			default: /* inactive */
			if ( fabs( qp->y[i] ) > maxKKTviolation )
					maxKKTviolation = fabs( qp->y[i] );
				break;
		}
	}

	/* constraints */
	for( i=0; i<nC; ++i )
	{
		switch ( qp->constraints->getStatus( i ) )
		{
			case ST_LOWER:
				if ( -qp->y[nV+i] > maxKKTviolation )
					maxKKTviolation = -qp->y[nV+i];
				if ( fabs( ( Ax[i] - qp->lbA[i] ) * qp->y[nV+i] ) > maxKKTviolation )
					maxKKTviolation = fabs( ( Ax[i] - qp->lbA[i] ) * qp->y[nV+i] );
				break;

			case ST_UPPER:
				if ( qp->y[nV+i] > maxKKTviolation )
					maxKKTviolation = qp->y[nV+i];
				if ( fabs( ( qp->ubA[i] - Ax[i] ) * qp->y[nV+i] ) > maxKKTviolation )
					maxKKTviolation = fabs( ( qp->ubA[i] - Ax[i] ) * qp->y[nV+i] );
				break;

			default: /* inactive */
			if ( fabs( qp->y[nV+i] ) > maxKKTviolation )
					maxKKTviolation = fabs( qp->y[nV+i] );
				break;
		}
	}

	delete[] Ax;

	return SUCCESSFUL_RETURN;
}


/*
 *	g e t M a x K K T v i o l a t i o n
 */
returnValue SolutionAnalysis::getMaxKKTviolation( SQProblem* qp, double& maxKKTviolation ) const
{
	return getMaxKKTviolation( (QProblem*)qp,maxKKTviolation );
}



/*
 *	g e t V a r i a n c e C o v a r i a n c e
 */
returnValue SolutionAnalysis::getVarianceCovariance( QProblemB* qp, double* g_b_bA_VAR, double* Primal_Dual_VAR ) const
{
	return THROWERROR( RET_NOT_YET_IMPLEMENTED );
}


/*
 *	g e t V a r i a n c e C o v a r i a n c e
 */
returnValue SolutionAnalysis::getVarianceCovariance( QProblem* qp, double* g_b_bA_VAR, double* Primal_Dual_VAR ) const
{

  // DEFINITION OF THE DIMENSIONS nV AND nC:
  // ---------------------------------------
  int nV  = qp->getNV( );                      // dimension of x / the bounds
  int nC  = qp->getNC( );                      // dimension of the constraints
  int dim = 2*nV+nC;                           // dimension of input and output
                                               // variance-covariance matrix
  int run1, run2, run3;                        // simple run variables (for loops).


  // ALLOCATION OF MEMORY:
  // ---------------------
  double* delta_g_cov    = new double[nV];     // a covariance-vector of g
  double* delta_lb_cov   = new double[nV];     // a covariance-vector of lb
  double* delta_ub_cov   = new double[nV];     // a covariance-vector of ub
  double* delta_lbA_cov  = new double[nC];     // a covariance-vector of lbA
  double* delta_ubA_cov  = new double[nC];     // a covariance-vector of ubA

  returnValue returnvalue;                     // the return value
  BooleanType Delta_bC_isZero = BT_FALSE;      // (just use FALSE here)
  BooleanType Delta_bB_isZero = BT_FALSE;      // (just use FALSE here)



  // ASK FOR THE NUMBER OF FREE AND FIXED VARIABLES:
  // (ASSUMES THAT ACTIVE SET IS CONSTANT FOR THE
  //  VARIANCE-COVARIANCE EVALUATION)
  // -----------------------------------------------
  int nFR, nFX, nAC;

  nFR = qp->getNFR( );
  nFX = qp->getNFX( );
  nAC = qp->getNAC( );


  // ASK FOR THE CORRESPONDING INDEX ARRAYS:
  // ---------------------------------------
  int *FR_idx, *FX_idx, *AC_idx;

  FR_idx = new int[nFR];
  if ( qp->bounds->getFree( )->getNumberArray( FR_idx ) != SUCCESSFUL_RETURN )
       return THROWERROR( RET_HOTSTART_FAILED );

  FX_idx = new int[nFX];
  if ( qp->bounds->getFixed( )->getNumberArray( FX_idx ) != SUCCESSFUL_RETURN )
       return THROWERROR( RET_HOTSTART_FAILED );

  AC_idx = new int[nAC];
  if ( qp->constraints->getActive( )->getNumberArray( AC_idx ) != SUCCESSFUL_RETURN )
       return THROWERROR( RET_HOTSTART_FAILED );



  // INTRODUCE VARIABLES TO MEASURE THE REACTION OF THE QP-SOLUTION TO
  // THE VARIANCE-COVARIANCE DISTURBANCE:
  // --------------------------------------------------------------
  double *delta_xFR = new double[nFR];
  double *delta_xFX = new double[nFX];
  double *delta_yAC = new double[nAC];
  double *delta_yFX = new double[nFX];

  double* K             = new double[dim*dim];  // matrix to store
                                                // an intermediate
                                                // result.

  // SOME INITIALIZATIONS:
  // ---------------------
  for( run1 = 0; run1 < dim*dim; run1++ ){
    K              [run1] = 0.0;
    Primal_Dual_VAR[run1] = 0.0;
  }


  // =================================================================

  // FIRST MATRIX MULTIPLICATION (OBTAINS THE INTERMEDIATE RESULT
  //  K := [ ("ACTIVE" KKT-MATRIX OF THE QP)^(-1) * g_b_bA_VAR ]^T )
  // THE EVALUATION OF THE INVERSE OF THE KKT-MATRIX OF THE QP
  // WITH RESPECT TO THE CURRENT ACTIVE SET
  // USES THE EXISTING CHOLESKY AND TQ-DECOMPOSITIONS. FOR DETAILS
  // cf. THE (protected) FUNCTION determineStepDirection.

  for( run3 = 0; run3 < dim; run3++ ){


    for( run1 = 0; run1 < nV; run1++ ){                             //
      delta_g_cov  [run1]   = g_b_bA_VAR[run3*dim+run1];            //
      delta_lb_cov [run1]   = g_b_bA_VAR[run3*dim+nV+run1];         //  LINE-WISE LOADING OF THE INPUT
      delta_ub_cov [run1]   = g_b_bA_VAR[run3*dim+nV+run1];         //  VARIANCE-COVARIANCE
    }                                                               //
    for( run1 = 0; run1 < nC; run1++ ){                             //
      delta_lbA_cov [run1]  = g_b_bA_VAR[run3*dim+2*nV+run1];       //
      delta_ubA_cov [run1]  = g_b_bA_VAR[run3*dim+2*nV+run1];       //
    }


    // EVALUATION OF THE STEP:
    // ------------------------------------------------------------------------------

    returnvalue = qp->determineStepDirection( FR_idx, FX_idx, AC_idx,
                                              delta_g_cov, delta_lbA_cov, delta_ubA_cov, delta_lb_cov, delta_ub_cov,
                                              Delta_bC_isZero, Delta_bB_isZero, delta_xFX,delta_xFR,
                                              delta_yAC,delta_yFX );

    // ------------------------------------------------------------------------------


    // STOP THE ALGORITHM IN THE CASE OF NO SUCCESFUL RETURN:
    // ------------------------------------------------------
    if ( returnvalue != SUCCESSFUL_RETURN ){

      delete[] delta_g_cov;
      delete[] delta_lb_cov;
      delete[] delta_ub_cov;
      delete[] delta_lbA_cov;
      delete[] delta_ubA_cov;
      delete[] FR_idx;
      delete[] FX_idx;
      delete[] AC_idx;
      delete[] delta_xFR;
      delete[] delta_xFX;
      delete[] delta_yAC;
      delete[] delta_yFX;
      delete[] K;

      THROWERROR( RET_STEPDIRECTION_DETERMINATION_FAILED );
      return returnvalue;
    }



    for( run1=0; run1<nFR; run1++ ){                                //
      run2                  = FR_idx[run1];                         //
      K[run3*dim+run2]      = delta_xFR[run1];                      //
    }                                                               //  LINE WISE
    for( run1=0; run1<nFX; run1++ ){                                //  STORAGE OF THE QP-REACTION
      run2                  = FX_idx[run1];                         //  (uses the index list)
      K[run3*dim+run2]      = delta_xFX[run1];                      //
      K[run3*dim+nV+run2]   = delta_yFX[run1];                      //
    }                                                               //
    for( run1=0; run1<nAC; run1++ ){                                //
      run2                  = AC_idx[run1];                         //
      K[run3*dim+2*nV+run2] = delta_yAC[run1];                      //
    }                                                               //

  }


  // =================================================================

  // SECOND MATRIX MULTIPLICATION (OBTAINS THE FINAL RESULT
  // Primal_Dual_VAR := ("ACTIVE" KKT-MATRIX OF THE QP)^(-1) * K )
  // THE APPLICATION OF THE KKT-INVERSE IS AGAIN REALIZED
  // BY USING THE PROTECTED FUNCTION
  // determineStepDirection

  for( run3 = 0; run3 < dim; run3++ ){

    for( run1 = 0; run1 < nV; run1++ ){                             //
      delta_g_cov  [run1]   = K[run3+     run1*dim];                //
      delta_lb_cov [run1]   = K[run3+(nV+run1)*dim];                //  ROW WISE LOADING OF THE
      delta_ub_cov [run1]   = K[run3+(nV+run1)*dim];                //  INTERMEDIATE RESULT K
    }                                                               //
    for( run1 = 0; run1 < nC; run1++ ){                             //
      delta_lbA_cov [run1]  = K[run3+(2*nV+run1)*dim];              //
      delta_ubA_cov [run1]  = K[run3+(2*nV+run1)*dim];              //
    }


    // EVALUATION OF THE STEP:
    // ------------------------------------------------------------------------------

    returnvalue = qp->determineStepDirection( FR_idx, FX_idx, AC_idx,
                                              delta_g_cov, delta_lbA_cov, delta_ubA_cov, delta_lb_cov, delta_ub_cov,
                                              Delta_bC_isZero, Delta_bB_isZero, delta_xFX,delta_xFR,
                                              delta_yAC,delta_yFX);


    // ------------------------------------------------------------------------------


    // STOP THE ALGORITHM IN THE CASE OF NO SUCCESFUL RETURN:
    // ------------------------------------------------------
    if ( returnvalue != SUCCESSFUL_RETURN ){

      delete[] delta_g_cov;
      delete[] delta_lb_cov;
      delete[] delta_ub_cov;
      delete[] delta_lbA_cov;
      delete[] delta_ubA_cov;
      delete[] FR_idx;
      delete[] FX_idx;
      delete[] AC_idx;
      delete[] delta_xFR;
      delete[] delta_xFX;
      delete[] delta_yAC;
      delete[] delta_yFX;
      delete[] K;

      THROWERROR( RET_STEPDIRECTION_DETERMINATION_FAILED );
      return returnvalue;
    }



    for( run1=0; run1<nFR; run1++ ){                                 //
      run2                                = FR_idx[run1];            //
      Primal_Dual_VAR[run3+run2*dim]      = delta_xFR[run1];         //
    }                                                                //
    for( run1=0; run1<nFX; run1++ ){                                 //  ROW-WISE STORAGE
      run2                  = FX_idx[run1];                          //  OF THE RESULT.
      Primal_Dual_VAR[run3+run2*dim     ]   = delta_xFX[run1];       //
      Primal_Dual_VAR[run3+(nV+run2)*dim]   = delta_yFX[run1];       //
    }                                                                //
    for( run1=0; run1<nAC; run1++ ){                                 //
      run2                                  = AC_idx[run1];          //
      Primal_Dual_VAR[run3+(2*nV+run2)*dim] = delta_yAC[run1];       //
    }

  }


  // DEALOCATE THE MEMORY:
  // ---------------------

  delete[] delta_g_cov;
  delete[] delta_lb_cov;
  delete[] delta_ub_cov;
  delete[] delta_lbA_cov;
  delete[] delta_ubA_cov;
  delete[] FR_idx;
  delete[] FX_idx;
  delete[] AC_idx;
  delete[] delta_xFR;
  delete[] delta_xFX;
  delete[] delta_yAC;
  delete[] delta_yFX;
  delete[] K;

  return SUCCESSFUL_RETURN;
}


/*
 *	g e t V a r i a n c e C o v a r i a n c e
 */
returnValue SolutionAnalysis::getVarianceCovariance( SQProblem* qp, double* g_b_bA_VAR, double* Primal_Dual_VAR ) const
{
	/* Call QProblem variant. */
	return getVarianceCovariance( (QProblem*)qp,g_b_bA_VAR,Primal_Dual_VAR );
}


#ifndef __DSPACE__
} /* qpOASES */
#endif


/*
 *	end of file
 */
