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
 *    \file external_packages/src/acado_qpoases/qp_solver_qpoases.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 19.08.2008
 */


#include <include/acado_qpoases/qp_solver_qpoases.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

QPsolver_qpOASES::QPsolver_qpOASES( ) : DenseQPsolver( )
{
	qp = 0;
}


QPsolver_qpOASES::QPsolver_qpOASES( UserInteraction* _userInteraction ) : DenseQPsolver( _userInteraction )
{
	qp = 0;
}


QPsolver_qpOASES::QPsolver_qpOASES( const QPsolver_qpOASES& rhs ) : DenseQPsolver( rhs )
{
	if ( rhs.qp != 0 )
		qp = new qpOASES::SQProblem( *(rhs.qp) );
	else
		qp = 0;
}


QPsolver_qpOASES::~QPsolver_qpOASES( )
{
	if ( qp != 0 )
		delete qp;
}


QPsolver_qpOASES& QPsolver_qpOASES::operator=( const QPsolver_qpOASES& rhs )
{
    if ( this != &rhs )
    {
		DenseQPsolver::operator=( rhs );

		if ( qp != 0 )
			delete qp;


		if ( rhs.qp != 0 )
			qp = new qpOASES::SQProblem( *(rhs.qp) );
		else
			qp = 0;

    }

    return *this;
}


DenseCPsolver* QPsolver_qpOASES::clone( ) const
{
	return new QPsolver_qpOASES(*this);
}


DenseQPsolver* QPsolver_qpOASES::cloneDenseQPsolver( ) const
{
	return new QPsolver_qpOASES(*this);
}


returnValue QPsolver_qpOASES::solve( DenseCP *cp_  )
{
	return DenseQPsolver::solveCP( cp_ );
}


returnValue QPsolver_qpOASES::solve(	const double* const H,
										const double* const A,
										const double* const g,
										const double* const lb,
										const double* const ub,
										const double* const lbA,
										const double* const ubA,
										uint maxIter
										)
{
	if ( qp == 0 )
		return ACADOERROR( RET_INITIALIZE_FIRST );

	/* call to qpOASES, using hotstart if possible and desired */
	numberOfSteps = maxIter;
	qpOASES::returnValue returnvalue;
	qpStatus = QPS_SOLVING;

	//printf( "nV: %d,  nC: %d \n",qp->getNV(),qp->getNC() );

	if ( qp->isInitialised( ) == qpOASES::BT_FALSE )
	{
		returnvalue = qp->init( H,g,A,lb,ub,lbA,ubA,numberOfSteps,0 );
	}
	else
	{
		int performHotstart = 0;
		get( HOTSTART_QP,performHotstart );

		if ( performHotstart == 1 )
		{
			 returnvalue = qp->hotstart( H,g,A,lb,ub,lbA,ubA,numberOfSteps,0 );
		}
		else
		{
			/* if no hotstart is desired, reset QP and use cold start */
			qp->reset( );
			returnvalue = qp->init( H,g,A,lb,ub,lbA,ubA,numberOfSteps,0 );
		}
	}
	setLast( LOG_NUM_QP_ITERATIONS, numberOfSteps );

//	acadoPrintf( "nEC: %d\n", qp->getNEC( ) );

	/* update QP status and determine return value */
	return updateQPstatus( returnvalue );
}


returnValue QPsolver_qpOASES::solve( const Matrix *H,
                                     const Matrix *A,
                                     const Vector *g,
                                     const Vector *lb,
                                     const Vector *ub,
                                     const Vector *lbA,
                                     const Vector *ubA,
                                     uint maxIter       ){

    double* H_tmp   = 0;
    double* A_tmp   = 0;
    double* g_tmp   = 0;
    double* lb_tmp  = 0;
    double* ub_tmp  = 0;
    double* lbA_tmp = 0;
    double* ubA_tmp = 0;

    returnValue returnvalue;
	returnvalue = convertQPdata( 	H,A,g,lb,ub,lbA,ubA,
									&H_tmp,&A_tmp,&g_tmp,&lb_tmp,&ub_tmp,&lbA_tmp,&ubA_tmp
									);
    returnvalue = solve( H_tmp,A_tmp,g_tmp,lb_tmp,ub_tmp,lbA_tmp,ubA_tmp,maxIter );

    if( H_tmp   != 0 ) delete[] H_tmp  ;
    if( A_tmp   != 0 ) delete[] A_tmp  ;
    if( g_tmp   != 0 ) delete[] g_tmp  ;
    if( lb_tmp  != 0 ) delete[] lb_tmp ;
    if( ub_tmp  != 0 ) delete[] ub_tmp ;
    if( lbA_tmp != 0 ) delete[] lbA_tmp;
    if( ubA_tmp != 0 ) delete[] ubA_tmp;

    return returnvalue;
}



returnValue QPsolver_qpOASES::step(	const double* const H,
									const double* const A,
									const double* const g,
									const double* const lb,
									const double* const ub,
									const double* const lbA,
									const double* const ubA
									)
{
	/* perform a single QP iteration */
	return solve( H,A,g,lb,ub,lbA,ubA,1 );
}


returnValue QPsolver_qpOASES::step(	const Matrix *H,
                                     const Matrix *A,
                                     const Vector *g,
                                     const Vector *lb,
                                     const Vector *ub,
                                     const Vector *lbA,
                                     const Vector *ubA
									)
{
	/* perform a single QP iteration */
	return solve( H,A,g,lb,ub,lbA,ubA,1 );
}


returnValue QPsolver_qpOASES::getPrimalSolution( Vector& xOpt ) const
{
	if ( qp == 0 )
		return ACADOERROR( RET_INITIALIZE_FIRST );

	uint dim = qp->getNV( );
	double* xOptTmp = new double[dim];

	if ( qp->getPrimalSolution( xOptTmp ) == qpOASES::SUCCESSFUL_RETURN )
	{
		xOpt.init( dim,xOptTmp );
		delete[] xOptTmp;
		return SUCCESSFUL_RETURN;
	}
	else
	{
		delete[] xOptTmp;
		return ACADOERROR( RET_QP_NOT_SOLVED );
	}
}


returnValue QPsolver_qpOASES::getDualSolution( Vector& yOpt ) const
{
	if ( qp == 0 )
		return ACADOERROR( RET_INITIALIZE_FIRST );

	uint dim = qp->getNV( ) + qp->getNC( );
	double* yOptTmp = new double[dim];

	if ( qp->getDualSolution( yOptTmp ) == qpOASES::SUCCESSFUL_RETURN )
	{
		yOpt.init( dim,yOptTmp );
		delete[] yOptTmp;
		return SUCCESSFUL_RETURN;
	}
	else
	{
		delete[] yOptTmp;
		return ACADOERROR( RET_QP_NOT_SOLVED );
	}
}


double QPsolver_qpOASES::getObjVal( ) const
{
	if ( isUnbounded( ) == BT_TRUE )
		return -INFTY;

	if ( ( isSolved( ) == BT_FALSE ) || ( qp == 0 ) )
		return INFTY;

	return qp->getObjVal( );
}


returnValue QPsolver_qpOASES::getVarianceCovariance( Matrix &var ){

    return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
}


uint QPsolver_qpOASES::getNumberOfVariables( ) const
{
	return qp->getNV( );
}

uint QPsolver_qpOASES::getNumberOfConstraints( ) const
{
	return qp->getNC( );
}


returnValue QPsolver_qpOASES::getVarianceCovariance( Matrix &H, Matrix &var ){

    if ( qp == 0 )
        return ACADOERROR( RET_INITIALIZE_FIRST );

    if ( isSolved( ) == BT_FALSE ) return ACADOERROR( RET_QP_NOT_SOLVED );

    qpOASES::returnValue      returnvalue;
    qpOASES::SolutionAnalysis analyser   ;

    uint             NV, NC     ;
    uint             run1, run2 ;

    NV = qp->getNV();
    NC = qp->getNC();

    double *Var            = new double[(2*NV+NC)*(2*NV+NC)];
    double *PrimalDualVar  = new double[(2*NV+NC)*(2*NV+NC)];

    for( run1 = 0; run1 < (2*NV+NC)*(2*NV+NC); run1++ )
        Var[run1] = 0.0;

    for( run1 = 0; run1 < NV; run1++ )
        for( run2 = 0; run2 < NV; run2++ )
            Var[run1*(2*NV+NC)+run2] = H(run1,run2);

    returnvalue = analyser.getVarianceCovariance( qp, Var, PrimalDualVar );

    if( returnvalue != qpOASES::SUCCESSFUL_RETURN ){
        delete[] Var          ;
        delete[] PrimalDualVar;
        return ACADOERROR(RET_QP_NOT_SOLVED);
    }

    var.init( NV, NV );

    for( run1 = 0; run1 < NV; run1++ )
        for( run2 = 0; run2 < NV; run2++ )
            var( run1, run2 ) = PrimalDualVar[run1*(2*NV+NC)+run2];

    delete[] Var          ;
    delete[] PrimalDualVar;
    return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue QPsolver_qpOASES::setupQPobject( uint nV, uint nC )
{
	if ( qp != 0 )
		delete qp;

	/* create new qpOASES QP object... */
	qp = new qpOASES::SQProblem( nV,nC );
	
	/* ... and define its printLevel */
	int printLevel = 0;
	//get( PRINTLEVEL,printLevel );

	switch( (PrintLevel) printLevel )
	{
		case HIGH:
			qp->setPrintLevel( qpOASES::PL_MEDIUM );

		case DEBUG:
			qp->setPrintLevel( qpOASES::PL_HIGH );

		// PL_NONE, PL_LOW, PL_MEDIUM
		default:
			qp->setPrintLevel( qpOASES::PL_NONE );
	}

	qpStatus = QPS_INITIALIZED;

	return SUCCESSFUL_RETURN;
}


returnValue QPsolver_qpOASES::updateQPstatus( qpOASES::returnValue returnvalue )
{
	switch ( returnvalue )
	{
		case qpOASES::SUCCESSFUL_RETURN:
			qpStatus = QPS_SOLVED;
			return SUCCESSFUL_RETURN;

		case qpOASES::RET_MAX_NWSR_REACHED:
			qpStatus = QPS_NOTSOLVED;
			return RET_QP_SOLUTION_REACHED_LIMIT;

		default:
			/* check for infeasibility */
			if ( qp->isInfeasible( ) == qpOASES::BT_TRUE )
			{
				qpStatus = QPS_INFEASIBLE;
				return RET_QP_INFEASIBLE;
			}

			/* check for unboundedness */
			if ( qp->isUnbounded( ) == qpOASES::BT_TRUE )
			{
				qpStatus = QPS_UNBOUNDED;
				return RET_QP_UNBOUNDED;
			}

			return RET_QP_SOLUTION_FAILED;
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
