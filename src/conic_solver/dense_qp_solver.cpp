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
 *    \file src/conic_solver/dense_qp_solver.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date   2010
 */


#include <acado/conic_solver/dense_qp_solver.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DenseQPsolver::DenseQPsolver( ) : DenseCPsolver( )
{
	setupLogging( );
	
    qpStatus = QPS_NOT_INITIALIZED;
    numberOfSteps = 0;
}


DenseQPsolver::DenseQPsolver( UserInteraction* _userInteraction ) : DenseCPsolver( _userInteraction )
{
	setupLogging( );
	
    qpStatus = QPS_NOT_INITIALIZED;
    numberOfSteps = 0;
}


DenseQPsolver::DenseQPsolver( const DenseQPsolver& rhs ) : DenseCPsolver( rhs )
{
    qpStatus = rhs.qpStatus;
    numberOfSteps = rhs.numberOfSteps;
}


DenseQPsolver::~DenseQPsolver( ){

}


DenseQPsolver& DenseQPsolver::operator=( const DenseQPsolver& rhs ){

    if ( this != &rhs ){

        DenseCPsolver::operator=( rhs );

        qpStatus = rhs.qpStatus;
    }
    return *this;
}


returnValue DenseQPsolver::init( uint nV, uint nC ){

	//printf( "nV: %d,   nC: %d!!\n",nV,nC );
    return setupQPobject( nV,nC );
}


returnValue DenseQPsolver::init( const DenseCP *cp ){

    ASSERT( cp != 0 );

    if( cp->isQP() == BT_FALSE )
        return ACADOERROR( RET_QP_SOLVER_CAN_ONLY_SOLVE_QP );

    return init( cp->getNV(), cp->getNC() );
}


//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue DenseQPsolver::setupLogging( )
{
  	//printf( "DenseQPsolver::setupLogging( ) called!\n" );
	LogRecord tmp( LOG_AT_EACH_ITERATION,stdout,PS_DEFAULT );

	tmp.addItem( LOG_NUM_QP_ITERATIONS );
	tmp.addItem( LOG_IS_QP_RELAXED );

	addLogRecord( tmp );
  
	return SUCCESSFUL_RETURN;
}


returnValue DenseQPsolver::solveCP( DenseCP *cp )
{
    ASSERT( cp != 0 );

    if( cp->isQP() == BT_FALSE )
        return ACADOERROR( RET_QP_SOLVER_CAN_ONLY_SOLVE_QP );


    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // QUICK HACK: SOLVE CALL SHOULD AVOID PASSING THE MAX-ITER ARGUMENT !!!


    const uint maxIter = 1000;

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    returnValue returnvalue;
    returnvalue = solve( &cp->H, &cp->A, &cp->g, &cp->lb, &cp->ub, &cp->lbA, &cp->ubA, maxIter );

    if( returnvalue != SUCCESSFUL_RETURN )
        return returnvalue;



    // GET THE PRIMAL AND DUAL SOLUTION FROM THE QP SOLVER AND
    // STORE THEM IN THE RIGHT FORMAT:
    // -------------------------------------------------------
    Vector xOpt, yOpt;

    getPrimalSolution( xOpt );
    getDualSolution  ( yOpt );
// 	printf( "DeltaU0 = [ %e, %e ]\n", xOpt(4+0),xOpt(4+1) );

// 	cp->lb.print("lb");

    cp->setQPsolution( xOpt, yOpt );
	
    return SUCCESSFUL_RETURN;
}


returnValue DenseQPsolver::convertQPdata( 	const Matrix *H,
											const Matrix *A,
											const Vector *g,
											const Vector *lb,
											const Vector *ub,
											const Vector *lbA,
											const Vector *ubA,
											double** H_tmp,
											double** A_tmp,
											double** g_tmp,
											double** lb_tmp,
											double** ub_tmp,
											double** lbA_tmp,
											double** ubA_tmp
											) const
{
    if( H == 0 ) *H_tmp = 0;
    else{
        *H_tmp = new double[H->getDim()];
        H->convert(*H_tmp);
    }
    if( A == 0 ) *A_tmp = 0;
    else{
        *A_tmp = new double[A->getDim()];
        A->convert(*A_tmp);
    }
    if( g == 0 ) *g_tmp = 0;
    else{
        *g_tmp = new double[g->getDim()];
        g->convert(*g_tmp);
    }
    if( lb == 0 ) *lb_tmp = 0;
    else{
        *lb_tmp = new double[lb->getDim()];
        lb->convert(*lb_tmp);
    }
    if( ub == 0 ) *ub_tmp = 0;
    else{
        *ub_tmp = new double[ub->getDim()];
        ub->convert(*ub_tmp);
    }
    if( lbA == 0 ) *lbA_tmp = 0;
    else{
        *lbA_tmp = new double[lbA->getDim()];
        lbA->convert(*lbA_tmp);
    }
    if( ubA == 0 ) *ubA_tmp = 0;
    else{
        *ubA_tmp = new double[ubA->getDim()];
        ubA->convert(*ubA_tmp);
    }

	return SUCCESSFUL_RETURN;
}


uint DenseQPsolver::getNumberOfIterations( ) const{

    return numberOfSteps;
}



CLOSE_NAMESPACE_ACADO

// end of file.
