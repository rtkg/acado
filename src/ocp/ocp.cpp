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
 *    \file src/ocp/ocp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008-2010
 */

#include <acado/ocp/ocp.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


OCP::OCP( const double &tStart_, const double &tEnd_, const int &N_ )
    :MultiObjectiveFunctionality(){

    setupGrid( tStart_, tEnd_, N_+1 );
}


OCP::OCP( const Grid &grid_ )
    :MultiObjectiveFunctionality(){

    grid = grid_;
    objective.init ( grid );
    constraint.init( grid );
}


OCP::OCP( const double    &tStart_,
          const Parameter &tEnd_  ,
          const int       &N_       )
    :MultiObjectiveFunctionality(){

    setupGrid( tStart_, tStart_ + 1.0, N_+1);
}


void OCP::copy( const OCP &rhs ){

    QQ = rhs.QQ;
    RR = rhs.RR;
    QF = rhs.QF;

    grid                 = rhs.grid                ;
    objective            = rhs.objective           ;
    differentialEquation = rhs.differentialEquation;
    constraint           = rhs.constraint          ;
}

OCP::OCP( const OCP& rhs )
    :MultiObjectiveFunctionality( rhs ){

    copy( rhs );
}


OCP::~OCP( ){ }


OCP& OCP::operator=( const OCP& rhs ){

    if ( this != &rhs ){

        MultiObjectiveFunctionality::operator=(rhs);
        copy(rhs);
    }
    return *this;
}



returnValue OCP::minimizeMayerTerm( const int &multiObjectiveIdx,  const Expression& arg ){

    return MultiObjectiveFunctionality::minimizeMayerTerm( multiObjectiveIdx, arg );
}


returnValue OCP::minimizeLSQ( const MatrixVariablesGrid &S,
                              const Function            &h,
                              const char*        rFilename ){

    VariablesGrid r = readFromFile( rFilename );

    if( r.isEmpty() == BT_TRUE )
        return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

    return minimizeLSQ( S,h,r );
}


returnValue OCP::minimizeLSQ( const Matrix        &S,
                              const Function      &h,
                              const char*  rFilename  ){

    VariablesGrid r = readFromFile( rFilename );

    if( r.isEmpty() == BT_TRUE )
        return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

    return minimizeLSQ( S,h,r );
}


returnValue OCP::minimizeLSQ( const Function      &h,
                              const char*  rFilename  ){

    VariablesGrid r = readFromFile( rFilename );

    if( r.isEmpty() == BT_TRUE )
        return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

    return minimizeLSQ( h,r );
}


returnValue OCP::subjectTo( const DifferentialEquation& differentialEquation_ ){

    differentialEquation = differentialEquation_;
    return SUCCESSFUL_RETURN;
}



returnValue OCP::subjectTo( const ConstraintComponent& component ){

    for( uint i=0; i<component.getDim(); ++i )
        constraint.add( component(i) );

    return SUCCESSFUL_RETURN;
}


returnValue OCP::subjectTo( const int index_, const ConstraintComponent& component ){

    for( uint i=0; i<component.getDim(); ++i )
        constraint.add( index_,component(i) );

    return SUCCESSFUL_RETURN;
}


returnValue OCP::subjectTo( const TimeHorizonElement index_, const ConstraintComponent& component ){

    uint i;

    switch( index_ ){

        case AT_START:
             for( i = 0; i < component.getDim(); i++ )
                 ACADO_TRY( constraint.add( 0,component(i) ) );
             return SUCCESSFUL_RETURN;

        case AT_END:
             for( i = 0; i < component.getDim(); i++ )
                 ACADO_TRY( constraint.add( grid.getLastIndex(),component(i) ) );
             return SUCCESSFUL_RETURN;

        default:
             return ACADOERROR(RET_UNKNOWN_BUG);
    }
    return SUCCESSFUL_RETURN;
}


returnValue OCP::subjectTo( const double lb_, const Expression& arg1,
                                   const Expression& arg2, const double ub_ ){

    return constraint.add( lb_, arg1, arg2, ub_ );
}


returnValue OCP::subjectTo( const double lb_, const Expression *arguments, const double ub_ ){

    return constraint.add( lb_, arguments, ub_ );
}


returnValue OCP::minimizeLSQ( const Matrix &Q_, const Matrix &R_ ){ QQ = Q_; RR = R_; return SUCCESSFUL_RETURN; }
returnValue OCP::minimizeLSQEndTerm( const Matrix &S_ ){ QF = S_; return SUCCESSFUL_RETURN;}

returnValue OCP::getQRS( Matrix &Q_, Matrix &R_, Matrix &S_ ) const{

    Q_ = QQ; R_ = RR; S_ = QF;
    return SUCCESSFUL_RETURN;
}


returnValue OCP::minimizeMayerTerm   ( const Expression& arg ){ return objective.addMayerTerm   ( arg ); }
returnValue OCP::maximizeMayerTerm   ( const Expression& arg ){ return objective.addMayerTerm   (-arg ); }
returnValue OCP::minimizeLagrangeTerm( const Expression& arg ){ return objective.addLagrangeTerm( arg ); }
returnValue OCP::maximizeLagrangeTerm( const Expression& arg ){ return objective.addLagrangeTerm(-arg ); }


returnValue OCP::minimizeLSQ( const Matrix&S, const Function &h, const Vector &r ){

    MatrixVariablesGrid tmpS(S);
    VariablesGrid       tmpR(r);

    return objective.addLSQ( &tmpS, h, &tmpR );
}

returnValue OCP::minimizeLSQ( const Function &h, const Vector &r ){

    Matrix S( h.getDim( ),h.getDim( ) );
    S.setIdentity( );

    return minimizeLSQ( S, h, r );
}

returnValue OCP::minimizeLSQ( const Function &h ){

    Matrix S( h.getDim( ),h.getDim( ) );
    S.setIdentity( );

    Vector r(h.getDim());
    r.setZero();

    return minimizeLSQ( S, h, r );
}


returnValue OCP::minimizeLSQ( const MatrixVariablesGrid &S,
                              const Function            &h,
                              const VariablesGrid       &r  ){

    return objective.addLSQ( &S, h, &r );
}

returnValue OCP::minimizeLSQ( const Matrix        &S,
                              const Function      &h,
                              const VariablesGrid &r ){

    MatrixVariablesGrid tmpS(S);
    return objective.addLSQ( &tmpS, h, &r );
}

returnValue OCP::minimizeLSQ( const Function      &h,
                              const VariablesGrid &r ){

    return objective.addLSQ( 0, h, &r );
}


returnValue OCP::minimizeLSQEndTerm( const Matrix   & S,
                                     const Function & m,
                                     const Vector   & r  ){

    return objective.addLSQEndTerm( S, m, r );
}

returnValue OCP::minimizeLSQEndTerm( const Function & m,
                                     const Vector   & r  ){

    Matrix S( m.getDim( ),m.getDim( ) );
    S.setIdentity( );
    return minimizeLSQEndTerm( S, m, r );
}


BooleanType OCP::hasObjective() const{

    if( objective.isEmpty() == BT_TRUE ) return BT_FALSE;
    return BT_TRUE;
}

BooleanType OCP::hasDifferentialEquation() const{

    if( differentialEquation.getDim() == 0 ) return BT_FALSE;
    return BT_TRUE;
}

BooleanType OCP::hasConstraint() const{

    if( constraint.isEmpty() == BT_TRUE ) return BT_FALSE;
    return BT_TRUE;
}


returnValue OCP::getGrid     ( Grid       & grid_      ) const{ grid_       = grid      ; return SUCCESSFUL_RETURN; }
returnValue OCP::getObjective( Objective  & objective_ ) const{ objective_  = objective ; return SUCCESSFUL_RETURN; }
returnValue OCP::getConstraint( Constraint& constraint_) const{ constraint_ = constraint; return SUCCESSFUL_RETURN; }


returnValue OCP::setObjective ( const Objective & objective_  ){ objective   = objective_; return SUCCESSFUL_RETURN; }
returnValue OCP::setConstraint( const Constraint& constraint_ ){ constraint = constraint_; return SUCCESSFUL_RETURN; }



returnValue OCP::getDifferentialEquation( DifferentialEquation &differentialEquation_ ) const{

    differentialEquation_ = differentialEquation;
    return SUCCESSFUL_RETURN;
}


returnValue OCP::getObjective( const int &multiObjectiveIdx, Expression **arg ) const{

    return MultiObjectiveFunctionality::getObjective( multiObjectiveIdx, arg );
}


double OCP::getStartTime ( ) const{ return grid.getFirstTime(); }
double OCP::getEndTime   ( ) const{ return grid.getLastTime (); }



// PROTECTED FUNCTIONS:
// --------------------
void OCP::setupGrid( double tStart, double tEnd, int N ){

    grid.init( tStart, tEnd, N );
    objective.init ( grid );
    constraint.init( grid );
}



CLOSE_NAMESPACE_ACADO

// end of file.
