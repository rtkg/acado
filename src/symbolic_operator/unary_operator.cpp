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
 *    \file   src/symbolic_operator/unary_operator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


BEGIN_NAMESPACE_ACADO


UnaryOperator::UnaryOperator() : SmoothOperator( )
{
    fcn   = 0;
    dfcn  = 0;
    ddfcn = 0;
    nCount = 0;
}

UnaryOperator::UnaryOperator( Operator *_argument ) : SmoothOperator( )
{
    fcn   = 0;
    dfcn  = 0;
    ddfcn = 0;

    argument          = _argument                        ;
    dargument         = NULL                             ;
    argument_result   = (double*)calloc(1,sizeof(double));
    dargument_result  = (double*)calloc(1,sizeof(double));
    bufferSize        = 1                                ;
    curvature         = CT_UNKNOWN                       ;
    monotonicity      = MT_UNKNOWN                       ;

    nCount = 0;
}


UnaryOperator::UnaryOperator( const UnaryOperator &arg ){

    int run1;

    fcn   = 0;
    dfcn  = 0;
    ddfcn = 0;

    bufferSize       = arg.bufferSize;
    argument         = arg.argument->clone();

    if( arg.dargument == 0 ) dargument = 0;
    else                     dargument = arg.dargument->clone();

    argument_result  = (double*)calloc(bufferSize,sizeof(double));
    dargument_result = (double*)calloc(bufferSize,sizeof(double));

    for( run1 = 0; run1 < bufferSize; run1++ ){

        argument_result[run1] = arg.argument_result[run1];
        dargument_result[run1] = arg.dargument_result[run1];
    }

    curvature    = arg.curvature   ;
    monotonicity = arg.monotonicity;
    cName        = arg.cName       ;

    nCount = 0;
}


UnaryOperator::~UnaryOperator(){

    if(  argument != 0 ) delete  argument;
    if( dargument != 0 ) delete dargument;

    free(  argument_result );
    free( dargument_result );
}


UnaryOperator& UnaryOperator::operator=( const UnaryOperator &arg ){

    if( this != &arg ){

        if(  argument != 0 ) delete  argument;
        if( dargument != 0 ) delete dargument;

        free(  argument_result );
        free( dargument_result );

        argument          = arg.argument->clone()              ;
        dargument         = NULL                               ;
        bufferSize        = arg.bufferSize                     ;
        argument_result   = (double*)calloc(bufferSize,sizeof(double))  ;
        dargument_result  = (double*)calloc(bufferSize,sizeof(double))  ;

        curvature    = arg.curvature   ;
        monotonicity = arg.monotonicity;
        cName        = arg.cName       ;

        nCount = 0;
    }
    return *this;
}


returnValue UnaryOperator::evaluate( int number, double *x, double *result ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument_result  = (double*)realloc( argument_result,bufferSize*sizeof(double));
        dargument_result = (double*)realloc(dargument_result,bufferSize*sizeof(double));
    }
    argument->evaluate( number, x , &argument_result[number] );
    result[0] = (*fcn)( argument_result[number] );
    return SUCCESSFUL_RETURN;

}


Operator* UnaryOperator::AD_forward( int                dim      ,
                                       VariableType      *varType  ,
                                       int               *component,
                                       Operator       **seed     ,
                                       int                &nNewIS  ,
                                       TreeProjection ***newIS   ){

    return ADforwardProtected( dim, varType, component, seed, nNewIS, newIS );
}


returnValue UnaryOperator::AD_backward( int           dim      ,
                                        VariableType *varType  ,
                                        int          *component,
                                        Operator   *seed     ,
                                        Operator  **df         ){

    return ADbackwardProtected( dim, varType, component, seed, df );
}


NeutralElement UnaryOperator::isOneOrZero() const{

    if ( argument->isOneOrZero() == NE_ZERO ){
        return NE_ZERO;
    }
    return NE_NEITHER_ONE_NOR_ZERO;

}


BooleanType UnaryOperator::isDependingOn( VariableType var ) const{

    return argument->isDependingOn(var);
}


BooleanType UnaryOperator::isDependingOn( int dim,
                                  VariableType *varType,
                                  int *component,
                                  BooleanType   *implicit_dep ){

    return argument->isDependingOn( dim, varType, component, implicit_dep );

}


BooleanType UnaryOperator::isLinearIn( int dim,
                               VariableType *varType,
                               int *component,
                               BooleanType   *implicit_dep ){

    if( argument->isDependingOn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_FALSE;
    }

    return BT_TRUE;
}


BooleanType UnaryOperator::isPolynomialIn( int dim,
                                   VariableType *varType,
                                   int *component,
                                   BooleanType   *implicit_dep ){

    if( argument->isDependingOn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_FALSE;
    }

    return BT_TRUE;
}


BooleanType UnaryOperator::isRationalIn( int dim,
                                 VariableType *varType,
                                 int *component,
                                 BooleanType   *implicit_dep ){

    if( argument->isDependingOn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_FALSE;
    }

    return BT_TRUE;
}


MonotonicityType UnaryOperator::getMonotonicity( ){

    if( monotonicity                != MT_UNKNOWN  )  return monotonicity;
    if( argument->getMonotonicity() == MT_CONSTANT )  return MT_CONSTANT ;

    return MT_NONMONOTONIC;
}


CurvatureType UnaryOperator::getCurvature( ){

    if( curvature                != CT_UNKNOWN  )  return curvature  ;
    if( argument->getCurvature() == CT_CONSTANT )  return CT_CONSTANT;

    return CT_NEITHER_CONVEX_NOR_CONCAVE;
}


returnValue UnaryOperator::setMonotonicity( MonotonicityType monotonicity_ ){

    monotonicity = monotonicity_;
    return SUCCESSFUL_RETURN;
}


returnValue UnaryOperator::setCurvature( CurvatureType curvature_ ){

    curvature = curvature_;
    return SUCCESSFUL_RETURN;
}


returnValue UnaryOperator::AD_forward( int number, double *x, double *seed,
                              double *f, double *df ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument_result  = (double*)realloc( argument_result,bufferSize*sizeof(double));
        dargument_result = (double*)realloc(dargument_result,bufferSize*sizeof(double));
    }
    argument->AD_forward( number, x, seed, &argument_result[number],
                          &dargument_result[number] );

    f[0]  =  (*fcn)( argument_result[number] );
    df[0] =  (*dfcn)(argument_result[number])*dargument_result[number];

     return SUCCESSFUL_RETURN;
}



returnValue UnaryOperator::AD_forward( int number, double *seed, double *df ){


    argument->AD_forward( number, seed, &dargument_result[number] );

    df[0] =  (*dfcn)(argument_result[number])*dargument_result[number];

     return SUCCESSFUL_RETURN;
}


returnValue UnaryOperator::AD_backward( int number, double seed, double *df ){
  return argument->AD_backward( number, (*dfcn)(argument_result[number])*seed, df );
}


returnValue UnaryOperator::AD_forward2( int number, double *seed, double *dseed,
                              double *df, double *ddf ){

    double      ddargument_result;
    double      dargument_result2;

    argument->AD_forward2( number, seed, dseed,
                           &dargument_result2, &ddargument_result);

    const double nn = (*dfcn)(argument_result[number]);

     df[0] = nn*dargument_result2;
    ddf[0] = nn*ddargument_result
      +(*ddfcn)( argument_result[number] )
              *dargument_result2*dargument_result[number];

    return SUCCESSFUL_RETURN;
}


returnValue UnaryOperator::AD_backward2( int number, double seed1, double seed2,
                               double *df, double *ddf ){

  const double nn = (*dfcn)(argument_result[number]);

    argument->AD_backward2( number   ,
                            seed1*nn ,
                            seed2*nn +
                            seed1*(*ddfcn)(argument_result[number])*dargument_result[number],
                            df, ddf );

    return SUCCESSFUL_RETURN;
}


Stream UnaryOperator::print( Stream &stream ) const{

    return stream << "(" << cName << "(" << *argument << "))";
}


BooleanType UnaryOperator::isVariable( VariableType &varType, int &component ) const
{
    return BT_FALSE;
}

returnValue UnaryOperator::clearBuffer(){

    if( bufferSize > 1 ){
        bufferSize = 1;
        argument_result  = (double*)realloc( argument_result,bufferSize*sizeof(double));
        dargument_result = (double*)realloc(dargument_result,bufferSize*sizeof(double));
    }

    return SUCCESSFUL_RETURN;
}



returnValue UnaryOperator::enumerateVariables( SymbolicIndexList *indexList ){

    return argument->enumerateVariables( indexList );
}


// //
// // PROTECTED MEMBER FUNCTIONS:
// // ---------------------------


OperatorName UnaryOperator::getName(){

  return operatorName;
}


returnValue UnaryOperator::loadIndices( SymbolicIndexList *indexList ){

    return argument->loadIndices( indexList );
}


BooleanType UnaryOperator::isSymbolic() const{

    if( argument->isSymbolic() == BT_FALSE ) return BT_FALSE;
    return BT_TRUE;
}


CLOSE_NAMESPACE_ACADO

// end of file.
