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
*    \file src/symbolic_operator/powerint.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO




Power_Int::Power_Int() : SmoothOperator( )
{
    nCount = 0;
}

Power_Int::Power_Int( Operator *_argument, int _exponent ) : SmoothOperator( )
{
    argument          = _argument                        ;
    exponent          = _exponent                        ;
    dargument         = NULL                             ;
    argument_result   = (double*)calloc(1,sizeof(double));
    dargument_result  = (double*)calloc(1,sizeof(double));
    bufferSize        = 1                                ;
    curvature         = CT_UNKNOWN                       ;
    monotonicity      = MT_UNKNOWN                       ;

    nCount = 0;
}


Power_Int::Power_Int( const Power_Int &arg ){

    int run1;

    bufferSize       = arg.bufferSize;
    argument         = arg.argument->clone();
    exponent         = arg.exponent;

    if( arg.dargument == NULL ){
        dargument = NULL;
    }
    else{
        dargument = arg.dargument->clone();
    }

    argument_result  = (double*)calloc(bufferSize,sizeof(double));
    dargument_result = (double*)calloc(bufferSize,sizeof(double));

    for( run1 = 0; run1 < bufferSize; run1++ ){

        argument_result[run1] = arg.argument_result[run1];
       dargument_result[run1] = arg.dargument_result[run1];

    }
    curvature         = arg.curvature   ;
    monotonicity      = arg.monotonicity;

    nCount = 0;
}


Power_Int::~Power_Int(){

    delete argument;

    if( dargument != NULL ){
        delete dargument;
    }

    free(  argument_result );
    free( dargument_result );

}

Power_Int& Power_Int::operator=( const Power_Int &arg ){

    if( this != &arg ){

        delete argument;

        if( dargument != NULL ){
            delete dargument;
        }

        free(  argument_result );
        free( dargument_result );

        argument          = arg.argument->clone()              ;
        exponent          = arg.exponent                       ;
        dargument         = NULL                               ;
        bufferSize        = arg.bufferSize                     ;
        argument_result   = (double*)calloc(bufferSize,sizeof(double))  ;
        dargument_result  = (double*)calloc(bufferSize,sizeof(double))  ;

        curvature         = arg.curvature   ;
        monotonicity      = arg.monotonicity;

        nCount = 0;
    }

    return *this;
}


returnValue Power_Int::evaluate( int number, double *x, double *result ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument_result  = (double*)realloc( argument_result,bufferSize*sizeof(double));
        dargument_result = (double*)realloc(dargument_result,bufferSize*sizeof(double));
    }
    argument->evaluate( number, x , &argument_result[number] );

    result[0] = pow( argument_result[number], exponent );

    return SUCCESSFUL_RETURN;
}



Operator* Power_Int::differentiate( int index ){

  dargument = argument->differentiate( index );
  if ( dargument->isOneOrZero() == NE_ZERO ){
    return new DoubleConstant( 0.0 , NE_ZERO );
  }
  if ( dargument->isOneOrZero() == NE_ONE ){
  return new Product(
           new DoubleConstant(
             (double) exponent,
             NE_NEITHER_ONE_NOR_ZERO
           ),
           new Power_Int(
             argument->clone(),
             exponent-1
           )
         );
  }
  return new Product(
           new Product(
             new DoubleConstant(
               (double) exponent,
               NE_NEITHER_ONE_NOR_ZERO
             ),
             new Power_Int(
               argument->clone(),
               exponent-1
             )
           ),
           dargument->clone()
         );

}


Operator* Power_Int::AD_forward( int dim,
                                   VariableType *varType,
                                   int *component,
                                   Operator **seed,
                                   int &nNewIS,
                                   TreeProjection ***newIS ){

    if( dargument != 0 )
        delete dargument;

    dargument = argument->AD_forward(dim,varType,component,seed,nNewIS,newIS);

    if( dargument->isOneOrZero() == NE_ZERO ){
        return new DoubleConstant( 0.0 , NE_ZERO );
    }
    if( dargument->isOneOrZero() == NE_ONE ){
        return new Product(
                 new DoubleConstant(
                     (double) exponent,
                     NE_NEITHER_ONE_NOR_ZERO
                 ),
                 new Power_Int(
                     argument->clone(),
                     exponent-1
                 )
             );
    }
    return new Product(
           new Product(
             new DoubleConstant(
               (double) exponent,
               NE_NEITHER_ONE_NOR_ZERO
             ),
             new Power_Int(
               argument->clone(),
               exponent-1
             )
           ),
           dargument->clone()
         );
}



returnValue Power_Int::AD_backward( int dim,
                                    VariableType *varType,
                                    int *component,
                                    Operator *seed,
                                    Operator **df         ){


    if( seed->isOneOrZero() == NE_ZERO ){
            argument->AD_backward( dim,
                                          varType,
                                          component,
                                          new DoubleConstant( 0.0 , NE_ZERO ),
                                          df
            );

        delete seed;
        return SUCCESSFUL_RETURN;
    }
    if( seed->isOneOrZero() == NE_ONE ){
            argument->AD_backward( dim,
                                          varType,
                                          component,
                                          new Product(
                                              new DoubleConstant(
                                                  (double) exponent,
                                                  NE_NEITHER_ONE_NOR_ZERO
                                              ),
                                              new Power_Int(
                                                  argument->clone(),
                                                  exponent-1
                                              )
                                          ),
                                          df
            );
        delete seed;
        return SUCCESSFUL_RETURN;
    }
    argument->AD_backward( dim,
                                  varType,
                                  component,
                                  new Product(
                                      new Product(
                                          new DoubleConstant(
                                              (double) exponent,
                                              NE_NEITHER_ONE_NOR_ZERO
                                          ),
                                          new Power_Int(
                                              argument->clone(),
                                              exponent-1
                                          )
                                      ),
                                      seed->clone()
                                  ),
                                  df
            );

    delete seed;
    return SUCCESSFUL_RETURN;
}



Operator* Power_Int::substitute( int index, const Operator *sub ){

    return new Power_Int( argument->substitute( index , sub ), exponent );

}



NeutralElement Power_Int::isOneOrZero() const{

    if ( argument->isOneOrZero() == NE_ONE ){
       return NE_ONE;
    }
    if ( argument->isOneOrZero() == NE_ZERO && exponent != 0 ){
        return NE_ZERO;
    }
    if ( exponent == 0 ){
        return NE_ONE;
    }
    return NE_NEITHER_ONE_NOR_ZERO;

}


BooleanType Power_Int::isDependingOn( VariableType var ) const{

    return argument->isDependingOn(var);
}


BooleanType Power_Int::isDependingOn( int dim,
                                        VariableType *varType,
                                        int *component,
                                        BooleanType   *implicit_dep ){

    if( exponent == 0 ){
        return BT_FALSE;
    }

    return argument->isDependingOn( dim, varType, component, implicit_dep );
}


BooleanType Power_Int::isLinearIn( int dim,
                                     VariableType *varType,
                                     int *component,
                                     BooleanType   *implicit_dep ){

    if( exponent == 0 ){
        return BT_TRUE;
    }

    if( exponent == 1 && argument->isLinearIn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Power_Int::isPolynomialIn( int dim,
                                         VariableType *varType,
                                         int *component,
                                         BooleanType   *implicit_dep ){

    if(  argument->isPolynomialIn( dim, varType, component, implicit_dep )    == BT_TRUE &&
         exponent >= 0 ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Power_Int::isRationalIn( int dim,
                                       VariableType *varType,
                                       int *component,
                                       BooleanType   *implicit_dep ){

    if(  argument->isRationalIn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


MonotonicityType Power_Int::getMonotonicity( ){

    if( monotonicity != MT_UNKNOWN )  return monotonicity;

    const MonotonicityType m = argument->getMonotonicity();

    if( m == MT_CONSTANT )  return MT_CONSTANT;

    // if exponent is even:
    if( fabs( ceil( ((double) exponent)/2.0 - EPS ) - ((double) exponent)/2.0 ) < 10.0*EPS ){

        if( exponent == 0 ) return MT_CONSTANT;

        return MT_NONMONOTONIC;
    }
    else{

        if( exponent > 0  )  return m;
        return MT_NONMONOTONIC;
    }

    return MT_NONMONOTONIC;
}


CurvatureType Power_Int::getCurvature( ){

    if( curvature != CT_UNKNOWN )  return curvature;

    const CurvatureType cc = argument->getCurvature();

    if( cc == CT_CONSTANT )  return CT_CONSTANT;

    // if exponent is even:
    if( fabs( ceil( ((double) exponent)/2.0 - EPS ) - ((double) exponent)/2.0 ) < 10.0*EPS ){

        if( exponent  < 0   ) return CT_NEITHER_CONVEX_NOR_CONCAVE;
        if( exponent == 0   ) return CT_CONSTANT;
        if( cc == CT_AFFINE ) return CT_CONVEX  ;

        return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }
    else{

        if( exponent == 1 ) return cc;
        return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }

    return CT_NEITHER_CONVEX_NOR_CONCAVE;
}


returnValue Power_Int::setMonotonicity( MonotonicityType monotonicity_ ){

    monotonicity = monotonicity_;
    return SUCCESSFUL_RETURN;
}


returnValue Power_Int::setCurvature( CurvatureType curvature_ ){

    curvature = curvature_;
    return SUCCESSFUL_RETURN;
}


returnValue Power_Int::AD_forward( int number, double *x, double *seed,
                                   double *f, double *df ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument_result  = (double*)realloc( argument_result,bufferSize*sizeof(double));
        dargument_result = (double*)realloc(dargument_result,bufferSize*sizeof(double));
    }
    argument->AD_forward( number, x, seed, &argument_result[number],
                                  &dargument_result[number] );

      f[0] =  pow( argument_result[number],exponent );
     df[0] =  exponent*pow( argument_result[number],exponent-1 )*dargument_result[number];

     return SUCCESSFUL_RETURN;
}



returnValue Power_Int::AD_forward( int number, double *seed, double *df ){


    argument->AD_forward( number, seed, &dargument_result[number] );

     df[0] =  exponent*pow( argument_result[number],exponent-1 )*dargument_result[number];

     return SUCCESSFUL_RETURN;
}


returnValue Power_Int::AD_backward( int number, double seed, double *df ){
    return argument->AD_backward( number, exponent*pow( argument_result[number],
                                  exponent-1 )*seed, df );
}


returnValue Power_Int::AD_forward2( int number, double *seed, double *dseed,
                                    double *df, double *ddf ){

    double      ddargument_result;
    double      dargument_result2;

    argument->AD_forward2( number, seed, dseed, &dargument_result2, &ddargument_result);

    const double nn = exponent*pow( argument_result[number],exponent-1 );
     df[0] = nn*dargument_result2;
    ddf[0] = nn*ddargument_result + exponent*(exponent-1)*dargument_result[number]
                                  * dargument_result2*
                                    pow( argument_result[number],exponent-2 )   ;

    return SUCCESSFUL_RETURN;
}


returnValue Power_Int::AD_backward2( int number, double seed1, double seed2,
                               double *df, double *ddf ){

    const double nn = exponent*pow(argument_result[number],exponent-1);

    argument->AD_backward2( number,
                            seed1*nn,
                            seed2*nn +
                            seed1*exponent*(exponent-1)
                                 *pow(argument_result[number],exponent-2)
                                 *dargument_result[number],
                            df, ddf );

    return SUCCESSFUL_RETURN;
}


Stream Power_Int::print( Stream &stream ) const{

    return stream << "(pow(" << *argument << "," << exponent << "))";
}


Operator* Power_Int::clone() const{

    return new Power_Int(*this);
}


returnValue Power_Int::clearBuffer(){

    if( bufferSize > 1 ){
        bufferSize = 1;
        argument_result  = (double*)realloc( argument_result,bufferSize*sizeof(double));
        dargument_result = (double*)realloc(dargument_result,bufferSize*sizeof(double));
    }

    return SUCCESSFUL_RETURN;
}


returnValue Power_Int::enumerateVariables( SymbolicIndexList *indexList ){

    return argument->enumerateVariables( indexList );
}


//
// PROTECTED MEMBER FUNCTIONS:
// ---------------------------


OperatorName Power_Int::getName(){

    return ON_POWER_INT;
}

BooleanType Power_Int::isVariable( VariableType &varType, int &component ) const
{
    return BT_FALSE;
}

returnValue Power_Int::loadIndices( SymbolicIndexList *indexList ){

    return argument->loadIndices( indexList );
}


BooleanType Power_Int::isSymbolic() const{

    if( argument->isSymbolic() == BT_FALSE ) return BT_FALSE;
    return BT_TRUE;
}


CLOSE_NAMESPACE_ACADO

// end of file.
