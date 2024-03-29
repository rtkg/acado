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
*    \file src/symbolic_operator/subtraction.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO



Subtraction::Subtraction():BinaryOperator(){}

Subtraction::Subtraction( Operator *_argument1, Operator *_argument2 )
            :BinaryOperator( _argument1, _argument2 ){

}

Subtraction::Subtraction( const Subtraction &arg ):BinaryOperator( arg ){ }

Subtraction::~Subtraction(){ }



Subtraction& Subtraction::operator=( const Subtraction &arg ){

    if( this != &arg ){

        BinaryOperator::operator=( arg );
    }
    return *this;
}


returnValue Subtraction::evaluate( int number, double *x, double *result ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument1_result  = (double*)realloc( argument1_result,bufferSize*sizeof(double));
        argument2_result  = (double*)realloc( argument2_result,bufferSize*sizeof(double));
        dargument1_result = (double*)realloc(dargument1_result,bufferSize*sizeof(double));
        dargument2_result = (double*)realloc(dargument2_result,bufferSize*sizeof(double));
    }

    argument1->evaluate( number, x , &argument1_result[number] );
    argument2->evaluate( number, x , &argument2_result[number] );

    result[0] = argument1_result[number] - argument2_result[number];

    return SUCCESSFUL_RETURN;
}



Operator* Subtraction::differentiate( int index ){

  dargument1 = argument1->differentiate( index );
  dargument2 = argument2->differentiate( index );
  if ( dargument1->isOneOrZero() == NE_ZERO && dargument2->isOneOrZero() == NE_ZERO ){
    return new DoubleConstant( 0.0 , NE_ZERO );
  }
  if ( dargument1->isOneOrZero() == NE_ZERO ){
    return new Subtraction( new DoubleConstant( 0.0 , NE_ZERO ), dargument2->clone() );
  }
  if ( dargument2->isOneOrZero() == NE_ZERO ){
    return dargument1->clone();
  }
  return new Subtraction( dargument1->clone() , dargument2->clone() );

}


Operator* Subtraction::AD_forward( int dim,
                                     VariableType *varType,
                                     int *component,
                                     Operator **seed,
                                     int &nNewIS,
                                     TreeProjection ***newIS ){

    if( dargument1 != 0 )
        delete dargument1;

    if( dargument2 != 0 )
        delete dargument2;

    dargument1 = argument1->AD_forward(dim,varType,component,seed,nNewIS,newIS);
    dargument2 = argument2->AD_forward(dim,varType,component,seed,nNewIS,newIS);

    if ( dargument1->isOneOrZero() == NE_ZERO && dargument2->isOneOrZero() == NE_ZERO ){
        return new DoubleConstant( 0.0 , NE_ZERO );
    }
    if ( dargument1->isOneOrZero() == NE_ZERO ){
        return new Subtraction( new DoubleConstant( 0.0 , NE_ZERO ), dargument2->clone() );
    }
    if ( dargument2->isOneOrZero() == NE_ZERO ){
        return dargument1->clone();
    }

    return new Subtraction( dargument1->clone() , dargument2->clone() );
}


returnValue Subtraction::AD_backward( int dim,
                                      VariableType *varType,
                                      int *component,
                                      Operator *seed,
                                      Operator **df         ){

    TreeProjection tmp;
    tmp = *seed;

    argument1->AD_backward( dim, varType, component, tmp.clone(), df );

    if( seed->isOneOrZero() != NE_ZERO ){
        argument2->AD_backward( dim, varType, component,
                                new Subtraction( new DoubleConstant( 0.0 , NE_ZERO ), tmp.clone() ), df );
    }

    delete seed;
    return SUCCESSFUL_RETURN;
}


Operator* Subtraction::substitute( int index, const Operator *sub ){

    return new Subtraction( argument1->substitute( index , sub ),
                            argument2->substitute( index , sub ) );

}



NeutralElement Subtraction::isOneOrZero() const{

    if ( argument1->isOneOrZero() == NE_ONE && argument2->isOneOrZero() == NE_ZERO ){
        return NE_ONE;
    }
    if ( argument1->isOneOrZero() == NE_ZERO && argument2->isOneOrZero() == NE_ZERO ){
      return NE_ZERO;
    }
    return NE_NEITHER_ONE_NOR_ZERO;

}


BooleanType Subtraction::isLinearIn( int dim,
                                       VariableType *varType,
                                       int *component,
                                       BooleanType   *implicit_dep ){

    if( argument1->isLinearIn( dim, varType, component, implicit_dep ) == BT_TRUE &&
        argument2->isLinearIn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Subtraction::isPolynomialIn( int dim,
                                           VariableType *varType,
                                           int *component,
                                           BooleanType   *implicit_dep ){

    if( argument1->isPolynomialIn( dim, varType, component, implicit_dep ) == BT_TRUE &&
        argument2->isPolynomialIn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Subtraction::isRationalIn( int dim,
                                         VariableType *varType,
                                         int *component,
                                         BooleanType   *implicit_dep ){

    if( argument1->isRationalIn( dim, varType, component, implicit_dep ) == BT_TRUE &&
        argument2->isRationalIn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


MonotonicityType Subtraction::getMonotonicity( ){

    if( monotonicity != MT_UNKNOWN )  return monotonicity;

    MonotonicityType m1, m2;

    m1 = argument1->getMonotonicity();
    m2 = argument2->getMonotonicity();

    if( m1 == MT_CONSTANT ){

        if( m2 == MT_CONSTANT      )  return MT_CONSTANT     ;
        if( m2 == MT_NONDECREASING )  return MT_NONINCREASING;
        if( m2 == MT_NONINCREASING )  return MT_NONDECREASING;

        return MT_NONMONOTONIC;
    }

    if( m1 == MT_NONDECREASING ){

        if( m2 == MT_CONSTANT      )  return MT_NONDECREASING;
        if( m2 == MT_NONINCREASING )  return MT_NONDECREASING;

        return MT_NONMONOTONIC;
    }

    if( m1 == MT_NONINCREASING ){

        if( m2 == MT_CONSTANT      )  return MT_NONINCREASING;
        if( m2 == MT_NONDECREASING )  return MT_NONINCREASING;

        return MT_NONMONOTONIC;
    }

    return MT_NONMONOTONIC;
}


CurvatureType Subtraction::getCurvature( ){

    if( curvature != CT_UNKNOWN )  return curvature;

    CurvatureType c1, c2;

    c1 = argument1->getCurvature();
    c2 = argument2->getCurvature();

    if( c1 == CT_CONSTANT ){

        if( c2 == CT_CONSTANT )  return CT_CONSTANT;
        if( c2 == CT_AFFINE   )  return CT_AFFINE  ;
        if( c2 == CT_CONVEX   )  return CT_CONCAVE ;
        if( c2 == CT_CONCAVE  )  return CT_CONVEX  ;
    }

    if( c1 == CT_AFFINE ){

        if( c2 == CT_CONSTANT )  return CT_AFFINE  ;
        if( c2 == CT_AFFINE   )  return CT_AFFINE  ;
        if( c2 == CT_CONVEX   )  return CT_CONCAVE ;
        if( c2 == CT_CONCAVE  )  return CT_CONVEX  ;
    }

    if( c1 == CT_CONVEX ){

        if( c2 == CT_CONSTANT )  return CT_CONVEX  ;
        if( c2 == CT_AFFINE   )  return CT_CONVEX  ;
        if( c2 == CT_CONCAVE  )  return CT_CONVEX  ;

        return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }

    if( c1 == CT_CONCAVE ){

        if( c2 == CT_CONSTANT )  return CT_CONCAVE ;
        if( c2 == CT_AFFINE   )  return CT_CONCAVE ;
        if( c2 == CT_CONVEX   )  return CT_CONCAVE ;

        return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }

    return CT_NEITHER_CONVEX_NOR_CONCAVE;
}


returnValue Subtraction::AD_forward( int number, double *x, double *seed,
                                     double *f, double *df ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument1_result  = (double*)realloc( argument1_result,bufferSize*sizeof(double));
        argument2_result  = (double*)realloc( argument2_result,bufferSize*sizeof(double));
        dargument1_result = (double*)realloc(dargument1_result,bufferSize*sizeof(double));
        dargument2_result = (double*)realloc(dargument2_result,bufferSize*sizeof(double));
    }

    argument1->AD_forward( number, x, seed, &argument1_result[number],
                           &dargument1_result[number] );
    argument2->AD_forward( number, x, seed, &argument2_result[number],
                           &dargument2_result[number] );

      f[0] =  argument1_result[number] -  argument2_result[number];
     df[0] = dargument1_result[number] - dargument2_result[number];

     return SUCCESSFUL_RETURN;
}


returnValue Subtraction::AD_forward( int number, double *seed, double *df ){

    argument1->AD_forward( number, seed, &dargument1_result[number] );
    argument2->AD_forward( number, seed, &dargument2_result[number] );

     df[0] = dargument1_result[number] - dargument2_result[number];

     return SUCCESSFUL_RETURN;
}


returnValue Subtraction::AD_backward( int number, double seed, double *df ){

    argument1->AD_backward( number,  seed, df );
    argument2->AD_backward( number, -seed, df );

    return SUCCESSFUL_RETURN;
}


returnValue Subtraction::AD_forward2( int number, double *seed, double *dseed,
                                      double *df, double *ddf ){

    double      ddargument1_result;
    double      ddargument2_result;
    double      dargument_result1 ;
    double      dargument_result2 ;

    argument1->AD_forward2( number, seed, dseed,
                            &dargument_result1, &ddargument1_result);
    argument2->AD_forward2( number, seed, dseed,
                            &dargument_result2, &ddargument2_result);

     df[0] =  dargument_result1         -  dargument_result2   ;
    ddf[0] = ddargument1_result         - ddargument2_result   ;

    return SUCCESSFUL_RETURN;
}


returnValue Subtraction::AD_backward2( int number, double seed1, double seed2,
                                       double *df, double *ddf ){

    argument1->AD_backward2( number,  seed1,  seed2, df, ddf );
    argument2->AD_backward2( number, -seed1, -seed2, df, ddf );

    return SUCCESSFUL_RETURN;
}


Stream Subtraction::print( Stream &stream ) const{

    return stream << "(" << *argument1 << "-" << *argument2 << ")";
}


Operator* Subtraction::clone() const{

    return new Subtraction(*this);
}


//
// PROTECTED MEMBER FUNCTIONS:
// ---------------------------

OperatorName Subtraction::getName(){

    return ON_SUBTRACTION;
}


CLOSE_NAMESPACE_ACADO

// end of file.
