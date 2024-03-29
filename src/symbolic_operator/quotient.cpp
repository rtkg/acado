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
 *    \file src/symbolic_operator/quotient.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO




Quotient::Quotient():BinaryOperator(){ }

Quotient::Quotient( Operator *_argument1, Operator *_argument2 )
         :BinaryOperator( _argument1, _argument2 ){

}


Quotient::Quotient( const Quotient &arg ):BinaryOperator( arg ){

}


Quotient::~Quotient(){

}

Quotient& Quotient::operator=( const Quotient &arg ){

    if( this != &arg ){

        BinaryOperator::operator=( arg );
    }
    return *this;
}



returnValue Quotient::evaluate( int number, double *x, double *result ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument1_result  = (double*)realloc( argument1_result,bufferSize*sizeof(double));
        argument2_result  = (double*)realloc( argument2_result,bufferSize*sizeof(double));
        dargument1_result = (double*)realloc(dargument1_result,bufferSize*sizeof(double));
        dargument2_result = (double*)realloc(dargument2_result,bufferSize*sizeof(double));
    }

    argument1->evaluate( number, x , &argument1_result[number] );
    argument2->evaluate( number, x , &argument2_result[number] );

    result[0] = argument1_result[number] / argument2_result[number];

    return SUCCESSFUL_RETURN;
}



Operator* Quotient::differentiate( int index ){

  dargument1 = argument1->differentiate( index );
  dargument2 = argument2->differentiate( index );
  if ( dargument1->isOneOrZero() == NE_ZERO && dargument2->isOneOrZero() == NE_ZERO ){
    return new DoubleConstant( 0.0 , NE_ZERO );
  }
  if ( dargument1->isOneOrZero() == NE_ONE && dargument2->isOneOrZero() == NE_ZERO ){
    return new Quotient(
                   new DoubleConstant( 1.0, NE_ONE ),
                   argument2->clone()
               );
  }
  if ( dargument1->isOneOrZero() == NE_ZERO && dargument2->isOneOrZero() == NE_ONE ){
    return new Quotient(
                   new Subtraction(
                           new DoubleConstant( 0.0 , NE_ZERO ),
                           argument1->clone()
                       ),
                   new Power_Int(
                           argument2->clone(),
                           2
                       )
               );
  }
  if ( dargument1->isOneOrZero() == NE_ONE && dargument2->isOneOrZero() == NE_ONE ){
      return new Subtraction(
                     new Quotient(
                         new DoubleConstant( 1.0, NE_ONE ),
                         argument2->clone()
                     ),
                     new Quotient(
                             argument1->clone(),
                             new Power_Int(
                                     argument2->clone(),
                                     2
                                 )
                         )
                 );
  }
  if ( dargument1->isOneOrZero() == NE_ONE ){
      return new Subtraction(
                     new Quotient(
                         new DoubleConstant( 1.0, NE_ONE ),
                         argument2->clone()
                     ),
                     new Quotient(
                             new Product(
                                     argument1->clone(),
                                     dargument2->clone()
                                 ),
                             new Power_Int(
                                     argument2->clone(),
                                     2
                                 )
                         )
                  );
  }
  if ( dargument1->isOneOrZero() == NE_ZERO ){
      return new Subtraction(
                     new DoubleConstant( 0.0, NE_ZERO ),
                     new Quotient(
                             new Product(
                                     argument1->clone(),
                                     dargument2->clone()
                                 ),
                             new Power_Int(
                                     argument2->clone(),
                                     2
                                 )
                         )
                  );
  }
  if ( dargument2->isOneOrZero() == NE_ONE ){
      return new Subtraction(
                     new Quotient(
                         dargument1->clone(),
                         argument2->clone()
                     ),
                     new Quotient(
                             argument1->clone(),
                             new Power_Int(
                                     argument2->clone(),
                                     2
                                 )
                         )
                 );
  }
  if ( dargument2->isOneOrZero() == NE_ZERO ){
      return new Quotient(
                     dargument1->clone(),
                     argument2->clone()
                 );
  }
  return new Subtraction(
                   new Quotient(
                       dargument1->clone(),
                       argument2->clone()
                   ),
                   new Quotient(
                           new Product(
                                   argument1->clone(),
                                   dargument2->clone()
                               ),
                           new Power_Int(
                                   argument2->clone(),
                                   2
                               )
                       )
              );

}




Operator* Quotient::AD_forward( int dim,
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

    if ( dargument1->isOneOrZero() == NE_ONE && dargument2->isOneOrZero() == NE_ZERO ){
        return new Quotient(
                   new DoubleConstant( 1.0, NE_ONE ),
                   argument2->clone()
               );
    }
    if ( dargument1->isOneOrZero() == NE_ZERO && dargument2->isOneOrZero() == NE_ONE ){
       return new Quotient(
                   new Subtraction(
                           new DoubleConstant( 0.0 , NE_ZERO ),
                           argument1->clone()
                       ),
                   new Power_Int(
                           argument2->clone(),
                           2
                       )
               );
    }
    if ( dargument1->isOneOrZero() == NE_ONE && dargument2->isOneOrZero() == NE_ONE ){
        return new Subtraction(
                     new Quotient(
                         new DoubleConstant( 1.0, NE_ONE ),
                         argument2->clone()
                     ),
                     new Quotient(
                             argument1->clone(),
                             new Power_Int(
                                     argument2->clone(),
                                     2
                                 )
                         )
                 );
    }
    if ( dargument1->isOneOrZero() == NE_ONE ){
        return new Subtraction(
                     new Quotient(
                         new DoubleConstant( 1.0, NE_ONE ),
                         argument2->clone()
                     ),
                     new Quotient(
                             new Product(
                                     argument1->clone(),
                                     dargument2->clone()
                                 ),
                             new Power_Int(
                                     argument2->clone(),
                                     2
                                 )
                         )
                  );
    }
    if ( dargument1->isOneOrZero() == NE_ZERO ){
        return new Subtraction(
                     new DoubleConstant( 0.0, NE_ZERO ),
                     new Quotient(
                             new Product(
                                     argument1->clone(),
                                     dargument2->clone()
                                 ),
                             new Power_Int(
                                     argument2->clone(),
                                     2
                                 )
                         )
                  );
    }
    if ( dargument2->isOneOrZero() == NE_ONE ){
        return new Subtraction(
                     new Quotient(
                         dargument1->clone(),
                         argument2->clone()
                     ),
                     new Quotient(
                             argument1->clone(),
                             new Power_Int(
                                     argument2->clone(),
                                     2
                                 )
                         )
                 );
    }
    if ( dargument2->isOneOrZero() == NE_ZERO ){
        return new Quotient(
                     dargument1->clone(),
                     argument2->clone()
                 );
    }
    return new Subtraction(
                   new Quotient(
                       dargument1->clone(),
                       argument2->clone()
                   ),
                   new Quotient(
                           new Product(
                                   argument1->clone(),
                                   dargument2->clone()
                               ),
                           new Power_Int(
                                   argument2->clone(),
                                   2
                               )
                       )
              );
}


returnValue Quotient::AD_backward( int dim,
                                   VariableType *varType,
                                   int *component,
                                   Operator *seed,
                                   Operator **df         ){


    if( seed->isOneOrZero() != NE_ZERO ){

        TreeProjection tmp;
        tmp = *seed;

        argument1->AD_backward( dim, varType, component,
                                new Quotient(
                                    tmp.clone(),
                                    argument2->clone()
                                ),
                                df );

        argument2->AD_backward( dim, varType, component,
                                new Subtraction(
                                    new DoubleConstant( 0.0, NE_ZERO ),
                                    new Quotient(
                                        new Product(
                                            argument1->clone(),
                                            tmp.clone()
                                        ),
                                        new Power_Int(
                                            argument2->clone(),
                                            2
                                        )
                                    )
                                ),
                                df );
    }

    delete seed;
    return SUCCESSFUL_RETURN;
}




Operator* Quotient::substitute( int index, const Operator *sub ){

    return new Quotient( argument1->substitute( index , sub ),
                         argument2->substitute( index , sub ) );

}



NeutralElement Quotient::isOneOrZero() const{

    if ( argument1->isOneOrZero() == NE_ZERO ){
      return NE_ZERO;
    }
    if ( argument1->isOneOrZero() == NE_ONE && argument2->isOneOrZero() == NE_ONE ){
      return NE_ONE;
    }
    return NE_NEITHER_ONE_NOR_ZERO;

}



BooleanType Quotient::isLinearIn( int dim,
                                    VariableType *varType,
                                    int *component,
                                    BooleanType   *implicit_dep ){

    if(  argument1->isLinearIn( dim, varType, component, implicit_dep )    == BT_TRUE &&
         argument2->isDependingOn( dim, varType, component, implicit_dep ) == BT_FALSE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Quotient::isPolynomialIn( int dim,
                                        VariableType *varType,
                                        int *component,
                                        BooleanType   *implicit_dep ){

    if(  argument1->isPolynomialIn( dim, varType, component, implicit_dep )    == BT_TRUE  &&
         argument2->isDependingOn( dim, varType, component, implicit_dep )     == BT_FALSE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Quotient::isRationalIn( int dim,
                                      VariableType *varType,
                                      int *component,
                                      BooleanType   *implicit_dep ){

    if(  argument1->isRationalIn( dim, varType, component, implicit_dep )    == BT_TRUE  &&
         argument2->isRationalIn( dim, varType, component, implicit_dep )    == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


MonotonicityType Quotient::getMonotonicity( ){

    if( monotonicity != MT_UNKNOWN )  return monotonicity;

    MonotonicityType m1, m2;

    m1 = argument1->getMonotonicity();
    m2 = argument2->getMonotonicity();

    if( m2 == MT_CONSTANT ){

        if( m1 == MT_CONSTANT )  return MT_CONSTANT;

        double res;
        argument2->evaluate(0,0,&res);

        if( res >= 0.0 ) return m1;

        if( m1 == MT_NONDECREASING ) return MT_NONINCREASING;
        if( m1 == MT_NONINCREASING ) return MT_NONDECREASING;

        return MT_NONMONOTONIC;
    }

    return MT_NONMONOTONIC;
}


CurvatureType Quotient::getCurvature( ){

    if( curvature != CT_UNKNOWN )  return curvature;

    CurvatureType c1, c2;

    c1 = argument1->getCurvature();
    c2 = argument2->getCurvature();

    if( c2 == CT_CONSTANT ){

        if( c1 == CT_CONSTANT )  return CT_CONSTANT;

        double res;
        argument2->evaluate(0,0,&res);

        if( res >= 0.0 ) return c1;

        if( c1 == CT_AFFINE  ) return CT_AFFINE ;
        if( c1 == CT_CONVEX  ) return CT_CONCAVE;
        if( c1 == CT_CONCAVE ) return CT_CONVEX ;

        return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }

    return CT_NEITHER_CONVEX_NOR_CONCAVE;
}


returnValue Quotient::AD_forward( int number, double *x, double *seed,
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
    argument2->AD_forward( number,
                           x, seed, &argument2_result[number], &dargument2_result[number] );

      f[0] =  argument1_result[number]/argument2_result[number];
     df[0] =  dargument1_result[number]/argument2_result[number]
             -(argument1_result[number]*dargument2_result[number])/
              (argument2_result[number]*argument2_result[number] );

     return SUCCESSFUL_RETURN;
}



returnValue Quotient::AD_forward( int number, double *seed, double *df ){

    argument1->AD_forward( number, seed, &dargument1_result[number] );
    argument2->AD_forward( number, seed, &dargument2_result[number] );

     df[0] =  dargument1_result[number]/argument2_result[number]
             -(argument1_result[number]*dargument2_result[number])/
              (argument2_result[number]*argument2_result[number] );

     return SUCCESSFUL_RETURN;
}


returnValue Quotient::AD_backward( int number, double seed, double *df ){

    argument1->AD_backward( number, seed/argument2_result[number], df );
    argument2->AD_backward( number, -argument1_result[number]*seed/
                            (argument2_result[number]*argument2_result[number]), df );

    return SUCCESSFUL_RETURN;
}


returnValue Quotient::AD_forward2( int number, double *seed, double *dseed,
                                   double *df, double *ddf ){

    double      ddargument1_result;
    double      ddargument2_result;
    double      dargument_result1;
    double      dargument_result2;

    argument1->AD_forward2( number, seed, dseed,
                            &dargument_result1, &ddargument1_result);
    argument2->AD_forward2( number, seed, dseed,
                            &dargument_result2, &ddargument2_result);

    const double gg  =   argument2_result[number]*argument2_result[number];
    const double ggg =   dargument_result2/gg;

     df[0] =   dargument_result1/argument2_result[number]
              -argument1_result[number]*ggg;

    ddf[0] =   ddargument1_result/argument2_result[number]
              -argument1_result[number]*ddargument2_result/gg
              -dargument_result2*dargument1_result[number]/gg
              -dargument_result1*dargument2_result[number]/gg
              +2.0*argument1_result[number]/(gg*argument2_result[number])
              *dargument_result2*dargument2_result[number];

    return SUCCESSFUL_RETURN;
}


returnValue Quotient::AD_backward2( int number, double seed1, double seed2,
                                       double *df, double *ddf ){

    const double gg  =   argument2_result[number]*argument2_result[number];
    const double ggg =   argument1_result[number]/gg;

    argument1->AD_backward2(  number, seed1/argument2_result[number],
                                      seed2/argument2_result[number] -
                                      seed1*dargument2_result[number]/gg, df, ddf );

    argument2->AD_backward2( number, -seed1*ggg,
                                     -seed2*ggg
                                     -seed1*dargument1_result[number]/gg
                                     +2.0*ggg/argument2_result[number]
                                      *seed1*dargument2_result[number],
                             df, ddf );

    return SUCCESSFUL_RETURN;
}


Stream Quotient::print( Stream &stream ) const{

    return stream << "(" << *argument1 << "/" << *argument2 << ")";
}


Operator* Quotient::clone() const{

    return new Quotient(*this);
}


OperatorName Quotient::getName(){

    return ON_QUOTIENT;
}


CLOSE_NAMESPACE_ACADO

// end of file.
