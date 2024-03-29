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
*    \file src/symbolic_operator/doubleconstant.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO




DoubleConstant::DoubleConstant( ) : SmoothOperator( )
{
	nCount = 0;
}

DoubleConstant::DoubleConstant( double value_, NeutralElement neutralElement_ ) : SmoothOperator( )
{
    value           = value_          ;
    neutralElement  = neutralElement_ ;

    if( fabs(value    ) < 10.0*EPS ) neutralElement = NE_ZERO;
    if( fabs(1.0-value) < 10.0*EPS ) neutralElement = NE_ONE;

    nCount = 0;
}


DoubleConstant::DoubleConstant( const DoubleConstant &arg ){

    value           = arg.value          ;
    neutralElement  = arg.neutralElement ;

    nCount = 0;
}


DoubleConstant::~DoubleConstant(){

}


DoubleConstant& DoubleConstant::operator=( const DoubleConstant &arg ){

    if( this != &arg ){

        value           = arg.value          ;
        neutralElement  = arg.neutralElement ;

        nCount = 0;
    }

    return *this;
}


returnValue DoubleConstant::evaluate( int number, double *x, double *result ){

    result[0] = value;
    return SUCCESSFUL_RETURN;
}



Operator* DoubleConstant::differentiate( int index ){

  return new DoubleConstant( 0.0 , NE_ZERO );
}


Operator* DoubleConstant::AD_forward( int dim,
                                        VariableType *varType,
                                        int *component,
                                        Operator **seed,
                                        int &nNewIS,
                                        TreeProjection ***newIS ){

    return new DoubleConstant( 0.0, NE_ZERO );
}



returnValue DoubleConstant::AD_backward( int dim,
                                         VariableType *varType,
                                         int *component,
                                         Operator *seed,
                                         Operator **df         ){

    delete seed;
    return SUCCESSFUL_RETURN;
}


Operator* DoubleConstant::substitute( int index, const Operator *sub ){

    return clone();
}



NeutralElement DoubleConstant::isOneOrZero() const{

    return neutralElement;
}


BooleanType DoubleConstant::isDependingOn( VariableType var ) const{

    return BT_FALSE;
}


BooleanType DoubleConstant::isDependingOn( int dim,
                                             VariableType *varType,
                                             int *component,
                                             BooleanType   *implicit_dep ){

    return BT_FALSE;
}


BooleanType DoubleConstant::isLinearIn( int dim,
                                          VariableType *varType,
                                          int *component,
                                          BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType DoubleConstant::isPolynomialIn( int dim,
                                              VariableType *varType,
                                              int *component,
                                              BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType DoubleConstant::isRationalIn( int dim,
                                            VariableType *varType,
                                            int *component,
                                            BooleanType   *implicit_dep ){

    return BT_TRUE;
}


MonotonicityType DoubleConstant::getMonotonicity( ){

    return MT_CONSTANT;
}


CurvatureType DoubleConstant::getCurvature( ){

    return CT_CONSTANT;
}


returnValue DoubleConstant::setMonotonicity( MonotonicityType monotonicity_ ){

    return SUCCESSFUL_RETURN;
}


returnValue DoubleConstant::setCurvature( CurvatureType curvature_ ){

    return SUCCESSFUL_RETURN;
}


returnValue DoubleConstant::AD_forward( int number, double *x, double *seed,
                                        double *f, double *df ){

      f[0] =  value;
     df[0] =  0.0;

     return SUCCESSFUL_RETURN;
}



returnValue DoubleConstant::AD_forward( int number, double *seed, double *df ){

     df[0] =  0.0;
     return SUCCESSFUL_RETURN;
}


returnValue DoubleConstant::AD_backward( int number, double seed, double *df ){

     return SUCCESSFUL_RETURN;
}


returnValue DoubleConstant::AD_forward2( int number, double *seed, double *dseed,
                                         double *df, double *ddf ){

     df[0] = 0.0;
    ddf[0] = 0.0;

    return SUCCESSFUL_RETURN;
}


returnValue DoubleConstant::AD_backward2( int number, double seed1, double seed2,
                                          double *df, double *ddf ){

    return SUCCESSFUL_RETURN;
}



Stream DoubleConstant::print( Stream &stream ) const{

    return stream << "(" << value << ")";
}


Operator* DoubleConstant::clone() const{

    return new DoubleConstant(*this);
}


returnValue DoubleConstant::clearBuffer(){

    return SUCCESSFUL_RETURN;
}


returnValue DoubleConstant::enumerateVariables( SymbolicIndexList *indexList ){

    return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
// ---------------------------

OperatorName DoubleConstant::getName(){

    return ON_DOUBLE_CONSTANT;
}

BooleanType DoubleConstant::isVariable( VariableType &varType, int &component ) const
{
    return BT_FALSE;
}


returnValue DoubleConstant::loadIndices( SymbolicIndexList *indexList ){

    return SUCCESSFUL_RETURN;
}


BooleanType DoubleConstant::isSymbolic() const{

    return BT_TRUE;
}


double DoubleConstant::getValue() const{

    return value;
}




CLOSE_NAMESPACE_ACADO

// end of file.
