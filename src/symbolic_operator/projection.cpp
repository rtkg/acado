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
 *    \file src/symbolic_operator/projection.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO


Projection::Projection()
           :SmoothOperator( ){

    scale          = 1.0           ;
    curvature      = CT_AFFINE     ;
    monotonicity = MT_NONDECREASING;
    operatorName = ON_VARIABLE     ;

    nCount = 0;
}


Projection::Projection( const String &name_ )
           :SmoothOperator( ){

    scale          = 1.0             ;
    curvature      = CT_AFFINE       ;
    monotonicity   = MT_NONDECREASING;
    operatorName   = ON_VARIABLE     ;
    name           = name_           ;

    nCount = 0;
}


Projection::Projection( VariableType variableType_, int vIndex_, const String &name_ ) :SmoothOperator( )
{
    variableType   = variableType_ ;
    vIndex         = vIndex_       ;
    variableIndex  = vIndex        ;

    scale          = 1.0           ;
    curvature      = CT_AFFINE     ;
    monotonicity = MT_NONDECREASING;
    operatorName = ON_VARIABLE     ;

    switch(variableType){

         case VT_DIFFERENTIAL_STATE:
              name = "acado_xd";
              break;

         case VT_ALGEBRAIC_STATE:
              name = "acado_xa";
              break;

         case VT_CONTROL:
              name = "acado_u";
              break;

         case VT_INTEGER_CONTROL:
              name = "acado_v";
              break;

         case VT_PARAMETER:
              name = "acado_p";
              break;

         case VT_INTEGER_PARAMETER:
              name = "acado_q";
              break;

         case VT_DISTURBANCE:
              name = "acado_w";
              break;

         case VT_TIME:
              name = "acado_t";
              break;

         case VT_INTERMEDIATE_STATE:
              name = "acadoWorkspace.acado_aux";
              break;

         case VT_DDIFFERENTIAL_STATE:
              name = "acado_dx";
              break;

         default: break;
    }
    nCount = 0;
}


Projection::~Projection(){ }


Projection::Projection( const Projection& arg ){

    copy( arg );
}


Operator* Projection::clone() const{

    return new Projection(*this);
}


void Projection::copy( const Projection &arg ){

    if( this != &arg ){

        variableType   = arg.variableType ;
        variableIndex  = arg.variableIndex;
        vIndex         = arg.vIndex       ;
        scale          = arg.scale        ;
        name           = arg.name         ;
        unit           = arg.unit         ;
        operatorName   = arg.operatorName ;
        curvature      = arg.curvature    ;
        monotonicity   = arg.monotonicity ;

        nCount = 0;
    }
}


returnValue Projection::evaluate( int number, double *x, double *result ){

    result[0] = x[variableIndex];
    return SUCCESSFUL_RETURN;
}



Operator* Projection::differentiate( int index ){

    if( variableIndex == index ){
        return new DoubleConstant( 1.0 , NE_ONE );
    }
    else{
        return new DoubleConstant( 0.0 , NE_ZERO );
    }
}


Operator* Projection::AD_forward( int dim,
                                          VariableType *varType,
                                          int *component,
                                          Operator **seed,
                                          int &nNewIS,
                                          TreeProjection ***newIS ){

    return ADforwardProtected( dim, varType, component, seed, nNewIS, newIS );
}



returnValue Projection::AD_backward( int dim,
                                           VariableType *varType,
                                           int *component,
                                           Operator *seed,
                                           Operator **df         ){

    return ADbackwardProtected( dim, varType, component, seed, df );
}


Operator* Projection::substitute( int index, const Operator *sub ){

    if( variableIndex == index ){
        return sub->clone();
    }
    else{
        return clone();
    }
}



NeutralElement Projection::isOneOrZero() const{

    return NE_NEITHER_ONE_NOR_ZERO;
}




BooleanType Projection::isDependingOn( VariableType var ) const{

    if( variableType == var )  return BT_TRUE;
    return BT_FALSE;
}




BooleanType Projection::isDependingOn( int dim,
                                               VariableType *varType,
                                               int *component,
                                               BooleanType   *implicit_dep ){

    int run1 = 0;

    while( run1 < dim ){

        if( varType[run1] == variableType && component[run1] == vIndex ){
            return BT_TRUE;
        }
        run1++;
    }
    return BT_FALSE;
}


BooleanType Projection::isLinearIn( int dim,
                                            VariableType *varType,
                                            int *component,
                                            BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType Projection::isPolynomialIn( int dim,
                                                VariableType *varType,
                                                int *component,
                                                BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType Projection::isRationalIn( int dim,
                                              VariableType *varType,
                                              int *component,
                                              BooleanType   *implicit_dep ){

    return BT_TRUE;
}


MonotonicityType Projection::getMonotonicity( ){

    return monotonicity;
}


CurvatureType Projection::getCurvature( ){

    return curvature;
}


returnValue Projection::setMonotonicity( MonotonicityType monotonicity_ ){

    monotonicity = monotonicity_;
    return SUCCESSFUL_RETURN;
}


returnValue Projection::setCurvature( CurvatureType curvature_ ){

    curvature = curvature_;
    return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_forward( int number, double *x, double *seed,
                                          double *f, double *df ){

      f[0] =  x[variableIndex];
     df[0] =  seed[variableIndex];

     return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_forward( int number, double *seed, double *df ){

     df[0] =  seed[variableIndex];
     return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_backward( int number, double seed, double *df ){

    df[variableIndex] = df[variableIndex] + seed;
    return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_forward2( int number, double *seed, double *dseed,
                                           double *df, double *ddf ){

     df[0] = seed [variableIndex];
    ddf[0] = dseed[variableIndex];
    return SUCCESSFUL_RETURN;
}

returnValue Projection::AD_backward2( int number, double seed1, double seed2,
                                            double *df, double *ddf ){

     df[variableIndex] =  df[variableIndex] + seed1;
    ddf[variableIndex] = ddf[variableIndex] + seed2;

    return SUCCESSFUL_RETURN;
}



Stream Projection::print( Stream &stream ) const{

    return stream << name << "[" << vIndex << "]";
}


returnValue Projection::clearBuffer(){

    return SUCCESSFUL_RETURN;
}



OperatorName Projection::getName(){

    return operatorName;
}



returnValue Projection::setName(const char *name_){

	name = name_;
    return SUCCESSFUL_RETURN;
}


double Projection::getScale() const{

    return scale;
}


returnValue Projection::setScale( const double &scale_ ){

    scale = scale_;
    return SUCCESSFUL_RETURN;
}


returnValue Projection::setUnit( const char *unit_){

	unit = unit_;
    return SUCCESSFUL_RETURN;
}


BooleanType Projection::isVariable( VariableType &varType, int &component ) const
{
  varType   = variableType;
  component = vIndex    ;

  return BT_TRUE;
}

returnValue Projection::enumerateVariables( SymbolicIndexList *indexList ){

  variableIndex = indexList->determineVariableIndex( variableType,
						     vIndex, scale         );
  return SUCCESSFUL_RETURN;
}


int Projection::getVariableIndex( ) const{

    return variableIndex;
}


int Projection::getGlobalTypeID() const{

    return vIndex;
}


VariableType Projection::getType( ) const
{
	return variableType;
}


returnValue Projection::loadIndices( SymbolicIndexList *indexList ){

    indexList->addNewElement( variableType, vIndex );
    return SUCCESSFUL_RETURN;
}


BooleanType Projection::isSymbolic() const{

    return BT_TRUE;
}


Operator* Projection::ADforwardProtected( int dim,
                                                  VariableType *varType,
                                                  int *component,
                                                  Operator **seed,
                                                  int &nNewIS,
                                                  TreeProjection ***newIS ){

    int run1 = 0;

    while( run1 < dim ){

        if( varType[run1] == variableType && component[run1] == vIndex ){
            return seed[run1]->clone();
        }
        run1++;
    }

    return new DoubleConstant( 0.0 , NE_ZERO );
}



returnValue Projection::ADbackwardProtected( int dim,
                                                   VariableType *varType,
                                                   int *component,
                                                   Operator *seed,
                                                   Operator **df         ){

    int run1 = 0;

    while( run1 < dim ){

        if( varType[run1] == variableType && component[run1] == vIndex ){

            if(   df[run1]->isOneOrZero() == NE_ZERO ){
                  delete df[run1];
                  df[run1] = seed->clone();
            }
            else{

                if( seed-> isOneOrZero() != NE_ZERO ){

                    Operator *tmp = df[run1]->clone();
                    delete df[run1];
                    df[run1] = new Addition(tmp->clone(),seed->clone());
                    delete tmp;
                }
            }

            break;
        }
        run1++;
    }

    delete seed;
    return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

// end of file.
