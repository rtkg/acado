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
*    \file include/acado/symbolic_operator/product.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#ifndef ACADO_TOOLKIT_PRODUCT_HPP
#define ACADO_TOOLKIT_PRODUCT_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements the scalar product operator within the symbolic operators family.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Product implements the scalar product operator within the 
 *	symbolic operators family.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */
class Product : public BinaryOperator{

public:

    /** Default constructor. */
    Product();

    /** Default constructor. */
    Product( Operator *_argument1, Operator *_argument2 );

    /** Copy constructor (deep copy). */
    Product( const Product &arg );

    /** Default destructor. */
    ~Product();

    /** Assignment Operator (deep copy). */
    Product& operator=( const Product &arg );



    /** Evaluates the expression and stores the intermediate      \n
     *  results in a buffer (needed for automatic differentiation \n
     *  in backward mode)                                         \n
     *  \return SUCCESFUL_RETURN                   \n
     *          RET_NAN                            \n
     * */
    virtual returnValue evaluate( int     number    /**< storage position     */,
                                  double *x         /**< the input variable x */,
                                  double *result    /**< the result           */  );



    /** Returns the derivative of the expression with respect     \n
     *  to the variable var(index).                               \n
     *  \return The expression for the derivative.                \n
     *
     */
     virtual Operator* differentiate( int index  /**< diff. index    */ );



    /** Automatic Differentiation in forward mode on the symbolic \n
     *  level. This function generates an expression for a        \n
     *  forward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                 \n
     */
     virtual Operator* AD_forward( int                dim      , /**< dimension of the seed */
                                     VariableType      *varType  , /**< the variable types    */
                                     int               *component, /**< and their components  */
                                     Operator       **seed     , /**< the forward seed      */
                                     int                &nNewIS  , /**< the number of new IS  */
                                     TreeProjection ***newIS    /**< the new IS-pointer    */ );



    /** Automatic Differentiation in backward mode on the symbolic \n
     *  level. This function generates an expression for a         \n
     *  backward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                  \n
     */
     virtual returnValue AD_backward( int           dim      , /**< number of directions  */
                                      VariableType *varType  , /**< the variable types    */
                                      int          *component, /**< and their components  */
                                      Operator   *seed     , /**< the backward seed     */
                                      Operator  **df         /**< the result            */ );


    /** Substitutes var(index) with the expression sub.           \n
     *  \return The substituted expression.                       \n
     *
     */
     virtual Operator* substitute( int   index           /**< subst. index    */,
                                     const Operator *sub /**< the substitution*/);



    /** Checks whether the expression is zero or one              \n
     *  \return NE_ZERO                                           \n
     *          NE_ONE                                            \n
     *          NE_NEITHER_ONE_NOR_ZERO                           \n
     *
     */
     virtual NeutralElement isOneOrZero() const;




    /** Checks whether the expression is linear in                \n
     *  (or not depending on) a variable                          \n
     *  \return BT_FALSE if no linearity is                       \n
     *                detected                                    \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isLinearIn( int           dim      ,    /**< number of directions  */
                                       VariableType *varType  ,    /**< the variable types    */
                                       int          *component,    /**< and their components  */
                                       BooleanType  *implicit_dep  /**< implicit dependencies */ );



    /** Checks whether the expression is polynomial in            \n
     *  the specified variables                                   \n
     *  \return BT_FALSE if the expression is not  polynomial     \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isPolynomialIn( int           dim      ,    /**< number of directions  */
                                           VariableType *varType  ,    /**< the variable types    */
                                           int          *component,    /**< and their components  */
                                           BooleanType  *implicit_dep  /**< implicit dependencies */ );



    /** Checks whether the expression is rational in              \n
     *  the specified variables                                   \n
     *  \return BT_FALSE if the expression is not rational        \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isRationalIn( int           dim      ,    /**< number of directions  */
                                         VariableType *varType  ,    /**< the variable types    */
                                         int          *component,    /**< and their components  */
                                         BooleanType  *implicit_dep  /**< implicit dependencies */ );



    /** Returns the monotonicity of the expression.               \n
     *  \return MT_NONDECREASING                                  \n
     *          MT_NONINCREASING                                  \n
     *          MT_NONMONOTONIC                                   \n
     *
     */
     virtual MonotonicityType getMonotonicity( );



    /** Returns the curvature of the expression                   \n
     *  \return CT_CONSTANT                                       \n
     *          CT_AFFINE                                         \n
     *          CT_CONVEX                                         \n
     *          CT_CONCAVE                                        \n
     *
     */
     virtual CurvatureType getCurvature( );



    /** Automatic Differentiation in forward mode.                \n
     *  This function uses the intermediate                       \n
     *  results from a buffer                                     \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_forward( int     number  /**< storage position */,
                                     double *seed    /**< the seed         */,
                                     double *df      /**< the derivative of
                                                          the expression   */  );




    /** Automatic Differentiation in forward mode.                \n
     *  This function stores the intermediate                     \n
     *  results in a buffer (needed for 2nd order automatic       \n
     *  differentiation in backward mode)                         \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_forward( int     number  /**< storage position */,
                                     double *x       /**< The evaluation
                                                          point x          */,
                                     double *seed    /**< the seed         */,
                                     double *f       /**< the value of the
                                                          expression at x  */,
                                     double *df      /**< the derivative of
                                                          the expression   */  );


    // IMPORTANT REMARK FOR AD_BACKWARD: run evaluate first to define
    //                                   the point x and to compute f.

    /** Automatic Differentiation in backward mode based on       \n
     *  buffered values                                           \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_backward( int    number /**< the buffer
                                                         position         */,
                                      double seed   /**< the seed         */,
                                      double  *df   /**< the derivative of
                                                         the expression   */);



    /** Automatic Differentiation in forward mode for             \n
     *  2nd derivatives.                                          \n
     *  This function uses intermediate                           \n
     *  results from a buffer.                                    \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_forward2( int    number  /**< the buffer
                                                          position         */,
                                      double *seed1  /**< the seed         */,
                                      double *seed2  /**< the seed for the
                                                          first derivative */,
                                      double *df     /**< the derivative of
                                                          the expression   */,
                                      double *ddf    /**< the 2nd derivative
                                                          of the expression*/);



    // IMPORTANT REMARK FOR AD_BACKWARD2: run AD_forward first to define
    //                                    the point x and to compute f and df.

    /** Automatic Differentiation in backward mode for 2nd order  \n
     *  derivatives based on buffered values.                     \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_backward2( int    number /**< the buffer
                                                          position           */,
                                       double seed1  /**< the seed1          */,
                                       double seed2  /**< the seed2          */,
                                       double   *df  /**< the 1st derivative
                                                          of the expression  */,
                                       double  *ddf  /**< the 2nd derivative
                                                          of the expression  */   );



    /** Prints the expression into a stream. \n
     *  \return SUCCESFUL_RETURN             \n
     */
     virtual Stream print( Stream &stream ) const;



     /** Provides a deep copy of the expression. \n
      *  \return a clone of the expression.      \n
      */
     virtual Operator* clone() const;


     /** Asks the expression for its name.   \n
      *  \return the name of the expression. \n
      */
     virtual OperatorName getName();


//
//  PROTECTED FUNCTIONS:
//

protected:


//
//  PROTECTED MEMBERS:
//

protected:

};


CLOSE_NAMESPACE_ACADO



#endif

