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
*    \file include/acado/symbolic_operator/projection.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#ifndef ACADO_TOOLKIT_PROJECTION_HPP
#define ACADO_TOOLKIT_PROJECTION_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements the projection operator within the symbolic operators family.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Projection implements the scalar-valued 
 *	projection operator within the symbolic operators family.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */
class Projection : public SmoothOperator{


friend class FunctionEvaluationTree;


public:


    /** Default constructor */
    Projection();

    /** Default constructor */
    Projection( const String &name_ );

    /** Default constructor */
    Projection( VariableType variableType_, int vIndex_, const String &name_ );


    /** Default constructor */
    Projection( const Projection& arg );


    /** Default destructor. */
    virtual ~Projection();



    /** Evaluates the expression and stores the intermediate      \n
     *  results in a buffer (needed for automatic differentiation \n
     *  in backward mode)                                         \n
     *  \return SUCCESSFUL_RETURN                  \n
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



     /** Asks the expression whether it is depending on a certian type of \n
       * variable.                                                        \n
       * \return BT_TRUE if a dependency is detected,                     \n
       *         BT_FALSE otherwise.                                      \n
       */
     virtual BooleanType isDependingOn( VariableType var ) const;



    /** Checks whether the expression is depending on a variable  \n
     *  \return BT_FALSE if no dependence is detected             \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isDependingOn( int           dim      ,    /**< number of directions  */
                                          VariableType *varType  ,    /**< the variable types    */
                                          int          *component,    /**< and their components  */
                                          BooleanType   *implicit_dep /**< implicit dependencies */ );




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



    /** Overwrites the monotonicity of the expression.            \n
     *  (For the case that the monotonicity is explicitly known)  \n
     *  \return SUCCESSFUL_RETURN                                 \n
     *
     */
     virtual returnValue setMonotonicity( MonotonicityType monotonicity_ );




    /** Overwrites the curvature of the expression.               \n
     *  (For the case that the curvature is explicitly known)     \n
     *  \return SUCCESSFUL_RETURN                                 \n
     *
     */
     virtual returnValue setCurvature( CurvatureType curvature_  );



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
     *  \return SUCCESSFUL_RETURN                                 \n
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
     *  \return SUCCESSFUL_RETURN                                 \n
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


     /** Clears the buffer and resets the buffer size \n
      *  to 1.                                        \n
      *  \return SUCCESSFUL_RETURN                    \n
      */
     virtual returnValue clearBuffer();



     /** Enumerates all variables based on a common   \n
      *  IndexList.                                   \n
      *  \return SUCCESFUL_RETURN
      */
     virtual returnValue enumerateVariables( SymbolicIndexList *indexList );


     /** Asks the expression for its name.   \n
      *  \return the name of the expression. \n
      */
     virtual OperatorName getName();


     /** Sets the name of the variable.   \n
      *  \returns void \n
      */
     virtual returnValue setName( const char *name_ );


     /** Asks the expression for its scale.   \n
      *  \return the scale of the expression. \n
      */
     virtual double getScale() const;


     /** Sets the scale of the variable.   \n
      *  \returns void \n
      */
     virtual returnValue setScale( const double &scale_ );



     /** Sets the unit of the variable.   \n
      *  \returns void \n
      */
     virtual returnValue setUnit( const char *unit_ );


     /** Asks the expression whether it is a variable.   \n
      *  \return The answer. \n
      */
     virtual BooleanType isVariable( VariableType &varType,
                                     int &component          ) const;



     /** Asks the variable for its relative index. \n
      */
     virtual int getVariableIndex( ) const;


     /** Returns the global type ID of this variable \n
      *                                              \n
      *  \return the requested global type ID.       \n
      */
     virtual int getGlobalTypeID() const;


     /** Asks the variable for its type. \n
      */
     VariableType getType( ) const;


     /** The function loadIndices passes an IndexList through    \n
      *  the whole expression tree. Whenever a variable gets the \n
      *  IndexList it tries to make an entry. However if a       \n
      *  variable recognices that it has already been added      \n
      *  before, it will not be allowed to make a second entry.  \n
      *  Note that all variables, in paticular the intermediate  \n
      *  states, will keep in mind whether they were allowed     \n
      *  to make an entry or not. This guarantees that           \n
      *  intermediate states are never evaluated twice if they   \n
      *  occur at several knots of the tree.                     \n
      *                                                          \n
      *  THIS FUNCTION IS FOR INTERNAL USE ONLY.                 \n
      *                                                          \n
      *  PLEASE CALL THIS FUNTION AT MOST ONCE FOR AN EXPRESSION \n
      *  AS A KIND OF INIT ROUTINE.                              \n
      *                                                          \n
      *  \return the name of the expression.                     \n
      */
     virtual returnValue loadIndices( SymbolicIndexList *indexList
                                                           /**< The index list to be
                                                             *  filled with entries  */ );


    /** Asks whether all elements are purely symbolic.                \n
      *                                                               \n
      * \return BT_TRUE  if the complete tree is symbolic.            \n
      *         BT_FALSE otherwise (e.g. if C functions are linked).  \n
      */
    virtual BooleanType isSymbolic() const;



//
//  PROTECTED FUNCTIONS:
//

protected:


    /** Automatic Differentiation in forward mode on the symbolic \n
     *  level. This function generates an expression for a        \n
     *  forward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                 \n
     */
     virtual Operator* ADforwardProtected( int                dim      , /**< dimension of the seed */
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
     virtual returnValue ADbackwardProtected( int           dim      , /**< number of directions  */
                                              VariableType *varType  , /**< the variable types    */
                                              int          *component, /**< and their components  */
                                              Operator   *seed     , /**< the backward seed     */
                                              Operator  **df         /**< the result            */ );



//  PROTECTED MEMBER FUNCTIONS:
// ---------------------------------------------

    /** Copy function. */
    virtual void copy( const Projection &arg );


//
//  PROTECTED MEMBERS:
//

protected:

    VariableType      variableType ;
    int               variableIndex;
    int               vIndex       ;
    double            scale        ;
    String            name         ;
    String            unit         ;

    OperatorName      operatorName ;

    CurvatureType     curvature    ;
    MonotonicityType  monotonicity ;

    int printIdx;
};


CLOSE_NAMESPACE_ACADO



#endif
