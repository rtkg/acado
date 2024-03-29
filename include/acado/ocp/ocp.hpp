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
 *    \file include/acado/ocp/ocp.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 */


#ifndef ACADO_TOOLKIT_OCP_HPP
#define ACADO_TOOLKIT_OCP_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/function/function.hpp>

#include <acado/variables_grid/grid.hpp>
#include <acado/constraint/constraint.hpp>
#include <acado/objective/objective.hpp>
#include <acado/ocp/multi_objective_functionality.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Data class for defining optimal control problems.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class OCP is a data class for defining optimal control problems.
 *  In the most easiest an optimal control problem can consists of an
 *  objecive only - i.e. in principle we can set up NLP's as well if no
 *  dynamic system is specified. However, in general the objective
 *  functional is optimized subject to a dynamic equation and different
 *  kind of constraints. \n
 *  \n
 *  Note that the OCP class is only designed for the user and collects
 *  data only. In order to solve an OCP this class must be setup and
 *  passed to an appropriate OptimizationAlgorithm.\n
 *  \n
 *  For setting up an optimal control problem (OCP), we should first specify
 *  the time horizon on which the OCP is defined. Note that there are
 *  several constructors available which allow to construct an OCP directly
 *  with the corresponding time interval. Here, the interval can consist of
 *  of given bounds, but in another variant a parameter can be passed in
 *  order to allow the setup of optimal control problems for which the end
 *  time is optimizaed, too.\n
 *  \n
 *  Constraints can be specified with the "subjectTo" syntax. Please note
 *  that every parameter, state, control etc which is not fixed via a
 *  constraint will be regarded as an optimization variable. In particular,
 *  initial value or boundary constraints appear in many OCP formulations
 *  and of course all these constraints should all be set explicitly.
 *  Moreover, the dynamic equations (model) is regarded as a constraint, too.\n
 *  \n
 *  Please note that the OCP class only collects the formulation of the
 *  problem. If initial values for non-linear problems should be specified,
 *  this needs to be done on the algorithm dealing with the OCP.
 *  (cf. OptimizationAlgorithm for more details.)\n
 *  \n
 *  For advanced users and developers it might be important to know that the
 *  class OCP inherits the MultiObjectiveFunctionality which is needed if
 *  more than one objective should be specified.\n
 *  \n
 *  Please check the tutorial examples as well as the class reference below,
 *  to learn about the usage of this class in more detail.\n
 *  \n
 *
 *  \author Boris Houska, Hans Joachim Ferreau
 */

class OCP: public MultiObjectiveFunctionality{


friend class OptimizationAlgorithmBase;


//
// PUBLIC MEMBER FUNCTIONS:
//
public:


    /** Default Constructor which can optionally set the time-horizon of the problem.
     */
    OCP( const double &tStart_ = 0.0,  /**< start of the time horizon of the OCP */
         const double &tEnd_   = 1.0,  /**< end   of the time horizon of the OCP */
         const int    &N_      = 20    /**< number of discretization intervals   */ );


    /** Constructor that takes a parametric version of the time horizon. This contructor
     *  should be used if the end time should be optimized, too.
     */
    OCP( const double    &tStart_,  /**< start of the time horizon of the OCP */
         const Parameter &tEnd_,    /**< end   of the time horizon of the OCP */
         const int       &N_ = 20   /**< number of discretization intervals   */ );


    /** Constructor that takes the time horizon in form of a Grid.
     */
    OCP( const Grid &grid_  /**< discretization grid  */ );


    /** Copy constructor (makes a deep copy of the class). */
    OCP( const OCP& rhs );


    /** Destructor (deletes everything). */
    ~OCP( );


    /** Assignment operator (deep copy). */
    OCP& operator=( const OCP& rhs );



    /** Adds an expression as a the Mayer term to be minimized.
     *  \return SUCCESSFUL_RETURN
     */
    returnValue minimizeMayerTerm( const Expression& arg );


    /** Adds an expression as a the Mayer term to be minimized. In this version
     *  the number of the objective can be specified as well. This functionality
     *  is needed for multi-objective optimal control problems.
     *  \return SUCCESSFUL_RETURN
     */
    returnValue minimizeMayerTerm( const int &multiObjectiveIdx,  const Expression& arg );


    /** Adds an expression as a the Mayer term to be maximized.
     *  Note that this function is introduced for convenience only.
     *  A call of the form maximizeMayerTerm( arg ) is equivalent to
     *  calling minimizeMayerTerm( -arg ).\n
     *
     *  \return SUCCESSFUL_RETURN
     */
    returnValue maximizeMayerTerm( const Expression& arg );


     /** Adds an expression as a the Lagrange term to be minimized.
      *  \return SUCCESSFUL_RETURN
      */
     returnValue minimizeLagrangeTerm( const Expression& arg );


     /** Adds an expression as a the Lagrange term to be maximized.
      *  \return SUCCESSFUL_RETURN
      */
     returnValue maximizeLagrangeTerm( const Expression& arg );




     /** Adds an Least Square term of the form                                \n
      *                                                                       \n
      *   0.5* sum_i || ( h(t_i,x(t_i),u(t_i),p(t_i),...) - r ) ||^2_S_i      \n
      *                                                                       \n
      *  Here the sum is over all grid points of the objective grid. The      \n
      *  Matrix S is assumed to be symmetric and positive (semi-) definite.   \n
      *  The Function  r  is called reference and can be                      \n
      *  specified by the user. The map  h  is a standard Function            \n
      *  (cf. function_.hpp).                                                 \n
      *
      *  \return SUCCESSFUL_RETURN
      */
     returnValue minimizeLSQ( const Matrix   &S,   /**< a weighting matrix */
                              const Function &h,   /**< the LSQ-Function   */
                              const Vector   &r    /**< the reference      */ );

     returnValue minimizeLSQ( const Function &h,   /**< the LSQ-Function   */
                              const Vector   &r    /**< the reference      */ );

     returnValue minimizeLSQ( const Function &h    /**< the LSQ-Function   */ );

     returnValue minimizeLSQ( const MatrixVariablesGrid &S,   /**< a weighting matrix */
                              const Function            &h,   /**< the LSQ-Function   */
                              const VariablesGrid       &r    /**< the reference      */ );

     returnValue minimizeLSQ( const Matrix        &S,   /**< a weighting matrix */
                              const Function      &h,   /**< the LSQ-Function   */
                              const VariablesGrid &r    /**< the reference      */ );

     returnValue minimizeLSQ( const Function      &h,   /**< the LSQ-Function   */
                              const VariablesGrid &r    /**< the reference      */ );

     returnValue minimizeLSQ( const MatrixVariablesGrid &S,   /**< a weighting matrix */
                              const Function            &h,   /**< the LSQ-Function   */
                              const char*        rFilename    /**< filename where the reference is stored */ );


     returnValue minimizeLSQ( const Matrix        &S,   /**< a weighting matrix */
                              const Function      &h,   /**< the LSQ-Function   */
                              const char*  rFilename    /**< filename where the reference is stored */ );


     returnValue minimizeLSQ( const Function      &h,   /**< the LSQ-Function   */
                              const char*  rFilename    /**< filename where the reference is stored */ );


     returnValue minimizeLSQ       ( const Matrix &Q_, const Matrix &R_ );
     returnValue minimizeLSQEndTerm( const Matrix &S_                   );
     returnValue getQRS            ( Matrix &Q_, Matrix &R_, Matrix &S_ ) const;

     /** Adds an Least Square term that is only evaluated at the end:          \n
      *                                                                        \n
      *        0.5* || ( m(T,x(T),u(T),p(T),...) - r ) ||^2_S                  \n
      *                                                                        \n
      *  where  S  is a weighting matrix, r a reference vector and T the time  \n
      *  at the last objective grid point.                                     \n
      *                                                                        \n
      *  \return SUCCESSFUL_RETURN                                             \n
      */
     returnValue minimizeLSQEndTerm( const Matrix   & S,  /**< a weighting matrix */
                                     const Function & m,  /**< the LSQ-Function   */
                                     const Vector   & r   /**< the reference      */ );

     returnValue minimizeLSQEndTerm( const Function & m,  /**< the LSQ-Function   */
                                     const Vector   & r   /**< the reference      */ );


     /** Adds an differential equation (as a continous equality constraint). \n
      *                                                                      \n
      *  \param differentialEquation_ the differential equation to be added  \n
      *  \param n_                    the number of control intervals        \n
      *                                                                      \n
      *  \return SUCCESSFUL_RETURN
      */
     returnValue subjectTo( const DifferentialEquation& differentialEquation_ );


     /**  Adds a (continuous) contraint.                \n
       *  \return SUCCESSFUL_RETURN                     \n
       *          RET_INFEASIBLE_CONSTRAINT             \n
       */
     returnValue subjectTo( const ConstraintComponent& component );


     /**< Adds a (discrete) contraint.                  \n
       *  \return SUCCESSFUL_RETURN                     \n
       *          RET_INFEASIBLE_CONSTRAINT             \n
       */
     returnValue subjectTo( const int index_, const ConstraintComponent& component );


     /**  Adds a (discrete) contraint.                  \n
       *  \return SUCCESSFUL_RETURN                     \n
       *          RET_INFEASIBLE_CONSTRAINT             \n
       */
     returnValue subjectTo( const TimeHorizonElement index_, const ConstraintComponent& component );


     // ===========================================================================
     //
     //                       COUPLED BOUNDARY CONSTRAINTS
     //                       ----------------------------
     //
     //
     //   (general form  lb <=   h_1( t_0,x(t_0),u(t_0),p,... )
     //                        + h_2( t_e,x(t_e),u(t_e),p,... ) <= ub(i)  )
     //
     //    where t_0 is the first and t_e the last time point in the grid.
     //
     // ===========================================================================

     /**< Adds a constraint of the form  lb_ <= arg1(0) + arg_2(T) <= ub  with   \n
      *   constant lower and upper bounds.                                       \n
      *
      *   \return  SUCCESSFUL_RETURN
      *            RET_INFEASIBLE_CONSTRAINT
      *
      */
     returnValue subjectTo( const double lb_, const Expression& arg1,
                            const Expression& arg2, const double ub_ );



     // ===========================================================================
     //
     //                         GENERAL COUPLED CONSTRAINTS
     //                       -------------------------------
     //
     //
     //   (general form  lb <= sum_i  h_i( t_i,x(t_i),u(t_i),p,... ) <= ub(i)  )
     //
     //
     // ===========================================================================

     /**< Adds a constraint of the form  lb_ <= sum_i arg_i(t_i) <= ub  with     \n
      *   constant lower and upper bounds.                                       \n
      *
      *   \return  SUCCESSFUL_RETURN
      *            RET_INFEASIBLE_CONSTRAINT
      *
      */
     returnValue subjectTo( const double lb_, const Expression *arguments, const double ub_ );


     BooleanType hasObjective           () const;
     BooleanType hasDifferentialEquation() const;
     BooleanType hasConstraint          () const;

     returnValue getGrid                ( Grid&      grid_                               ) const;
     returnValue getObjective           ( Objective& objective_                          ) const;
     returnValue getObjective           ( const int &multiObjectiveIdx, Expression **arg ) const;


     /** Returns the differential equation. \n
      *  \return SUCCESSFUL_RETURN          \n
      */
     returnValue getDifferentialEquation( DifferentialEquation &differentialEquation_ ) const;



     returnValue getConstraint( Constraint& constraint_ ) const;



     returnValue setObjective ( const Objective & objective_  );
     returnValue setConstraint( const Constraint& constraint_ );


     double getStartTime ( ) const;
     double getEndTime( ) const;




    //
    // PROTECTED FUNCTIONS:
    //
    protected:

        void setupGrid( double tStart, double tEnd, int N );
        void copy( const OCP &rhs );


    //
    // DATA MEMBERS:
    //
    protected:

        Grid                   grid                 ;   /**< Common discretization grid            */
        Objective              objective            ;   /**< The Objective.                        */
        DifferentialEquation   differentialEquation ;   /**< Dynamic Equations.                    */
        Constraint             constraint           ;   /**< The Constraints.                      */

        Matrix QQ,RR,QF;
};


CLOSE_NAMESPACE_ACADO



#include <acado/ocp/ocp.ipp>
#include <acado/ocp/nlp.hpp>

#endif  // ACADO_TOOLKIT_OCP_HPP

/*
 *   end of file
 */
