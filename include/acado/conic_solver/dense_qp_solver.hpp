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
 *    \file include/acado/conic_solver/dense_qp_solver.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 */


#ifndef ACADO_TOOLKIT_DENSE_QP_SOLVER_HPP
#define ACADO_TOOLKIT_DENSE_QP_SOLVER_HPP


#include <acado/nlp_solver/nlp_solver.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/conic_solver/dense_cp_solver.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Abstract base class for algorithms solving quadratic programs.
 *
 *	\ingroup AlgorithmInterfaces
 *
 *  The class DenseQPsolver provides an abstract base class for different
 *  algorithms for solving quadratic programming (QP) problems.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class DenseQPsolver : public DenseCPsolver
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        DenseQPsolver( );

        DenseQPsolver(	UserInteraction* _userInteraction
						);

        /** Copy constructor (deep copy). */
        DenseQPsolver( const DenseQPsolver& rhs );

        /** Destructor. */
        ~DenseQPsolver( );

        /** Assignment operator (deep copy). */
        DenseQPsolver& operator=( const DenseQPsolver& rhs );

        virtual DenseCPsolver* clone( ) const = 0;

        virtual DenseQPsolver* cloneDenseQPsolver( ) const = 0;



        /** Initializes QP object.     \n
         *                             \n
         *  \return SUCCESSFUL_RETURN  \n
         */
        virtual returnValue init( const DenseCP *cp );



        /** Alternative way to initialize QP object. */
        virtual returnValue init(	uint nV,	/**< Number of QP variables. */
									uint nC		/**< Number of QP constraints (without bounds). */
									);



        /** Solves the QP. */
        virtual returnValue solve( DenseCP *cp_  ) = 0;


        /** Solves QP using at most <maxIter> iterations. */
        virtual returnValue solve(	const double* const H,		/**< Hessian matrix of neighbouring QP to be solved. */
									const double* const A,		/**< Constraint matrix of neighbouring QP to be solved. */
									const double* const g,		/**< Gradient of neighbouring QP to be solved. */
									const double* const lb,		/**< Lower bounds of neighbouring QP to be solved. */
									const double* const ub,		/**< Upper bounds of neighbouring QP to be solved. */
									const double* const lbA,	/**< Lower constraints' bounds of neighbouring QP to be solved. */
									const double* const ubA,	/**< Upper constraints' bounds of neighbouring QP to be solved. */
									uint maxIter				/**< Maximum number of iterations. */
									) = 0;

        /** Solves QP using at most <maxIter> iterations. */
        virtual returnValue solve(  const Matrix *H,    /**< Hessian matrix of neighbouring QP to be solved. */
                                    const Matrix *A,    /**< Constraint matrix of neighbouring QP to be solved. */
                                    const Vector *g,    /**< Gradient of neighbouring QP to be solved. */
                                    const Vector *lb,   /**< Lower bounds of neighbouring QP to be solved. */
                                    const Vector *ub,   /**< Upper bounds of neighbouring QP to be solved. */
                                    const Vector *lbA,  /**< Lower constraints' bounds of neighbouring QP to be solved. */
                                    const Vector *ubA,  /**< Upper constraints' bounds of neighbouring QP to be solved. */
                                    uint maxIter        /**< Maximum number of iterations. */
									) = 0;


        /** Performs exactly one QP iteration. */
        virtual returnValue step(	const double* const H,		/**< Hessian matrix of neighbouring QP to be solved. */
									const double* const A,		/**< Constraint matrix of neighbouring QP to be solved. */
									const double* const g,		/**< Gradient of neighbouring QP to be solved. */
									const double* const lb,		/**< Lower bounds of neighbouring QP to be solved. */
									const double* const ub,		/**< Upper bounds of neighbouring QP to be solved. */
									const double* const lbA,	/**< Lower constraints' bounds of neighbouring QP to be solved. */
									const double* const ubA		/**< Upper constraints' bounds of neighbouring QP to be solved. */
									) = 0;


		/** Performs exactly one QP iteration. */
        virtual returnValue step( 	const Matrix *H,    /**< Hessian matrix of neighbouring QP to be solved. */
                                    const Matrix *A,    /**< Constraint matrix of neighbouring QP to be solved. */
                                    const Vector *g,    /**< Gradient of neighbouring QP to be solved. */
                                    const Vector *lb,   /**< Lower bounds of neighbouring QP to be solved. */
                                    const Vector *ub,   /**< Upper bounds of neighbouring QP to be solved. */
                                    const Vector *lbA,  /**< Lower constraints' bounds of neighbouring QP to be solved. */
                                    const Vector *ubA  /**< Upper constraints' bounds of neighbouring QP to be solved. */
									) = 0;


		/** Returns QP status.
		 * \return QP status */
		inline QPStatus getStatus( ) const;

		/** Returns if QP (or its relaxation) has been solved.
		 * \return BT_TRUE iff QP has been solved */
		inline BooleanType isSolved( ) const;

		/** Returns if QP has been found to be infeasible.
		 * \return BT_TRUE if QP is infeasible */
		inline BooleanType isInfeasible( ) const;

		/** Returns if QP has been found to be unbounded.
		 * \return BT_TRUE if QP is unbounded */
		inline BooleanType isUnbounded( ) const;


		/** Returns primal solution vector if QP has been solved.
		 * \return SUCCESSFUL_RETURN \n
		 *         RET_QP_NOT_SOLVED */
		virtual returnValue getPrimalSolution(	Vector& xOpt	/**< OUTPUT: primal solution vector. */
												) const = 0;

		/** Returns dual solution vector if QP has been solved.
		 * \return SUCCESSFUL_RETURN \n
		 *         RET_QP_NOT_SOLVED */
		virtual returnValue getDualSolution(	Vector& yOpt	/**< OUTPUT: dual solution vector. */
												) const = 0;


		/** Returns optimal objective function value.
		 *	\return finite value: Optimal objective function value (QP has been solved) \n
		 			+INFTY:	      QP has not been solved or is infeasible \n
					-INFTY:	      QP is unbounded */
		virtual double getObjVal( ) const = 0;


		/** Returns number of iterations performed at last QP solution.
		 * \return Number of iterations performed at last QP solution */
		virtual uint getNumberOfIterations( ) const;

		virtual uint getNumberOfVariables( ) const = 0;
		virtual uint getNumberOfConstraints( ) const = 0;


        /** Returns a variance-covariance estimate if possible or an error message otherwise.
         *
         *  \return SUCCESSFUL_RETURN
         *          RET_MEMBER_NOT_INITIALISED
         */
        virtual returnValue getVarianceCovariance( Matrix &var ) = 0;


        /** Returns a variance-covariance estimate if possible or an error message otherwise.
         *
         *  \return SUCCESSFUL_RETURN
         *          RET_MEMBER_NOT_INITIALISED
         */
        virtual returnValue getVarianceCovariance( Matrix &H, Matrix &var ) = 0;



    //
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		virtual returnValue setupLogging( );
	  
	  
	  
		returnValue solveCP( DenseCP *cp );


        /** Setups QP object.
		 *  \return SUCCESSFUL_RETURN \n
		 *          RET_QP_INIT_FAILED */
        virtual returnValue setupQPobject(	uint nV,	/**< Number of QP variables. */
											uint nC		/**< Number of QP constraints (without bounds). */
											) = 0;

		virtual returnValue convertQPdata( 	const Matrix *H,
											const Matrix *A,
											const Vector *g,
											const Vector *lb,
											const Vector *ub,
											const Vector *lbA,
											const Vector *ubA,
											double** H_tmp,
											double** A_tmp,
											double** g_tmp,
											double** lb_tmp,
											double** ub_tmp,
											double** lbA_tmp,
											double** ubA_tmp
											) const;


    //
    // DATA MEMBERS:
    //
    protected:

        QPStatus qpStatus;
        int numberOfSteps;
};


CLOSE_NAMESPACE_ACADO



#include <acado/conic_solver/dense_qp_solver.ipp>


#endif  // ACADO_TOOLKIT_QP_SOLVER_HPP

/*
 *	end of file
 */
