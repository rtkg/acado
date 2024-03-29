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
 *    \file external_packages/include/acado_qpoases/qp_solver_qpoases.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 19.08.2008
 */


#ifndef ACADO_TOOLKIT_QP_SOLVER_QPOASES_HPP
#define ACADO_TOOLKIT_QP_SOLVER_QPOASES_HPP


#include <acado/conic_solver/dense_qp_solver.hpp>
#include <qpOASES-2.0/INCLUDE/SQProblem.hpp>
#include <qpOASES-2.0/INCLUDE/EXTRAS/SolutionAnalysis.hpp>

BEGIN_NAMESPACE_ACADO


/**
 *	\brief (not yet documented)
 *
 *	\ingroup ExternalFunctionality
 *
 *  The class QPsolver_qpOASES interfaces the qpOASES software package
 *  for solving convex quadratic programming (QP) problems.
 *
 *	 \author Boris Houska, Hans Joachim Ferreau
 */
class QPsolver_qpOASES : public DenseQPsolver
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:
        /** Default constructor. */
        QPsolver_qpOASES( );
		
        QPsolver_qpOASES(	UserInteraction* _userInteraction
							);

        /** Copy constructor (deep copy). */
        QPsolver_qpOASES( const QPsolver_qpOASES& rhs );

        /** Destructor. */
        virtual ~QPsolver_qpOASES( );

        /** Assignment operator (deep copy). */
        QPsolver_qpOASES& operator=( const QPsolver_qpOASES& rhs );


        virtual DenseCPsolver* clone( ) const;

        virtual DenseQPsolver* cloneDenseQPsolver( ) const;


        /** Solves the QP. */
        virtual returnValue solve( DenseCP *cp_  );


        /** Solves QP using at most <maxIter> iterations.
		 * \return SUCCESSFUL_RETURN \n
		 *         RET_QP_SOLUTION_REACHED_LIMIT \n
		 *         RET_QP_SOLUTION_FAILED \n
		 *         RET_INITIALIZE_FIRST */
        virtual returnValue solve(	const double* const H,		/**< Hessian matrix of neighbouring QP to be solved. */
									const double* const A,		/**< Constraint matrix of neighbouring QP to be solved. */
									const double* const g,		/**< Gradient of neighbouring QP to be solved. */
									const double* const lb,		/**< Lower bounds of neighbouring QP to be solved. */
									const double* const ub,		/**< Upper bounds of neighbouring QP to be solved. */
									const double* const lbA,	/**< Lower constraints' bounds of neighbouring QP to be solved. */
									const double* const ubA,	/**< Upper constraints' bounds of neighbouring QP to be solved. */
									uint maxIter				/**< Maximum number of iterations. */
									);

        /** Solves QP using at most <maxIter> iterations. */
        virtual returnValue solve(  const Matrix *H,    /**< Hessian matrix of neighbouring QP to be solved. */
                                    const Matrix *A,    /**< Constraint matrix of neighbouring QP to be solved. */
                                    const Vector *g,    /**< Gradient of neighbouring QP to be solved. */
                                    const Vector *lb,   /**< Lower bounds of neighbouring QP to be solved. */
                                    const Vector *ub,   /**< Upper bounds of neighbouring QP to be solved. */
                                    const Vector *lbA,  /**< Lower constraints' bounds of neighbouring QP to be solved. */
                                    const Vector *ubA,  /**< Upper constraints' bounds of neighbouring QP to be solved. */
                                    uint maxIter        /**< Maximum number of iterations. */  
									);


        /** Performs exactly one QP iteration.
		 * \return SUCCESSFUL_RETURN \n
		 *         RET_QP_SOLUTION_REACHED_LIMIT \n
		 *         RET_QP_SOLUTION_FAILED \n
		 *         RET_INITIALIZE_FIRST */
        virtual returnValue step(	const double* const H,		/**< Hessian matrix of neighbouring QP to be solved. */
									const double* const A,		/**< Constraint matrix of neighbouring QP to be solved. */
									const double* const g,		/**< Gradient of neighbouring QP to be solved. */
									const double* const lb,		/**< Lower bounds of neighbouring QP to be solved. */
									const double* const ub,		/**< Upper bounds of neighbouring QP to be solved. */
									const double* const lbA,	/**< Lower constraints' bounds of neighbouring QP to be solved. */
									const double* const ubA	/**< Upper constraints' bounds of neighbouring QP to be solved. */
									);

        /** Performs exactly one QP iteration.
		 * \return SUCCESSFUL_RETURN \n
		 *         RET_QP_SOLUTION_REACHED_LIMIT \n
		 *         RET_QP_SOLUTION_FAILED \n
		 *         RET_INITIALIZE_FIRST */
        virtual returnValue step(	const Matrix *H,    /**< Hessian matrix of neighbouring QP to be solved. */
                                    const Matrix *A,    /**< Constraint matrix of neighbouring QP to be solved. */
                                    const Vector *g,    /**< Gradient of neighbouring QP to be solved. */
                                    const Vector *lb,   /**< Lower bounds of neighbouring QP to be solved. */
                                    const Vector *ub,   /**< Upper bounds of neighbouring QP to be solved. */
                                    const Vector *lbA,  /**< Lower constraints' bounds of neighbouring QP to be solved. */
                                    const Vector *ubA   /**< Upper constraints' bounds of neighbouring QP to be solved. */
									);


		/** Returns primal solution vector if QP has been solved.
		 * \return SUCCESSFUL_RETURN \n
		 *         RET_QP_NOT_SOLVED */
		virtual returnValue getPrimalSolution(	Vector& xOpt	/**< OUTPUT: primal solution vector. */
												) const;

		/** Returns dual solution vector if QP has been solved.
		 * \return SUCCESSFUL_RETURN \n
		 *         RET_QP_NOT_SOLVED */
		virtual returnValue getDualSolution(	Vector& yOpt	/**< OUTPUT: dual solution vector. */
												) const;

		/** Returns optimal objective function value.
		 *	\return finite value: Optimal objective function value (QP has been solved) \n
		 			+INFTY:	      QP has not been solved or is infeasible \n
					-INFTY:	      QP is unbounded */
		virtual double getObjVal( ) const;


		virtual uint getNumberOfVariables( ) const;
		virtual uint getNumberOfConstraints( ) const;


        /** Returns a variance-covariance estimate if possible or an error message otherwise.
         *
         *  \return SUCCESSFUL_RETURN
         *          RET_MEMBER_NOT_INITIALISED
         */
        virtual returnValue getVarianceCovariance( Matrix &var );


        /** Returns a variance-covariance estimate if possible or an error message otherwise.
         *
         *  \return SUCCESSFUL_RETURN
         *          RET_MEMBER_NOT_INITIALISED
         */
        virtual returnValue getVarianceCovariance( Matrix &H, Matrix &var );



    //
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:
        /** Setups QP object.
		 *  \return SUCCESSFUL_RETURN \n
		 *          RET_QP_INIT_FAILED */
        virtual returnValue setupQPobject(	uint nV,	/**< Number of QP variables. */
											uint nC		/**< Number of QP constraints (without bounds). */
											);

		returnValue updateQPstatus(	qpOASES::returnValue returnvalue
									);



    //
    // DATA MEMBERS:
    //
    protected:
		qpOASES::SQProblem* qp;
};


CLOSE_NAMESPACE_ACADO


#include <include/acado_qpoases/qp_solver_qpoases.ipp>


#endif  // ACADO_TOOLKIT_QP_SOLVER_QPOASES_HPP

/*
 *	end of file
 */
