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
 *    \file include/acado/conic_solver/condensing_based_cp_solver.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_CONDENSING_BASED_CP_SOLVER_HPP
#define ACADO_TOOLKIT_CONDENSING_BASED_CP_SOLVER_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/conic_solver/banded_cp_solver.hpp>
#include <acado/conic_solver/dense_qp_solver.hpp>



BEGIN_NAMESPACE_ACADO


/**
 *	\brief Solves banded conic programs arising in optimal control using condensing.
 *
 *	\ingroup NumericalAlgorithm
 *
 *  The class condensing based CP solver is a special solver for
 *  band structured conic programs that can be solved via a
 *  condensing technique.
 *
 *  \author Boris Houska, Hans Joachim Ferreau
 */

class CondensingBasedCPsolver: public BandedCPsolver {


    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        CondensingBasedCPsolver( );
		
        CondensingBasedCPsolver(	UserInteraction* _userInteraction,
									uint nConstraints_,
        							const Vector& blockDims_
        							);

        /** Copy constructor (deep copy). */
        CondensingBasedCPsolver( const CondensingBasedCPsolver& rhs );

        /** Destructor. */
        virtual ~CondensingBasedCPsolver( );

        /** Assignment operator (deep copy). */
        CondensingBasedCPsolver& operator=( const CondensingBasedCPsolver& rhs );


        /** Assignment operator (deep copy). */
        virtual BandedCPsolver* clone() const;


        /** initializes the banded conic solver */
        virtual returnValue init( const OCPiterate &iter_ );


        /** Solves a given banded conic program */
        virtual returnValue prepareSolve(	BandedCP& cp
											);

		/** Solves a given banded conic program in feedback mode:                   \n
         *                                                                          \n
         *  \param cp     the banded conic program to be solved                     \n
         *  \param DeltaX difference between state estimate and previous prediction \n
         *  \param DeltaP difference between current and previous parameter value   \n
         *                                                                          \n
         *  \return SUCCESSFUL_RETURN   (if successful)                             \n
         *          or a specific error message from the dense CP solver.           \n
         */
        virtual returnValue solve(	BandedCP& cp
									);

        /** Solves a given banded conic program */
        virtual returnValue finalizeSolve(	BandedCP& cp
											);


		inline uint getNX( ) const;
		inline uint getNXA( ) const;
		inline uint getNP( ) const;
		inline uint getNU( ) const;
		inline uint getNW( ) const;

		inline uint getNC( ) const;
		inline uint getNF( ) const;
		inline uint getNA( ) const;

		inline uint getNumPoints( ) const;


		virtual returnValue getParameters        ( Vector        &p_  ) const;
		virtual returnValue getFirstControl      ( Vector        &u0_ ) const;


        /** Returns a variance-covariance estimate if possible or an error message otherwise.
         *
         *  \return SUCCESSFUL_RETURN
         *          RET_MEMBER_NOT_INITIALISED
         */
        virtual returnValue getVarianceCovariance( Matrix &var );

		
		virtual returnValue setRealTimeParameters(	const Vector& DeltaX,
													const Vector& DeltaP = emptyConstVector
													);

		inline BooleanType areRealTimeParametersDefined( ) const;


		virtual returnValue freezeCondensing( );

		virtual returnValue unfreezeCondensing( );



    //
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

        /** Initializes QP objects.
		 *  \return SUCCESSFUL_RETURN \n
		 *          RET_QP_INIT_FAILED */
        virtual returnValue initializeCPsolver(	InfeasibleQPhandling infeasibleQPhandling
												);


        /** Solves current QP using at most <maxIter> iterations. */
		virtual returnValue solveQP(	uint maxIter				/**< Maximum number of iterations. */
										);

        /** Solves a relaxation of the current QP using at most <maxIter> iterations. */
		virtual returnValue solveQP(	uint maxIter,				/**< Maximum number of iterations. */
										InfeasibleQPhandling infeasibleQPhandling
										);





        virtual returnValue solveQPsubproblem( );



        /** Checks whether the Hessian is positive definite and projects \n
         *  the Hessian based on a heuristic damping factor. If this     \n
         *  damping factor is smaller than 0, the routine does nothing.  \n
         *                                                               \n
         *  \return SUCCESSFUL_RETURN.                                   \n
         */
        returnValue projectHessian( Matrix &H_, double dampingFactor );


		// --------
		// SQP DATA
		// --------

        /** Performes the condensing of the dynamic system if necessary.
         */
        returnValue condense(	BandedCP& cp
								);


        /** Expands the KKT system if necessary.
         */
        returnValue expand(		BandedCP& cp
								);


        returnValue generateHessianBlockLine   ( uint nn, uint rowOffset, uint& rowOffset1 );
        returnValue generateConstraintBlockLine( uint nn, uint rowOffset, uint& rowOffset1 );
        returnValue generateStateBoundBlockLine( uint nn, uint rowOffset, uint& rowOffset1 );
        returnValue generateConstraintVectors  ( uint nn, uint rowOffset, uint& rowOffset1 );
        returnValue generateStateBoundVectors  ( uint nn, uint rowOffset, uint& rowOffset1 );


        returnValue generateBoundVectors     ( );
        returnValue generateObjectiveGradient( );


        returnValue initializeCondensingOperator( );
		
		returnValue computeCondensingOperator(	BandedCP& cp
												);


        virtual returnValue getRelaxedQPdimensions(	InfeasibleQPhandling infeasibleQPhandling,
													uint& _nV,	/**< OUTPUT: Number of relaxed QP variables. */
													uint& _nC	/**< OUTPUT: Number of relaxed QP constraints (without bounds). */
													) const;


        /** Determines relaxed (constraints') bounds of an infeasible QP. */
        virtual returnValue getRelaxedQPdata(	InfeasibleQPhandling infeasibleQPhandling,
												Matrix** H_relaxed,		/**< OUTPUT: Relaxed Hessian matrix. */
                                    			Matrix** A_relaxed,		/**< OUTPUT: Relaxed constraint matrix. */
                                    			Vector** g_relaxed,		/**< OUTPUT: Relaxed gradient. */
                                    			Vector** lb_relaxed,		/**< OUTPUT: Relaxed lower bounds. */
												Vector** ub_relaxed,		/**< OUTPUT: Relaxed upper bounds. */
												Vector** lbA_relaxed,	/**< OUTPUT: Relaxed lower constraints' bounds. */
												Vector** ubA_relaxed		/**< OUTPUT: Relaxed upper constraints' bounds. */
												) const;


        /** Determines relaxed (constraints') bounds of an infeasible QP. */
        virtual returnValue getRelaxedQPdataL2(	Matrix** H_relaxed,		/**< OUTPUT: Relaxed Hessian matrix. */
                                    			Matrix** A_relaxed,		/**< OUTPUT: Relaxed constraint matrix. */
                                    			Vector** g_relaxed,		/**< OUTPUT: Relaxed gradient. */
                                    			Vector** lb_relaxed,		/**< OUTPUT: Relaxed lower bounds. */
												Vector** ub_relaxed,		/**< OUTPUT: Relaxed upper bounds. */
												Vector** lbA_relaxed,	/**< OUTPUT: Relaxed lower constraints' bounds. */
												Vector** ubA_relaxed		/**< OUTPUT: Relaxed upper constraints' bounds. */
												) const;



    //
    // DATA MEMBERS:
    //
    protected:

        OCPiterate iter;
        Vector blockDims;
        uint nConstraints;

		CondensingStatus condensingStatus;


        // THE CONDENSING OPERATORS:
        // -----------------------------------------------------
        BlockMatrix   T;    /**< the condensing operator */
        BlockMatrix   d;    /**< the condensing offset   */

		BlockMatrix  hT;
        // ------------------------------------------------


        // DENSE QP IN BLOCK-MATRIX FORM:
        // ----------------------------------------------------------------------

        BlockMatrix        HDense;    /**< Hessian after condensing            */
        BlockMatrix        gDense;    /**< Objective gradient after condensing */
        BlockMatrix        ADense;    /**< Constraint matrix                   */
        BlockMatrix      lbADense;    /**< Constraint lower bounds             */
        BlockMatrix      ubADense;    /**< Constraint upper bounds             */
        BlockMatrix       lbDense;    /**< Simple lower bounds                 */
        BlockMatrix       ubDense;    /**< Simple upper bounds                 */
        // ----------------------------------------------------------------------


        DenseCPsolver *cpSolver ;
        DenseQPsolver *qpRelaxed;
        DenseCP        denseCP  ;

		Vector deltaX;
		Vector deltaP;
};


CLOSE_NAMESPACE_ACADO


#include <acado/conic_solver/condensing_based_cp_solver.ipp>


#endif  // ACADO_TOOLKIT_CONDENSING_BASED_CP_SOLVER_HPP

/*
 *  end of file
 */
