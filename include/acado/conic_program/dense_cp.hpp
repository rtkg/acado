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
 *    \file include/acado/conic_program/dense_cp.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_DENSE_CP_HPP
#define ACADO_TOOLKIT_DENSE_CP_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Data class for storing generic conic programs.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class DenseCP (dense conic program) is a data class
 *  to store generic conic programs in a convenient format.
 *
 *  \author Boris Houska, Hans Joachim Ferreau
 */

class DenseCP{


    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        DenseCP( );

        /** Copy constructor (deep copy). */
        DenseCP( const DenseCP& rhs );

        /** Destructor. */
        virtual ~DenseCP( );

        /** Assignment operator (deep copy). */
        DenseCP& operator=( const DenseCP& rhs );


        /** Constructs an empty QP with dimensions nV and nC */
        returnValue init( uint nV_, uint nC_ );



//         returnValue setBounds( const Vector &lb,
//                                const Vector &ub  );


        /** Returns whether or not the conic program is an LP */
        inline BooleanType isLP () const;

        /** Returns whether or not the conic program is an LP */
        inline BooleanType isQP () const;

        /** Returns whether or not the conic program is an SDP */
        inline BooleanType isSDP() const;



        /** Returns the number of variables */
        inline uint getNV() const;

        /** Returns the number of linear constraints */
        inline uint getNC() const;


        /** Sets the primal and dual solution converting the dual solution   \n
         *  into the internal format (this routine expects a vector y of     \n
         *  dimension nV + nC, where nC is number of linear constraints and  \n
         *  nV the number of variables (= number of bounds) ).               \n
         *                                                                   \n
         *  \return SUCCESSFUL_RETURN                                        \n
         */
        returnValue setQPsolution( const Vector &x_, const Vector &y_ );


        /** Sets the primal and dual solution converting the dual solution   \n
         *  into the internal format (this routine expects a vector y of     \n
         *  dimension nV + nC, where nC is number of linear constraints and  \n
         *  nV the number of variables (= number of bounds) ).               \n
         *                                                                   \n
         *  \return SUCCESSFUL_RETURN                                        \n
         */
        Vector getMergedDualSolution( ) const;





    //
    // PUBLIC DATA MEMBERS:
    //
    public:


    // DIMENSIONS OF THE CP:
    // ---------------------

    uint         nS;    /**< Number of SDP constraints      */


    // DENSE CP IN MATRIX-VECTOR FORMAT:
    // -------------------------------------------------------

    Matrix        H;    /**< The Hessian matrix             */
    Vector        g;    /**< The objective gradient         */

    Vector       lb;    /**< Simple lower bounds            */
    Vector       ub;    /**< Simple upper bounds            */

    Matrix        A;    /**< Constraint matrix              */
    Vector      lbA;    /**< Constraint lower bounds        */
    Vector      ubA;    /**< Constraint upper bounds        */

    Matrix      **B;    /**< SDP constraint tensor          */
    Vector     *lbB;    /**< SDP lower bounds               */
    Vector     *ubB;    /**< SDP upper bounds               */


    // SOLUTION OF THE DENSE CP:
    // -------------------------------------------------------
    Vector       *x;    /**< Primal Solution                */

    Vector     *ylb;    /**< Dual solution, lower bound     */
    Vector     *yub;    /**< Dual solution, upper bound     */

    Vector    *ylbA;    /**< Dual solution, LP lower bound  */
    Vector    *yubA;    /**< Dual solution, LP upper bound  */

    Vector   **ylbB;    /**< Dual solution, SDB lower bound */
    Vector   **yubB;    /**< Dual solution, SDP upper bound */



    // PROTECTED MEMBER FUNCTIONS:
    // ---------------------------
    protected:

    void copy (const DenseCP& rhs);
    void clean();
};


CLOSE_NAMESPACE_ACADO


#include <acado/conic_program/dense_cp.ipp>


#endif  // ACADO_TOOLKIT_DENSE_CP_HPP

/*
 *  end of file
 */
