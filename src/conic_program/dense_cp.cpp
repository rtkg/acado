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
 *    \file src/conic_program/dense_cp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/conic_program/dense_cp.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DenseCP::DenseCP( ){

    nS = 0;

    B   = 0;
    lbB = 0;
    ubB = 0;

    x   = 0;
    ylb = 0;
    yub = 0;

    ylbA = 0;
    yubA = 0;

    ylbB = 0;
    yubB = 0;
}


DenseCP::DenseCP( const DenseCP& rhs ){

    copy(rhs);
}


DenseCP::~DenseCP( ){

    clean();
}


DenseCP& DenseCP::operator=( const DenseCP& rhs ){

    if ( this != &rhs ){

        clean()  ;
        copy(rhs);
    }
    return *this;
}



void DenseCP::copy( const DenseCP& rhs ){

    uint run1, run2;

    nS = rhs.nS;

    H   = rhs.H;
    g   = rhs.g;

    lb  = rhs.lb;
    ub  = rhs.ub;

    A   = rhs.A;
    lbA = rhs.lbA;
    ubA = rhs.ubA;


    if( rhs.B != 0 ){
        B = (Matrix**)calloc(nS,sizeof(Matrix*));
        for( run1 = 0; run1 < nS; run1++ ){
            B[run1] = new Matrix[ getNV() ];
            for( run2 = 0; run2 < getNV(); run2++ )
                B[run1][run2] = rhs.B[run1][run2];
        }
    }
    else B = 0;


    if( rhs.lbB != 0 ){
        lbB = (Vector*)calloc(nS,sizeof(Vector));
        for( run1 = 0; run1 < nS; run1++ )
            lbB[run1] = rhs.lbB[run1];
    }
    else lbB = 0;

    if( rhs.ubB != 0 ){
        ubB = (Vector*)calloc(nS,sizeof(Vector));
        for( run1 = 0; run1 < nS; run1++ )
            ubB[run1] = rhs.ubB[run1];
    }
    else ubB = 0;


    if( rhs.x != 0 ) x = new Vector(*rhs.x);
    else             x = 0                 ;

    if( rhs.ylb != 0 ) ylb = new Vector(*rhs.ylb);
    else               ylb = 0                   ;

    if( rhs.yub != 0 ) yub = new Vector(*rhs.yub);
    else               yub = 0                   ;

    if( rhs.ylbA != 0 ) ylbA = new Vector(*rhs.ylbA);
    else                ylbA = 0                    ;

    if( rhs.yubA != 0 ) yubA = new Vector(*rhs.yubA);
    else                yubA = 0                    ;

    if( nS > 0 ){

        ylbB = (Vector**)calloc(nS,sizeof(Vector*));
        yubB = (Vector**)calloc(nS,sizeof(Vector*));

        for( run1 = 0; run1 < nS; run1++ ){
            if( rhs.ylbB[run1] != 0 ) ylbB[run1] = new Vector(*rhs.ylbB[run1]);
            else                      ylbB[run1] = 0                          ;

            if( rhs.yubB[run1] != 0 ) yubB[run1] = new Vector(*rhs.yubB[run1]);
            else                      yubB[run1] = 0                          ;
        }
    }
    else{
        ylbB = 0;
        yubB = 0;
    }
}



void DenseCP::clean(){

    uint run1;

    for( run1 = 0; run1 < nS; run1++ ){

        if( B[run1]    != 0 ) delete[]    B[run1];
        if( ylbB[run1] != 0 ) delete   ylbB[run1];
        if( yubB[run1] != 0 ) delete   yubB[run1];
    }

    if( B    != 0 ) free(B)   ;
    if( ylbB != 0 ) free(ylbB);
    if( yubB != 0 ) free(yubB);

    if( lbB != 0 ) free(lbB);
    if( ubB != 0 ) free(ubB);

    if( ylbA != 0 ) delete ylbA;
    if( yubA != 0 ) delete yubA;

    if( ylb != 0 ) delete ylb;
    if( yub != 0 ) delete yub;

    if( x != 0 ) delete x;
}


returnValue DenseCP::init( uint nV_, uint nC_ ){

    H.init(nV_,nV_);
    A.init(nC_,nV_);

    return SUCCESSFUL_RETURN;
}


returnValue DenseCP::setQPsolution( const Vector &x_, const Vector &y_ ){

    uint run1;
    clean();

    ASSERT( x_.getDim() == getNV()           );
    ASSERT( y_.getDim() == getNV() + getNC() );


    // SET THE PRIMAL SOLUTION:
    // ------------------------
    x = new Vector(x_);


    // SET THE DUAL SOLUTION FOR THE BOUNDS:
    // -------------------------------------
    ylb = new Vector( getNV() );
    yub = new Vector( getNV() );

    for( run1 = 0; run1 < getNV(); run1++ ){
        if( fabs(x_(run1)-lb(run1)) <= BOUNDTOL ){
            ylb->operator()(run1) = y_(run1);
            yub->operator()(run1) = 0.0     ;
        }
        else{
            ylb->operator()(run1) = 0.0     ;
            yub->operator()(run1) = y_(run1);
        }
    }


    // SET THE DUAL SOLUTION FOR THE CONSTRAINTS:
    // ------------------------------------------
    Vector tmp = A*x_;
    ylbA = new Vector( getNC() );
    yubA = new Vector( getNC() );

    for( run1 = 0; run1 < getNC(); run1++ ){
        if( fabs(tmp(run1)-lbA(run1)) <= BOUNDTOL ){
            ylbA->operator()(run1) = y_(getNV()+run1);
            yubA->operator()(run1) = 0.0             ;
        }
        else{
            ylbA->operator()(run1) = 0.0             ;
            yubA->operator()(run1) = y_(getNV()+run1);
        }
    }


    return SUCCESSFUL_RETURN;
}


Vector DenseCP::getMergedDualSolution( ) const
{
	Vector dualSolution( getNV()+getNC() );

	uint i;
	for( i=0; i<getNV(); ++i )
	{
		if ( acadoIsGreater( fabs( (*ylb)(i) ),1e-10 ) == BT_TRUE )
			dualSolution(i) = (*ylb)(i);
		else
			dualSolution(i) = (*yub)(i);
	}

	for( i=0; i<getNC(); ++i )
	{
		if ( acadoIsGreater( fabs( (*ylbA)(i) ),1e-10 ) == BT_TRUE )
			dualSolution( getNV()+i ) = (*ylbA)(i);
		else
			dualSolution( getNV()+i ) = (*yubA)(i);
	}

	return dualSolution;
}



CLOSE_NAMESPACE_ACADO

// end of file.
