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
 *    \file src/constraint/coupled_path_constraint.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/constraint/coupled_path_constraint.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


CoupledPathConstraint::CoupledPathConstraint( )
                      :ConstraintElement(){

}

CoupledPathConstraint::CoupledPathConstraint( const Grid& grid_ )
                      :ConstraintElement(grid_, grid_.getNumPoints(), 1 ){

}

CoupledPathConstraint::CoupledPathConstraint( const CoupledPathConstraint& rhs )
                      :ConstraintElement(rhs){

}

CoupledPathConstraint::~CoupledPathConstraint( ){

}

CoupledPathConstraint& CoupledPathConstraint::operator=( const CoupledPathConstraint& rhs ){

    if( this != &rhs ){

        ConstraintElement::operator=(rhs);
    }
    return *this;
}



returnValue CoupledPathConstraint::evaluate( const OCPiterate& iter ){

    int run1, run2;

    const int nc = getNC();
    const int T  = grid.getLastIndex();

    if( nc == 0 )  return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

	Matrix resL( nc, 1 );
	Matrix resU( nc, 1 );

    for( run1 = 0; run1 < nc; run1++ ){
         resL( run1, 0 ) = lb[0][run1];
         resU( run1, 0 ) = ub[0][run1];
	}
    

	Vector result;

    for( run2 = 0; run2 <= T; run2++ ){

        z[run2].setZ( run2, iter );
		result = fcn[run2].evaluate( z[run2] );

        for( run1 = 0; run1 < nc; run1++ ){
             resL( run1, 0 ) -= result(run1);
             resU( run1, 0 ) -= result(run1);
        }
    }

    // STORE THE RESULTS:
    // ------------------

    residuumL.init(1,1);
    residuumU.init(1,1);

    residuumL.setDense( 0, 0, resL );
    residuumU.setDense( 0, 0, resU );

    return SUCCESSFUL_RETURN;
}


returnValue CoupledPathConstraint::evaluateSensitivities(){


    // EVALUATION OF THE SENSITIVITIES:
    // --------------------------------

    int run1,/*run2, */run3;

//     const int nc = getNC();
    const int N  = grid.getNumPoints();


    // EVALUATION OF THE SENSITIVITIES:
    // --------------------------------

    if( bSeed != 0 )
	{

        if( xSeed  != 0 || pSeed  != 0 || uSeed  != 0 || wSeed  != 0 ||
            xSeed2 != 0 || pSeed2 != 0 || uSeed2 != 0 || wSeed2 != 0 )
            return ACADOERROR( RET_WRONG_DEFINITION_OF_SEEDS );

//         double*   bseed1 = new double[nc];

        Matrix bseed_;
        bSeed->getSubBlock( 0, 0, bseed_);

        dBackward.init( 1, 5*N );

        int nBDirs = bSeed->getNumRows( 0, 0 );

        Matrix Dx ( nBDirs, nx );
        Matrix Dxa( nBDirs, na );
        Matrix Dp ( nBDirs, np );
        Matrix Du ( nBDirs, nu );
        Matrix Dw ( nBDirs, nw );

        for( run3 = 0; run3 < N; run3++ )
		{

//             double* dresult1 = new double[fcn[run3].getNumberOfVariables()+1];
// 
            for( run1 = 0; run1 < nBDirs; run1++ )
			{
// 
//                 for( run2 = 0; run2 < nc; run2++ )
//                     bseed1[run2] = bseed_(run1,run2);
// 
//                 for( run2 = 0; run2 <= fcn[run3].getNumberOfVariables(); run2++ )
//                     dresult1[run2] = 0.0;

//                 fcn[run3].AD_backward( 0, bseed1, dresult1 );
				ACADO_TRY( fcn[run3].AD_backward( bseed_.getRow(run1),JJ[run3],0 ) );

				if( nx > 0 ) Dx .setRow( run1, JJ[run3].getX () );
				if( na > 0 ) Dxa.setRow( run1, JJ[run3].getXA() );
				if( np > 0 ) Dp .setRow( run1, JJ[run3].getP () );
				if( nu > 0 ) Du .setRow( run1, JJ[run3].getU () );
				if( nw > 0 ) Dw .setRow( run1, JJ[run3].getW () );

				JJ[run3].setZero( );


//                 for( run2 = 0; run2 < nx; run2++ ){
//                      Dx ( run1, run2 ) = dresult1[y_index[run3][run2]];
//                 }
//                 for( run2 = nx; run2 < nx+na; run2++ ){
//                      Dxa( run1, run2-nx ) = dresult1[y_index[run3][run2]];
//                 }
//                 for( run2 = nx+na; run2 < nx+na+np; run2++ ){
//                      Dp ( run1, run2-nx-na ) = dresult1[y_index[run3][run2]];
//                 }
//                 for( run2 = nx+na+np; run2 < nx+na+np+nu; run2++ ){
//                      Du ( run1, run2-nx-na-np ) = dresult1[y_index[run3][run2]];
//                 }
//                 for( run2 = nx+na+np+nu; run2 < nx+na+np+nu+nw; run2++ ){
//                      Dw ( run1, run2-nx-na-np-nu ) = dresult1[y_index[run3][run2]];
//                 }
			}

			if( nx > 0 ) 
				dBackward.setDense( 0,     run3, Dx  );

			if( na > 0 ) 
				dBackward.setDense( 0,   N+run3, Dxa );
			
			if( np > 0 ) 
				dBackward.setDense( 0, 2*N+run3, Dp  );
			
			if( nu > 0 ) 
				dBackward.setDense( 0, 3*N+run3, Du  );
			
			if( nw > 0 ) 
				dBackward.setDense( 0, 4*N+run3, Dw  );

//             delete[] dresult1;
		}

//         delete[] bseed1  ;
        return SUCCESSFUL_RETURN;
    }
	
	// TODO: implement forward mode

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue CoupledPathConstraint::evaluateSensitivities( const Matrix &seed, BlockMatrix &hessian ){

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



CLOSE_NAMESPACE_ACADO

// end of file.
