/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file INTERFACES/MATLAB/qpOASES.cpp
 *	\author Hans Joachim Ferreau, Aude Perrin
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function.
 *
 */



#include "QProblem.hpp"


using namespace qpOASES;

#include <qpOASES_matlab_utils.cpp>


/*
 *	q p O A S E S m e x _ c o n s t r a i n t s
 */
void qpOASESmex_constraints(	int nV, int nC, int nP,
								double* H, double* g, double* A,
								double* lb, double* ub, double* lbA, double* ubA,
								int nWSRin,
								double* x0,
								int nOutputs,
								double* obj, double* x, double* y, double* status, double* nWSRout
								)
{
	/* 1) Setup initial QP. */
	QProblem QP( nV,nC );

	QP.setPrintLevel( PL_LOW );
	#ifdef __DEBUG__
	QP.setPrintLevel( PL_HIGH );
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	QP.setPrintLevel( PL_NONE );
	#endif

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	if ( x0 == 0 )
		returnvalue = QP.init( H,g,A,lb,ub,lbA,ubA, nWSRin,0 );
	else
		returnvalue = QP.init( H,g,A,lb,ub,lbA,ubA, nWSRin,0, x0,0,0,0 );

	/* 3) Solve remaining QPs and assign lhs arguments. */
	/*    Set up pointers to the current QP vectors */
	double* g_current   = g;
	double* lb_current  = lb;
	double* ub_current  = ub;
	double* lbA_current = lbA;
	double* ubA_current = ubA;

	/* Loop through QP sequence. */
	for ( int k=0; k<nP; ++k )
	{
		if ( k != 0 )
		{
			/* update pointers to the current QP vectors */
			g_current = &(g[k*nV]);
			if ( lb != 0 )
				lb_current = &(lb[k*nV]);
			if ( ub != 0 )
				ub_current = &(ub[k*nV]);
			if ( lbA != 0 )
				lbA_current = &(lbA[k*nC]);
			if ( ubA != 0 )
				ubA_current = &(ubA[k*nC]);

			returnvalue = QP.hotstart( g_current,lb_current,ub_current,lbA_current,ubA_current, nWSRin,0 );
		}

		/* write results into output vectors */
		obj[k] = QP.getObjVal( );

		if ( nOutputs >= 2 )
		{
			QP.getPrimalSolution( &(x[k*nV]) );

			if ( nOutputs >= 3 )
			{
				QP.getDualSolution( &(y[k*(nV+nC)]) );

				if ( nOutputs >= 4 )
				{
					status[k] = getStatus( returnvalue );

					if ( nOutputs == 5 )
						nWSRout[k] = (double) nWSRin;
				}
			}
		}
	}

	return;
}


/*
 *	q p O A S E S m e x _ b o u n d s
 */
void qpOASESmex_bounds(	int nV, int nP,
						double* H, double* g,
						double* lb, double* ub,
						int nWSRin,
						double* x0,
						int nOutputs,
						double* obj, double* x, double* y, double* status, double* nWSRout
						)
{
	/* 1) Setup initial QP. */
	QProblemB QP( nV );

	QP.setPrintLevel( PL_LOW );
	#ifdef __DEBUG__
	QP.setPrintLevel( PL_HIGH );
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	QP.setPrintLevel( PL_NONE );
	#endif

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	if ( x0 == 0 )
		returnvalue = QP.init( H,g,lb,ub, nWSRin,0 );
	else
		returnvalue = QP.init( H,g,lb,ub, nWSRin,0, x0,0,0 );

	/* 3) Solve remaining QPs and assign lhs arguments. */
	/*    Set up pointers to the current QP vectors */
	double* g_current  = g;
	double* lb_current = lb;
	double* ub_current = ub;

	/* Loop through QP sequence. */
	for ( int k=0; k<nP; ++k )
	{
		if ( k != 0 )
		{
			/* update pointers to the current QP vectors */
			g_current = &(g[k*nV]);
			if ( lb != 0 )
				lb_current = &(lb[k*nV]);
			if ( ub != 0 )
				ub_current = &(ub[k*nV]);

			returnvalue = QP.hotstart( g_current,lb_current,ub_current, nWSRin,0 );
		}

		/* write results into output vectors */
		obj[k] = QP.getObjVal( );

		if ( nOutputs >= 2 )
		{
			QP.getPrimalSolution( &(x[k*nV]) );

			if ( nOutputs >= 3 )
			{
				QP.getDualSolution( &(y[k*(nV)]) );

				if ( nOutputs >= 4 )
				{
					status[k] = getStatus( returnvalue );

					if ( nOutputs == 5 )
						nWSRout[k] = (double) nWSRin;
				}
			}
		}
	}

	return;
}



/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	unsigned int i, j;

	/* inputs */
	double *H, *g, *A, *A_for, *lb, *ub, *lbA, *ubA, *nWSRin, *x0;
	int H_idx, g_idx, A_idx, lb_idx, ub_idx, lbA_idx, ubA_idx, nWSR_idx=-1, x0_idx=-1;

	/* outputs */
	double *obj, *x=0, *y=0, *status=0, *nWSRout=0;

	/* dimensions */
	unsigned int nV, nC, nP;

	bool withConstraints;


	/* I) CONSISTENCY CHECKS: */
	/* 1) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs != 4 ) && ( nrhs != 5 ) && ( nrhs != 6 ) && ( nrhs != 7 ) && ( nrhs != 8 ) && ( nrhs != 9 ) )
		mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\n    Type 'help qpOASES' for further information." );

	/* 2) Check for proper number of output arguments. */
	if ( nlhs > 5 )
		mexErrMsgTxt( "ERROR (qpOASES): At most five output arguments are allowed: \n    [obj,x,y,status,nWSRout]!" );
	if ( nlhs < 1 )
		mexErrMsgTxt( "ERROR (qpOASES): At least one output argument is required: [obj,...]!" );


	/* II) PREPARE RESPECTIVE QPOASES FUNCTION CALL: */
	/*     Choose between QProblem and QProblemB object and assign the corresponding
	 *     indices of the input pointer array in to order to access QP data correctly. */
	H_idx = 0;
	g_idx = 1;
	nV = mxGetM( prhs[ H_idx ] ); /* row number of Hessian matrix */
	nP = mxGetN( prhs[ g_idx ] ); /* number of columns of the gradient matrix (vectors series have to be stored columnwise!) */

	/* 1) Simply bounded QP. */
	if ( ( nrhs == 4 ) || ( nrhs == 5 ) || ( nrhs == 6 ) )
	{
		lb_idx   = 2;
		ub_idx   = 3;

		if ( nrhs == 4 ) /* nWSR not specified */
			nWSR_idx = -1;
		else
			nWSR_idx = 4;

		if ( nrhs == 6 ) /* x0 specified */
			x0_idx = 5;
		else
			x0_idx = -1;

		withConstraints = false;
	}

	/* 2) QP comprising constraint arguments (they might be empty!). */
	if ( ( nrhs == 7 ) || ( nrhs == 8 ) || ( nrhs == 9 ) )
	{
		A_idx = 2;
		nC = mxGetM( prhs[ A_idx ] ); /* row number of constraint matrix */

		/* If constraint matrix is empty, use a QProblemB object! */
		if ( mxIsEmpty( prhs[ A_idx ] ) )
		{
			lb_idx   = 3;
			ub_idx   = 4;

			if ( nrhs == 7 ) /* nWSR not specified */
				nWSR_idx = -1;
			else
				nWSR_idx = 7;

			if ( nrhs == 9 ) /* x0 specified */
				x0_idx = 8;
			else
				x0_idx = -1;

			withConstraints = false;
		}
		else
		{
			lb_idx   = 3;
			ub_idx   = 4;
			lbA_idx  = 5;
			ubA_idx  = 6;

			if ( nrhs == 7 ) /* nWSR not specified */
				nWSR_idx = -1;
			else
				nWSR_idx = 7;

			if ( nrhs == 9 ) /* x0 specified */
				x0_idx = 8;
			else
				x0_idx = -1;

			withConstraints = true;
		}
	}


	/* III) ACTUALLY PERFORM QPOASES FUNCTION CALL: */
	if ( withConstraints == false )
	{
		/* 1) Call special bound version (using QProblemB class). */

		/* ensure that data is given in double precision */
		if ( ( mxIsDouble( prhs[ H_idx ] ) == 0 ) ||
			 ( mxIsDouble( prhs[ g_idx ] ) == 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in double precision!" );

		/* ensure that matrices are stored in dense format */
		if ( mxIsSparse( prhs[ H_idx ] ) != 0 )
			mexErrMsgTxt( "ERROR (qpOASES): Matrices must not be stored in sparse format!" );

		/* Check inputs dimensions and assign pointers to inputs. */
		if ( mxGetN( prhs[ H_idx ] ) != nV )
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );

		H = (double*) mxGetPr( prhs[ H_idx ] );

		if ( smartDimensionCheck( &g,nV,nP, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,nP, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,nP, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( nWSR_idx != -1 )
		{
			if ( smartDimensionCheck( &nWSRin,1,1, BT_FALSE,prhs,nWSR_idx ) != SUCCESSFUL_RETURN )
				return;
		}
		else
		{
			/* use default value for nWSR */
			nWSRin = new double[1];
			nWSRin[0] = 5.0 * ((double) nV);
		}

		if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,x0_idx ) != SUCCESSFUL_RETURN )
			return;

		/* Create output vectors and assign pointers to them. */
		plhs[0] = mxCreateDoubleMatrix( 1, nP, mxREAL );
		obj = mxGetPr( plhs[0] );
		if ( nlhs >= 2 )
		{
			plhs[1] = mxCreateDoubleMatrix( nV, nP, mxREAL );
			x       = mxGetPr( plhs[1] );

			if ( nlhs >= 3 )
			{
				plhs[2] = mxCreateDoubleMatrix( nV, nP, mxREAL );
				y  = mxGetPr( plhs[2] );

				if ( nlhs >= 4 )
				{
					plhs[3] = mxCreateDoubleMatrix( 1, nP, mxREAL );
					status  = mxGetPr( plhs[3] );

					if ( nlhs == 5 )
					{
						plhs[4] = mxCreateDoubleMatrix( 1, nP, mxREAL );
						nWSRout = mxGetPr( plhs[4] );
					}
				}
			}
		}

		/* call qpOASES */
		qpOASESmex_bounds(	nV,nP,
							H,g,
							lb,ub,
							((int) *nWSRin),
							x0,
							nlhs,
							obj,x,y,status,nWSRout
							);

		if ( nWSR_idx == -1 )
			delete[] nWSRin;
		return;
	}
	else
	{
		/* 2) Call usual version including constraints (using QProblem class) */

		/* ensure that data is given in double precision */
		if ( ( mxIsDouble( prhs[ H_idx ] ) == 0 ) ||
			 ( mxIsDouble( prhs[ A_idx ] ) == 0 ) ||
			 ( mxIsDouble( prhs[ g_idx ] ) == 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in double precision!" );

		/* ensure that matrices are stored in dense format */
		if ( ( mxIsSparse( prhs[ H_idx ] ) != 0 ) || ( mxIsSparse( prhs[ A_idx ] ) != 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Matrices must not be stored in sparse format!" );

		/* Check inputs dimensions and assign pointers to inputs. */
		if ( ( mxGetN( prhs[ H_idx ] ) != nV ) || ( mxGetN( prhs[ A_idx ] ) != nV ) )
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );

		H     = (double*) mxGetPr( prhs[ H_idx ] );
		A_for = (double*) mxGetPr( prhs[ A_idx ] );

		if ( smartDimensionCheck( &g,nV,nP, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,nP, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,nP, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,nP, BT_TRUE,prhs,lbA_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,nP, BT_TRUE,prhs,ubA_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( nWSR_idx != -1 )
		{
			if ( smartDimensionCheck( &nWSRin,1,1, BT_FALSE,prhs,nWSR_idx ) != SUCCESSFUL_RETURN )
				return;
		}
		else
		{
			/* use default value for nWSR */
			nWSRin = new double[1];
			nWSRin[0] = 5.0 * ((double) nV+nC);
		}

		if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,x0_idx ) != SUCCESSFUL_RETURN )
			return;


		/* Convert constraint matrix A from FORTRAN to C style
		 * (not necessary for H as it should be symmetric!). */
		if ( nC > 0 )
		{
			A   = new double[nC*nV];
			for ( i=0; i<nC; ++i )
				for ( j=0; j<nV; ++j )
					A[i*nV + j] = A_for[j*nC + i];
		}
		else
		{
			A = 0;
		}


		/* Create output vectors and assign pointers to them. */
		plhs[0] = mxCreateDoubleMatrix( 1, nP, mxREAL );
		obj     = mxGetPr( plhs[0] );

		if ( nlhs >= 2 )
		{
			plhs[1] = mxCreateDoubleMatrix( nV, nP, mxREAL );
			x       = mxGetPr( plhs[1] );

			if ( nlhs >= 3 )
			{
				plhs[2] = mxCreateDoubleMatrix( nV+nC, nP, mxREAL );
				y  = mxGetPr( plhs[2] );

				if ( nlhs >= 4 )
				{
					plhs[3] = mxCreateDoubleMatrix( 1, nP, mxREAL );
					status  = mxGetPr( plhs[3] );

					if ( nlhs == 5 )
					{
						plhs[4] = mxCreateDoubleMatrix( 1, nP, mxREAL );
						nWSRout = mxGetPr( plhs[4] );
					}
				}
			}
		}


		/* Call qpOASES. */
		qpOASESmex_constraints(	nV,nC,nP,
								H,g,A,
								lb,ub,lbA,ubA,
								((int) *nWSRin),
								x0,
								nlhs,
								obj,x,y,status,nWSRout
								);


		if ( nWSR_idx == -1 )
			delete[] nWSRin;
		if ( nC > 0 )
			delete[] A;
		return;
	}
}

/*
 *	end of file
 */
