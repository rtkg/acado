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
 *	\file INTERFACES/MATLAB/qpOASES_sequenceVM.cpp
 *	\author Hans Joachim Ferreau, Aude Perrin
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function
 *  (variant for QPs with varying matrices).
 *
 */



#include <SQProblem.hpp>


using namespace qpOASES;

#include <qpOASES_matlab_utils.cpp>


/* global pointer to QP object */
static SQProblem* globalSQP = 0;


/*
 *	a l l o c a t e G l o b a l S Q P r o b l e m I n s t a n c e
 */
void allocateGlobalSQProblemInstance(	int nV, int nC
										)
{
	globalSQP = new SQProblem( nV,nC );

	globalSQP->setPrintLevel( PL_LOW );
	#ifdef __DEBUG__
	globalSQP->setPrintLevel( PL_HIGH );
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	globalSQP->setPrintLevel( PL_NONE );
	#endif

	return;
}


/*
 *	d e l e t e G l o b a l S Q P r o b l e m I n s t a n c e
 */
void deleteGlobalSQProblemInstance( )
{
	if ( globalSQP != 0 )
	{
		delete globalSQP;
		globalSQP = 0;
	}

	return;
}


/*
 *	i n i t V M
 */
void initVM(	int nV, int nC,
				const double* const H, const double* const g, const double* const A,
				const double* const lb, const double* const ub, const double* const lbA, const double* const ubA,
				int nWSR,
				const double* const x0,
				int nOutputs,
				double* obj, double* x, double* y, double* status, double* nWSRout
				)
{
	/* 1) Setup initial QP. */
	deleteGlobalSQProblemInstance( );
	allocateGlobalSQProblemInstance( nV,nC );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	if ( x0 == 0 )
		returnvalue = globalSQP->init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
	else
		returnvalue = globalSQP->init( H,g,A,lb,ub,lbA,ubA, nWSR,0, x0,0,0,0 );

	/* 3) Assign lhs arguments. */
	*obj = globalSQP->getObjVal( );
	if ( nOutputs >= 2 )
	{
		globalSQP->getPrimalSolution( x );

		if ( nOutputs >= 3 )
		{
			globalSQP->getDualSolution( y );

			if ( nOutputs >= 4 )
			{
				*status = getStatus( returnvalue );

				if ( nOutputs >= 5 )
					*nWSRout = ((double) nWSR);
			}
		}
	}

	return;
}


/*
 *	h o t s t a r t V M
 */
void hotstartVM(	const double* const H, const double* const g, const double* const A,
					const double* const lb, const double* const ub, const double* const lbA, const double* const ubA,
					int nWSR,
					int nOutputs,
					double* obj, double* x, double* y, double* status, double* nWSRout
					)
{
	/* 1) Solve QP. */
	returnValue returnvalue = globalSQP->hotstart( H,g,A,lb,ub,lbA,ubA, nWSR,0 );

	/* 2) Assign lhs arguments. */
	*obj = globalSQP->getObjVal( );
	if ( nOutputs >= 2 )
	{
		globalSQP->getPrimalSolution( x );

		if ( nOutputs >= 3 )
		{
			globalSQP->getDualSolution( y );

			if ( nOutputs >= 4 )
			{
				*status = getStatus( returnvalue );

				if ( nOutputs >= 5 )
					*nWSRout = ((double) nWSR);
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
	char* typeString;
	double *H, *g, *A, *A_for, *lb, *ub, *lbA, *ubA, *nWSRin, *x0;

	/* outputs */
	double *obj, *x=0, *y=0, *status=0, *nWSRout=0;

	/* dimensions */
	unsigned int nV, nC;


	/* I) CONSISTENCY CHECKS: */
	/* 1) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs != 1 ) && ( nrhs != 8 ) && ( nrhs != 9 ) && ( nrhs != 10 ) )
		mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

	/* 2) Ensure that first input is a string (and if so, read it). */
	if ( mxIsChar( prhs[0] ) != 1 )
		mexErrMsgTxt( "ERROR (qpOASES): First input argument must be a string!" );

	typeString = (char*) mxGetPr( prhs[0] );


	/* II) SELECT RESPECTIVE QPOASES FUNCTION CALL: */
	/* 1) Cleanup. */
	if ( ( strcmp( typeString,"c" ) == 0 ) || ( strcmp( typeString,"C" ) == 0 ) )
	{
		/* consistency checks */
		if ( nlhs != 0 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		/* Cleanup global SQProblem instance. */
		deleteGlobalSQProblemInstance( );
		return;
	}


	/* 2) Hotstart OR
	 * 3) Init (without or with initial guess for primal solution). */
	if ( ( strcmp( typeString,"h" ) == 0 ) || ( strcmp( typeString,"H" ) == 0 ) ||
		 ( strcmp( typeString,"i" ) == 0 ) || ( strcmp( typeString,"I" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		if ( ( nrhs != 8 ) && ( nrhs != 9 ) && ( nrhs != 10 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

		/* ensure that data is given in double precision */
		if ( ( mxIsDouble( prhs[1] ) == 0 ) ||
			 ( mxIsDouble( prhs[2] ) == 0 ) ||
			 ( mxIsDouble( prhs[3] ) == 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in double precision!" );

		/* ensure that matrices are stored in dense format */
		if ( ( mxIsSparse( prhs[1] ) != 0 ) || ( mxIsSparse( prhs[3] ) != 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Matrices must not be stored in sparse format!" );


		/* Check inputs dimensions and assign pointers to inputs. */
		nV = mxGetM( prhs[1] ); /* row number of Hessian matrix */
		nC = mxGetM( prhs[3] ); /* row number of constraint matrix */

		if ( ( mxGetN( prhs[1] ) != nV ) || ( ( mxGetN( prhs[3] ) != 0 ) && ( mxGetN( prhs[3] ) != nV ) ) )
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );

		H     = (double*) mxGetPr( prhs[1] );
		A_for = (double*) mxGetPr( prhs[3] );


		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,5 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,6 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,7 ) != SUCCESSFUL_RETURN )
			return;

		if ( nrhs == 8 )
		{
			/* no nWSR, no x0 */
			nWSRin = new double[1];
			x0 = 0;
			/* use default value for nWSR */
			nWSRin[0] = 5.0 * ((double) nV + nC);
		}
		else
		{
			if ( nrhs == 9 )
			{
				/* nWSR, but no x0 */
				if ( smartDimensionCheck( &nWSRin,1,1, BT_FALSE,prhs,8 ) != SUCCESSFUL_RETURN )
					return;
				x0 = 0;
			}
			else
			{
				/* both nWSR and x0 (not possible for hotstart!) */
				if ( ( strcmp( typeString,"h" ) == 0 ) || ( strcmp( typeString,"H" ) == 0 ) )
					mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceVM' for further information." );

				if ( smartDimensionCheck( &nWSRin,1,1, BT_FALSE,prhs,8 ) != SUCCESSFUL_RETURN )
					return;

				if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,9 ) != SUCCESSFUL_RETURN )
					return;
			}
		}


		/* Convert constraint matrix A from FORTRAN to C style
		 * (not necessary for H as it should be symmetric!). */
		if ( nC > 0 )
		{
			/* convert constraint matrix A from FORTRAN to C style (not necessary for H as it should be symmetric!) */
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
		plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL );
		obj = mxGetPr( plhs[0] );
		if ( nlhs >= 2 )
		{
			plhs[1] = mxCreateDoubleMatrix( nV, 1, mxREAL );
			x       = mxGetPr( plhs[1] );

			if ( nlhs >= 3 )
			{
				plhs[2] = mxCreateDoubleMatrix( nV+nC, 1, mxREAL );
				y  = mxGetPr( plhs[2] );

				if ( nlhs >= 4 )
				{
					plhs[3] = mxCreateDoubleMatrix( 1, 1, mxREAL );
					status  = mxGetPr( plhs[3] );

					if ( nlhs == 5 )
					{
						plhs[4] = mxCreateDoubleMatrix( 1, 1, mxREAL );
						nWSRout = mxGetPr( plhs[4] );
					}
				}
			}
		}


		/* Call qpOASES */
		if ( ( ( strcmp( typeString,"h" ) == 0 ) ) || ( strcmp( typeString,"H" ) == 0 ) )
		{
			/* hotstart */
			if ( globalSQP == 0 )
			{
				if ( nC > 0 )
					delete[] A;
				mexErrMsgTxt( "ERROR (qpOASES): QP needs to be initialised first!" );
			}

			if ( ( (int)nV != globalSQP->getNV( ) ) || ( (int)nC != globalSQP->getNC( ) ) )
			{
				if ( nC > 0 )
					delete[] A;
				mexErrMsgTxt( "ERROR (qpOASES): QP dimensions must be constant during a sequence!" );
			}

			hotstartVM(	H,g,A,
						lb,ub,lbA,ubA,
						((int) *nWSRin),
						nlhs,
						obj,x,y,status,nWSRout
						);
		}
		else
		{
			/* init */
			initVM(	nV,nC,
					H,g,A,
					lb,ub,lbA,ubA,
					((int) *nWSRin),
					x0,
					nlhs,
					obj,x,y,status,nWSRout
					);
		}

		if ( nrhs == 8 )
			delete[] nWSRin;
		if ( nC > 0 )
			delete[] A;
		return;
	}
}


/*
 *	end of file
 */
