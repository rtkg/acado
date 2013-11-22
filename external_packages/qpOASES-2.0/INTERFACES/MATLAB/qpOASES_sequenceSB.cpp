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
 *	\file INTERFACES/MATLAB/qpOASES_sequenceSB.cpp
 *	\author Hans Joachim Ferreau, Aude Perrin
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function
 *  (variant for simply bounded QPs).
 *
 */



#include <QProblemB.hpp>


using namespace qpOASES;

#include <qpOASES_matlab_utils.cpp>


/* global pointer to QP object */
static QProblemB* globalQPB = 0;


/*
 *	a l l o c a t e G l o b a l Q P r o b l e m B I n s t a n c e
 */
void allocateGlobalQProblemBInstance(	int nV
										)
{
	globalQPB = new QProblemB( nV );

	globalQPB->setPrintLevel( PL_LOW );
	#ifdef __DEBUG__
	globalQPB->setPrintLevel( PL_HIGH );
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	globalQPB->setPrintLevel( PL_NONE );
	#endif

	return;
}


/*
 *	d e l e t e G l o b a l Q P r o b l e m B I n s t a n c e
 */
void deleteGlobalQProblemBInstance( )
{
	if ( globalQPB != 0 )
	{
		delete globalQPB;
		globalQPB = 0;
	}

	return;
}


/*
 *	i n i t S B
 */
void initSB(	int nV,
				const double* const H, const double* const g,
				const double* const lb, const double* const ub,
				int nWSR,
				const double* const x0,
				int nOutputs,
				double* obj, double* x, double* y, double* status, double* nWSRout
				)
{
	/* 1) Setup initial QP. */
	deleteGlobalQProblemBInstance( );
	allocateGlobalQProblemBInstance( nV );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

	if ( x0 == 0 )
		returnvalue = globalQPB->init( H,g,lb,ub, nWSR,0 );
	else
		returnvalue = globalQPB->init( H,g,lb,ub, nWSR,0, x0,0,0 );

	/* 3) Assign lhs arguments. */
	*obj = globalQPB->getObjVal( );
	if ( nOutputs >= 2 )
	{
		globalQPB->getPrimalSolution( x );

		if ( nOutputs >= 3 )
		{
			globalQPB->getDualSolution( y );

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
 *	h o t s t a r t S B
 */
void hotstartSB(	const double* const g,
					const double* const lb, const double* const ub,
					int nWSR,
					int nOutputs,
					double* obj, double* x, double* y, double* status, double* nWSRout
					)
{
	/* 1) Solve QP. */
	returnValue returnvalue = globalQPB->hotstart( g,lb,ub, nWSR,0 );

	/* 2) Assign lhs arguments. */
	*obj = globalQPB->getObjVal( );
	if ( nOutputs >= 2 )
	{
		globalQPB->getPrimalSolution( x );

		if ( nOutputs >= 3 )
		{
			globalQPB->getDualSolution( y );

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
	/* inputs */
	char* typeString;
	double *H, *g, *lb, *ub, *nWSRin, *x0;

	/* outputs */
	double *obj, *x=0, *y=0, *status=0, *nWSRout=0;

	/* dimensions */
	unsigned int nV;


	/* I) CONSISTENCY CHECKS: */
	/* 1) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs != 1 ) && ( nrhs != 4 ) && ( nrhs != 5 ) && ( nrhs != 6 ) && ( nrhs != 7 ) )
		mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceSB' for further information." );

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
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		if ( nrhs != 1 )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		/* Cleanup global QProblemB instance. */
		deleteGlobalQProblemBInstance( );
		return;
	}

	/* 2) Hotstart. */
	if ( ( strcmp( typeString,"h" ) == 0 ) || ( strcmp( typeString,"H" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		if ( ( nrhs != 4 ) && ( nrhs != 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		/* has QP been initialised? */
		if ( globalQPB == 0 )
			mexErrMsgTxt( "ERROR (qpOASES): QP sequence needs to be initialised first!" );


		/* Check inputs dimensions and assign pointers to inputs. */
		nV = globalQPB->getNV( );

		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,1 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,3 ) != SUCCESSFUL_RETURN )
			return;

		if ( nrhs == 4 )
		{
			/* use default value for nWSR */
			nWSRin = new double[1];
			nWSRin[0] = 5.0 * ((double) nV);
		}
		else
		{
			/* nWSR specified by the user */
			if ( smartDimensionCheck( &nWSRin,1,1, BT_FALSE,prhs,4 ) != SUCCESSFUL_RETURN )
				return;
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
				plhs[2] = mxCreateDoubleMatrix( nV, 1, mxREAL );
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


		/* call qpOASES */
		hotstartSB(	g,
					lb,ub,
					((int) *nWSRin),
					nlhs,
					obj,x,y,status,nWSRout
					);

		if ( nrhs == 4 )
			delete[] nWSRin;
		return;
	}

	/* 3) Init (without or with initial guess for primal solution). */
	if ( ( strcmp( typeString,"i" ) == 0 ) || ( strcmp( typeString,"I" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 5 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		if ( ( nrhs != 5 ) && ( nrhs != 6 ) && ( nrhs != 7 ) )
			mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequenceSB' for further information." );

		/* ensure that data is given in double precision */
		if ( ( mxIsDouble( prhs[1] ) == 0 ) ||
			 ( mxIsDouble( prhs[2] ) == 0 ) )
			mexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in double precision!" );

		/* ensure that Hessian matrix is stored in dense format */
		if ( mxIsSparse( prhs[1] ) != 0 )
			mexErrMsgTxt( "ERROR (qpOASES): Matrices must not be stored in sparse format!" );


		/* Check inputs dimensions and assign pointers to inputs. */
		nV = mxGetM( prhs[1] ); /* row number of Hessian matrix */

		if ( mxGetN( prhs[1] ) != nV )
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );

		H = (double*) mxGetPr( prhs[1] );

		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,2 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,3 ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
			return;

		if ( nrhs == 5 )
		{
			/* no nWSR, no x0 */
			nWSRin = new double[1];
			x0 = 0;
			/* use default value for nWSR */
			nWSRin[0] = 5.0 * ((double) nV);
		}
		else
		{
			if ( nrhs == 6 )
			{
				/* nWSR, but no x0 */
				if ( smartDimensionCheck( &nWSRin,1,1, BT_FALSE,prhs,5 ) != SUCCESSFUL_RETURN )
					return;
				x0 = 0;
			}
			else
			{
				/* both nWSR and x0 */
				if ( smartDimensionCheck( &nWSRin,1,1, BT_FALSE,prhs,5 ) != SUCCESSFUL_RETURN )
					return;

			if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,6 ) != SUCCESSFUL_RETURN )
				return;
			}
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
				plhs[2] = mxCreateDoubleMatrix( nV, 1, mxREAL );
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


		/* call qpOASES */
		initSB(	nV,
				H,g,
				lb,ub,
				((int) *nWSRin),
				x0,
				nlhs,
				obj,x,y,status,nWSRout
				);

		if ( nrhs == 5 )
			delete[] nWSRin;
		return;
	}
}

/*
 *	end of file
 */
