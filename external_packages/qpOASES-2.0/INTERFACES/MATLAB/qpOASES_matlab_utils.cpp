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
 *	\file INTERFACES/MATLAB/qpOASES_matlab_utils.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Collects utility functions for Interface to Matlab(R) that 
 *	enables to call qpOASES as a MEX function.
 *
 */


#include "mex.h"
#include "matrix.h"
#include "string.h"


/*
 *	g e t S t a t u s
 */
double getStatus( returnValue returnvalue )
{
	/* determine status from returnvalue */
	switch ( returnvalue )
	{
		case SUCCESSFUL_RETURN:
			return 0.0;

		case RET_MAX_NWSR_REACHED:
			return 1.0;

		default:
			return -1.0;
	}
}


/*
 *	s m a r t D i m e n s i o n C h e c k
 */
returnValue smartDimensionCheck(	double** input, unsigned int m, unsigned int n, BooleanType emptyAllowed,
									const mxArray* prhs[], int idx
									)
{
	/* If index is negative, the input does not exist. */
	if ( idx < 0 )
	{
		*input = 0;
		return SUCCESSFUL_RETURN;
	}

	/* Otherwise the input has been passed by the user. */
	if ( mxIsEmpty( prhs[ idx ] ) )
	{
		/* input is empty */
		if ( emptyAllowed == BT_TRUE )
		{
			*input = 0;
			return SUCCESSFUL_RETURN;
		}
		else
		{
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );
			return RET_INVALID_ARGUMENTS;
		}
	}
	else
	{
		/* input is non-empty */
		if ( ( mxGetM( prhs[ idx ] ) == m ) && ( mxGetN( prhs[ idx ] ) == n ) )
		{
			*input = (double*) mxGetPr( prhs[ idx ] );
			return SUCCESSFUL_RETURN;
		}
		else
		{
			mexErrMsgTxt( "ERROR (qpOASES): Input dimension mismatch!" );
			return RET_INVALID_ARGUMENTS;
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	end of file
 */
