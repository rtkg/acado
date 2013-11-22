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
 *	\file SRC/EXTRAS/OQPinterface.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2008-2009
 *
 *	Implementation of an interface comprising several utility functions
 *	for solving test problems from the Online QP Benchmark Collection
 *	(see http://homes.esat.kuleuven.be/~optec/software/onlineQP/).
 *
 */


#include <stdio.h>

#include <EXTRAS/OQPinterface.hpp>
#include <QProblem.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/*
 *	r e a d O Q P d i m e n s i o n s
 */
returnValue readOQPdimensions(	const char* path,
								int& nQP, int& nV, int& nC, int& nEC
								)
{
	/* 1) Setup file name where dimensions are stored. */
	char filename[160];
	snprintf( filename,160,"%sdims.oqp",path );

	/* 2) Load dimensions from file. */
	int dims[4];
	if ( readFromFile( dims,4,filename ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_UNABLE_TO_READ_FILE );

	nQP = dims[0];
	nV  = dims[1];
	nC  = dims[2];
	nEC = dims[3];


	/* consistency check */
	if ( ( nQP <= 0 ) || ( nV <= 0 ) || ( nC < 0 ) || ( nEC < 0 ) )
		return THROWERROR( RET_FILEDATA_INCONSISTENT );

	return SUCCESSFUL_RETURN;
}


/*
 *	r e a d O Q P d a t a
 */
returnValue readOQPdata(	const char* path,
							int& nQP, int& nV, int& nC, int& nEC,
							double** H, double** g, double** A, double** lb, double** ub, double** lbA, double** ubA,
							double** xOpt, double** yOpt, double** objOpt
							)
{
	char filename[160];

	/* consistency check */
	if ( ( H == 0 ) || ( g == 0 ) || ( lb == 0 ) || ( ub == 0 ) )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	/* 1) Obtain OQP dimensions. */
	if ( readOQPdimensions( path, nQP,nV,nC,nEC ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_UNABLE_TO_READ_FILE );


	/* another consistency check */
	if ( ( nC > 0 ) && ( ( A == 0 ) || ( lbA == 0 ) || ( ubA == 0 ) ) )
		return THROWERROR( RET_FILEDATA_INCONSISTENT );


	/* 2) Allocate memory and load OQP data: */
	/* Hessian matrix */
	*H  = new double[nV*nV];
	snprintf( filename,160,"%sH.oqp",path );
	if ( readFromFile( *H,nV,nV,filename ) != SUCCESSFUL_RETURN )
	{
		delete[] *H;
		return THROWERROR( RET_UNABLE_TO_READ_FILE );
	}

	/* gradient vector sequence */
	*g  = new double[nQP*nV];
	snprintf( filename,160,"%sg.oqp",path );
	if ( readFromFile( *g,nQP,nV,filename ) != SUCCESSFUL_RETURN )
	{
		delete[] *g; delete[] *H;
		return THROWERROR( RET_UNABLE_TO_READ_FILE );
	}

	/* lower bound vector sequence */
	*lb  = new double[nQP*nV];
	snprintf( filename,160,"%slb.oqp",path );
	if ( readFromFile( *lb,nQP,nV,filename ) != SUCCESSFUL_RETURN )
	{
		delete[] *lb; delete[] *g; delete[] *H;
		return THROWERROR( RET_UNABLE_TO_READ_FILE );
	}

	/* upper bound vector sequence */
	*ub  = new double[nQP*nV];
	snprintf( filename,160,"%sub.oqp",path );
	if ( readFromFile( *ub,nQP,nV,filename ) != SUCCESSFUL_RETURN )
	{
		delete[] *ub; delete[] *lb; delete[] *g; delete[] *H;
		return THROWERROR( RET_UNABLE_TO_READ_FILE );
	}

	if ( nC > 0 )
	{
		/* Constraint matrix */
		*A   = new double[nC*nV];
		snprintf( filename,160,"%sA.oqp",path );
		if ( readFromFile( *A,nC,nV,filename ) != SUCCESSFUL_RETURN )
		{
			delete[] *A;
			delete[] *ub; delete[] *lb; delete[] *g; delete[] *H;
			return THROWERROR( RET_UNABLE_TO_READ_FILE );
		}

		/* lower constraints' bound vector sequence */
		*lbA = new double[nQP*nC];
		snprintf( filename,160,"%slbA.oqp",path );
		if ( readFromFile( *lbA,nQP,nC,filename ) != SUCCESSFUL_RETURN )
		{
			delete[] *lbA; delete[] *A;
			delete[] *ub; delete[] *lb; delete[] *g; delete[] *H;
			return THROWERROR( RET_UNABLE_TO_READ_FILE );
		}

		/* upper constraints' bound vector sequence */
		*ubA = new double[nQP*nC];
		snprintf( filename,160,"%subA.oqp",path );
		if ( readFromFile( *ubA,nQP,nC,filename ) != SUCCESSFUL_RETURN )
		{
			delete[] *ubA; delete[] *lbA; delete[] *A;
			delete[] *ub; delete[] *lb; delete[] *g; delete[] *H;
			return THROWERROR( RET_UNABLE_TO_READ_FILE );
		}
	}
	else
	{
		*A = 0;
		*lbA = 0;
		*ubA = 0;
	}

	if ( xOpt != 0 )
	{
		/* primal solution vector sequence */
		*xOpt = new double[nQP*nV];
		snprintf( filename,160,"%sx_opt.oqp",path );
		if ( readFromFile( *xOpt,nQP,nV,filename ) != SUCCESSFUL_RETURN )
		{
			delete[] xOpt;
			if ( nC > 0 ) { delete[] *ubA; delete[] *lbA; delete[] *A; };
			delete[] *ub; delete[] *lb; delete[] *g; delete[] *H;
			return THROWERROR( RET_UNABLE_TO_READ_FILE );
		}
	}

	if ( yOpt != 0 )
	{
		/* dual solution vector sequence */
		*yOpt = new double[nQP*(nV+nC)];
		snprintf( filename,160,"%sy_opt.oqp",path );
		if ( readFromFile( *yOpt,nQP,nV+nC,filename ) != SUCCESSFUL_RETURN )
		{
			delete[] yOpt;
			if ( xOpt != 0 ) { delete[] xOpt; };
			if ( nC > 0 ) { delete[] *ubA; delete[] *lbA; delete[] *A; };
			delete[] *ub; delete[] *lb; delete[] *g; delete[] *H;
			return THROWERROR( RET_UNABLE_TO_READ_FILE );
		}
	}

	if ( objOpt != 0 )
	{
		/* dual solution vector sequence */
		*objOpt = new double[nQP];
		snprintf( filename,160,"%sobj_opt.oqp",path );
		if ( readFromFile( *objOpt,nQP,1,filename ) != SUCCESSFUL_RETURN )
		{
			delete[] objOpt;
			if ( yOpt != 0 ) { delete[] yOpt; };
			if ( xOpt != 0 ) { delete[] xOpt; };
			if ( nC > 0 ) { delete[] *ubA; delete[] *lbA; delete[] *A; };
			delete[] *ub; delete[] *lb; delete[] *g; delete[] *H;
			return THROWERROR( RET_UNABLE_TO_READ_FILE );
		}
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s o l v e O Q P b e n c h m a r k
 */
returnValue solveOQPbenchmark(	int nQP, int nV, int nC, int nEC,
								const double* const H, const double* const g, const double* const A,
								const double* const lb, const double* const ub,
								const double* const lbA, const double* const ubA,
								const double* const xOpt, const double* const yOpt, const double* const objOpt,
								int& nWSR, double& maxCPUtime,
								double& maxPrimalDeviation, double& maxDualDeviation, double& maxObjDeviation
								)
{
	int i, k;

	/* I) SETUP AUXILIARY VARIABLES: */
	/* 1) Keep nWSR and store current and maximum number of
	 *    working set recalculations in temporary variables */
	int nWSRcur;
	int maxNWSR = 0;

	double CPUtimeLimit = maxCPUtime;
	double CPUtimeCur = CPUtimeLimit;
	maxCPUtime = 0.0;

	/* 2) Maximum deviation (infinity norm) */
	maxPrimalDeviation = 0.0;
	maxDualDeviation   = 0.0;
	maxObjDeviation    = 0.0;

	/* 3) Pointers to data of current QP ... */
	const double* gCur;
	const double* lbCur;
	const double* ubCur;
	const double* lbACur;
	const double* ubACur;

	/* ... and to its optimal solution. */
	const double* xOptCur;
	const double* yOptCur;
	double        objOptCur;

	/* 4) Vectors for solution obtained by qpOASES. */
	double* x = new double[nV];
	double* y = new double[nV+nC];
	double  obj;


	/* II) SETUP QPROBLEM OBJECT */
	QProblem qp( nV,nC );
	qp.setPrintLevel( PL_LOW );


	/* III) RUN BENCHMARK SEQUENCE: */
	returnValue returnvalue;

	for( k=0; k<nQP; ++k )
	{
		/* 1) Update pointers to current QP data. */
		gCur   = &( g[k*nV] );
		lbCur  = &( lb[k*nV] );
		ubCur  = &( ub[k*nV] );
		lbACur = &( lbA[k*nC] );
		ubACur = &( ubA[k*nC] );

		xOptCur = &( xOpt[k*nV] );
		yOptCur = &( yOpt[k*(nV+nC)] );
		objOptCur = objOpt[k];

		/* 2) Set nWSR and maximum CPU time. */
		nWSRcur = nWSR;
		CPUtimeCur = CPUtimeLimit;

		/* 3) Solve current QP. */
		if ( k == 0 )
		{
			/* initialise */
			returnvalue = qp.init( H,gCur,A,lbCur,ubCur,lbACur,ubACur, nWSRcur,&CPUtimeCur );
			if ( ( returnvalue != SUCCESSFUL_RETURN ) && ( returnvalue != RET_MAX_NWSR_REACHED ) )
			{
				delete[] y; delete[] x;
				return THROWERROR( RET_BENCHMARK_ABORTED );
			}
		}
		else
		{
			/* hotstart */
			returnvalue = qp.hotstart( gCur,lbCur,ubCur,lbACur,ubACur, nWSRcur,&CPUtimeCur );
			if ( ( returnvalue != SUCCESSFUL_RETURN ) && ( returnvalue != RET_MAX_NWSR_REACHED ) )
			{
				delete[] y; delete[] x;
				return THROWERROR( RET_BENCHMARK_ABORTED );
			}
		}

		/* 4) Obtain solution vectors and objective function value ... */
		qp.getPrimalSolution( x );
		qp.getDualSolution( y );
		obj = qp.getObjVal( );

		/* ... and update maximum values. */
		for( i=0; i<nV; ++i )
		{
			if ( fabs( x[i] - xOptCur[i] ) > maxPrimalDeviation )
				maxPrimalDeviation = fabs( x[i] - xOptCur[i] );
		}

		for( i=0; i<nV+nC; ++i )
		{
			if ( fabs( y[i] - yOptCur[i] ) > maxDualDeviation )
				maxDualDeviation = fabs( y[i] - yOptCur[i] );
		}

		if ( fabs( obj - objOptCur ) > maxObjDeviation )
			maxObjDeviation = fabs( obj - objOptCur );

		if ( nWSRcur > maxNWSR )
			maxNWSR = nWSRcur;

		if ( CPUtimeCur > maxCPUtime )
			maxCPUtime = CPUtimeCur;
	}

	delete[] y; delete[] x;

	return SUCCESSFUL_RETURN;
}


/*
 *	s o l v e O Q P b e n c h m a r k
 */
returnValue solveOQPbenchmark(	int nQP, int nV,
								const double* const H, const double* const g,
								const double* const lb, const double* const ub,
								const double* const xOpt, const double* const yOpt, const double* const objOpt,
								int& nWSR, double& maxCPUtime,
								double& maxPrimalDeviation, double& maxDualDeviation, double& maxObjDeviation
								)
{
	int i, k;

	/* I) SETUP AUXILIARY VARIABLES: */
	/* 1) Keep nWSR and store current and maximum number of
	 *    working set recalculations in temporary variables */
	int nWSRcur;
	int maxNWSR = 0;

	double CPUtimeLimit = maxCPUtime;
	double CPUtimeCur = CPUtimeLimit;
	maxCPUtime = 0.0;

	/* 2) Maximum deviation (infinity norm) */
	maxPrimalDeviation = 0.0;
	maxDualDeviation   = 0.0;
	maxObjDeviation    = 0.0;

	/* 3) Pointers to data of current QP ... */
	const double* gCur;
	const double* lbCur;
	const double* ubCur;

	/* ... and to its optimal solution. */
	const double* xOptCur;
	const double* yOptCur;
	double        objOptCur;

	/* 4) Vectors for solution obtained by qpOASES. */
	double* x = new double[nV];
	double* y = new double[nV];
	double  obj;


	/* II) SETUP QPROBLEM OBJECT */
	QProblemB qp( nV );
	qp.setPrintLevel( PL_LOW );


	/* III) RUN BENCHMARK SEQUENCE: */
	returnValue returnvalue;

	for( k=0; k<nQP; ++k )
	{
		/* 1) Update pointers to current QP data. */
		gCur   = &( g[k*nV] );
		lbCur  = &( lb[k*nV] );
		ubCur  = &( ub[k*nV] );

		xOptCur = &( xOpt[k*nV] );
		yOptCur = &( yOpt[k*nV] );
		objOptCur = objOpt[k];

		/* 2) Set nWSR and maximum CPU time. */
		nWSRcur = nWSR;
		CPUtimeCur = CPUtimeLimit;

		/* 3) Solve current QP. */
		if ( k == 0 )
		{
			/* initialise */
			returnvalue = qp.init( H,gCur,lbCur,ubCur, nWSRcur,&CPUtimeCur );
			if ( ( returnvalue != SUCCESSFUL_RETURN ) && ( returnvalue != RET_MAX_NWSR_REACHED ) )
			{
				delete[] y; delete[] x;
				return THROWERROR( RET_BENCHMARK_ABORTED );
			}
		}
		else
		{
			/* hotstart */
			returnvalue = qp.hotstart( gCur,lbCur,ubCur, nWSRcur,&CPUtimeCur );
			if ( ( returnvalue != SUCCESSFUL_RETURN ) && ( returnvalue != RET_MAX_NWSR_REACHED ) )
			{
				delete[] y; delete[] x;
				return THROWERROR( RET_BENCHMARK_ABORTED );
			}
		}

		/* 4) Obtain solution vectors and objective function value ... */
		qp.getPrimalSolution( x );
		qp.getDualSolution( y );
		obj = qp.getObjVal( );

		/* ... and update maximum values. */
		for( i=0; i<nV; ++i )
		{
			if ( fabs( x[i] - xOptCur[i] ) > maxPrimalDeviation )
				maxPrimalDeviation = fabs( x[i] - xOptCur[i] );
		}

		for( i=0; i<nV; ++i )
		{
			if ( fabs( y[i] - yOptCur[i] ) > maxDualDeviation )
				maxDualDeviation = fabs( y[i] - yOptCur[i] );
		}

		if ( fabs( obj - objOptCur ) > maxObjDeviation )
			maxObjDeviation = fabs( obj - objOptCur );

		if ( nWSRcur > maxNWSR )
			maxNWSR = nWSRcur;

		if ( CPUtimeCur > maxCPUtime )
			maxCPUtime = CPUtimeCur;
	}

	delete[] y; delete[] x;

	return SUCCESSFUL_RETURN;
}


/*
 *	r u n O Q P b e n c h m a r k
 */
returnValue runOQPbenchmark(	const char* path,
								int& nWSR, double& maxCPUtime,
								double& maxPrimalDeviation, double& maxDualDeviation, double& maxObjDeviation
								)
{
	int nQP, nV, nC, nEC;

	double *H, *g, *A, *lb, *ub, *lbA, *ubA;
	double *xOpt, *yOpt, *objOpt;


	/* I) SETUP BENCHMARK: */
	/* 1) Obtain QP sequence dimensions. */
	if ( readOQPdimensions( path, nQP,nV,nC,nEC ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_BENCHMARK_ABORTED );

	/* 2) Read OQP benchmark data. */
	if ( readOQPdata(	path,
						nQP,nV,nC,nEC,
						&H,&g,&A,&lb,&ub,&lbA,&ubA,
						&xOpt,&yOpt,&objOpt
						) != SUCCESSFUL_RETURN )
	{
		return THROWERROR( RET_UNABLE_TO_READ_BENCHMARK );
	}


	/* II) SOLVE BENCHMARK */
	if ( nC > 0 )
	{
		if ( solveOQPbenchmark(	nQP,nV,nC,nEC,
								H,g,A,lb,ub,lbA,ubA,
								xOpt,yOpt,objOpt,
								nWSR,maxCPUtime,
								maxPrimalDeviation,maxDualDeviation,maxObjDeviation
								) != SUCCESSFUL_RETURN )
		{
			delete[] objOpt; delete[] yOpt; delete[] xOpt;
			delete[] ubA; delete[] lbA; delete[] ub; delete[] lb; delete[] A; delete[] g; delete[] H;

			return THROWERROR( RET_BENCHMARK_ABORTED );
		}
	}
	else
	{
		if ( solveOQPbenchmark(	nQP,nV,
								H,g,lb,ub,
								xOpt,yOpt,objOpt,
								nWSR,maxCPUtime,
								maxPrimalDeviation,maxDualDeviation,maxObjDeviation
								) != SUCCESSFUL_RETURN )
		{
			delete[] objOpt; delete[] yOpt; delete[] xOpt;
			delete[] ub; delete[] lb; delete[] g; delete[] H;

			return THROWERROR( RET_BENCHMARK_ABORTED );
		}
	}


	delete[] objOpt; delete[] yOpt; delete[] xOpt;
	delete[] ubA; delete[] lbA; delete[] ub; delete[] lb; delete[] A; delete[] g; delete[] H;

	return SUCCESSFUL_RETURN;
}

#ifndef __DSPACE__
} /* qpOASES */
#endif


/*
 *	end of file
 */
