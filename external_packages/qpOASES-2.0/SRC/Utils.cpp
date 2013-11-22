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
 *	\file SRC/Utils.cpp
 *	\author Hans Joachim Ferreau, Eckhard Arnold
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Implementation of some utilities for working with the different QProblem classes.
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#if defined(__WIN32__) || defined(WIN32)
  #include <windows.h>
#elif defined(LINUX)
  #include <sys/stat.h>
  #include <sys/time.h>
#endif

#ifdef __MATLAB__
  #include "mex.h"
#endif

#ifdef __MODELICA__
extern "C" {
#include <ModelicaUtilities.h>
}
#endif

#include <Utils.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif


/*
 *	p r i n t
 */
returnValue print( const double* const v, int n )
{
	#ifndef __XPCTARGET__
	int i;
	char myPrintfString[160];

	/* Print a vector. */
	myPrintf( "[\t" );
	for( i=0; i<n; ++i )
	{
		snprintf( myPrintfString,160," %.16e\t", v[i] );
		myPrintf( myPrintfString );
	}
	myPrintf( "]\n" );
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t
 */
returnValue print(	const double* const v, int n,
					const int* const V_idx
					)
{
	#ifndef __XPCTARGET__
	int i;
	char myPrintfString[160];

	/* Print a permuted vector. */
	myPrintf( "[\t" );
	for( i=0; i<n; ++i )
	{
		snprintf( myPrintfString,160," %.16e\t", v[ V_idx[i] ] );
		myPrintf( myPrintfString );
	}
	myPrintf( "]\n" );
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t
 */
returnValue print(	const double* const v, int n,
					const char* name
					)
{
	#ifndef __XPCTARGET__
	char myPrintfString[160];

	/* Print vector name ... */
	snprintf( myPrintfString,160,"%s = ", name );
	myPrintf( myPrintfString );
	#endif

	/* ... and the vector itself. */
	return print( v, n );
}


/*
 *	p r i n t
 */
returnValue print( const double* const M, int nrow, int ncol )
{
	#ifndef __XPCTARGET__
	int i;

	/* Print a matrix as a collection of row vectors. */
	for( i=0; i<nrow; ++i )
		print( &(M[i*ncol]), ncol );
	myPrintf( "\n" );
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t
 */
returnValue print(	const double* const M, int nrow, int ncol,
					const int* const ROW_idx, const int* const COL_idx
					)
{
	#ifndef __XPCTARGET__
	int i;

	/* Print a permuted matrix as a collection of permuted row vectors. */
	for( i=0; i<nrow; ++i )
		print( &( M[ ROW_idx[i]*ncol ] ), ncol, COL_idx );
	myPrintf( "\n" );
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t
 */
returnValue print(	const double* const M, int nrow, int ncol,
					const char* name
					)
{
	#ifndef __XPCTARGET__
	char myPrintfString[160];

	/* Print matrix name ... */
	snprintf( myPrintfString,160,"%s = ", name );
	myPrintf( myPrintfString );
	#endif

	/* ... and the matrix itself. */
	return print( M, nrow, ncol );
}


/*
 *	p r i n t
 */
returnValue print( const int* const index, int n )
{
	#ifndef __XPCTARGET__
	int i;
	char myPrintfString[160];

	/* Print a indexlist. */
	myPrintf( "[\t" );
	for( i=0; i<n; ++i )
	{
		snprintf( myPrintfString,160," %d\t", index[i] );
		myPrintf( myPrintfString );
	}
	myPrintf( "]\n" );
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t
 */
returnValue print(	const int* const index, int n,
					const char* name
					)
{
	#ifndef __XPCTARGET__
	char myPrintfString[160];

	/* Print indexlist name ... */
	snprintf( myPrintfString,160,"%s = ", name );
	myPrintf( myPrintfString );
	#endif

	/* ... and the indexlist itself. */
	return print( index, n );
}


/*
 *	m y P r i n t f
 */
returnValue myPrintf( const char* s )
{
	#ifdef __MATLAB__
	mexPrintf( s );
        #elif defined __MODELICA__
	ModelicaFormatMessage("%s",s);
	#else
	FILE* outputfile = getGlobalMessageHandler( )->getOutputFile( );
	if ( outputfile == 0 )
		return THROWERROR( RET_NO_GLOBAL_MESSAGE_OUTPUTFILE );

	fprintf( outputfile, "%s", s );
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t C o p y r i g h t N o t i c e
 */
returnValue printCopyrightNotice( )
{
	#ifndef __NO_COPYRIGHT__
	myPrintf( "\nqpOASES -- An Implementation of the Online Active Set Strategy.\nCopyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.\n\nqpOASES is distributed under the terms of the \nGNU Lesser General Public License 2.1 in the hope that it will be \nuseful, but WITHOUT ANY WARRANTY; without even the implied warranty \nof MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. \nSee the GNU Lesser General Public License for more details.\n\n" );
	#endif
	return SUCCESSFUL_RETURN;
}


/*
 *	r e a d F r o m F i l e
 */
returnValue readFromFile(	double* data, int nrow, int ncol,
							const char* datafilename
							)
{
	#ifndef __XPCTARGET__
	int i, j;
	float float_data;
	FILE* datafile;

	/* 1) Open file. */
	if ( ( datafile = fopen( datafilename, "r" ) ) == 0 )
	{
		char errstr[80];
		snprintf( errstr,80,"(%s)",datafilename );
		return getGlobalMessageHandler( )->throwError( RET_UNABLE_TO_OPEN_FILE,errstr,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
	}

	/* 2) Read data from file. */
	for( i=0; i<nrow; ++i )
	{
		for( j=0; j<ncol; ++j )
		{
			if ( fscanf( datafile, "%f ", &float_data ) == 0 )
			{
				fclose( datafile );
				char errstr[80];
				snprintf( errstr,80,"(%s)",datafilename );
				return getGlobalMessageHandler( )->throwError( RET_UNABLE_TO_READ_FILE,errstr,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
			}
			data[i*ncol + j] = ( (double) float_data );
		}
	}

	/* 3) Close file. */
	fclose( datafile );

	return SUCCESSFUL_RETURN;
	#else

	return RET_NOT_YET_IMPLEMENTED;

	#endif
}


/*
 *	r e a d F r o m F i l e
 */
returnValue readFromFile(	double* data, int n,
							const char* datafilename
							)
{
	return readFromFile( data, n, 1, datafilename );
}



/*
 *	r e a d F r o m F i l e
 */
returnValue readFromFile(	int* data, int n,
							const char* datafilename
							)
{
	#ifndef __XPCTARGET__
	int i;
	FILE* datafile;

	/* 1) Open file. */
	if ( ( datafile = fopen( datafilename, "r" ) ) == 0 )
	{
		char errstr[80];
		snprintf( errstr,80,"(%s)",datafilename );
		return getGlobalMessageHandler( )->throwError( RET_UNABLE_TO_OPEN_FILE,errstr,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
	}

	/* 2) Read data from file. */
	for( i=0; i<n; ++i )
	{
		if ( fscanf( datafile, "%d\n", &(data[i]) ) == 0 )
		{
			fclose( datafile );
			char errstr[80];
			snprintf( errstr,80,"(%s)",datafilename );
			return getGlobalMessageHandler( )->throwError( RET_UNABLE_TO_READ_FILE,errstr,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		}
	}

	/* 3) Close file. */
	fclose( datafile );

	return SUCCESSFUL_RETURN;
	#else

	return RET_NOT_YET_IMPLEMENTED;

	#endif
}


/*
 *	w r i t e I n t o F i l e
 */
returnValue writeIntoFile(	const double* const data, int nrow, int ncol,
							const char* datafilename, BooleanType append
							)
{
	#ifndef __XPCTARGET__
	int i, j;
	FILE* datafile;

	/* 1) Open file. */
	if ( append == BT_TRUE )
	{
		/* append data */
		if ( ( datafile = fopen( datafilename, "a" ) ) == 0 )
		{
			char errstr[80];
			snprintf( errstr,80,"(%s)",datafilename );
			return getGlobalMessageHandler( )->throwError( RET_UNABLE_TO_OPEN_FILE,errstr,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		}
	}
	else
	{
		/* do not append data */
		if ( ( datafile = fopen( datafilename, "w" ) ) == 0 )
		{
			char errstr[80];
			snprintf( errstr,80,"(%s)",datafilename );
			return getGlobalMessageHandler( )->throwError( RET_UNABLE_TO_OPEN_FILE,errstr,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		}
	}

	/* 2) Write data into file. */
	for( i=0; i<nrow; ++i )
	{
		for( j=0; j<ncol; ++j )
		 	fprintf( datafile, "%.16e ", data[i*ncol+j] );

		fprintf( datafile, "\n" );
	}

	/* 3) Close file. */
	fclose( datafile );

	return SUCCESSFUL_RETURN;
	#else

	return RET_NOT_YET_IMPLEMENTED;

	#endif
}


/*
 *	w r i t e I n t o F i l e
 */
returnValue writeIntoFile(	const double* const data, int n,
							const char* datafilename, BooleanType append
							)
{
	return writeIntoFile( data,1,n,datafilename,append );
}


/*
 *	w r i t e I n t o F i l e
 */
returnValue writeIntoFile(	const int* const data, int n,
							const char* datafilename, BooleanType append
							)
{
	#ifndef __XPCTARGET__
	int i;

	FILE* datafile;

	/* 1) Open file. */
	if ( append == BT_TRUE )
	{
		/* append data */
		if ( ( datafile = fopen( datafilename, "a" ) ) == 0 )
		{
			char errstr[80];
			snprintf( errstr,80,"(%s)",datafilename );
			return getGlobalMessageHandler( )->throwError( RET_UNABLE_TO_OPEN_FILE,errstr,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		}
	}
	else
	{
		/* do not append data */
		if ( ( datafile = fopen( datafilename, "w" ) ) == 0 )
		{
			char errstr[80];
			snprintf( errstr,80,"(%s)",datafilename );
			return getGlobalMessageHandler( )->throwError( RET_UNABLE_TO_OPEN_FILE,errstr,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		}
	}

	/* 2) Write data into file. */
	for( i=0; i<n; ++i )
		fprintf( datafile, "%d\n", data[i] );

	/* 3) Close file. */
	fclose( datafile );

	return SUCCESSFUL_RETURN;
	#else

	return RET_NOT_YET_IMPLEMENTED;

	#endif
}


/*
 *	g e t C P U t i m e
 */
double getCPUtime( )
{
	double current_time = -1.0;

	#if defined(__WIN32__) || defined(WIN32)
	LARGE_INTEGER counter, frequency;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&counter);
	current_time = ((double) counter.QuadPart) / ((double) frequency.QuadPart);
	#elif defined(LINUX)
	struct timeval theclock;
	gettimeofday( &theclock,0 );
	current_time = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;
	#endif

	return current_time;
}


/*
 *	g e t N o r m
 */
double getNorm( const double* const v, int n )
{
	int i;

	double norm = 0.0;

	for( i=0; i<n; ++i )
		norm += v[i]*v[i];

	return sqrt( norm );
}


#ifndef __DSPACE__
} /* qpOASES */
#endif


/*
 *	end of file
 */
