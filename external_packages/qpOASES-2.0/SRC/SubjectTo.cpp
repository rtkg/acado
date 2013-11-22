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
 *	\file SRC/SubjectTo.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Implementation of the SubjectTo class designed to manage working sets of
 *	constraints and bounds within a QProblem.
 */


#include <stdio.h>

#include <SubjectTo.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	S u b j e c t T o
 */
SubjectTo::SubjectTo( ) :	n( 0 ),
							noLower( BT_TRUE ),
							noUpper( BT_TRUE )
{
	type   = 0;
	status = 0;
}


/*
 *	S u b j e c t T o
 */
SubjectTo::SubjectTo( int _n ) :	n( _n ),
									noLower( BT_TRUE ),
									noUpper( BT_TRUE )
{
	int i;

	type   = new SubjectToType[n];
	status = new SubjectToStatus[n];

	for( i=0; i<n; ++i )
	{
		type[i]   = ST_UNKNOWN;
		status[i] = ST_UNDEFINED;
	}
}


/*
 *	S u b j e c t T o
 */
SubjectTo::SubjectTo( const SubjectTo& rhs ) :	n( rhs.n ),
												noLower( rhs.noLower ),
												noUpper( rhs.noUpper )
{
	int i;

	if ( rhs.n != 0 )
	{
		type   = new SubjectToType[n];
		status = new SubjectToStatus[n];

		for( i=0; i<n; ++i )
		{
			type[i]   = rhs.type[i];
			status[i] = rhs.status[i];
		}
	}
	else
	{
		type   = 0;
		status = 0;
	}
}


/*
 *	~ S u b j e c t T o
 */
SubjectTo::~SubjectTo( )
{
	if ( type != 0 )
		delete[] type;

	if ( status != 0 )
		delete[] status;
}


/*
 *	o p e r a t o r =
 */
SubjectTo& SubjectTo::operator=( const SubjectTo& rhs )
{
	int i;

	if ( this != &rhs )
	{
		if ( type != 0 )
			delete[] type;

		if ( status != 0 )
			delete[] status;

		n = rhs.n;

		if ( rhs.n != 0 )
		{
			type   = new SubjectToType[n];
			status = new SubjectToStatus[n];

			for( i=0; i<n; ++i )
			{
				type[i]   = rhs.type[i];
				status[i] = rhs.status[i];
			}
		}
		else
		{
			type   = 0;
			status = 0;
		}

		noLower = rhs.noLower;
		noUpper = rhs.noUpper;
	}

	return *this;
}



/*****************************************************************************
 *  P R O T E C T E D                                                        *
 *****************************************************************************/

/*
 *	a d d I n d e x
 */
returnValue SubjectTo::addIndex(	Indexlist* const indexlist,
									int newnumber, SubjectToStatus newstatus
									)
{
	if ( status != 0 )
	{
		/* consistency check */
		if ( status[newnumber] == newstatus )
			return THROWERROR( RET_INDEX_ALREADY_OF_DESIRED_STATUS );

		status[newnumber] = newstatus;
	}
	else
		return THROWERROR( RET_ADDINDEX_FAILED );

	if ( indexlist != 0 )
	{
		if ( indexlist->addNumber( newnumber ) == RET_INDEXLIST_EXCEEDS_MAX_LENGTH )
			return THROWERROR( RET_ADDINDEX_FAILED );
	}
	else
		return THROWERROR( RET_INVALID_ARGUMENTS );

	return SUCCESSFUL_RETURN;
}


/*
 *	r e m o v e I n d e x
 */
returnValue SubjectTo::removeIndex(	Indexlist* const indexlist,
									int removenumber
									)
{
	if ( status != 0 )
		status[removenumber] = ST_UNDEFINED;
	else
		return THROWERROR( RET_REMOVEINDEX_FAILED );

	if ( indexlist != 0 )
	{
		if ( indexlist->removeNumber( removenumber ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_REMOVEINDEX_FAILED );
	}
	else
		return THROWERROR( RET_INVALID_ARGUMENTS );

	return SUCCESSFUL_RETURN;
}


/*
 *	s w a p I n d e x
 */
returnValue SubjectTo::swapIndex(	Indexlist* const indexlist,
									int number1, int number2
									)
{
	/* consistency checks */
	if ( status != 0 )
	{
		if ( status[number1] != status[number2] )
			return THROWERROR( RET_SWAPINDEX_FAILED );
	}
	else
		return THROWERROR( RET_SWAPINDEX_FAILED );

	if ( number1 == number2 )
	{
		THROWWARNING( RET_NOTHING_TO_DO );
		return SUCCESSFUL_RETURN;
	}

	if ( indexlist != 0 )
	{
		if ( indexlist->swapNumbers( number1,number2 ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_SWAPINDEX_FAILED );
	}
	else
		return THROWERROR( RET_INVALID_ARGUMENTS );

	return SUCCESSFUL_RETURN;
}

#ifndef __DSPACE__
} /* qpOASES */
#endif


/*
 *	end of file
 */
