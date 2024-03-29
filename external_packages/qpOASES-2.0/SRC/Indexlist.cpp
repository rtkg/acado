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
 *	\file SRC/Indexlist.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Implementation of the Indexlist class designed to manage index lists of
 *	constraints and bounds within a QProblem_SubjectTo.
 */


#include <Indexlist.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	I n d e x l i s t
 */
Indexlist::Indexlist( ) :	length( 0 ),
							first( -1 ),
							last( -1 ),
							lastusedindex( -1 ),
							physicallength( 0 )
{
	number 		= new int;
	next 		= new int;
	previous 	= new int;
}


/*
 *	I n d e x l i s t
 */
Indexlist::Indexlist( int n ) :	length( 0 ),
								first( -1 ),
								last( -1 ),
								lastusedindex( -1 ),
								physicallength( n )
{
	number 		= new int[n];
	next 		= new int[n];
	previous 	= new int[n];
}


/*
 *	I n d e x l i s t
 */
Indexlist::Indexlist( const Indexlist& rhs ) :	length( rhs.length ),
												first( rhs.first ),
												last( rhs.last ),
												lastusedindex( rhs.lastusedindex ),
												physicallength( rhs.physicallength )
{
	int i;

	number 		= new int[physicallength];
	next 		= new int[physicallength];
	previous 	= new int[physicallength];

	for( i=0; i<physicallength; ++i )
	{
		number[i] = rhs.number[i];
		next[i] = rhs.next[i];
		previous[i] = rhs.previous[i];
	}
}


/*
 *	~ I n d e x l i s t
 */
Indexlist::~Indexlist( )
{
	delete[] number;
	delete[] next;
	delete[] previous;
}


/*
 *	o p e r a t o r =
 */
Indexlist& Indexlist::operator=( const Indexlist& rhs )
{
	int i;

	if ( this != &rhs )
	{
		delete[] number;
		delete[] next;
		delete[] previous;

		length = rhs.length;
		first = rhs.first;
		last = rhs.last;
		lastusedindex = rhs.lastusedindex;
		physicallength = rhs.physicallength;

		number 		= new int[physicallength];
		next 		= new int[physicallength];
		previous 	= new int[physicallength];

		for( i=0; i<physicallength; ++i )
		{
			number[i] = rhs.number[i];
			next[i] = rhs.next[i];
			previous[i] = rhs.previous[i];
		}
	}

	return *this;
}


/*
 *	g e t N u m b e r A r r a y
 */
returnValue Indexlist::getNumberArray( int* const numberarray ) const
{
	int i;
	int n = first;

	/* Run trough indexlist and store numbers in numberarray. */
	for( i=0; i<length; ++i )
	{
		if ( ( n >= 0 ) && ( number[n] >= 0 ) )
			numberarray[i] = number[n];
		else
			return THROWERROR( RET_INDEXLIST_CORRUPTED );

		n = next[n];
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	g e t I n d e x
 */
int Indexlist::getIndex( int givennumber ) const
{
	int i;
	int n = first;
	int index = -1;	/* return -1 by default */

	/* Run trough indexlist until number is found, if so return it index. */
	for ( i=0; i<length; ++i )
	{
		if ( number[n] == givennumber )
		{
			index = i;
			break;
		}

		n = next[n];
	}

	return index;
}


/*
 *	g e t P h y s i c a l I n d e x
 */
int Indexlist::getPhysicalIndex( int givennumber ) const
{
	int i;
	int n = first;
	int index = -1;	/* return -1 by default */

	/* Run trough indexlist until number is found, if so return it physicalindex. */
	for ( i=0; i<length; ++i )
	{
		if ( number[n] == givennumber )
		{
			index = n;
			break;
		}

		n = next[n];
	}

	return index;
}


/*
 *	a d d N u m b e r
 */
returnValue Indexlist::addNumber( int addnumber )
{
	int i;

	if ( lastusedindex+1 < physicallength )
	{
		/* If there is enough storage, add number to indexlist. */
		++lastusedindex;
		number[lastusedindex] = addnumber;
		next[lastusedindex] = 0;

		if ( length == 0 )
		{
			first = lastusedindex;
			previous[lastusedindex] = 0;
		}
		else
		{
			next[last] = lastusedindex;
			previous[lastusedindex] = last;
		}

		last = lastusedindex;
		++length;

		return SUCCESSFUL_RETURN;
	}
	else
	{
		/* Rearrangement of index list necessary! */
		if ( length >= physicallength )
			return THROWERROR( RET_INDEXLIST_EXCEEDS_MAX_LENGTH );
		else
		{
			int* numberArray = new int[length];
			getNumberArray( numberArray );

			/* copy existing elements */
			for ( i=0; i<length; ++i )
			{
				number[i] = numberArray[i];
				next[i] = i+1;
				previous[i] = i-1;
			}

			/* add new number at end of list */
			number[length] = addnumber;
			next[length] = -1;
			previous[length] = length-1;

			/* and set remaining entries to empty */
			for ( i=length+1; i<physicallength; ++i )
			{
				number[i] = -1;
				next[i] = -1;
				previous[i] = -1;
			}

			first = 0;
			last = length;
			lastusedindex = length;
			++length;

			delete[] numberArray;

			return THROWWARNING( RET_INDEXLIST_MUST_BE_REORDERD );
		}
	}
}


/*
 *	r e m o v e N u m b e r
 */
returnValue Indexlist::removeNumber( int removenumber )
{
	int i = getPhysicalIndex( removenumber );

	/* nothing to be done if number is not contained in index set */
	if ( i < 0 )
		return SUCCESSFUL_RETURN;

	int p = previous[i];
	int n = next[i];

	if ( i == last )
		last = p;
	else
		previous[n] = p;

	if ( i == first )
		first = n;
	else
		next[p] = n;

	number[i] = -1;
	next[i] = -1;
	previous[i] = -1;
	--length;

	return SUCCESSFUL_RETURN;
}


/*
 *	s w a p N u m b e r s
 */
returnValue Indexlist::swapNumbers( int number1, int number2 )
{
	int index1 = getPhysicalIndex( number1 );
	int index2 = getPhysicalIndex( number2 );

	/* consistency check */
	if ( ( index1 < 0 ) || ( index2 < 0 ) )
		return THROWERROR( RET_INDEXLIST_CORRUPTED );

	int tmp = number[index1];
	number[index1] = number[index2];
	number[index2] = tmp;

	return SUCCESSFUL_RETURN;
}


/*
 *	a d d
 */
returnValue Indexlist::add( const Indexlist* const IS )
{
	/* if this functionality is used more often, one should think about a
	 * different implementation to avoid the O(n^2) complexity... */
	int i;
	int n = IS->getLength( );
	int newNumber;

	for ( i=0; i<n; ++i )
	{
		newNumber = IS->getNumber( i );

		if ( this->isMember( newNumber ) == BT_FALSE )	/* check if index is already contained in current index set ... */
			if ( this->addNumber( newNumber ) != SUCCESSFUL_RETURN ) /* ... if not, add it! */
				return THROWERROR( RET_INDEXLIST_ADD_FAILED );
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	i n t e r s e c t
 */
returnValue Indexlist::intersect( const Indexlist* const IS )
{
	/* if this functionality is used more often, one should think about a
	 * different implementation to avoid the O(n^2) complexity... */
	int i;
	int n = IS->getLength( );
	int newNumber;

	for ( i=0; i<n; ++i )
	{
		newNumber = this->getNumber( i );

		if ( IS->isMember( newNumber ) == BT_FALSE )	/* remove number if it is NOT contained in IS */
			if ( this->removeNumber( newNumber ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_INDEXLIST_ADD_FAILED );
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t m i n u s
 */
returnValue Indexlist::setminus( const Indexlist* const IS )
{
	/* if this functionality is used more often, one should think about a
	 * different implementation to avoid the O(n^2) complexity... */
	int i;
	int n = IS->getLength( );

	for ( i=0; i<n; ++i )
	{
		if ( this->removeNumber( IS->getNumber( i ) ) != SUCCESSFUL_RETURN )
			return THROWERROR( RET_INDEXLIST_ADD_FAILED );
	}

	return SUCCESSFUL_RETURN;
}


#ifndef __DSPACE__
} /* qpOASES */
#endif


/*
 *	end of file
 */
