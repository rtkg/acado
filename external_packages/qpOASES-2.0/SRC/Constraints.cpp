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
 *	\file SRC/Constraints.cpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Implementation of the Constraints class designed to manage working sets of
 *	constraints within a QProblem.
 */


#include <stdio.h>

#include <Constraints.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	C o n s t r a i n t s
 */
Constraints::Constraints( ) : SubjectTo( )
{
	active   = 0;
	inactive = 0;
	disabled = 0;
}


/*
 *	C o n s t r a i n t s
 */
Constraints::Constraints( int _n ) : SubjectTo( _n )

{
	active   = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );
	inactive = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );
	disabled = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );
}


/*
 *	C o n s t r a i n t s
 */
Constraints::Constraints( const Constraints& rhs ) : SubjectTo( rhs )
{
	if ( rhs.active != 0 )
		active = new Indexlist( *(rhs.active) );
	else
		active = 0;

	if ( rhs.inactive != 0 )
		inactive = new Indexlist( *(rhs.inactive) );
	else
		inactive = 0;

	if ( rhs.disabled != 0 )
		disabled = new Indexlist( *(rhs.disabled) );
	else
		disabled = 0;
}


/*
 *	~ C o n s t r a i n t s
 */
Constraints::~Constraints( )
{
	if ( active != 0 )
		delete active;

	if ( inactive != 0 )
		delete inactive;

	if ( disabled != 0 )
		delete disabled;
}


/*
 *	o p e r a t o r =
 */
Constraints& Constraints::operator=( const Constraints& rhs )
{
	if ( this != &rhs )
	{
		SubjectTo::operator=( rhs );

		if ( active != 0 )
			delete active;

		if ( inactive != 0 )
			delete inactive;

		if ( disabled != 0 )
			delete disabled;

		if ( rhs.active != 0 )
			active = new Indexlist( *(rhs.active) );
		else
			active = 0;

		if ( rhs.inactive != 0 )
			inactive = new Indexlist( *(rhs.inactive) );
		else
			inactive = 0;

		if ( rhs.disabled != 0 )
			disabled = new Indexlist( *(rhs.disabled) );
		else
			disabled = 0;
	}

	return *this;
}


/*
 *	s e t u p C o n s t r a i n t
 */
returnValue Constraints::setupConstraint(	int number, SubjectToStatus _status
											)
{
	/* consistency check */
	if ( ( number < 0 ) || ( number >= n ) )
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );

	/* Add constraint index to respective index list. */
	switch ( _status )
	{
		case ST_INACTIVE:
			if ( this->addIndex( this->getInactive( ),number,_status ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
			break;

		case ST_LOWER:
			if ( this->addIndex( this->getActive( ),number,_status ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
			break;

		case ST_UPPER:
			if ( this->addIndex( this->getActive( ),number,_status ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
			break;

		case ST_DISABLED:
			if ( this->addIndex( this->getDisabled( ),number,_status ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
			break;

		default:
			return THROWERROR( RET_INVALID_ARGUMENTS );
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t u p A l l I n a c t i v e
 */
returnValue Constraints::setupAllInactive( )
{
	return setupAll( ST_INACTIVE );
}


/*
 *	s e t u p A l l L o w e r
 */
returnValue Constraints::setupAllLower( )
{
	return setupAll( ST_LOWER );
}


/*
 *	s e t u p A l l U p p e r
 */
returnValue Constraints::setupAllUpper( )
{
	return setupAll( ST_UPPER );
}


/*
 *	m o v e A c t i v e T o I n a c t i v e
 */
returnValue Constraints::moveActiveToInactive( int number )
{
	/* consistency check */
	if ( ( number < 0 ) || ( number >= n ) )
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );

	/* Move index from indexlist of active constraints to that of inactive ones. */
	if ( this->removeIndex( this->getActive( ),number ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	if ( this->addIndex( this->getInactive( ),number,ST_INACTIVE ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	return SUCCESSFUL_RETURN;
}


/*
 *	m o v e I n a c t i v e T o A c t i v e
 */
returnValue Constraints::moveInactiveToActive(	int number, SubjectToStatus _status
												)
{
	/* consistency check */
	if ( ( number < 0 ) || ( number >= n ) )
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );

	/* Move index from indexlist of inactive constraints to that of active ones. */
	if ( this->removeIndex( this->getInactive( ),number ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	if ( this->addIndex( this->getActive( ),number,_status ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	return SUCCESSFUL_RETURN;
}


/*
 *	m o v e A c t i v e T o D i s a b l e d
 */
returnValue Constraints::moveActiveToDisabled( int number )
{
	/* consistency check */
	if ( ( number < 0 ) || ( number >= n ) )
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );

	/* Move index from indexlist of active constraints to that of disabled ones. */
	if ( this->removeIndex( this->getActive( ),number ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	if ( this->addIndex( this->getDisabled( ),number,ST_DISABLED ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	return SUCCESSFUL_RETURN;
}


/*
 *	m o v e I n a c t i v e T o D i s a b l e d
 */
returnValue Constraints::moveInactiveToDisabled( int number )
{
	/* consistency check */
	if ( ( number < 0 ) || ( number >= n ) )
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );

	/* Move index from indexlist of inactive constraints to that of disabled ones. */
	if ( this->removeIndex( this->getInactive( ),number ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	if ( this->addIndex( this->getDisabled( ),number,ST_DISABLED ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	return SUCCESSFUL_RETURN;
}


/*
 *	m o v e D i s a b l e d T o I n a c t i v e
 */
returnValue Constraints::moveDisabledToInactive( int number )
{
	/* consistency check */
	if ( ( number < 0 ) || ( number >= n ) )
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );

	/* Move index from indexlist of disabled constraints to that of inactive ones. */
	if ( this->removeIndex( this->getDisabled( ),number ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	if ( this->addIndex( this->getInactive( ),number,ST_INACTIVE ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_MOVING_BOUND_FAILED );

	return SUCCESSFUL_RETURN;
}


/*
 *	s h i f t
 */
returnValue Constraints::shift( int offset )
{
	int i;

	/* consistency check */
	if ( ( offset == 0 ) || ( n <= 1 ) )
		return SUCCESSFUL_RETURN;

	if ( ( offset < 0 ) || ( offset > n/2 ) )
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );

	if ( ( n % offset ) != 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );


	/* 1) Shift types and status. */
	for( i=0; i<n-offset; ++i )
	{
		setType( i,getType( i+offset ) );
		setStatus( i,getStatus( i+offset ) );
	}

	/* 2) Construct shifted index lists of free and fixed variables. */
	Indexlist* shiftedActive   = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );
	Indexlist* shiftedInactive = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );
	Indexlist* shiftedDisabled = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );

	for( i=0; i<n; ++i )
	{
		switch ( getStatus( i ) )
		{
			case ST_INACTIVE:
				if ( shiftedInactive->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete shiftedActive; delete shiftedInactive; delete shiftedDisabled;
					return THROWERROR( RET_SHIFTING_FAILED );
				}
				break;

			case ST_LOWER:
				if ( shiftedActive->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete shiftedActive; delete shiftedInactive; delete shiftedDisabled;
					return THROWERROR( RET_SHIFTING_FAILED );
				}
				break;

			case ST_UPPER:
				if ( shiftedActive->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete shiftedActive; delete shiftedInactive; delete shiftedDisabled;
					return THROWERROR( RET_SHIFTING_FAILED );
				}
				break;

			case ST_DISABLING:
				if ( shiftedActive->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete shiftedActive; delete shiftedInactive; delete shiftedDisabled;
					return THROWERROR( RET_SHIFTING_FAILED );
				}
				break;

			case ST_DISABLED:
				if ( shiftedDisabled->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete shiftedActive; delete shiftedInactive; delete shiftedDisabled;
					return THROWERROR( RET_SHIFTING_FAILED );
				}
				break;

			default:
				delete shiftedActive; delete shiftedInactive; delete shiftedDisabled;
				return THROWERROR( RET_SHIFTING_FAILED );
		}
	}

	/* 3) Assign shifted index list. */
	delete active;
	active = shiftedActive;

	delete inactive;
	inactive = shiftedInactive;

	delete disabled;
	disabled = shiftedDisabled;

	return SUCCESSFUL_RETURN;
}


/*
 *	r o t a t e
 */
returnValue Constraints::rotate( int offset )
{
	int i;

	/* consistency check */
	if ( ( offset == 0 ) || ( offset == n ) || ( n <= 1 ) )
		return SUCCESSFUL_RETURN;

	if ( ( offset < 0 ) || ( offset > n ) )
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );


	/* 1) Rotate types and status. */
	SubjectToType*   typeTmp   = new SubjectToType[offset];
	SubjectToStatus* statusTmp = new SubjectToStatus[offset];

	for( i=0; i<offset; ++i )
	{
		typeTmp[i] = getType( i );
		statusTmp[i] = getStatus( i );
	}

	for( i=0; i<n-offset; ++i )
	{
		setType( i,getType( i+offset ) );
		setStatus( i,getStatus( i+offset ) );
	}

	for( i=n-offset; i<n; ++i )
	{
		setType( i,typeTmp[i-n+offset] );
		setStatus( i,statusTmp[i-n+offset] );
	}

	delete[] statusTmp; delete[] typeTmp;

	/* 2) Construct shifted index lists of free and fixed variables. */
	Indexlist* rotatedActive   = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );
	Indexlist* rotatedInactive = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );
	Indexlist* rotatedDisabled = new Indexlist( INDEXLISTPHYSICALLENGTHFACTOR*n );

	for( i=0; i<n; ++i )
	{
		switch ( getStatus( i ) )
		{
			case ST_INACTIVE:
				if ( rotatedInactive->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete rotatedActive; delete rotatedInactive; delete rotatedDisabled;
					return THROWERROR( RET_ROTATING_FAILED );
				}
				break;

			case ST_LOWER:
				if ( rotatedActive->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete rotatedActive; delete rotatedInactive; delete rotatedDisabled;
					return THROWERROR( RET_ROTATING_FAILED );
				}
				break;

			case ST_UPPER:
				if ( rotatedActive->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete rotatedActive; delete rotatedInactive; delete rotatedDisabled;
					return THROWERROR( RET_ROTATING_FAILED );
				}
				break;

			case ST_DISABLING:
				if ( rotatedActive->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete rotatedActive; delete rotatedInactive; delete rotatedDisabled;
					return THROWERROR( RET_ROTATING_FAILED );
				}
				break;

			case ST_DISABLED:
				if ( rotatedDisabled->addNumber( i ) != SUCCESSFUL_RETURN )
				{
					delete rotatedActive; delete rotatedInactive; delete rotatedDisabled;
					return THROWERROR( RET_ROTATING_FAILED );
				}
				break;

			default:
				delete rotatedActive; delete rotatedInactive; delete rotatedDisabled;
				return THROWERROR( RET_ROTATING_FAILED );
		}
	}

	/* 3) Assign shifted index list. */
	delete active;
	active = rotatedActive;

	delete inactive;
	inactive = rotatedInactive;

	delete disabled;
	disabled = rotatedDisabled;

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t
 */
returnValue Constraints::print( ) const
{
	if ( n == 0 )
		return SUCCESSFUL_RETURN;

	#ifndef __XPCTARGET__
	#ifndef __DSPACE__
	char myPrintfString[160];

	int nIAC = getNIAC( );
	int nAC  = getNAC( );

	int* IAC_idx = new int[nIAC];
	if ( getInactive( )->getNumberArray( IAC_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] IAC_idx;
		return THROWERROR( RET_INDEXLIST_CORRUPTED );
	}

	int* AC_idx = new int[nAC];
	if ( getActive( )->getNumberArray( AC_idx ) != SUCCESSFUL_RETURN )
	{
		delete[] AC_idx; delete[] IAC_idx;
		return THROWERROR( RET_INDEXLIST_CORRUPTED );
	}

	snprintf( myPrintfString,160,"Constraints object comprising %d constraints (%d inactive, %d active):\n",n,nIAC,nAC );
	myPrintf( myPrintfString );

	qpOASES::print( IAC_idx,nIAC,"inactive" );
	qpOASES::print( AC_idx, nAC, "active  " );

	delete[] AC_idx; delete[] IAC_idx;
	#endif
	#endif

	return SUCCESSFUL_RETURN;
}



/*****************************************************************************
 *  P R O T E C T E D                                                        *
 *****************************************************************************/

/*
 *	s e t u p A l l
 */
returnValue Constraints::setupAll( SubjectToStatus _status )
{
	int i;

	/* 1) Place unbounded constraints at the beginning of the index list of inactive constraints. */
	for( i=0; i<n; ++i )
	{
		if ( ( getType( i ) == ST_UNBOUNDED ) && ( isEnabled( i ) == BT_TRUE ) )
		{
			if ( setupConstraint( i,_status ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
		}
	}

	/* 2) Add remaining (i.e. "real" inequality) constraints to the index list of inactive constraints. */
	for( i=0; i<n; ++i )
	{
		if ( ( getType( i ) == ST_BOUNDED ) && ( isEnabled( i ) == BT_TRUE ) )
		{
			if ( setupConstraint( i,_status ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
		}
	}

	/* 3) Place implicit equality constraints at the end of the index list of inactive constraints. */
	for( i=0; i<n; ++i )
	{
		if ( ( getType( i ) == ST_EQUALITY ) && ( isEnabled( i ) == BT_TRUE ) )
		{
			if ( setupConstraint( i,_status ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
		}
	}

	/* 4) Moreover, add all constraints of unknown type. */
	for( i=0; i<n; ++i )
	{
		if ( ( getType( i ) == ST_UNKNOWN ) && ( isEnabled( i ) == BT_TRUE ) )
		{
			if ( setupConstraint( i,_status ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
		}
	}

	/* 5) Finally, add disabled constraints to index list of disabled constraints! */
	for( i=0; i<n; ++i )
	{
		if ( isEnabled( i ) == BT_FALSE )
		{
			if ( setupConstraint( i,ST_DISABLED ) != SUCCESSFUL_RETURN )
				return THROWERROR( RET_SETUP_CONSTRAINT_FAILED );
		}
	}

	return SUCCESSFUL_RETURN;
}


#ifndef __DSPACE__
} /* qpOASES */
#endif


/*
 *	end of file
 */
