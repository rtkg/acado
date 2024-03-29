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
 *	\file INCLUDE/Bounds.hpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Declaration of the Bounds class designed to manage working sets of
 *	bounds within a QProblem.
 */


#ifndef QPOASES_BOUNDS_HPP
#define QPOASES_BOUNDS_HPP


#include <SubjectTo.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/** This class manages working sets of bounds by storing
 *	index sets and other status information.
 *
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 */
class Bounds : public SubjectTo
{
	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		Bounds( );

		/** Constructor which takes the number of bounds. */
		Bounds(	int _n									/**< Number of bounds. */
				);

		/** Copy constructor (deep copy). */
		Bounds(	const Bounds& rhs						/**< Rhs object. */
				);

		/** Destructor. */
		virtual ~Bounds( );

		/** Assignment operator (deep copy). */
		Bounds& operator=(	const Bounds& rhs			/**< Rhs object. */
							);


		/** Initially adds number of a new (i.e. not yet in the list) bound to
		 *  given index set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS \n
					RET_INVALID_ARGUMENTS */
		returnValue setupBound(	int number,				/**< Number of new bound. */
								SubjectToStatus _status	/**< Status of new bound. */
								);

		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set of free bounds; the order depends on the SujectToType
		 *  of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED */
		returnValue setupAllFree( );

		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set of fixed bounds (on their lower bounds);
		 *  the order depends on the SujectToType of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED */
		returnValue setupAllLower( );

		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set of fixed bounds (on their upper bounds);
		 *  the order depends on the SujectToType of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED */
		returnValue setupAllUpper( );


		/** Moves index of a bound from index list of fixed to that of free bounds.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MOVING_BOUND_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS */
		returnValue moveFixedToFree(	int number				/**< Number of bound to be freed. */
										);

		/** Moves index of a bound from index list of free to that of fixed bounds.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MOVING_BOUND_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS */
		returnValue moveFreeToFixed(	int number,				/**< Number of bound to be fixed. */
										SubjectToStatus _status	/**< Status of bound to be fixed. */
										);

		/** Swaps the indices of two free bounds within the index set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SWAPINDEX_FAILED */
		returnValue swapFree(	int number1,					/**< Number of first constraint or bound. */
								int number2						/**< Number of second constraint or bound. */
								);


		/** Returns number of variables.
		 *	\return Number of variables. */
		inline int getNV( ) const;

		/** Returns number of implicitly fixed variables.
		 *	\return Number of implicitly fixed variables. */
		inline int getNFV( ) const;

		/** Returns number of bounded (but possibly free) variables.
		 *	\return Number of bounded (but possibly free) variables. */
		inline int getNBV( ) const;

		/** Returns number of unbounded variables.
		 *	\return Number of unbounded variables. */
		inline int getNUV( ) const;

		/** Returns number of free variables.
		 *	\return Number of free variables. */
		inline int getNFR( ) const;

		/** Returns number of fixed variables.
		 *	\return Number of fixed variables. */
		inline int getNFX( ) const;


		/** Returns a pointer to free variables index list.
		 *	\return Pointer to free variables index list. */
		inline Indexlist* getFree( ) const;

		/** Returns a pointer to fixed variables index list.
		 *	\return Pointer to fixed variables index list. */
		inline Indexlist* getFixed( ) const;


		/** Shifts forward type and status of all bounds by a given
		 *  offset. This offset has to lie within the range [0,n/2] and has to
		 *  be an integer divisor of the total number of bounds n.
		 *  Type and status of the first \<offset\> bounds is thrown away,
		 *  type and status of the last \<offset\> bounds is doubled,
		 *  e.g. for offset = 2: \n
		 *  shift( {c/b1,c/b2,c/b3,c/b4,c/b5,c/b6} ) = {c/b3,c/b4,c/b5,c/b6,c/b5,c/b6}
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS \n
		 			RET_INVALID_ARGUMENTS \n
		 			RET_SHIFTING_FAILED */
		virtual returnValue shift(	int offset		/**< Shift offset within the range [0,n/2] and integer divisor of n. */
									);

		/** Rotates forward type and status of all bounds by a given
		 *  offset. This offset has to lie within the range [0,n].
		 *  Example for offset = 2: \n
		 *  rotate( {c/b1,c/b2,c/b3,c/b4,c/b5,c/b6} ) = {c/b3,c/b4,c/b5,c/b6,c/b1,c/b2}
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS \n
		 			RET_ROTATING_FAILED */
		virtual returnValue rotate(	int offset		/**< Rotation offset within the range [0,n]. */
									);


		/** Prints information on bounds object
		 *  (in particular, lists of free and fixed bounds.
		 * \return SUCCESSFUL_RETURN \n
				   RET_INDEXLIST_CORRUPTED */
		returnValue print( ) const;


	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:
		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set corresponding to the desired status;
		 *  the order depends on the SujectToType of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED */
		returnValue setupAll(	SubjectToStatus _status	/**< Desired initial status for all bounds. */
								);


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		Indexlist* freee;		/**< Index list of free variables. */
		Indexlist* fixed;		/**< Index list of fixed variables. */
};

#ifndef __DSPACE__
} /* qpOASES */
#endif

#include <Bounds.ipp>

#endif	/* QPOASES_BOUNDS_HPP */


/*
 *	end of file
 */
