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
 *	\file INCLUDE/Constants.hpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2007-2009
 *
 *	Definition of all global constants.
 */


#ifndef QPOASES_CONSTANTS_HPP
#define QPOASES_CONSTANTS_HPP


#ifndef __DSPACE__
namespace qpOASES
{
#endif


/** Numerical value of machine precision (min eps, s.t. 1+eps > 1).
	Note: this value has to be positive! */
const double EPS = 2.221e-16;

/** Numerical value of zero (for situations in which it would be
 *	unreasonable to compare with 0.0).
 *  Note: this value has to be positive! */
const double ZERO = 1.0e-50;

/** Numerical value of infinity (e.g. for non-existing bounds).
	Note: this value has to be positive! */
const double INFTY = 1.0e12;


/** Lower/upper (constraints') bound tolerance (an inequality constraint
 *	whose lower and upper bound differ by less than BOUNDTOL is regarded
 *	to be an equality constraint).
 *  Note: this value has to be positive! */
const double BOUNDTOL = 1.0e-10;

/** Offset for relaxing (constraints') bounds at beginning of
 *  an initial homotopy.
 *  Note: this value has to be positive! */
const double BOUNDRELAXATION = 1.0e3;


/** When enabling a constraint, it is relaxed to
 *  (1 +/- ENABLINGFACTOR ) * lbA/ubA +/- ENABLINGOFFSET.
 *  Note: this value has to be positive! */
const double ENABLINGFACTOR = 1.0e-1;

/** When enabling a constraint, it is relaxed to
 *  (1 +/- ENABLINGFACTOR ) * lbA/ubA +/- ENABLINGOFFSET.
 *  Note: this value has to be positive! */
const double ENABLINGOFFSET = 1.0e-6;


/** Factor that determines physical lengths of index lists.
 *  Note: this value has to be larger than 1 (i.e. >= 2)! */
const int INDEXLISTPHYSICALLENGTHFACTOR = 10;


/** Scaling factor of identity matrix used for Hessian regularisation.
 *  Note: this value has to be positive! */
const double EPS_FOR_REGULARISATION = 1.0e-12;

/** Maximum number of successive regularisation steps.
 *  Note: this value should be larger than 0 (i.e. >= 1)! */
const int MAX_NUMBER_OF_REGULARISATION_STEPS = 2;


#ifndef __DSPACE__
} /* qpOASES */
#endif


#endif	/* QPOASES_CONSTANTS_HPP */


/*
 *	end of file
 */
