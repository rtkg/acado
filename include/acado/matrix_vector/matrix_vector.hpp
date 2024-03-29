/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *    \file include/acado/matrix_vector/matrix_vector.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 31.05.2008
 */


#ifndef ACADO_TOOLKIT_MATRIX_VECTOR_HPP
#define ACADO_TOOLKIT_MATRIX_VECTOR_HPP


#include <acado/utils/acado_utils.hpp>

#include <acado/matrix_vector/vectorspace_element.hpp>

#include <acado/matrix_vector/vector.hpp>
#include <acado/matrix_vector/matrix.hpp>
#include <acado/matrix_vector/block_matrix.hpp>

#include <acado/matrix_vector/vector.ipp>
#include <acado/matrix_vector/matrix.ipp>
#include <acado/matrix_vector/block_matrix.ipp>


BEGIN_NAMESPACE_ACADO

static       VectorspaceElement emptyVectorspaceElement;
static const VectorspaceElement emptyConstVectorspaceElement;

static       Vector emptyVector;
static const Vector emptyConstVector;

static       Matrix emptyMatrix;
static const Matrix emptyConstMatrix;

static       BlockMatrix emptyBlockMatrix;
//static const BlockMatrix emptyConstBlockMatrix;

CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_MATRIX_VECTOR_HPP

/*
 *	end of file
 */
