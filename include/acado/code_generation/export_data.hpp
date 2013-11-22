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
 *    \file include/acado/code_generation/export_data.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_DATA_HPP
#define ACADO_TOOLKIT_EXPORT_DATA_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_data_argument.hpp>
#include <acado/code_generation/export_index.hpp>



BEGIN_NAMESPACE_ACADO


class ExportArithmeticStatement;


/** 
 *	\brief ....
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class ....
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */

class ExportData : public ExportDataArgument
{
	friend class ExportFunction;

    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        ExportData( );

		ExportData(	const char* const _name,
					const char* const _typeString = "int",
					uint _nRows = 1,
					uint _nCols = 1,
					BooleanType _callItByValue = BT_FALSE
					);
					
		ExportData(	const char* const _name,
					const char* const _typeString,
					const Matrix& _data,
					BooleanType _callItByValue = BT_FALSE
					);

        /** Copy constructor (deep copy). */
        ExportData(	const ExportData& arg
					);

        /** Destructor. */
        ~ExportData( );

        /** Assignment operator (deep copy). */
        ExportData& operator=(	const ExportData& arg
								);
								
		ExportData& operator=(	const Matrix& arg
								);


		double& operator()(	uint rowIdx,
							uint colIdx
							);
							
		double operator()(	uint rowIdx,
							uint colIdx
							) const;

		BooleanType isZero( const ExportIndex& rowIdx,
							const ExportIndex& colIdx
							) const;

		BooleanType isZero( const ExportIndex& rowIdx,
							uint colIdx
							) const;

		BooleanType isZero( uint rowIdx,
							const ExportIndex& colIdx
							) const;

		BooleanType isZero( uint rowIdx,
							uint colIdx
							) const;

							
		BooleanType isOne(	const ExportIndex& rowIdx,
							const ExportIndex& colIdx
							) const;

		BooleanType isOne(	const ExportIndex& rowIdx,
							uint colIdx
							) const;

		BooleanType isOne(	uint rowIdx,
							const ExportIndex& colIdx
							) const;

		BooleanType isOne(	uint rowIdx,
							uint colIdx
							) const;

		const char* get(	const ExportIndex& rowIdx,
							const ExportIndex& colIdx
							) const;

		const char* get(	const ExportIndex& rowIdx,
							uint colIdx
							) const;
							
		const char* get(	uint rowIdx,
							const ExportIndex& colIdx
							) const;
							
		const char* get(	uint rowIdx,
							uint colIdx
							) const;


		virtual uint getNumRows( ) const;
		
		virtual uint getNumCols( ) const;

		virtual uint getDim( ) const;


		ExportArithmeticStatement operator+(	const ExportData& arg
												) const;

		ExportArithmeticStatement operator-(	const ExportData& arg
												) const;

		ExportArithmeticStatement operator+=(	const ExportData& arg
												) const;

		ExportArithmeticStatement operator-=(	const ExportData& arg
												) const;

		ExportArithmeticStatement operator*(	const ExportData& arg
												) const;

		ExportArithmeticStatement operator^(	const ExportData& arg
												) const;

		ExportArithmeticStatement operator==(	const ExportData& arg
												) const;


		ExportArithmeticStatement operator==(	ExportArithmeticStatement arg
												) const;

		ExportArithmeticStatement operator+(	ExportArithmeticStatement arg
												) const;

		ExportArithmeticStatement operator-(	ExportArithmeticStatement arg
												) const;

		ExportArithmeticStatement operator+=(	ExportArithmeticStatement arg
												) const;

		ExportArithmeticStatement operator-=(	ExportArithmeticStatement arg
												) const;


		ExportArithmeticStatement operator+(	const Matrix& arg
												) const;

		ExportArithmeticStatement operator-(	const Matrix& arg
												) const;

		ExportArithmeticStatement operator+=(	const Matrix& arg
												) const;

		ExportArithmeticStatement operator-=(	const Matrix& arg
												) const;

		ExportArithmeticStatement operator*(	const Matrix& arg
												) const;
												
		ExportArithmeticStatement operator^(	const Matrix& arg
												) const;

		ExportArithmeticStatement operator==(	const Matrix& arg
												) const;


		ExportData getTranspose( ) const;


		ExportData getRow(	uint idx
							) const;

		ExportData getRow(	const ExportIndex& idx
							) const;

		ExportData getCol(	uint idx
							) const;

		ExportData getCol(	const ExportIndex& idx
							) const;


		ExportData getRows(	uint idx1,
							uint idx2
							) const;

		ExportData getRows(	const ExportIndex& idx1,
							const ExportIndex& idx2
							) const;

		ExportData getCols(	uint idx1,
							uint idx2
							) const;
							
		ExportData getCols(	const ExportIndex& idx1,
							const ExportIndex& idx2
							) const;

		ExportData getSubMatrix(	uint rowIdx1,
									uint rowIdx2,
									uint colIdx1,
									uint colIdx2
									) const;

		ExportData getSubMatrix(	const ExportIndex& rowIdx1,
									const ExportIndex& rowIdx2,
									uint colIdx1,
									uint colIdx2
									) const;

		ExportData getSubMatrix(	uint rowIdx1,
									uint rowIdx2,
									const ExportIndex& colIdx1,
									const ExportIndex& colIdx2
									) const;

		ExportData getSubMatrix(	const ExportIndex& rowIdx1,
									const ExportIndex& rowIdx2,
									const ExportIndex& colIdx1,
									const ExportIndex& colIdx2
									) const;

									
		ExportData makeRowVector( ) const;

		ExportData makeColVector( ) const;


		BooleanType isVector( ) const;

		returnValue print( ) const;


	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		returnValue setSubmatrixOffsets(	uint _rowOffset,
											uint _colOffset,
											uint _colDim,
											uint _nRows = 0,
											uint _nCols = 0
											);

		returnValue setSubmatrixOffsets(	const ExportIndex& _rowOffset,
											const ExportIndex& _colOffset,
											uint _colDim,
											uint _nRows = 0,
											uint _nCols = 0
											);

    protected:

		ExportIndex rowOffset;
		ExportIndex colOffset;
		uint colDim;
		uint nRows;
		uint nCols;
};


static const ExportData emptyConstExportData;


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_DATA_HPP

// end of file.
