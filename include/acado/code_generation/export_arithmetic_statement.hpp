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
 *    \file include/acado/code_generation/export_arithmetic_statement.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_ARITHMETIC_STATEMENT_HPP
#define ACADO_TOOLKIT_EXPORT_ARITHMETIC_STATEMENT_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_statement.hpp>
#include <acado/code_generation/export_data.hpp>



BEGIN_NAMESPACE_ACADO




/** 
 *	\brief ....
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class ....
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */

class ExportArithmeticStatement : public ExportStatement
{
	friend class ExportData;
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        ExportArithmeticStatement( );

        /** Default constructor. */
		ExportArithmeticStatement(	const ExportData* const _lhs,
									ExportStatementOperator _op0,
									const ExportData* const _rhs1,
									ExportStatementOperator _op1,
									const ExportData* const _rhs2,
									ExportStatementOperator _op2 = ESO_UNDEFINED,
									const ExportData* const _rhs3 = 0
									);

        /** Copy constructor (deep copy). */
        ExportArithmeticStatement(	const ExportArithmeticStatement& arg
							);

        /** Destructor. */
        virtual ~ExportArithmeticStatement( );

        /** Assignment operator (deep copy). */
        ExportArithmeticStatement& operator=(	const ExportArithmeticStatement& arg
									);

		/** Clone constructor (deep copy). */
		virtual ExportStatement* clone( ) const;


		uint getNumRows( ) const;
		
		uint getNumCols( ) const;


		virtual returnValue exportDataDeclaration(	FILE *file,
													const char* _realString = "double",
													const char* _intString = "int",
													int _precision = 16
													) const;

		virtual returnValue exportCode(	FILE* file,
										const char* _realString = "double",
										const char* _intString = "int",
										int _precision = 16
										) const;

		
		 ExportArithmeticStatement& unrollOuterLoop( );


	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		returnValue exportCodeAddSubtract(	FILE* file,
											const char* const _sign = "+",
											const char* _realString = "double",
											const char* _intString = "int",
											int _precision = 16
											) const;

		returnValue exportCodeMultiply(	FILE* file,
										BooleanType transposeRhs1 = BT_FALSE,
										const char* _realString = "double",
										const char* _intString = "int",
										int _precision = 16
										) const;
										
		returnValue exportCodeAssign(	FILE* file,
										const char* const _op = "=",
										const char* _realString = "double",
										const char* _intString = "int",
										int _precision = 16
										) const;


    protected:

		ExportData* lhs;
		ExportData* rhs1;
		ExportData* rhs2;
		ExportData* rhs3;

		ExportStatementOperator op0;    // code expression of form:
		ExportStatementOperator op1;    // code expression of form:
		ExportStatementOperator op2;    //  lhs <op0> rhs1 <op1> rhs2 <op2> rhs3
		
		ExportIndex outerLoopVariable;
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_ARITHMETIC_STATEMENT_HPP

// end of file.
