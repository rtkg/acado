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
 *    \file src/code_generation/gauss_newton_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/gauss_newton_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussNewtonExport::GaussNewtonExport( )
{
}

GaussNewtonExport::GaussNewtonExport( const GaussNewtonExport& arg )
{
	condenser = arg.condenser;
	uBounds   = arg.uBounds;
}


GaussNewtonExport::~GaussNewtonExport( ){ }


GaussNewtonExport& GaussNewtonExport::operator=( const GaussNewtonExport& arg ){

    if( this != &arg ){
		condenser  = arg.condenser ;
		uBounds = arg.uBounds;
    }
    return *this;
}

returnValue GaussNewtonExport::setCondensingExport( const CondensingExport &condenser_ ){

     condenser = condenser_;
     return SUCCESSFUL_RETURN;
}



returnValue GaussNewtonExport::setControlBounds(	const VariablesGrid &_uBounds
													)
{
	BooleanType isFinite = BT_FALSE;
	Vector lbTmp;
	Vector ubTmp;

	for( uint i=0; i<_uBounds.getNumPoints(); ++i )
	{
		lbTmp = _uBounds.getLowerBounds(i);
		ubTmp = _uBounds.getUpperBounds(i);

		if ( (ubTmp-lbTmp).isPositive() == BT_FALSE )
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		
		if ( ( lbTmp.isFinite( ) == BT_TRUE ) || ( ubTmp.isFinite( ) == BT_TRUE ) )
			isFinite = BT_TRUE;
	}

	if ( isFinite == BT_TRUE )
		uBounds = _uBounds;
	else
		uBounds.init();

	return SUCCESSFUL_RETURN;
}



returnValue GaussNewtonExport::exportHeader( FILE* file ) const
{
    condenser.exportHeader( file );

    acadoFPrintf( file, "       real_t deltaX0[%d];\n", getNX() );

    return SUCCESSFUL_RETURN;
}


returnValue GaussNewtonExport::exportForwardDeclarations( FILE* file ) const
{
    condenser.exportForwardDeclarations( file );

    acadoFPrintf( file, "   void   preparationStep( );\n");
    acadoFPrintf( file, "   void   initialValueEmbedding( );\n");
    acadoFPrintf( file, "   void   feedbackStep( real_t* );\n");
    acadoFPrintf( file, "   void   shiftControls( real_t* );\n");
    acadoFPrintf( file, "   void   shiftStates( real_t* );\n");
    acadoFPrintf( file, "   real_t getKKT( );\n");

    return SUCCESSFUL_RETURN;
}


returnValue GaussNewtonExport::exportCode( const String& dirName ) const
{
    uint run1, run2;

    condenser.exportCode( dirName );

    String fileName( dirName );
    fileName += "/gauss_newton_method.c";

    FILE *file = acadoFOpen( fileName.getName(), "w" );
	if ( file == 0 )
		return ACADOERROR( RET_DOES_DIRECTORY_EXISTS );

    acadoPrintAutoGenerationNotice( file );

    acadoFPrintf( file, "#include \"acado.h\"\n\n\n");

	
	Matrix lbValuesMatrix( getN(),getNU() );
	Matrix ubValuesMatrix( getN(),getNU() );

	for( run1=0; run1<getN(); ++run1 )
		for( run2=0; run2<getNU(); ++run2 )
		{
			lbValuesMatrix( run1,run2 ) = uBounds.getLowerBound(run1,run2);
			ubValuesMatrix( run1,run2 ) = uBounds.getUpperBound(run1,run2);
		}

	
	ExportData x ( "acadoVariables.x",    "real_t", getN()+1,getNX() );
	ExportData u ( "acadoVariables.u",    "real_t", getN(),getNU() );
	ExportData xEnd( "xEnd",              "real_t", 1,getNX() );
	ExportData uEnd( "uEnd",              "real_t", 1,getNU() );
	
	ExportData x0 ( "x0",                     "real_t", getNX(),1 );
	ExportData dX0( "acadoWorkspace.deltaX0", "real_t", getNX(),1 );
	
	ExportData H01( "acadoWorkspace.H01",     "real_t", getNX(),getNU()*getN() );
	ExportData H11( "acadoWorkspace.H11",     "real_t", getNU()*getN(),getNU()*getN() );
	
	ExportData H ( "params.H",            "real_t", getNU()*getN(),getNU()*getN() );
	ExportData A ( "params.A",            "real_t", getNumStateBounds( ),getN()*getNU() );
	ExportData C ( "acadoWorkspace.C",    "real_t", getN()*getNX(),getNX() );
	ExportData E ( "acadoWorkspace.E",    "real_t", getN()*getNX(),getN()*getNU() );
	
	ExportData g ( "params.g",            "real_t", getN()*getNU(),1 );
	ExportData g1( "acadoWorkspace.g1",   "real_t", getN()*getNU(),1 );
	
	ExportData lb( "params.lb", "real_t", getN(),getNU() );
	ExportData ub( "params.ub", "real_t", getN(),getNU() );
	ExportData lbValues( "lb",  "real_t", lbValuesMatrix );
	ExportData ubValues( "ub",  "real_t", ubValuesMatrix );
	
	ExportData lbA( "params.lbA", "real_t", getNumStateBounds( ),1 );
	ExportData ubA( "params.ubA", "real_t", getNumStateBounds( ),1 );
	ExportData lbAValues( "acadoWorkspace.lbA", "real_t", getNumStateBounds( ),1 );
	ExportData ubAValues( "acadoWorkspace.ubA", "real_t", getNumStateBounds( ),1 );
	
	ExportData xVars( "vars.x",    "real_t", getN()*getNU(),1 );
	
	ExportData tmp( "tmp", "real_t", 1,1, BT_TRUE );


	ExportFunction preparationStep( "preparationStep" );
	
	preparationStep.addFunctionCall( "setupQP" );
	
	preparationStep.addStatement( lb == lbValues - u );
	preparationStep.addStatement( ub == ubValues - u );

	preparationStep.exportCode( file );


	ExportFunction initialValueEmbedding( "initialValueEmbedding" );

	initialValueEmbedding.addStatement( "real_t tmp;\n" );
	initialValueEmbedding.addStatement( g == g1 + (H01^dX0) );
	initialValueEmbedding.addLinebreak( );

	if( getNumStateBounds( ) > 0 )
	{
		for( run1 = 0; run1 < getNumStateBounds( ); run1++ )
			initialValueEmbedding.addStatement( A.getRow(run1) == E.getRow( condenser.xBoundsIdx[run1]-getNX() ) );
		
		// shift constraint bounds by first interval
		for( run1 = 0; run1 < getNumStateBounds( ); run1++ )
		{
			initialValueEmbedding.addStatement( tmp == x.makeColVector().getRow( condenser.xBoundsIdx[run1] ) + C.getRow( condenser.xBoundsIdx[run1]-getNX() )*dX0 );
			initialValueEmbedding.addStatement( lbA.getRow(run1) == lbAValues.getRow(run1) - tmp );
			initialValueEmbedding.addStatement( ubA.getRow(run1) == ubAValues.getRow(run1) - tmp );
		}
	}
	initialValueEmbedding.addLinebreak( );
    
	initialValueEmbedding.addStatement( H == H11 ) ;

	initialValueEmbedding.exportCode( file );


	ExportFunction feedbackStep( "feedbackStep",x0 );

	feedbackStep.addStatement( dX0 == x0 - x.makeColVector().getRows( 0,getNX() ) );
	feedbackStep.addLinebreak( );
	
	feedbackStep.addFunctionCall( "initialValueEmbedding" );
	feedbackStep.addFunctionCall( "solve" );
	feedbackStep.addLinebreak( );

	feedbackStep.addStatement( u.makeColVector() += xVars );
	feedbackStep.addStatement( x.makeColVector().getRows( 0,getNX() ) += dX0 );
	
	feedbackStep.exportCode( file );


	ExportFunction shiftControls( "shiftControls",uEnd );
	
	for( run1 = 1; run1 < getN(); run1++ )
		shiftControls.addStatement( u.getRow(run1-1) == u.getRow(run1) );
	
	shiftControls.addStatement( u.getRow(getN()-1) == uEnd );

	shiftControls.exportCode( file );
	

//     acadoFPrintf( file, "void shiftStates( real_t *xEnd ){\n");

	ExportFunction shiftStates( "shiftStates",xEnd );
	
	shiftStates.addStatement( x.getRow(0) == x.getRow(1) );
	shiftStates.addStatement( x.makeColVector().getRows( 0,getNX() ) += C.getRows( 0,getNX() )*dX0 );
	shiftStates.addStatement( x.makeColVector().getRows( 0,getNX() ) += E.getSubMatrix( 0,getNX(),0,getNU() )*xVars.getRows( 0,getNU() ) );

	shiftStates.exportCode( file );


	ExportFunction getKKT( "getKKT" );
	getKKT.setReturnValue( tmp );
        
	getKKT.addStatement( "real_t tmp;\n" );
	getKKT.addStatement( tmp == (g1^xVars) );
	getKKT.addStatement( "tmp = fabs( tmp );\n" );
	
	getKKT.exportCode( file );

// 	acadoFPrintf( file, "\n");
/*	acadoFPrintf( file, "    int run1;\n");
    acadoFPrintf( file, "    for( run1 = 0; run1 < %d; run1++ ){\n",getN()*getNU() );
	acadoFPrintf( file, "        if ( vars.y[run1] > %e )\n", 1.0/INFTY );
	acadoFPrintf( file, "            tmp += fabs( (acadoVariables.u[run1]-params.lb[run1])*vars.y[run1] );\n");
	acadoFPrintf( file, "        if ( vars.y[run1] < %e )\n", -1.0/INFTY );
	acadoFPrintf( file, "            tmp += fabs( (acadoVariables.u[run1]-params.ub[run1])*vars.y[run1] );\n");
	acadoFPrintf( file, "    }\n");
	acadoFPrintf( file, "\n");*/
	if ( getNumStateBounds() > 0 )
	{
    /*acadoFPrintf( file, "    for( run1 = 0; run1 < %d; run1++ ){\n",getN()*getNX() );
	acadoFPrintf( file, "        if ( vars.y[%d+run1] > %e )\n",getN()*getNU(), 1.0/INFTY );
	acadoFPrintf( file, "            tmp += fabs( (acadoVariables.x[run1]-acadoWorkspace.lbA[run1])*vars.y[%d+run1] );\n",getN()*getNU() );
	acadoFPrintf( file, "        if ( vars.y[%d+run1] < %e )\n",getN()*getNU(), -1.0/INFTY );
	acadoFPrintf( file, "            tmp += fabs( (acadoVariables.x[run1]-acadoWorkspace.ubA[run1])*vars.y[%d+run1] );\n",getN()*getNU() );
	acadoFPrintf( file, "    }\n");
	acadoFPrintf( file, "\n");
	*/}


    fclose(file);

    return SUCCESSFUL_RETURN;
}


uint GaussNewtonExport::getNX() const{ return condenser.getNX(); }
uint GaussNewtonExport::getNU() const{ return condenser.getNU(); }
uint GaussNewtonExport::getN () const{ return condenser.getN() ; }


uint GaussNewtonExport::getNumStateBounds( ) const
{
	return condenser.getNumStateBounds();
}


CLOSE_NAMESPACE_ACADO

// end of file.
