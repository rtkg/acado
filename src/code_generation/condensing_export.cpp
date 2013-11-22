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
 *    \file src/code_generation/condensing_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/condensing_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

CondensingExport::CondensingExport()
{
	N = 0;
	
	xBoundsIdx = 0;
	nxBounds = 0;
}

CondensingExport::CondensingExport( const CondensingExport& arg )
{
	N           = arg.N          ;
	
	sqrtQ       = arg.sqrtQ      ;
	sqrtQplusQF = arg.sqrtQplusQF;
	R           = arg.R          ;
	
	xBounds     = arg.xBounds    ;
	
	if ( arg.xBoundsIdx != 0 )
	{
		xBoundsIdx = new int[arg.nxBounds];
		for( uint i=0; i<arg.nxBounds; ++i )
			xBoundsIdx[i] = arg.xBoundsIdx[i];
	}
	else
		xBoundsIdx = 0;

	nxBounds  = arg.nxBounds;
	
	integrator  = arg.integrator ;
}


CondensingExport::~CondensingExport( )
{
	if ( xBoundsIdx != 0 )
		delete[] xBoundsIdx;
}


CondensingExport& CondensingExport::operator=( const CondensingExport& arg )
{
    if( this != &arg )
	{
		if ( xBoundsIdx != 0 )
			delete[] xBoundsIdx;


		N           = arg.N          ;
		
		sqrtQ       = arg.sqrtQ      ;
		sqrtQplusQF = arg.sqrtQplusQF;
		R           = arg.R          ;
		
		xBounds     = arg.xBounds    ;
		
		if ( arg.xBoundsIdx != 0 )
		{
			xBoundsIdx = new int[arg.nxBounds];
			for( uint i=0; i<arg.nxBounds; ++i )
				xBoundsIdx[i] = arg.xBoundsIdx[i];
		}
		else
			xBoundsIdx = 0;

		nxBounds  = arg.nxBounds;
		
		integrator  = arg.integrator ;
	}

    return *this;
}


returnValue CondensingExport::setIntegratorExport( const IntegratorExport &integrator_, uint N_ ){

    integrator = integrator_;
    N          = N_         ;

    return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::setWeightingMatrices( const Matrix &Q_, const Matrix &QF_, const Matrix &R_ )
{
	Matrix Id = eye( Q_.getNumRows() );
	Id *= SQRT_EPS;

    sqrtQ = (Q_+Id).getCholeskyDecomposition( );
    sqrtQplusQF = (Q_+QF_+Id).getCholeskyDecomposition( );

    R  = R_ ;

    return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::setStateBounds(	const VariablesGrid &_xBounds
												)
{
	BooleanType isFinite = BT_FALSE;
	Vector lbTmp;
	Vector ubTmp;
	
	if ( xBoundsIdx != 0 )
		delete[] xBoundsIdx;
	xBoundsIdx = new int[_xBounds.getDim()+1];
// 	printf("dim=%d\n",_xBounds.getDim());
	
	for( uint i=1; i<_xBounds.getNumPoints(); ++i )
	{
		lbTmp = _xBounds.getLowerBounds(i);
		ubTmp = _xBounds.getUpperBounds(i);

		for( uint j=0; j<lbTmp.getDim(); ++j )
		{
			if ( acadoIsGreater( ubTmp(j),lbTmp(j) ) == BT_FALSE )
				return ACADOERROR( RET_INVALID_ARGUMENTS );
			
			if ( ( acadoIsFinite( ubTmp(j) ) == BT_TRUE ) || ( acadoIsFinite( lbTmp(j) ) == BT_TRUE ) )
			{
				xBoundsIdx[nxBounds] = i*lbTmp.getDim()+j;
				++nxBounds;
				isFinite = BT_TRUE;
			}
		}
	}

	xBoundsIdx[nxBounds] = -1;

// 	for ( uint ii=0; ii<nxBounds+1; ++ii )
// 		printf("%d\n",xBoundsIdx[ii] );
		
	if ( isFinite == BT_TRUE )
		xBounds = _xBounds;
	else
		xBounds.init();

	return SUCCESSFUL_RETURN;
}



uint CondensingExport::getNX() const{ return integrator.getNX(); }
uint CondensingExport::getNU() const{ return integrator.getNU(); }
uint CondensingExport::getN () const{ return N ; }


uint CondensingExport::getNumStateBounds( ) const
{
	return nxBounds;
}


returnValue CondensingExport::exportHeader( FILE* file ) const
{
    integrator.exportHeader( file );

    acadoFPrintf( file, "       real_t state[%d];\n", getNX() * (getNX()+getNU()+1) + getNU() );

	acadoFPrintf( file, "       real_t g1[%d];\n", getN()*getNU() );

    acadoFPrintf( file, "       real_t H01[%d];\n", getNX()*getN()*getNU() );
    acadoFPrintf( file, "       real_t H11[%d];\n", getN()*getN()*getNU()*getNU() );

    if( getNumStateBounds( ) > 0 ){
        acadoFPrintf( file, "       real_t lbA[%d];\n", nxBounds );
        acadoFPrintf( file, "       real_t ubA[%d];\n", nxBounds );
    }

	acadoFPrintf( file, "       real_t C[%d];\n", getNX()*getNX()*getN() );
    acadoFPrintf( file, "       real_t QC[%d];\n", getNX()*getNX()*getN() );
	
	acadoFPrintf( file, "       real_t Gx[%d];\n", getNX()*getNX() );
	
	acadoFPrintf( file, "       real_t E[%d];\n", getN()*getNX()*getN()*getNU() );
    acadoFPrintf( file, "       real_t QE[%d];\n", getN()*getN()*getNX()*getNU() );

	acadoFPrintf( file, "       real_t Gu[%d];\n", getNX()*getNU() );

    acadoFPrintf( file, "       real_t Dx[%d];\n", getNX()*getN() );
    acadoFPrintf( file, "       real_t QDx[%d];\n", getNX()*getN());

	acadoFPrintf( file, "       real_t Du[%d];\n", getNU()*getN() );
    acadoFPrintf( file, "       real_t RDu[%d];\n", getNU()*getN());

    return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::exportForwardDeclarations( FILE* file ) const
{
    integrator.exportForwardDeclarations( file );

    acadoFPrintf( file, "   void   multiplyQ1( real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiplyQ2( real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiplyQ3( real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiplyQ4( real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiplyQ5( real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiplyQ6( real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiplyQ7( real_t*, real_t* );\n");

    acadoFPrintf( file, "   void   multiply1 ( real_t*, real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiply2 ( real_t*, real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiply8 ( real_t*, real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiply10( real_t*, real_t*, real_t* );\n");
    acadoFPrintf( file, "   void   multiply11( real_t*, real_t*, real_t* );\n");

    acadoFPrintf( file, "   void   condense1 ( int    , real_t* );\n");
    acadoFPrintf( file, "   void   condense2 ( );\n" );
    acadoFPrintf( file, "   void   setupQP   ( );\n" );

    return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::exportCode( const String& dirName ) const
{
    integrator.exportCode( dirName );

    String fileName( dirName );
    fileName += "/condensing.c";

    FILE *file = acadoFOpen( fileName.getName(), "w" );
	if ( file == 0 )
		return ACADOERROR( RET_DOES_DIRECTORY_EXISTS );

    acadoPrintAutoGenerationNotice( file );

    acadoFPrintf( file, "#include \"acado.h\" \n\n\n" );

    exportMultiplicationRoutines( file );
    exportCondensing1( file );
    exportCondensing2( file );
    exportEvaluation( file );

    fclose( file );

    return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

void CondensingExport::exportMultiplicationRoutines( FILE *file ) const
{
	ExportData QQ ( "Q", "real_t",sqrtQ );
	ExportData RR ( "R", "real_t",R );
	ExportData QQF( "QQF","real_t",sqrtQplusQF );
	ExportData QC1( "CC","real_t",getNX(),getNX() );
	ExportData QE1( "CC","real_t",getNX(),getNU()*getN() );
	ExportData Dx1( "B", "real_t",getNX(),1 );
	ExportData QDx1("CC","real_t",getNX(),1 );
	ExportData Du1( "B", "real_t",getNU(),1 );
	ExportData RDu1("CC","real_t",getNU(),1 );
	ExportData Gx ( "A", "real_t",getNX(),getNX() );
	ExportData C1 ( "B", "real_t",getNX(),getNX() );
	ExportData C2 ( "CC","real_t",getNX(),getNX() );
	ExportData E1 ( "B", "real_t",getNX(),getNU()*getN() );
	ExportData E2 ( "CC","real_t",getNX(),getNU()*getN() );
	ExportData QEa( "A", "real_t",getNX()*getN(),getNU()*getN() );
	ExportData QEb( "B", "real_t",getNX()*getN(),getNU()*getN() );
	ExportData QDx( "B", "real_t",getNX()*getN(),1 );
	ExportData g1 ( "CC","real_t",getNU()*getN(),1 );
	ExportData QC ( "A", "real_t",getNX()*getN(),getNX() );
	ExportData H01( "CC","real_t",getNX(),getNU()*getN() );
	ExportData H11( "CC","real_t",getNU()*getN(),getNU()*getN() );
	
	ExportFunction multiplyQ1( "multiplyQ1", QQ,C1,QC1 );
	multiplyQ1.addStatement( QC1 == QQ*C1 );
	multiplyQ1.exportCode( file,"real_t" );

	ExportFunction multiplyQ2( "multiplyQ2", QQ,E1,QE1 );
	multiplyQ2.addStatement( QE1 == QQ*E1 );
	multiplyQ2.exportCode( file,"real_t" );

	ExportFunction multiplyQ3( "multiplyQ3", QQ,Dx1,QDx1 );
	multiplyQ3.addStatement( QDx1 == QQ*Dx1 );
	multiplyQ3.exportCode( file,"real_t" );

	ExportFunction multiplyQ4( "multiplyQ4", RR,Du1,RDu1 );
	multiplyQ4.addStatement( RDu1 == RR*Du1 );
	multiplyQ4.exportCode( file,"real_t" );

	ExportFunction multiplyQ5( "multiplyQ5", QQF,C1,QC1 );
	multiplyQ5.addStatement( QC1 == QQF*C1 );
	multiplyQ5.exportCode( file,"real_t" );

	ExportFunction multiplyQ6( "multiplyQ6", QQF,E1,QE1 );
	multiplyQ6.addStatement( QE1 == QQF*E1 );
	multiplyQ6.exportCode( file,"real_t" );

	ExportFunction multiplyQ7( "multiplyQ7", QQF,Dx1,QDx1 );
	multiplyQ7.addStatement( QDx1 == QQF*Dx1 );
	multiplyQ7.exportCode( file,"real_t" );

	ExportFunction multiply1( "multiply1", Gx,C1,C2 );
	multiply1.addStatement( C2 == Gx*C1 );
	multiply1.exportCode( file,"real_t" );

	ExportFunction multiply2( "multiply2", Gx,E1,E2 );
	multiply2.addStatement( E2 == Gx*E1 );
	multiply2.exportCode( file,"real_t" );

	ExportFunction multiply8( "multiply8", QEa,QDx,g1 );
	multiply8.addStatement( g1 == (QEa^QDx) );
	multiply8.exportCode( file,"real_t" );

	ExportFunction multiply10( "multiply10", QC,QEb,H01 );
	multiply10.addStatement( H01 == (QC^QEb) );
	multiply10.exportCode( file,"real_t" );

	ExportFunction multiply11( "multiply11", QEa,QEb,H11 );
	multiply11.addStatement( (H11 == (QEa^QEb)).unrollOuterLoop() );
	multiply11.exportCode( file,"real_t" );
}



void CondensingExport::exportCondensing1( FILE *file ) const
{
	ExportData Dx  ( "acadoWorkspace.Dx",  "real_t", getN(), getNX() );
	ExportData Du  ( "acadoWorkspace.Du",  "real_t", getN(), getNU() );
	ExportData xRef( "acadoVariables.xRef","real_t", getN(), getNX() );
	ExportData uRef( "acadoVariables.uRef","real_t", getN(), getNU() );
	ExportData yy  ( "yy",                 "real_t", 1, getNX()*(getNX()+getNU()+1)+getNU() );

	ExportData Gx  ( "acadoWorkspace.Gx",  "real_t", getNX(),getNX() );
	ExportData Gu  ( "acadoWorkspace.Gu",  "real_t", getNX(),getNU() );
	ExportData C   ( "acadoWorkspace.C",   "real_t", getN()*getNX(), getNX() );
	ExportData E   ( "acadoWorkspace.E",   "real_t", getN()*getNX(), getN()*getNU() );
	
	ExportIndex index( "index" );


	ExportFunction condense1( "condense1", index.makeArgument(),yy );
	
	condense1.addStatement( Dx.getRow(index) == yy.getCols( 0,getNX() ) - xRef.getRow(index) );
	
	uint uIdx = getNX()*(getNX()+getNU()+1);
	condense1.addStatement( Du.getRow(index) == yy.getCols( uIdx,uIdx+getNU() ) - uRef.getRow(index) );
	condense1.addStatement( Gx.makeRowVector() == yy.getCols( getNX(),getNX()+getNX()*getNX() ) );

	uIdx = getNX()*(getNX()+1);
	condense1.addStatement( Gu.makeRowVector() == yy.getCols( uIdx,uIdx+getNX()*getNU() ) );

    condense1.addStatement( "if( index != 0 ){\n" );
	condense1.addFunctionCall( "multiply1", Gx,C.getAddress((index-1)*getNX(),0), C.getAddress(index*getNX(),0) );
	condense1.addFunctionCall( "multiply2", Gx,E.getAddress((index-1)*getNX(),0), E.getAddress(index*getNX(),0) );
    condense1.addStatement( "}\nelse{\n");
	condense1.addStatement( C.getRows( 0,getNX() ) == Gx );
	condense1.addStatement( "}\n" );

	condense1.addStatement( E.getSubMatrix( index*getNX(),(index+1)*getNX(), index*getNU(),(index+1)*getNU() ) == Gu );

	condense1.exportCode( file );
}



void CondensingExport::exportCondensing2( FILE *file ) const{

    uint run1;
	
	ExportFunction condense2( "condense2" );

	ExportData QQ ( "Q", "real_t",sqrtQ );
	ExportData RR ( "R", "real_t",R );
	ExportData QQF( "QQF","real_t",sqrtQplusQF );

	ExportData C  ( "acadoWorkspace.C",   "real_t", getN()*getNX(), getNX() );
	ExportData QC ( "acadoWorkspace.QC",  "real_t", getN()*getNX(), getNX() );
	ExportData E  ( "acadoWorkspace.E",   "real_t", getN()*getNX(), getN()*getNU() );
	ExportData QE ( "acadoWorkspace.QE",  "real_t", getN()*getNX(), getN()*getNU() );
	ExportData Dx ( "acadoWorkspace.Dx",  "real_t", getN()*getNX(), 1 );
	ExportData QDx( "acadoWorkspace.QDx", "real_t", getN()*getNX(), 1 );
	ExportData Du ( "acadoWorkspace.Du",  "real_t", getN()*getNU(), 1 );
	ExportData RDu( "acadoWorkspace.RDu", "real_t", getN()*getNU(), 1 );
	ExportData g1 ( "acadoWorkspace.g1",  "real_t", getN()*getNU(), 1 );
	ExportData H01( "acadoWorkspace.H01", "real_t", getN()*getNX(), getNU() );
	ExportData H11( "acadoWorkspace.H11", "real_t", getN()*getNU(), getN()*getNU() );
 

    for( run1 = 0; run1 < getN()-1; run1++ )
	{
		condense2.addFunctionCall( "multiplyQ1", QQ, C.getAddress(run1*getNX(),0),  QC.getAddress(run1*getNX(),0)  );
		condense2.addFunctionCall( "multiplyQ2", QQ, E.getAddress(run1*getNX(),0),  QE.getAddress(run1*getNX(),0)  );
		condense2.addFunctionCall( "multiplyQ3", QQ, Dx.getAddress(run1*getNX(),0), QDx.getAddress(run1*getNX(),0) );
		condense2.addFunctionCall( "multiplyQ4", RR, Du.getAddress(run1*getNU(),0), RDu.getAddress(run1*getNU(),0) );
    }

	condense2.addFunctionCall( "multiplyQ5", QQF, C.getAddress((getN()-1)*getNX(),0),  QC.getAddress((getN()-1)*getNX(),0)  );
	condense2.addFunctionCall( "multiplyQ6", QQF, E.getAddress((getN()-1)*getNX(),0),  QE.getAddress((getN()-1)*getNX(),0)  );
	condense2.addFunctionCall( "multiplyQ7", QQF, Dx.getAddress((getN()-1)*getNX(),0), QDx.getAddress((getN()-1)*getNX(),0) );
	condense2.addFunctionCall( "multiplyQ4", RR,  Du.getAddress((getN()-1)*getNU(),0), RDu.getAddress((getN()-1)*getNU(),0) );

	condense2.addFunctionCall( "multiply8",  QE, QDx, g1 );
	condense2.addStatement( g1 += RDu );

	// HESSIAN CONDENSING:
	condense2.addFunctionCall( "multiply10", QC, QE, H01 );
	condense2.addFunctionCall( "multiply11", QE, QE, H11 );

	for( run1 = 0; run1 < getN(); run1++ )
		condense2.addStatement( H11.getSubMatrix( run1*getNU(),(run1+1)*getNU(), run1*getNU(),(run1+1)*getNU() ) += RR );

    condense2.exportCode( file,"real_t" );
}


void CondensingExport::exportEvaluation( FILE *file ) const
{
	ExportData state( "acadoWorkspace.state", "real_t", 1,getNX()*(getNX()+getNU()+1) + getNU() );
	ExportData x    ( "acadoVariables.x",     "real_t", (getN()+1), getNX() );
	ExportData u    ( "acadoVariables.u",     "real_t", getN(), getNU() );

	ExportData E    ( "acadoWorkspace.E",     "real_t", getN()*getNX(), getN()*getNU() );
	ExportData lbA  ( "acadoWorkspace.lbA",   "real_t", nxBounds, 1 );
	ExportData ubA  ( "acadoWorkspace.ubA",   "real_t", nxBounds, 1 );

	Matrix zeroXU = zeros( getNX(),getNU() );
	Matrix idX    = eye( getNX() );


	ExportFunction setupQP( "setupQP" );
    
	setupQP.addStatement( state.getCols( 0,getNX() ) == x.getRow(0) );

	uint run1, run2;
	for( run1 = 0; run1 < getN()-1; run1++ )
		for( run2 = 1+run1; run2 < getN(); run2++ )
			setupQP.addStatement( E.getSubMatrix( run1*getNX(),(run1+1)*getNX(), run2*getNU(),(run2+1)*getNU() ) == zeroXU );


    // WRITE STATE BOUNDS INTO THE FILE:
    // ---------------------------------
	if( getNumStateBounds( ) > 0 )
	{
		Vector xLowerBounds(nxBounds), xUpperBounds(nxBounds);
		for( run1 = 0; run1 < nxBounds; run1++ )
		{
			xLowerBounds(run1) = xBounds.getLowerBound( xBoundsIdx[run1]/getNX(),xBoundsIdx[run1]%getNX() );
			xUpperBounds(run1) = xBounds.getUpperBound( xBoundsIdx[run1]/getNX(),xBoundsIdx[run1]%getNX() );
		}
		
		setupQP.addStatement( lbA == xLowerBounds );
		setupQP.addStatement( ubA == xUpperBounds );
	}
	setupQP.addLinebreak( );
	

	ExportIndex run( "run1" );
	ExportForLoop loop( run, 0,getN() );
	
	uint uIdx = getNX()+getNX()*getNX();
	loop.addStatement( state.getCols( getNX(),uIdx ) == idX.makeVector().transpose() );
	loop.addStatement( state.getCols( uIdx,uIdx+getNX()*getNU() ) == zeroXU.makeVector().transpose() );

	uIdx = getNX()*(getNX()+getNU()+1);
	loop.addStatement( state.getCols( uIdx,uIdx+getNU() ) == u.getRow( run ) );
	loop.addLinebreak( );
	
	loop.addFunctionCall( "integrate", state );
	loop.addStatement( x.getRow( run+1 ) == state.getCols( 0,getNX() ) );
	loop.addLinebreak( );
	
	loop.addFunctionCall( "condense1", run.makeArgument(),state );

    setupQP.addStatement( loop );
	setupQP.addLinebreak( );
	
	setupQP.addFunctionCall( "condense2" );

	setupQP.exportCode( file );
}


CLOSE_NAMESPACE_ACADO

// end of file.
