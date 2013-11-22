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
 *    \file src/integrator/integrator_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010
 */

#include <acado/code_generation/integrator_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorExport::IntegratorExport(){

    NX = 0;
    NU = 0;
}

IntegratorExport::IntegratorExport( const IntegratorExport& arg ){

    NX   = arg.NX  ;
    NU   = arg.NU  ;
    ODE  = arg.ODE ;
    grid = arg.grid;
}


IntegratorExport::~IntegratorExport( )
{
}


IntegratorExport& IntegratorExport::operator=( const IntegratorExport& arg ){

    if( this != &arg ){

        NX   = arg.NX  ;
        NU   = arg.NU  ;
        ODE  = arg.ODE ;
        grid = arg.grid;
    }
    return *this;
}


returnValue IntegratorExport::setODE( const Expression        &rhs,
                                      const DifferentialState &x  ,
                                      const Control           &u    )
{
	NX = rhs.getDim();
	NU = u.getDim();

	DifferentialState Gx(NX,NX), Gu(NX,NU);

	DifferentialEquation f;
	f <<  rhs;
	f <<  forwardDerivative( rhs, x ) * Gx;
	f <<  forwardDerivative( rhs, x ) * Gu + forwardDerivative( rhs, u );

	return ODE.init( f,"acado_rhs" );
}


returnValue IntegratorExport::setGrid( const Grid &grid_ )
{
    grid = grid_;

    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::exportHeader( FILE* file ) const
{
    ODE.exportDataDeclaration( file,"real_t" );
    generateHeader( file, NX*(NX+NU+1), NX*(NX+NU+1) + NU );

    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::exportForwardDeclarations( FILE* file ) const
{
    acadoFPrintf(file,"   void   integrate( real_t* );\n");
    ODE.exportForwardDeclaration( file,"real_t" );

    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::exportCode( const String& dirName ) const
{
    String fileName( dirName );
    fileName += "/integrator.c";

	FILE *file = acadoFOpen( fileName.getName(), "w" );
	if ( file == 0 )
		return ACADOERROR( RET_DOES_DIRECTORY_EXISTS );

    acadoPrintAutoGenerationNotice( file );
    acadoFPrintf( file, "#include \"acado.h\" \n\n\n" );
    ODE.exportDefinition( file,"real_t" );
    generateCode( file, NX*(NX+NU+1), NX*(NX+NU+1) + NU );

    fclose(file);

    return SUCCESSFUL_RETURN;
}


uint IntegratorExport::getNX() const
{
    return NX;
}


uint IntegratorExport::getNU() const
{
    return NU;
}



// PROTECTED:


void IntegratorExport::generateHeader( FILE *file, uint rhsDim, uint xDim ) const
{
    acadoFPrintf(file,"       real_t rk_ttt;\n" );

    acadoFPrintf(file,"       real_t rk_xxx[%d];\n", xDim   );
    acadoFPrintf(file,"       real_t rk_kkk[%d];\n", 4*rhsDim );
}


void IntegratorExport::generateCode( FILE *file, uint rhsDim, uint xDim ) const
{
	uint run1;
	
    const uint dim = 4;

	Matrix AA(dim,dim), bb4(dim,1), cc(dim,1);
	
    AA(0,0) = 0.0;
    AA(0,1) = 0.0;
    AA(0,2) = 0.0;
    AA(0,3) = 0.0;

    AA(1,0) = 1.0/2.0;
    AA(1,1) = 0.0;
    AA(1,2) = 0.0;
    AA(1,3) = 0.0;

    AA(2,0) = 0.0;
    AA(2,1) = 1.0/2.0;
    AA(2,2) = 0.0;
    AA(2,3) = 0.0;

    AA(3,0) = 0.0;
    AA(3,1) = 0.0;
    AA(3,2) = 1.0;
    AA(3,3) = 0.0;

	bb4(0,0) = 1.0/6.0;
    bb4(1,0) = 1.0/3.0;
    bb4(2,0) = 1.0/3.0;
    bb4(3,0) = 1.0/6.0;

    cc(0,0) = 0.0;
    cc(1,0) = 1.0/2.0;
    cc(2,0) = 1.0/2.0;
    cc(3,0) = 1.0;

	//grid.print();
	   
    double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();    

	ExportData Ah ( "A*h",  "real_t", AA*=h );
	ExportData b4h( "b4*h", "real_t", bb4*=h );
	
	ExportIndex run( "run1" );

	ExportData  rk_ttt( "acadoWorkspace.rk_ttt", "real_t", 1,1, BT_TRUE );
	ExportData  rk_xxx( "acadoWorkspace.rk_xxx", "real_t", 1,xDim );
	ExportData  rk_kkk( "acadoWorkspace.rk_kkk", "real_t", dim,rhsDim );
	ExportData  rk_eta( "rk_eta",                "real_t", 1,xDim );

	
	ExportFunction integrate( "integrate", rk_eta );
	
	integrate.addStatement( rk_ttt == Matrix(grid.getFirstTime()) );
	integrate.addStatement( rk_xxx.getCols( rhsDim,xDim ) == rk_eta.getCols( rhsDim,xDim ) );
	integrate.addLinebreak( );

    // INTEGRATOR LOOP:
	ExportForLoop loop( run, 0,grid.getNumIntervals() );

	for( run1 = 0; run1 < dim; run1++ )
	{
		loop.addStatement( rk_xxx.getCols( 0,rhsDim ) == rk_eta.getCols( 0,rhsDim ) + Ah.getRow(run1)*rk_kkk );
		loop.addFunctionCall( "acado_rhs", rk_xxx,rk_kkk.getAddress(run1,0) ); 
	}
	loop.addStatement( rk_eta.getCols( 0,rhsDim ) += b4h^rk_kkk );
	loop.addStatement( rk_ttt += Matrix(h) );
    // END OF INTEGRATOR LOOP.

	integrate.addStatement( loop );
	integrate.exportCode( file );
}



CLOSE_NAMESPACE_ACADO

// end of file.
