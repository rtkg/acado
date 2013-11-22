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
 *    \file src/code_generation/mpc_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/mpc_export.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

MPCexport::MPCexport() : UserInteraction()
{ 
	setupOptions( ); 
	NX = 0; 
	NU = 0; 
	N = 0;
}


MPCexport::MPCexport( const OCP& _ocp ) : UserInteraction()
{ 
	setupOptions( );
	NX = 0; 
	NU = 0; 
	N = 0;

	ASSERT( setOCP( _ocp ) == SUCCESSFUL_RETURN );
}


MPCexport::MPCexport( const MPCexport& arg ) : UserInteraction( arg )
{
    NX       = arg.NX      ;
    NU       = arg.NU      ;
    N        = arg.N       ;
    gnExport = arg.gnExport;
}


MPCexport::~MPCexport( )
{
}


MPCexport& MPCexport::operator=( const MPCexport& arg )
{
    if( this != &arg ){

        UserInteraction::operator=( arg );

        NX       = arg.NX      ;
        NU       = arg.NU      ;
        N        = arg.N       ;
        gnExport = arg.gnExport;
    }
    return *this;
}


returnValue MPCexport::setupOptions( )
{
	addOption( HESSIAN_APPROXIMATION, GAUSS_NEWTON    );
	addOption( DISCRETIZATION_TYPE,   SINGLE_SHOOTING );
	addOption( INTEGRATOR_TYPE,       INT_RK4         );
	addOption( NUM_INTEGRATOR_STEPS,  30              );
	addOption( QP_SOLVER,             QP_QPOASES      );
	addOption( HOTSTART_QP,           BT_FALSE        );
	addOption( GENERATE_TEST_FILE,    BT_TRUE         );
	addOption( GENERATE_MAKE_FILE,    BT_TRUE         );
	addOption( USE_SINGLE_PRECISION,  BT_FALSE        );

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::setOCP( const OCP& _ocp )
{
	ocp = _ocp;
	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::exportCode( const char* const dirName )
{
	if ( setup( ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	int qpSolver;
	get( QP_SOLVER,qpSolver );

	int useSinglePrecision;
	get( USE_SINGLE_PRECISION,useSinglePrecision );

    String s( dirName ), fileName( dirName );
    fileName += "/acado.h";

    FILE *file = acadoFOpen( fileName.getName(), "w" );
	if ( file == 0 )
		return ACADOERROR( RET_DOES_DIRECTORY_EXISTS );

    acadoPrintAutoGenerationNotice( file );

    acadoFPrintf (file,"#include <stdio.h>\n");
    acadoFPrintf (file,"#include <math.h>\n");
    acadoFPrintf (file,"#include <time.h>\n");
    acadoFPrintf (file,"#include <sys/stat.h>\n");
    acadoFPrintf (file,"#include <sys/time.h>\n");
	acadoFPrintf (file,"\n");
	
	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			acadoFPrintf (file,"#define USE_CVXGEN\n");
			acadoFPrintf (file,"#include \"cvxgen/solver.h\"\n\n\n");
			
			if ( (BooleanType)useSinglePrecision == BT_TRUE )
				acadoFPrintf (file,"typedef float real_t;\n\n\n");
			else
				acadoFPrintf (file,"typedef double real_t;\n\n\n");
			break;
		
		case QP_QPOASES:
			acadoFPrintf (file,"#ifndef __MATLAB__\n");
			acadoFPrintf (file,"extern \"C\"\n");
			acadoFPrintf (file,"{\n");
			acadoFPrintf (file,"#endif");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"#include \"qpoases/solver.hpp\"\n\n\n");
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}


	// consistency checks
	int hessianApproximation;
	get( HESSIAN_APPROXIMATION,hessianApproximation );
	if ( (HessianApproximationMode)hessianApproximation != GAUSS_NEWTON )
		return ACADOERROR( RET_INVALID_OPTION );

	int discretizationType;
	get( DISCRETIZATION_TYPE,discretizationType );
	if ( (StateDiscretizationType)discretizationType != SINGLE_SHOOTING )
		return ACADOERROR( RET_INVALID_OPTION );


    acadoFPrintf( file, "/* GLOBAL VARIABLES: */\n");
    acadoFPrintf( file, "/* -------------------------------- */\n");
    acadoFPrintf( file, "   typedef struct ACADOvariables_ {\n\n");

    acadoFPrintf( file, "       real_t x[%d];\n", NX*(N+1) );
    acadoFPrintf( file, "       real_t u[%d];\n", NU*N );

    acadoFPrintf( file, "       real_t xRef[%d];\n", NX*N );
    acadoFPrintf( file, "       real_t uRef[%d];\n\n", NU*N );
    acadoFPrintf( file, "   } ACADOvariables;\n\n\n");


    acadoFPrintf (file, "/* GLOBAL WORKSPACE:                */\n");
    acadoFPrintf (file, "/* -------------------------------- */\n");
    acadoFPrintf (file, "   typedef struct ACADOworkspace_ {\n");
    acadoFPrintf(file,"\n");

                gnExport.exportHeader( file );

    acadoFPrintf(file,"\n   } ACADOworkspace;\n\n\n");

    acadoFPrintf (file,"/* GLOBAL FORWARD DECLARATIONS: */\n");
    acadoFPrintf (file,"/* ------------------------------------- */\n");

                gnExport.exportForwardDeclarations( file );

    acadoFPrintf (file,"/* ------------------------------------- */\n\n\n");


    acadoFPrintf (file,"/* EXTERN DECLARATIONS: */\n");
    acadoFPrintf (file,"/* ------------------------------------- */\n");
    acadoFPrintf (file,"   extern ACADOworkspace acadoWorkspace;\n");
    acadoFPrintf (file,"   extern ACADOvariables acadoVariables;\n\n");
    acadoFPrintf (file,"/* ------------------------------------- */\n");

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			break;
		
		case QP_QPOASES:
			acadoFPrintf (file,"#ifndef __MATLAB__");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"} // extern \"C\"\n");
			acadoFPrintf (file,"#endif" );
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}

	acadoFPrintf (file,"\n");
    acadoFPrintf (file,"/* END OF FILE. */\n\n");


    acadoFClose( file );

    gnExport.exportCode( s );


	// export template for main file and  ...
	int generateTestFile;
	get( GENERATE_TEST_FILE,generateTestFile );
	if ( (BooleanType)generateTestFile == BT_TRUE )
		if ( exportTemplateMain( s ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	// a basic Makefile, if desired
	int generateMakeFile;
	get( GENERATE_MAKE_FILE,generateMakeFile );
	if ( (BooleanType)generateMakeFile == BT_TRUE )
		if ( exportMakefile( s ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	if ( exportQPsolverInterface( s ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	if ( exportAuxiliaryFunctions( s ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	ACADOINFO( RET_CODE_EXPORT_SUCCESSFUL );

    return SUCCESSFUL_RETURN;
}



returnValue MPCexport::printDimensionsQP( )
{
	if ( setup( ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoPrintf( "\n***********************  ACADO CODE GENERATION  ***********************\n\n" );
	acadoPrintf( "The condensed QP comprises:\n" );
	acadoPrintf( "%d variables and\n", N*NU );
	acadoPrintf( "%d constraints.\n\n", gnExport.getNumStateBounds() );

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue MPCexport::setup( )
{
	returnValue returnvalue = checkConsistency( );
	if ( returnvalue != SUCCESSFUL_RETURN )
		return ACADOERROR( returnvalue );

     Grid grid;
     ocp.getGrid( grid );

     DifferentialEquation f;
     Expression rhs;

     ocp.getDifferentialEquation( f );
     f.getExpression( rhs );

     NX = rhs.getDim();
     NU = f.getNU();


	 // setup fixed integrator grid
	 double T = grid.getLastTime() - grid.getFirstTime();
     N = grid.getNumIntervals();

     int numSteps;
     get( NUM_INTEGRATOR_STEPS, numSteps );

	 if ( numSteps <= 0 )
		return ACADOERROR( RET_INVALID_OPTION );

// 	 printf("numSteps = %d \n", numSteps );
// 	 printf("N = %d \n", N );
// 	 printf("(int) ceil(numSteps/N - 10.0*EPS) = %d \n", (int) ceil((double)numSteps/((double) N) - 10.0*EPS) );
	 
     Grid integratorGrid( 0.0, ((double) T)/((double) N), (int) ceil((double)numSteps/((double) N) - 10.0*EPS) + 1 );


     Control               dummy1;
     DifferentialState     dummy2;
     dummy1.clearStaticCounters();
     dummy2.clearStaticCounters();

     DifferentialState x(NX);
     Control           u(NU);


	int integratorType;
	get( INTEGRATOR_TYPE, integratorType );

	if ( (IntegratorType)integratorType != INT_RK4 )
		return ACADOERROR( RET_INVALID_OPTION );
	
	IntegratorExport integratorExport;

	integratorExport.setODE  ( rhs, x, u );
	integratorExport.setGrid ( integratorGrid );


	// extract control/state bounds
	VariablesGrid ugrid( NU, grid );
	VariablesGrid xgrid( NX, grid );

	OCPiterate tmp;
	tmp.init( &xgrid,0,0,&ugrid,0 );

	Constraint constraint;
	ocp.getConstraint( constraint );
	constraint.getBounds( tmp );


	// setup condensing
	CondensingExport condenser;

	condenser.setIntegratorExport( integratorExport, N );

	Matrix Q,R,S;
	ocp.getQRS( Q, R, S );

	if( S.getDim() == 0 )
		 S = eye(NX);

	condenser.setWeightingMatrices( Q, S, R );
	condenser.setStateBounds( *(tmp.x) );


	// setup Gauss-Newton algorithm 
	gnExport.setCondensingExport( condenser );
	gnExport.setControlBounds( *(tmp.u) );

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::checkConsistency( ) const
{
	// consistency checks:
	// only standard LSQ objective supported!
	if ( ocp.hasObjective( ) == BT_TRUE )
		return ACADOERROR( RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT );
	
	// only time-continuous ODEs without parameter and disturbances supported!
	DifferentialEquation f;
	ocp.getDifferentialEquation( f );

	if ( f.isODE( ) == BT_FALSE )
		return ACADOERROR( RET_ONLY_ODE_FOR_CODE_EXPORT );

	if ( f.isDiscretized( ) == BT_TRUE )
		return ACADOERROR( RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT );
	
	if ( ( f.getNXA( ) > 0 ) || ( f.getNDX( ) > 0 ) || ( f.getNUI( ) > 0 ) || 
		 ( f.getNP( ) > 0 ) || ( f.getNPI( ) > 0 ) || ( f.getNW( ) > 0 ) )
		return ACADOERROR( RET_ONLY_STATES_AND_CONTROLS_FOR_CODE_EXPORT );

	// only equidistant evaluation grids supported!
	Grid grid;
	ocp.getGrid( grid );
	
	if ( grid.isEquidistant( ) == BT_FALSE )
		return ACADOERROR( RET_ONLY_EQUIDISTANT_GRID_FOR_CODE_EXPORT );

	// only state or control BOUNDS supported!
	Constraint constraint;
	ocp.getConstraint( constraint );
	
	if ( constraint.isBoxConstraint( ) == BT_FALSE )
		return ACADOERROR( RET_ONLY_BOUNDS_FOR_CODE_EXPORT );
	
	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::exportTemplateMain( const String &dirName ) const
{
    String fileName( dirName );
    fileName += "/test.c";

	int qpSolver;
	get( QP_SOLVER,qpSolver );

    FILE *file = fopen( fileName.getName(), "w" );

    acadoPrintAutoGenerationNotice( file );

    acadoFPrintf (file,"#include \"acado.h\"\n");
	acadoFPrintf (file,"#include \"auxiliary_functions.c\"\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"// SOME CONVENIENT DEFINTIONS:\n");
    acadoFPrintf (file,"// ---------------------------------------------------------------\n");
    acadoFPrintf (file,"   #define NX        %3.d      /* number of differential states  */\n", NX );
    acadoFPrintf (file,"   #define NU        %3.d      /* number of control inputs       */\n", NU );
    acadoFPrintf (file,"   #define N         %3.d      /* number of control intervals    */\n", N );
    acadoFPrintf (file,"   #define NUM_STEPS   5      /* number of real time iterations */\n");
    acadoFPrintf (file,"   #define VERBOSE     1      /* show iterations: 1, silent: 0  */\n");
    acadoFPrintf (file,"// ---------------------------------------------------------------\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"// GLOBAL VARIABLES FOR THE ACADO REAL-TIME ALGORITHM:\n");
    acadoFPrintf (file,"// ---------------------------------------------------\n");
    acadoFPrintf (file,"   ACADOvariables acadoVariables;\n");
	acadoFPrintf (file,"   ACADOworkspace acadoWorkspace;\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"// GLOBAL VARIABLES FOR THE QP SOLVER:\n");
    acadoFPrintf (file,"// -----------------------------------\n");
    acadoFPrintf (file,"   Vars         vars;\n");
    acadoFPrintf (file,"   Params       params;\n");

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			acadoFPrintf (file,"   Workspace    work;\n");
			acadoFPrintf (file,"   Settings     settings;\n");
			break;
		
		default:
			break;
	}

    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
	acadoFPrintf (file,"// A TEMPLATE FOR TESTING THE REAL-TIME IMPLEMENTATION:\n");
    acadoFPrintf (file,"// ----------------------------------------------------\n");
    acadoFPrintf (file,"int main(){\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"   // INTRODUCE AUXILIARY VAIRABLES:\n");
    acadoFPrintf (file,"   // ------------------------------\n");
    acadoFPrintf (file,"      int    i, iter        ;\n");
    acadoFPrintf (file,"      real_t measurement[NX];\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			acadoFPrintf (file,"   // CUSTOMIZE THE SOLVER SETTINGS:\n");
			acadoFPrintf (file,"   // ------------------------------\n");
			acadoFPrintf (file,"      set_defaults();\n");
			acadoFPrintf (file,"      if( !VERBOSE ) settings.verbose = 0;\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"\n");
			break;
		
		default:
			break;
	}

    acadoFPrintf (file,"   // INITIALIZE THE STATES AND CONTROLS:\n");
    acadoFPrintf (file,"   // ----------------------------------------\n");
    acadoFPrintf (file,"      for( i = 0; i < NX*N; i++ )  acadoVariables.x[i] = 0.0;\n");
    acadoFPrintf (file,"      for( i = 0; i < NU*N; i++ )  acadoVariables.u[i] = 0.0;\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"   // INITIALIZE THE STATES AND CONTROL REFERENCE:\n");
    acadoFPrintf (file,"   // --------------------------------------------\n");
    acadoFPrintf (file,"      for( i = 0; i < NX*N; i++ )  acadoVariables.xRef[i] =  0.0;\n");
    acadoFPrintf (file,"      for( i = 0; i < NU*N; i++ )  acadoVariables.uRef[i] =  0.0;\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"   // SETUP THE FIRST STATE MEASUREMENT:\n");
    acadoFPrintf (file,"   // ------------------------------------------------\n");
    acadoFPrintf (file,"      for( i = 0; i < NX; i++ )  measurement[i] = 0.0;\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"      acadoVariables.x[0] = 1.0;\n");
    acadoFPrintf (file,"      measurement     [0] = 1.0;\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"      if( VERBOSE ) printHeader();\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
	acadoFPrintf (file,"    // PREPARE FIRST STEP:\n");
    acadoFPrintf (file,"    // -------------------\n");
    acadoFPrintf (file,"       preparationStep();\n"); 
	acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"   // GET THE TIME BEFORE START THE LOOP:\n");
    acadoFPrintf (file,"   // ----------------------------------------------\n");
    acadoFPrintf (file,"      real_t t1 = getTime();\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"   // THE REAL-TIME ITERATION LOOP:\n");
    acadoFPrintf (file,"   // ----------------------------------------------\n");
    acadoFPrintf (file,"      for( iter = 0; iter < NUM_STEPS; iter++ ){\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"        // TAKE A MEASUREMENT:\n");
    acadoFPrintf (file,"        // -----------------------------\n");
    acadoFPrintf (file,"           /// meausrement = ...\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"        // PERFORM THE FEEDBACK STEP:\n");
    acadoFPrintf (file,"        // -----------------------------\n");
    acadoFPrintf (file,"           feedbackStep( measurement );\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"        // APPLY THE NEW CONTROL IMMEDIATELY TO THE PROCESS:\n");
    acadoFPrintf (file,"        // -------------------------------------------------\n");
    acadoFPrintf (file,"           /// send first piece of acadoVariables.u to process;\n");
    acadoFPrintf (file,"           if( VERBOSE ) printf(\"=================================================================\\n\\n\" );\n");
    acadoFPrintf (file,"           if( VERBOSE ) printf(\"      Real-Time Iteration %%d:  KKT Tolerance = %%.3e\\n\", iter, getKKT() );\n");
    acadoFPrintf (file,"           if( VERBOSE ) printf(\"\\n=================================================================\\n\" );\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"        // OPTIONAL: SHIFT THE INITIALIZATION:\n");
    acadoFPrintf (file,"        // -----------------------------------\n");
    acadoFPrintf (file,"           /// shiftControls( acadoVariables.uRef );\n");
    acadoFPrintf (file,"           /// shiftStates  ( acadoVariables.xRef );\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"        // PREPARE NEXT STEP:\n");
    acadoFPrintf (file,"        // ------------------\n");
    acadoFPrintf (file,"           preparationStep();\n"); 
    acadoFPrintf (file,"      }\n");
    acadoFPrintf (file,"      if( VERBOSE ) printf(\"\\n\\n              END OF THE REAL-TIME LOOP. \\n\\n\\n\");\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"   // GET THE TIME AT THE END OF THE LOOP:\n");
    acadoFPrintf (file,"   // ----------------------------------------------\n");
    acadoFPrintf (file,"      real_t t2 = getTime();\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"\n");
    acadoFPrintf (file,"   // PRINT DURATION AND RESULTS:\n");
    acadoFPrintf (file,"   // --------------------------------------------------------------------------------------------------\n");
    acadoFPrintf (file,"      if( !VERBOSE )\n");
    acadoFPrintf (file,"      printf(\"\\n\\n AVERAGE DURATION OF ONE REAL-TIME ITERATION:   %%.3g μs\\n\\n\", 1e6*(t2-t1)/NUM_STEPS );\n");
    acadoFPrintf (file,"\n");
	acadoFPrintf (file,"      printStates();\n");
	acadoFPrintf (file,"      printControls();\n");
	acadoFPrintf (file,"\n");
    acadoFPrintf (file,"    return 0;\n");
    acadoFPrintf (file,"}\n\n\n");

    fclose( file );
	
	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::exportMakefile( const String &dirName ) const
{
    String fileName( dirName );
    fileName += "/Makefile";

    int qpSolver;
	get( QP_SOLVER,qpSolver );

    FILE *file = fopen( fileName.getName() ,"w");

    acadoFPrintf (file,"##\n");
    acadoFPrintf (file,"##    This file was auto-generated by ACADO Toolkit.\n");
    acadoFPrintf (file,"##\n");
    acadoFPrintf (file,"##    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\n");
    acadoFPrintf (file,"##    Copyright (C) 2008-2010 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.\n");
    acadoFPrintf (file,"##    Developed within the Optimization in Engineering Center (OPTEC) under\n");
    acadoFPrintf (file,"##    supervision of Moritz Diehl. All rights reserved.\n");
    acadoFPrintf (file,"##\n");
    acadoFPrintf (file,"##\n");
    acadoFPrintf (file,"\n");
	acadoFPrintf (file,"\n");
	
	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			acadoFPrintf (file,"LDLIBS = -lm\n");
			acadoFPrintf (file,"CFLAGS = -Os\n");
			acadoFPrintf (file,"CC     = gcc\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"OBJECTS = ./cvxgen/solver.o       \\\n");
			acadoFPrintf (file,"\t./cvxgen/matrix_support.o      \\\n");
			acadoFPrintf (file,"\t./cvxgen/ldl.o                 \\\n");
			acadoFPrintf (file,"\t./cvxgen/util.o                \\\n");
			acadoFPrintf (file,"\tintegrator.o                   \\\n");
			acadoFPrintf (file,"\tcondensing.o                   \\\n");
			acadoFPrintf (file,"\tgauss_newton_method.o          \n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,".PHONY: all\n");
			acadoFPrintf (file,"all: test libacado_exported_rti.a\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"test: ${OBJECTS} test.o\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"./cvxgen/solver.o     : ./cvxgen/solver.h\n");
			acadoFPrintf (file,"integrator.o          : acado.h\n");
			acadoFPrintf (file,"condensing.o          : acado.h\n");
			acadoFPrintf (file,"gauss_newton_method.o : acado.h   ./cvxgen/solver.h\n");
			acadoFPrintf (file,"test.o                : acado.h   ./cvxgen/solver.h\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"libacado_exported_rti.a: ${OBJECTS}\n");
			acadoFPrintf (file,"\tar r $@ $\?\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,".PHONY : clean\n");
			acadoFPrintf (file,"clean :\n");
			acadoFPrintf (file,"\t-rm -f *.o *.a ./cvxgen/*.o test\n");
			acadoFPrintf (file,"\n");

			break;
		
		case QP_QPOASES:
			acadoFPrintf (file,"LDLIBS = -lm \n");
			acadoFPrintf (file,"CXXFLAGS = -O3 -I. -I./qpoases/INCLUDE -I./qpoases/SRC\n");
			acadoFPrintf (file,"CFLAGS = -Os\n");
			acadoFPrintf (file,"CC     = g++\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"OBJECTS = \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/QProblem.o        \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/QProblemB.o       \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/Bounds.o          \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/Constraints.o     \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/SubjectTo.o       \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/Indexlist.o       \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/CyclingManager.o  \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/Utils.o           \\\n");
			acadoFPrintf (file,"\t./qpoases/SRC/MessageHandling.o \\\n");
			acadoFPrintf (file,"\t./qpoases/solver.o              \\\n");
			acadoFPrintf (file,"\tintegrator.o                    \\\n");
			acadoFPrintf (file,"\tcondensing.o                    \\\n");
			acadoFPrintf (file,"\tgauss_newton_method.o \n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,".PHONY: all\n");
			acadoFPrintf (file,"all: test libacado_exported_rti.a\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"test: ${OBJECTS} test.o\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"./qpoases/solver.o    : ./qpoases/solver.hpp\n");
			acadoFPrintf (file,"integrator.o          : acado.h\n");
			acadoFPrintf (file,"condensing.o          : acado.h\n");
			acadoFPrintf (file,"gauss_newton_method.o : acado.h   ./qpoases/solver.hpp\n");
			acadoFPrintf (file,"test.o                : acado.h   ./qpoases/solver.hpp\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"libacado_exported_rti.a: ${OBJECTS}\n");
			acadoFPrintf (file,"\tar r $@ $\?\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,"${OBJECTS} : ./qpoases/solver.hpp\n");
			acadoFPrintf (file,"\n");
			acadoFPrintf (file,".PHONY : clean\n");
			acadoFPrintf (file,"clean :\n");
			acadoFPrintf (file,"\t-rm -f *.o *.a ./qpoases/SRC/*.o ./qpoases/SRC/*.a test\n");
			acadoFPrintf (file,"\n");
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}

    fclose( file );

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::exportQPsolverInterface( const String &dirName ) const
{
	int qpSolver;
	get( QP_SOLVER,qpSolver );

	int hotstartQP;
	get( HOTSTART_QP,hotstartQP );
	
	int useSinglePrecision;
	get( USE_SINGLE_PRECISION,useSinglePrecision );

	String fileNameHeader( dirName );
	String fileNameSource( dirName );
	
	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			if ( (BooleanType)hotstartQP == BT_TRUE )
				return ACADOERROR( RET_UNABLE_TO_HOTSTART_QP );
			else
				return SUCCESSFUL_RETURN;

			if ( (BooleanType)useSinglePrecision == BT_TRUE )
				return ACADOERROR( RET_INVALID_OPTION );
			else
				return SUCCESSFUL_RETURN;

		case QP_QPOASES:
			fileNameHeader += "/qpoases/solver.hpp";
			fileNameSource += "/qpoases/solver.cpp";
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}


	// generate header file
    FILE *file = fopen( fileNameHeader.getName() ,"w");

    if( file == 0 )
        return ACADOERROR( RET_QPOASES_EMBEDDED_NOT_FOUND );

    acadoPrintAutoGenerationNotice( file );
    acadoFPrintf (file,"#ifndef SOLVER_QPOASES_HPP\n");
	acadoFPrintf (file,"#define SOLVER_QPOASES_HPP\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"#include <stdio.h>\n");
	acadoFPrintf (file,"#include <math.h>\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"#define QPOASES_NVMAX   %d\n",N*NU );
	acadoFPrintf (file,"#define QPOASES_NCMAX   %d\n",gnExport.getNumStateBounds() );
	acadoFPrintf (file,"#define QPOASES_NWSRMAX %d\n",3*(N*NU+gnExport.getNumStateBounds()) );

	if ( (BooleanType)useSinglePrecision == BT_TRUE )
	{
		acadoFPrintf (file,"#define QPOASES_EPS     %e\n",1.193e-07 );
		acadoFPrintf (file,"\n");
		acadoFPrintf (file,"typedef float real_t;\n");
	}
	else
	{
		acadoFPrintf (file,"#define QPOASES_EPS     %e\n",2.221e-16 );
		acadoFPrintf (file,"\n");
		acadoFPrintf (file,"typedef double real_t;\n");
	}

	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"typedef struct Params_t {\n");
	acadoFPrintf (file,"  real_t H[%d];\n",  (N*NU)*(N*NU) );
	acadoFPrintf (file,"  real_t g[%d];\n",   N*NU );
	acadoFPrintf (file,"  real_t lb[%d];\n",  N*NU );
	acadoFPrintf (file,"  real_t ub[%d];\n",  N*NU );
	if ( gnExport.getNumStateBounds() > 0 )
	{
		acadoFPrintf (file,"  real_t A[%d];\n", gnExport.getNumStateBounds()*(N*NU) );
		acadoFPrintf (file,"  real_t lbA[%d];\n", gnExport.getNumStateBounds() );
		acadoFPrintf (file,"  real_t ubA[%d];\n", gnExport.getNumStateBounds() );
	}
	acadoFPrintf (file,"} Params;\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"typedef struct Vars_t {\n");
	acadoFPrintf (file,"  real_t x[%d];\n", N*NU );
	acadoFPrintf (file,"  real_t y[%d];\n", N*NU + gnExport.getNumStateBounds() );
	acadoFPrintf (file,"} Vars;\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"extern Params params;\n");
	acadoFPrintf (file,"extern Vars vars;\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"long solve(void);\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"#endif\n");
	
    fclose( file );

	// generate source file
    file = fopen( fileNameSource.getName() ,"w");

	acadoPrintAutoGenerationNotice( file );
	acadoFPrintf (file,"extern \"C\"{\n" );
	acadoFPrintf (file,"#include \"solver.hpp\"\n");
	acadoFPrintf (file,"}\n");
	acadoFPrintf (file,"#include \"INCLUDE/QProblem.hpp\"\n");
    acadoFPrintf (file,"\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"long solve( void )\n");
	acadoFPrintf (file,"{\n");
	acadoFPrintf (file,"   int nWSR = QPOASES_NWSRMAX;\n" );
	acadoFPrintf (file,"\n");

	if ( gnExport.getNumStateBounds() > 0 )
	{
		acadoFPrintf (file,"   QProblem example( %d,%d );\n", N*NU, gnExport.getNumStateBounds() );
		if ( (BooleanType)hotstartQP == BT_TRUE )
			acadoFPrintf (file,"   example.init( params.H,params.g,params.A,params.lb,params.ub,params.lbA,params.ubA, nWSR,vars.y );\n");
		else
			acadoFPrintf (file,"   example.init( params.H,params.g,params.A,params.lb,params.ub,params.lbA,params.ubA, nWSR );\n");
	}
	else
	{
		acadoFPrintf (file,"   QProblemB example( %d );\n", N*NU );
		if ( (BooleanType)hotstartQP == BT_TRUE )
			acadoFPrintf (file,"   example.init( params.H,params.g,params.lb,params.ub, nWSR,vars.y );\n");
		else
			acadoFPrintf (file,"   example.init( params.H,params.g,params.lb,params.ub, nWSR );\n");
	}

	acadoFPrintf (file,"   example.getPrimalSolution( vars.x );\n");
	acadoFPrintf (file,"   example.getDualSolution( vars.y );\n");
	acadoFPrintf (file,"\n");
	acadoFPrintf (file,"   return 0;\n");
	acadoFPrintf (file,"}\n");

    fclose( file );

	return SUCCESSFUL_RETURN;
}


returnValue MPCexport::exportAuxiliaryFunctions( const String &dirName ) const
{
    String fileName( dirName );
    fileName += "/auxiliary_functions.c";

	int qpSolver;
	get( QP_SOLVER,qpSolver );

    FILE *file = fopen( fileName.getName(), "w" );

    acadoPrintAutoGenerationNotice( file );

	acadoFPrintf( file,"real_t* getAcadoVariablesX( ){\n" );
	acadoFPrintf( file,"    return acadoVariables.x;\n" );
	acadoFPrintf( file,"}\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"real_t* getAcadoVariablesU( ){\n" );
	acadoFPrintf( file,"    return acadoVariables.u;\n" );
	acadoFPrintf( file,"}\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"real_t* getAcadoVariablesXRef( ){\n" );
	acadoFPrintf( file,"    return acadoVariables.xRef;\n" );
	acadoFPrintf( file,"}\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"real_t* getAcadoVariablesURef( ){\n" );
	acadoFPrintf( file,"    return acadoVariables.uRef;\n" );
	acadoFPrintf( file,"}\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"void printStates(){\n" );
	acadoFPrintf( file,"    int i,j;\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"    printf( \"states = \\n\" );\n" );
	acadoFPrintf( file,"    for( j=0; j<%d; ++j ){\n",NX );
	acadoFPrintf( file,"        for( i=0; i<%d; ++i )\n",N+1 );
	acadoFPrintf( file,"            printf( \"%%e \\t\", acadoVariables.x[i*%d+j] );\n",NX );
	acadoFPrintf( file,"        printf( \"\\n\" );\n" );
	acadoFPrintf( file,"    }\n" );
	acadoFPrintf( file,"}\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"void printControls(){\n" );
	acadoFPrintf( file,"    int i,j;\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"    printf( \"controls = \\n\" );\n" );
	acadoFPrintf( file,"    for( j=0; j<%d; ++j ){\n",NU );
	acadoFPrintf( file,"        for( i=0; i<%d; ++i )\n",N );
	acadoFPrintf( file,"            printf( \"%%e \\t\", acadoVariables.u[i*%d+j] );\n",NU );
	acadoFPrintf( file,"        printf( \"\\n\" );\n" );
	acadoFPrintf( file,"    }\n" );
	acadoFPrintf( file,"}\n" );
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"\n" );
    acadoFPrintf( file, "double getTime( )\n");
    acadoFPrintf( file, "{\n");
    acadoFPrintf( file, "    double current_time = 0.0;\n");
    acadoFPrintf( file, "    struct timeval theclock;\n");
    acadoFPrintf( file, "    gettimeofday( &theclock,0 );\n");
    acadoFPrintf( file, "    current_time = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;\n");
    acadoFPrintf( file, "    return current_time;\n");
    acadoFPrintf( file, "}\n");
	acadoFPrintf( file,"\n" );
	acadoFPrintf( file,"\n" );
    acadoFPrintf( file,"void printHeader(){\n");
    acadoFPrintf( file,"    printf(\"\\nACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\\nCopyright (C) 2008-2011 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.\\nDeveloped within the Optimization in Engineering Center (OPTEC) under\\nsupervision of Moritz Diehl. All rights reserved.\\n\\nACADO Toolkit is distributed under the terms of the GNU Lesser\\nGeneral Public License 3 in the hope that it will be useful,\\nbut WITHOUT ANY WARRANTY; without even the implied warranty of\\nMERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\\nGNU Lesser General Public License for more details.\\n\\n\" );\n");
    acadoFPrintf( file,"}\n\n");

    fclose( file );

	return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO

// end of file.
