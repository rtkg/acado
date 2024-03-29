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
*    \file src/simulation_environment/simulation_environment.cpp
*    \author Hans Joachim Ferreau, Boris Houska
*    \date 24.08.2009
*/



#include <acado/simulation_environment/simulation_environment.hpp>


BEGIN_NAMESPACE_ACADO



SimulationEnvironment::SimulationEnvironment( ) : SimulationBlock( BN_SIMULATION_ENVIRONMENT )
{
	setupOptions( );
	setupLogging( );

	startTime = 0.0;
	endTime   = 0.0;

	process    = 0;
	controller = 0;

	nSteps = 0;
	
	setStatus( BS_NOT_INITIALIZED );
}


SimulationEnvironment::SimulationEnvironment(	double _startTime,
												double _endTime,
												Process& _process,
												Controller& _controller
												) : SimulationBlock( BN_SIMULATION_ENVIRONMENT )
{
	setupOptions( );
	setupLogging( );
	
	startTime = _startTime;
	endTime   = _endTime;

	if ( _process.isDefined( ) == BT_TRUE )
	{
		process = &_process;
		setStatus( BS_NOT_INITIALIZED );
	}
	else
		process = 0;

	if ( _controller.isDefined( ) == BT_TRUE )
		controller = &_controller;
	else
		controller = 0;

	nSteps = 0;
}


SimulationEnvironment::SimulationEnvironment( const SimulationEnvironment &rhs ) : SimulationBlock( rhs )
{
	startTime = rhs.startTime;
	endTime   = rhs.endTime;

	if ( rhs.process != 0 )
		process = rhs.process;
	else
		process = 0;

	if ( rhs.controller != 0 )
		controller = rhs.controller;
	else
		controller = 0;

	simulationClock = rhs.simulationClock;

	processOutput     = rhs.processOutput;
	feedbackControl   = rhs.feedbackControl;
	feedbackParameter = rhs.feedbackParameter;

	nSteps = rhs.nSteps;
}


SimulationEnvironment::~SimulationEnvironment( )
{
}


SimulationEnvironment& SimulationEnvironment::operator=( const SimulationEnvironment &rhs )
{
	if( this != &rhs )
	{
		SimulationBlock::operator=( rhs );
		
		startTime = rhs.startTime;
		endTime   = rhs.endTime;
	
		if ( rhs.process != 0 )
			process = rhs.process;
		else
			process = 0;
	
		if ( rhs.controller != 0 )
			controller = rhs.controller;
		else
			controller = 0;

		simulationClock = rhs.simulationClock;
	
		processOutput     = rhs.processOutput;
		feedbackControl   = rhs.feedbackControl;
		feedbackParameter = rhs.feedbackParameter;

		nSteps = rhs.nSteps;
    }

    return *this;
}



returnValue SimulationEnvironment::setProcess(	Process& _process
												)
{
	if ( _process.isDefined( ) == BT_TRUE )
	{
		if ( process == 0 )
			process = &_process;
		else
			*process = _process;
	}

	if ( _process.isDefined( ) == BT_TRUE )
		setStatus( BS_NOT_INITIALIZED );

	return SUCCESSFUL_RETURN;
}


returnValue SimulationEnvironment::setController(	Controller& _controller
													)
{
	if ( _controller.isDefined( ) == BT_TRUE )
	{
		if ( controller == 0 )
			controller = &_controller;
		else
			*controller = _controller;
	}

	if ( getStatus( ) > BS_NOT_INITIALIZED )
		setStatus( BS_NOT_INITIALIZED );

	return SUCCESSFUL_RETURN;
}



returnValue SimulationEnvironment::initializeAlgebraicStates( const VariablesGrid& _xa_init )
{
	if ( controller != 0 )
		controller->initializeAlgebraicStates( _xa_init );

	if ( process != 0 )
		process->initializeAlgebraicStates( _xa_init.getVector(0) );

	return SUCCESSFUL_RETURN;
}


returnValue SimulationEnvironment::initializeAlgebraicStates( const char* fileName )
{
	VariablesGrid tmp = fopen( fileName,"r" );
	
	if ( tmp.isEmpty( ) == BT_TRUE )
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );
	
	return initializeAlgebraicStates( tmp );
}



returnValue SimulationEnvironment::init(	const Vector &x0_,
											const Vector &p_
											)
{
	// 1) initialise all sub-blocks and evaluate process at start time
	Vector uStart, pStart;
	VariablesGrid yStart;
	
	if ( controller != 0 )
	{
		if ( controller->init( startTime,x0_,p_ ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_INIT_FAILED );

		if ( controller->getU( uStart ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_INIT_FAILED );

		if ( controller->getP( pStart ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_INIT_FAILED );
	}
	else
		return ACADOERROR( RET_NO_CONTROLLER_SPECIFIED );
	

	if ( process != 0 )
	{
		if ( process->init( startTime,x0_,uStart,pStart ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_INIT_FAILED );

		if ( process->getY( yStart ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_INIT_FAILED );
	}
	else
		return ACADOERROR( RET_NO_PROCESS_SPECIFIED );

	simulationClock.init( startTime );


	// 2) consistency checks
	if ( ( process != 0 ) && ( controller != 0 ) )
	{
		if ( process->getNY( ) != controller->getNY( ) )
			return ACADOERROR( RET_BLOCK_DIMENSION_MISMATCH );

// 		printf( "%d  %d\n",process->getNU( ),controller->getNU( ) );
			
		if ( process->getNU( ) != controller->getNU( ) )
			return ACADOERROR( RET_BLOCK_DIMENSION_MISMATCH );

		if ( process->getNP( ) > controller->getNP( ) )
			return ACADOERROR( RET_BLOCK_DIMENSION_MISMATCH );
	}


	// initialise history...
	if ( getNU( ) > 0 )
		feedbackControl.  add( startTime-10.0*EPS,startTime,uStart );

	if ( getNP( ) > 0 )
		feedbackParameter.add( startTime-10.0*EPS,startTime,pStart );
	if ( getNY( ) > 0 )
		processOutput.    add( startTime-10.0*EPS,startTime,yStart.getVector(0) );

	// ... and update block status
	setStatus( BS_READY );
	return SUCCESSFUL_RETURN;
}



returnValue SimulationEnvironment::step( )
{
	/* Consistency check. */
	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );


	++nSteps;
	acadoPrintf( "\n*** Simulation Loop No. %d (starting at time %.3f) ***\n",nSteps,simulationClock.getTime( ) );

	/* Perform one single simulation loop */
	Vector u, p;
	Vector uPrevious, pPrevious;

	if ( getNU( ) > 0 )
		feedbackControl.evaluate( simulationClock.getTime( ),uPrevious );

	if ( getNP( ) > 0 )
		feedbackParameter.evaluate( simulationClock.getTime( ),pPrevious );

	VariablesGrid y;
	Vector yPrevious;

	if ( getNY( ) > 0 )
		processOutput.evaluate( simulationClock.getTime( ),yPrevious );


	// step controller
// 	yPrevious.print("controller input y");
	
	if ( controller->step( simulationClock.getTime( ),yPrevious ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_ENVIRONMENT_STEP_FAILED );

	double compDelay = determineComputationalDelay( controller->getPreviousRealRuntime( ) );
	double nextSamplingInstant = controller->getNextSamplingInstant( simulationClock.getTime( ) );
	nextSamplingInstant = round( nextSamplingInstant * 1.0e6 ) / 1.0e6;

	// obtain new controls and parameters
	if ( controller->getU( u ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_ENVIRONMENT_STEP_FAILED );

// 	u.print("controller output u");

	if ( controller->getP( p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_ENVIRONMENT_STEP_FAILED );

	if ( acadoIsEqual( simulationClock.getTime( ),endTime ) == BT_TRUE )
	{
		simulationClock.init( nextSamplingInstant );
		return SUCCESSFUL_RETURN;
	}
	
	if ( fabs( compDelay ) < 100.0*EPS )
	{
		// step process without computational delay
		if ( process->step( simulationClock.getTime( ),nextSamplingInstant,u,p ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_STEP_FAILED );

		// Obtain current process output
		if ( process->getY( y ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_STEP_FAILED );

// 		y.print("process output y");

		// update history
		if ( getNU( ) > 0 )
			feedbackControl.  add( simulationClock.getTime( ),nextSamplingInstant,u );
		if ( getNP( ) > 0 )
			feedbackParameter.add( simulationClock.getTime( ),nextSamplingInstant,p );
		if ( getNY( ) > 0 )
			processOutput.    add( y,IM_LINEAR );
	}
	else
	{
		// step process WITH computational delay
		if ( simulationClock.getTime( )+compDelay > nextSamplingInstant )
			return ACADOERROR( RET_COMPUTATIONAL_DELAY_TOO_BIG );

		Grid delayGrid( 3 );
		delayGrid.setTime( simulationClock.getTime( ) );
		delayGrid.setTime( simulationClock.getTime( )+compDelay );
		delayGrid.setTime( nextSamplingInstant );

		VariablesGrid uDelayed( u.getDim( ),delayGrid,VT_CONTROL );
		uDelayed.setVector( 0,uPrevious );
		uDelayed.setVector( 1,u );
		uDelayed.setVector( 2,u );

		VariablesGrid pDelayed( p.getDim( ),delayGrid,VT_PARAMETER );
		pDelayed.setVector( 0,pPrevious );
		pDelayed.setVector( 1,p );
		pDelayed.setVector( 2,p );

		if ( process->step( uDelayed,pDelayed ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_STEP_FAILED );

		// Obtain current process output
		if ( process->getY( y ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_STEP_FAILED );

		// update history
		if ( getNU( ) > 0 )
			feedbackControl.  add( uDelayed,IM_CONSTANT );

		if ( getNP( ) > 0 )
			feedbackParameter.add( pDelayed,IM_CONSTANT );

		if ( getNY( ) > 0 )
			processOutput.    add( y,IM_LINEAR );
	}

	// update simulation clock
	simulationClock.init( nextSamplingInstant );

	return SUCCESSFUL_RETURN;
}


returnValue SimulationEnvironment::step(	double nextTime
											)
{
	if ( ( nextTime < startTime ) || ( nextTime > endTime ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	while ( nextTime >= simulationClock.getTime( ) )
	{
		if ( step( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_ENVIRONMENT_STEP_FAILED );
	}

// 	MatrixVariablesGrid runtimes;
// 	controller->getAll( LOG_TIME_CONTROL_LAW,runtimes );
// 	runtimes.print( "runtime",PS_MATLAB );

	return SUCCESSFUL_RETURN;
}


returnValue SimulationEnvironment::run( )
{
	return step( endTime );
}



// PROTECTED FUCNTIONS:
// --------------------

returnValue SimulationEnvironment::setupOptions( )
{
	addOption( SIMULATE_COMPUTATIONAL_DELAY , defaultSimulateComputationalDelay );
	addOption( COMPUTATIONAL_DELAY_FACTOR   , defaultComputationalDelayFactor   );
	addOption( COMPUTATIONAL_DELAY_OFFSET   , defaultComputationalDelayOffset   );

	return SUCCESSFUL_RETURN;
}

returnValue SimulationEnvironment::setupLogging( )
{
// 	LogRecord tmp( LOG_AT_EACH_ITERATION,stdout,PS_DEFAULT );
// 
// 	tmp.addItem( LOG_FEEDBACK_CONTROL );
// 
// 	addLogRecord( tmp );

	return SUCCESSFUL_RETURN;
}


double SimulationEnvironment::determineComputationalDelay(	double controllerRuntime
															) const
{
	int simulateComputationalDelay;
	get( SIMULATE_COMPUTATIONAL_DELAY,simulateComputationalDelay );

	if ( (BooleanType)simulateComputationalDelay == BT_TRUE )
	{
		ACADOWARNING( RET_COMPUTATIONAL_DELAY_NOT_SUPPORTED );
		return 0.0;

		double computationalDelayFactor, computationalDelayOffset;
		get( COMPUTATIONAL_DELAY_FACTOR,computationalDelayFactor );
		get( COMPUTATIONAL_DELAY_OFFSET,computationalDelayOffset );

		return acadoMax( 0.0, controllerRuntime*computationalDelayFactor + computationalDelayOffset );
	}
	else
		return 0.0;
}




CLOSE_NAMESPACE_ACADO

// end of file.
