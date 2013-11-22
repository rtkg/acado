
#include <acado_toolkit.hpp>


int main( ){

	USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
	
	DifferentialState     C1_T, C2_T;	
	Control               u    ;
	Parameter 		T; //time horizon
	DifferentialEquation  f;
	

	const double C1 = 10.0; // capacity wall
	const double C2 = 10.0; // capacity
	const double R1 = 1.0; // wall resistance*0.5
	const double R2 = 1.0; // wall resistance*0.5
	
	
    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
	
	f << dot(C1_T) == (u-C1_T)/(R1*C1) + (C2_T-C1_T)/(R2*C1);
	f << dot(C2_T) == (C1_T-C2_T)/(R2*C2);
	
		
    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;

    h << C2_T;
    h << u;
    
    Vector r(2);
    r(0) = 320.0;
    r(1) = 320.0;
    
    Matrix S(2,2);
    S.setIdentity();
    S(1,1) = 0.00001;
    
       
    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
	OCP ocp(0.0, 10.0, 40);  //time horizon of the ocp
	
	ocp.minimizeLSQ( S, h, r );
	ocp.subjectTo( f );

	ocp.subjectTo( 280.0 <= u <=  370.0   );

    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity);

	Process process( dynamicSystem,INT_RK45 );

	
    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------

	RealTimeAlgorithm alg( ocp, 0.2 );
	alg.set( MAX_NUM_ITERATIONS, 1 );
	
	StaticReferenceTrajectory zeroReference;

	Controller controller( alg,zeroReference );
	controller.setSamplingTime(0.2); //Determines TISC sync rate.
	
    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	
	SimulationEnvironmentTISC sim(0.0, 100.0, controller);

	Vector x0(2);
	x0(0)=293.15;
	x0(1)=293.15;
	
	Vector u0(1);
	u0(0)=300.0;
	
	sim.init( x0, u0 );
	sim.run( );
	
    return 0;
}
