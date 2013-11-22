#include <acado_toolkit.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>

int main( ){

    USING_NAMESPACE_ACADO

	DifferentialState		x;     //alpha=TV position? model i used: alpha dot=A*alpha+B*u
	IntermediateState		alpha; //alpha=TV position? model i used: alpha dot=A*alpha+B*u
	Control					u;	   // input (0 to 1)
	DifferentialEquation	f;

	Matrix A(1,1);
	A.setZero();

	Vector b(1);
	b.setZero();

	Matrix C(1,1);
	C.setZero();
	
	Matrix D(1,1);
	D.setZero();

	double t_start	= 0.0;
	double t_end	= 1.0;
	double alphaD	= 0.8;	// alpha Desired


	// DEFINE THE MATRICES AND VECTORS:
	A(0,0)	= -43.548913059153811;	//-33.124100304944747;
	b(0)	=   1.0;				//1.0;
	C(0,0)	=  27.062532550322111;	//54.118715873296125;
    D(0,0)	=   0.377532234341255;	//-0.634266286517313;	

	f << dot(x) == A(0,0)*x +   b(0)*u;
	     alpha  =  C(0,0)*x + D(0,0)*u;


	Function h;
	h << alpha;
	h << u;

	Matrix Q(2,2);
	Q.setIdentity();
	Q(0,0) = 10.0;
	Q(1,1) = 0.01;

	Vector r(2);
	r(0) = alphaD;
	r(1) = 0.0;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
	OCP ocp( t_start, t_end, 10 );
    ocp.minimizeLSQ( Q,h,r );
	ocp.subjectTo( f );

	ocp.subjectTo( 0.0 <= u <= 1.0 );
	ocp.subjectTo( 0.0 <= alpha <= 1.0 );


    // Additionally, flush a plotting object
	GnuplotWindow window;
	window.addSubplot( x ,"State x" );
	window.addSubplot( alpha,"Output alpha" );
	window.addSubplot( u ,"Control u" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
	RealTimeAlgorithm algorithm(ocp);
	algorithm << window;

	//algorithm.set( INTEGRATOR_TYPE, INT_BDF );
    algorithm.set( MAX_NUM_ITERATIONS,3 );
	algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );

    //algorithm.initializeControls( "controls.txt" );

	Vector x0(1);
	x0(0) = 0.0;
	
// 	algorithm.solve( x0 );


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	DynamicFeedbackLaw feedbackLaw( algorithm,0.05 );
	
	StaticReferenceTrajectory zeroReference;

	Controller controller;
	controller.setFeedbackLaw( feedbackLaw );

	printf( "%d\n", controller.getNY( ) );
	printf( "%d\n", controller.getNU( ) );

	controller.init( x0 );
	controller.step( 0.0,x0 );

	printf( "%d\n", controller.getNY( ) );
	printf( "%d\n", controller.getNU( ) );

	return 0;
}
