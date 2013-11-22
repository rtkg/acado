#include <iostream>
#include <fstream>
#include <string>
#include <map>

#include <acado_integrators.hpp>
#include <acado/modeling_tools/kinetics_tools.hpp>


USING_NAMESPACE_ACADO
using namespace std;


// CART PENDULUM
//
// This is a trivial example to get stated with the kinematics toolbox.
// Spinning top.

int main(int argc, char *argv[]){

try{

// ------------------------------
// Symbolic variable definitions|
// ------------------------------
printf("Symbolic variable definitions\n");

DifferentialState q(3);  // The time changing variables of our system
DifferentialState dq(3); // The derivatives of these variables
DifferentialState ddq(3);

IntermediateState phi   = q(0);IntermediateState dphi   = dq(0);IntermediateState ddphi   = ddq(0);
IntermediateState theta = q(1);IntermediateState dtheta = dq(1);IntermediateState ddtheta = ddq(1);
IntermediateState delta = q(2);IntermediateState ddelta = dq(2);IntermediateState dddelta = ddq(2);

IntermediateState g=9.81;
IntermediateState r=1;				// Position of CM
IntermediateState m=2;				// mass of spinning top
IntermediateState I=eye(3);  // Inertia tensor in {1} coordinates

// -------------------
// Frame definitions |
// -------------------
printf("Frame definitions\n");

// We define three frames according to image spinningtop.svg
Frame f0("world frame",q,dq,ddq);
Frame f1("CM frame",f0,TRz(phi)*TRy(-theta)*tr(r,0,0)); 
Frame f2("rotating frame",f1,TRx(delta)); 

// -------------------
// Forces and moments|
// -------------------
printf("Forces and moments\n");

// Forces working on the system
KinVec Fg(0,0,-m*g,0,f0); 			// Gravity working at CM
// other forces will be constructed later




// ----------
// Kinetics |
// ----------
printf("Kinetics\n");

// Kinetics - all in inertial frame
KinVec v=rotVel(f1,f0,f0);	// linear velocity
KinVec a=rotVel(f1,f0,f0);	// linear acceleration
KinVec w=rotVel(f2,f0,f1);	// rotational velocity
KinVec alpha=rotAcc(f2,f0,f1);	// rotational acceleration

// ---------------------
// Equations of motion |
// ---------------------

printf("Equations of motion\n");

//  A) Newton approach
//M-(I*alpha+cross(w,I*w));

KinVec FR= acc(f1,f0,f0)*m-Fg;  // Reaction force
KinVec M=cross(pos(f1,f0),FR);   // Moment working on CM from reaction force of ground

printf("let's explicitize\n");
//IntermediateState ddqf=(M-(I*alpha+cross(w,I*w))).explicitize(ddq);

std::map<unsigned int,unsigned int> di;
di[0]=0;di[1]=1;di[2]=2;

IntermediateState ddqf=(M-(I*alpha+cross(w,I*w))).explicitize(di);

printf("okay\n");

Function f;
f << ddqf;
FILE *file = fopen("debug.cpp", "w" );
file << f;
fclose(file);


//  B) Lagrange-Equation approach approach
//Expression T=m*v.transpose()*v/2+w.transpose()*I*w/2;
//Expression V=pos(f1,f0,f0)*g*m;

//Expression L=T-V;

//L.jacobian(dq).der(t)-L.jacobian(q)=pos(f1,f0,f0).jacobian(q)*Fg;

// -------------
// Integration |
// -------------

} catch (const char * str){
  cerr << str << endl;
  return 1;
}
}
