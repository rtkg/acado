#include <iostream>
#include <fstream>
#include <string>

#include <acado_integrators.hpp>

#define PI 3.141592653589793238462643383279

using namespace std;

USING_NAMESPACE_ACADO

// CART PENDULUM
//
// This is a trivial example to get stated with the kinematics toolbox.
// pendulum on a cart

int main(int argc, char *argv[]){

try{
  
Expression t;			// Time
Expression x,theta;	// The states we will use
Expression dx,dtheta;
Expression ddx,ddtheta;

Expression L;			// Length of the cart

Expression q; q(0)=x;q(1)=theta;
Expression dq; dq(0)=dx;dq(1)=dtheta;
Expression ddq; ddq(0)=ddx;ddq(1)=ddtheta;

Frame f0("world frame",q,dq,ddq,t); // The world frame is an inertial frame
				// It has a notion of time.

// We chain transformations together as follows:

Frame f1("cart frame",f0,tr(x,0,0)); // The frame attached to the cart
Frame f2("pendulum anchor frame",f1,TRzp(-1)*TRz(theta)); // The frame attached to the pendulum's anchorpoint
Frame f3("pendulum CM frame",f2,tr(L/2,0,0)); // The frame attached to the pendulum's CM



cout << "We express all in the inertial frame" << endl;

cout << "== positions ==" << endl;
KinVec v=pos(f1,f0);
cout << "The position of the cart is: " << v.getCoords() << endl;
v=pos(f3,f0);
cout << "The position of the pendulum's CM is: " << v.getCoords() << endl;

cout << "== velocities ==" << endl;
v=vel(f1,f0,f0);
cout << "The velocity of the cart is: " << v.getCoords() << endl;
v=vel(f2,f0,f0);
cout << "The velocity of the pendulum's anchorpoint is: " << v.getCoords() << endl;
v=vel(f3,f0,f0);
cout << "The velocity of the pendulum's CM is: " << v.getCoords() << endl;

cout << "== accelerations ==" << endl;
v=acc(f3,f0,f0);
cout << "The velocity of the pendulum's CM is: " << v.getCoords() << endl;

cout << "== rotations ==" << endl;
v=rotVel(f2,f0,f0);
cout << "The velocity of the pendulum's CM is: " << v.getCoords() << endl;
v=rotVel(f3,f0,f0);
cout << "The velocity of the pendulum's CM is: " << v.getCoords() << endl;

} catch (const char * str){
  cerr << str << endl;
  return 1;
}
}
