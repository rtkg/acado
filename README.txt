The ACADO tookit package, originally available from here: http://git.mech.kuleuven.be/robotics/optimization.git, git://git.mech.kuleuven.be/robotics/optimization.git.

Here, the export flags in the manifest have been changed in order to be able to use qpOASES: <cpp cflags="-I${prefix}/include -I${prefix}/external_packages -I${prefix}/external_packages/qpOASES-2.0/INCLUDE -fPIC" lflags="-Wl,-rpath,${prefix}/libs -L${prefix}/libs -lacado_optimal_control -lacado_toolkit -lacado_integrators -lqpOASES2.0 -lqpOASESextras2.0 -lcsparse "/> 

