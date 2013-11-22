cmake_minimum_required(VERSION 2.8)
PROJECT(ACADO CXX C)
#SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)
SET(TESTING_OUTPUT_PATH ${PROJECT_BINARY_DIR}/testing/automatic_testing)
SET(CMAKE_CXX_FLAGS, -O3)
add_definitions(-DLINUX)

SET(DEBUG TRUE)

# Define packages
SET(INTEGRATOR_DIRS
   utils
   clock
   matrix_vector
   variable
   curve
   user_interaction
   symbolic_expression
   symbolic_operator
   function
   integrator
   sparse_solver
)
SET(OPTIMAL_CONTROL_DIRS
   ${INTEGRATOR_DIRS}
   code_generation
   dynamic_system
   dynamic_discretization
   objective
   constraint
   ocp
   conic_program
   conic_solver
   nlp_solver
   nlp_derivative_approximation
   optimization_algorithm
)
SET(TOOLKIT_DIRS
   ${OPTIMAL_CONTROL_DIRS}
   process
   noise
   transfer_device
   controller
   control_law
   estimator
   reference_trajectory
   output_transformator
   simulation_environment
   variables_grid
)

# Define example directories
SET(EXAMPLE_DIRS
#   curve
#   dsoa_test_suite
#   function
   getting_started
#   integrator
#   matrix_vector
#   modeling_tools
    multi_objective
#   multi_stage_ocp
#   nlp
    ocp
    parameter_estimation
    simulation_environment
#   sparse_solver
#   xml
)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/external_packages/ ${PROJECT_SOURCE_DIR}/external_packages/src/ ${PROJECT_SOURCE_DIR}/external_packages/include ${PROJECT_SOURCE_DIR}/include/  ${PROJECT_SOURCE_DIR}/external_packages/qpOASES-2.0/INCLUDE ${PROJECT_SOURCE_DIR}/new_features/include) 

# Compile stand-alone libraries if requested.
IF (WITH_LIBRARIES)

   MESSAGE( STATUS "COMPILING WITH LIBRARIES ... " )

   SET(PACKAGES INTEGRATOR_DIRS OPTIMAL_CONTROL_DIRS)
   SET(LIB_NAMES integrator optcont)
   LIST(LENGTH PACKAGES s)
   MATH(EXPR s ${s}-1)
# Collecting the source files and compiling them.
   FOREACH(i RANGE ${s})
      LIST(GET PACKAGES ${i} ACT_PACKAGE)
      LIST(GET LIB_NAMES ${i} ACT_LIBNAME)
      UNSET(SOURCES)
      FOREACH(DIR ${${ACT_PACKAGE}})
	 FILE(GLOB SRC src/${DIR}/*.cpp)
	 SET(SOURCES ${SOURCES} ${SRC})
      ENDFOREACH(DIR ${${ACT_PACKAGE}})
      ADD_LIBRARY(${ACT_LIBNAME} SHARED ${SOURCES})
   ENDFOREACH(i RANGE s)
ENDIF (WITH_LIBRARIES)




# 
# Adding interfaces to external packages
FILE(GLOB SRC external_packages/src/acado_qpoases/*.cpp)
SET(EXT_SOURCES ${EXT_SOURCES} ${SRC})
FILE(GLOB SRC external_packages/src/acado_csparse/acado_csparse.cpp)
SET(EXT_SOURCES ${EXT_SOURCES} ${SRC})

# Compiling acado as a static library
UNSET(SOURCES)
FOREACH(DIR ${TOOLKIT_DIRS})
   INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/acado/${DIR})
   FILE(GLOB SRC src/${DIR}/*.cpp)
   SET(SOURCES ${SOURCES} ${SRC})

   FILE(GLOB TSRC1 external_packages/src/acado_csparse/*cpp)
   SET(SOURCES ${SOURCES} ${TSRC1})
   FILE(GLOB TSRC2 external_packages/qpOASES-2.0/*cpp)
   SET(SOURCES ${SOURCES} ${TSRC2})

ENDFOREACH(DIR ${TOOLKIT_DIRS})
SET(SOURCES ${SOURCES} ${EXT_SOURCES})
#FILE(GLOB FIX_WEIRD_BUG external_packages/src/acado_csparse/acado_csparse_matlab.cpp)
ADD_LIBRARY(acado SHARED ${FIX_WEIRD_BUG} ${SOURCES})




# Compiling qpoases and qpoases-extras
ADD_SUBDIRECTORY(./external_packages/qpOASES-2.0/ ./ext_libs/qpoases)
SET(LIBRARIES ${LIBRARIES} qpoases-extras qpoases)

# Compiling gnuplot interface
ADD_SUBDIRECTORY(./external_packages/gnuplot/ ./ext_libs/gnuplot)
SET(LIBRARIES ${LIBRARIES} gnuplot)

# Compiling csparse
ADD_SUBDIRECTORY(./external_packages/csparse/ ./ext_libs/csparse)
SET(LIBRARIES ${LIBRARIES} csparse)

target_link_libraries( acado csparse qpoases-extras qpoases gnuplot )

# Compiling examples
FOREACH(ELEMENT ${EXAMPLE_DIRS})
#    MESSAGE( STATUS "-------------------------------" )
   FILE(GLOB SOURCES examples/${ELEMENT}/*.cpp)
#    MESSAGE( STATUS "SOURCES = ${SOURCES}" )
   FOREACH(SRC ${SOURCES})
      	 GET_FILENAME_COMPONENT(EXEC_NAME ${SRC} NAME_WE)
	 ADD_EXECUTABLE(${ELEMENT}_${EXEC_NAME} ${SRC})
	 TARGET_LINK_LIBRARIES( ${ELEMENT}_${EXEC_NAME} acado ${LIBRARIES} )
   ENDFOREACH(SRC ${SOURCES})
ENDFOREACH(ELEMENT ${EXAMPLE_DIRS})



IF(WITH_TESTING)
   MESSAGE( STATUS "COMPILING WITH TESTS ... " )
ELSE()
   MESSAGE( STATUS "COMPILING WITHOUT TESTS  ... " )
ENDIF()


# Tests
IF (WITH_TESTING)
  ENABLE_TESTING()
  INCLUDE(CTest)
  SET(DBUILD_DASHBOARD_REPORTS=ON)
  ADD_SUBDIRECTORY(./testing/automatic_testing ${TESTING_OUTPUT_PATH})
ENDIF (WITH_TESTING)




