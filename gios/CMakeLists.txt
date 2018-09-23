cmake_minimum_required(VERSION 2.8.3)
project(gios)
add_compile_options(-std=c++11)

set(ACADO_ROOT_DIR tests/solvers/drone_mpc_full_generated)

set(ACADO_INCLUDE_DIRS
     ${ACADO_ROOT_DIR}/
     ${ACADO_ROOT_DIR}/qpoases/
     ${ACADO_ROOT_DIR}/qpoases/INCLUDE
     ${ACADO_ROOT_DIR}/qpoases/SRC
     )
set(ACADO_SOURCES
  ${ACADO_ROOT_DIR}/qpoases/SRC/Bounds.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/Bounds.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/Constraints.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/Constraints.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/CyclingManager.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/CyclingManager.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/Indexlist.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/Indexlist.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/MessageHandling.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/MessageHandling.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/QProblem.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/QProblem.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/SubjectTo.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/SubjectTo.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/QProblemB.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/QProblemB.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/Utils.cpp
  ${ACADO_ROOT_DIR}/qpoases/SRC/Utils.ipp
  ${ACADO_ROOT_DIR}/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  ${ACADO_ROOT_DIR}/acado_auxiliary_functions.c
  ${ACADO_ROOT_DIR}/acado_integrator.c
  ${ACADO_ROOT_DIR}/acado_qpoases_interface.cpp
  ${ACADO_ROOT_DIR}/acado_solver.c
  )

set(${PROJECT_NAME}_INCLUDE_DIRS
     ${catkin_INCLUDE_DIRS}
     include/
   )

find_package(catkin REQUIRED)

# Exported Include dirs (I think)
catkin_package(
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS}
)

include_directories( ${${PROJECT_NAME}_INCLUDE_DIRS} )

# Library
add_library(${PROJECT_NAME}_lib src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# Tests 
if (CATKIN_ENABLE_TESTING)
  include_directories( ${ACADO_INCLUDE_DIRS} )
  catkin_add_gtest(test_acado ${ACADO_SOURCES} tests/test_acado.cpp)
  target_link_libraries(test_acado ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
endif()
