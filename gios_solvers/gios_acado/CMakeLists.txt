cmake_minimum_required(VERSION 2.8.3)
project(gios_acado)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS gios_common)

set(GIOS_ACADO_INCLUDE_DIRS
     ${catkin_INCLUDE_DIRS}
     src/include/
     src/external/qpoases/
     src/external/qpoases/INCLUDE
     src/external/qpoases/SRC
     src/external
     )
catkin_package(
  INCLUDE_DIRS ${GIOS_ACADO_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS gios_common
)

###########
## Build ##
###########

include_directories(
  ${GIOS_ACADO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/external/qpoases/SRC/Bounds.cpp
  src/external/qpoases/SRC/Bounds.ipp
  src/external/qpoases/SRC/Constraints.cpp
  src/external/qpoases/SRC/Constraints.ipp
  src/external/qpoases/SRC/CyclingManager.cpp
  src/external/qpoases/SRC/CyclingManager.ipp
  src/external/qpoases/SRC/Indexlist.cpp
  src/external/qpoases/SRC/Indexlist.ipp
  src/external/qpoases/SRC/MessageHandling.cpp
  src/external/qpoases/SRC/MessageHandling.ipp
  src/external/qpoases/SRC/QProblem.cpp
  src/external/qpoases/SRC/QProblem.ipp
  src/external/qpoases/SRC/SubjectTo.cpp
  src/external/qpoases/SRC/SubjectTo.ipp
  src/external/qpoases/SRC/QProblemB.cpp
  src/external/qpoases/SRC/QProblemB.ipp
  src/external/qpoases/SRC/Utils.cpp
  src/external/qpoases/SRC/Utils.ipp
  src/external/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp

  src/external/acado_auxiliary_functions.c
  src/external/acado_integrator.c
  src/external/acado_qpoases_interface.cpp
  src/external/acado_solver.c

  src/source/gios_acado.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

