cmake_minimum_required(VERSION 2.8.3)
project(gios)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED)

set(GIOS_INCLUDE_DIRS
     ${catkin_INCLUDE_DIRS}
     src/include/
     )

catkin_package(
  INCLUDE_DIRS ${GIOS_INCLUDE_DIRS}
)

include_directories(
  ${GIOS_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}_lib
  src/source/gios.cpp
)

add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

