cmake_minimum_required(VERSION 2.8.3)
project(gios_common)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED)

set(GIOS_COMMON_INCLUDE_DIRS
     ${catkin_INCLUDE_DIRS}
     src/include/
     )

catkin_package(
  INCLUDE_DIRS ${GIOS_COMMON_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${GIOS_COMMON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/source/gios_common.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

