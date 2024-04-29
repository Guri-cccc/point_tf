cmake_minimum_required(VERSION 3.0.2)
project(point_tf)

set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++11 -DNDEBUG")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp geometry_msgs std_msgs
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_library(${PROJECT_NAME}_lib
  src/point_tf.cpp
)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib
 ${catkin_LIBRARIES}
 ${Eigen_LIBRARIES}
)