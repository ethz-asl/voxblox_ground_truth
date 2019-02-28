cmake_minimum_required(VERSION 2.8.3)
project(voxblox_ground_truth)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

###############
## LIBRARIES ##
###############


##############
## BINARIES ##
##############
cs_add_executable(ply_to_sdf
    src/main.cpp
    src/io/tinyply.cpp)


#################
# GAZEBO PLUGIN #
#################
# TODO(victorr): Check if this is the nice way to do it
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(voxblox_ground_truth_plugin SHARED src/gazebo_plugin.cpp)
target_link_libraries(voxblox_ground_truth_plugin ${GAZEBO_LIBRARIES})
