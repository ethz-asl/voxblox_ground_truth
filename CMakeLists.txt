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
