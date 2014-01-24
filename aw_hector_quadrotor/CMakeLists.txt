cmake_minimum_required(VERSION 2.8.3)
project(aw_hector_quadrotor)


find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
)
