cmake_minimum_required(VERSION 2.8.3)
project(aw_taskplayer)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS std_msgs std_srvs)
# add_executable(aw_task_deligator
#   src/aw_taskplayer.cpp 
#   src/aw_drone.cpp)