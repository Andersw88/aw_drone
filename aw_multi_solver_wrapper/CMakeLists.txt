cmake_minimum_required(VERSION 2.8.3)
project(aw_multi_solver_wrapper)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs std_srvs  message_generation)


add_service_files(
  FILES
  MultiPlanner.srv
)

generate_messages(DEPENDENCIES std_msgs std_srvs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs std_srvs)

