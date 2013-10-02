cmake_minimum_required(VERSION 2.8.3)
project(aw_task_deligator)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg)



include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(
  FILES
  getObjectives.srv
)

add_message_files(
  FILES
  Objectives.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

add_executable(aw_task_deligator
  src/aw_task_deligator.cpp 
  src/munkres-cpp/munkres.cpp
  src/TaskDeligator.cpp
  src/aw_drone.cpp
  src/threshold.cpp)

catkin_package(
)

target_link_libraries(aw_task_deligator ${catkin_LIBRARIES})
