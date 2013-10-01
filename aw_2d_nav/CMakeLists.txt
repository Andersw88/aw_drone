cmake_minimum_required(VERSION 2.8.3)
project(aw_2d_nav)

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(aw_2d_nav src/aw_2d_nav.cpp)
target_link_libraries(aw_2d_nav ${catkin_LIBRARIES})