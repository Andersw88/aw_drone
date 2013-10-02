Requires ROS Hydro
http://wiki.ros.org/hydro/Installation/Ubuntu

Create a catkin workspace
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Run catkin_make in the root of your catkin workspace

Use "roslaunch aw_hector_quadrotor maze2.launch tgm:=2" to run. where tgm can be 0-2

some run configuration can be found in:
aw_hector_quadrotor/cfg/

goals in:
aw_hector_quadrotor/goals/

move_base maps are located in:
aw_hector_quadrotor/maps/

multiplanner "problem files" and problem designer in:
aw_multi_solver_wrapper/src
