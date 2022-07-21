#!/bin/sh

# rm ~/catkin_ws/src/FrankaArmROS/franka_arm_ros/robots/panda_arm.urdf
xacro /opt/ros/noetic/share/franka_description/robots/panda_arm.urdf.xacro > ~/catkin_ws/src/FrankaArmROS/franka_arm_ros/robots/panda_arm.urdf gazebo:=true hand:=true arm_id:="panda" xyz:='0 0 0' rpy:='0 0 0'