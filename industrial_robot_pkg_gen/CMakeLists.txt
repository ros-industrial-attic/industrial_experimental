cmake_minimum_required(VERSION 2.8.3)
project(industrial_robot_pkg_gen)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

