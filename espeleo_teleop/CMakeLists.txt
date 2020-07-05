cmake_minimum_required(VERSION 2.8.3)

project(espeleo_teleop)

set(CMAKE_CXX_STANDARD 11)

# Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS
        joy
        roscpp
        rospy
        std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
