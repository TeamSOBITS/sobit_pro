cmake_minimum_required(VERSION 3.0.2)
project(sobit_pro_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamixel_sdk
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sobit_pro_control src/sobit_pro_main.cpp
                                 src/sobit_pro_control.cpp
                                 src/sobit_pro_motor_driver.cpp
                                 src/sobit_pro_odometry.cpp)
target_link_libraries(sobit_pro_control ${catkin_LIBRARIES})
