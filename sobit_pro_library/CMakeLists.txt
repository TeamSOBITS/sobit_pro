cmake_minimum_required(VERSION 3.0.2)
project(sobit_pro_library)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamixel_sdk
  hardware_interface
  roscpp
  rospy
  sobit_common_msg
  std_msgs
  trajectory_msgs
  tf
  pybind11_catkin
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pybind11_catkin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/sobit_pro_joint_controller.cpp
                            src/sobit_pro_wheel_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

pybind_add_module(sobit_pro_module MODULE src/sobit_pro_library.cpp
                                          src/sobit_pro_joint_controller.cpp
                                          src/sobit_pro_wheel_controller.cpp)

add_executable(test_arm_controll test/test_arm_controll.cpp
                        src/sobit_pro_joint_controller.cpp
                        src/sobit_pro_wheel_controller.cpp)
target_link_libraries(test_arm_controll ${catkin_LIBRARIES})
