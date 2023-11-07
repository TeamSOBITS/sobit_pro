cmake_minimum_required(VERSION 3.0.2)
project(sobit_pro_library)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamixel_sdk
  hardware_interface
  roscpp
  rospy
  std_msgs
  sobits_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  tf
  pybind11_catkin
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  controller_manager
  dynamixel_sdk
  hardware_interface
  roscpp
  rospy
  std_msgs
  sobits_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  tf
  pybind11_catkin
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

add_executable(test_controll_arm example/test_controll_arm.cpp
                                 src/sobit_pro_joint_controller.cpp
                                 src/sobit_pro_wheel_controller.cpp)
target_link_libraries(test_controll_arm ${catkin_LIBRARIES})

add_executable(test_put_on_table example/test_put_on_table.cpp
                                 src/sobit_pro_joint_controller.cpp
                                 src/sobit_pro_wheel_controller.cpp)
target_link_libraries(test_put_on_table ${catkin_LIBRARIES})

add_executable(test_grasp_on_floor example/test_grasp_on_floor.cpp
                                   src/sobit_pro_joint_controller.cpp
                                   src/sobit_pro_wheel_controller.cpp)
target_link_libraries(test_grasp_on_floor ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
