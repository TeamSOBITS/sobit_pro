cmake_minimum_required(VERSION 3.10)
project(sobit_pro_library)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pybind11 REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sobits_interfaces REQUIRED)

###########
## Build ##
###########

include_directories(
  include
)

# ライブラリの作成
add_library(${PROJECT_NAME} SHARED
  # src/sobit_pro_joint_controller.cpp
  # src/sobit_pro_wheel_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  sobits_interfaces
)

# Pybind11 モジュールの作成
# pybind11_add_module(sobit_pro_module MODULE
#   src/sobit_pro_library.cpp
#   src/sobit_pro_joint_controller.cpp
#   src/sobit_pro_wheel_controller.cpp
# )
ament_target_dependencies(sobit_pro_module
  rclcpp
  hardware_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  sobits_interfaces
)

# 実行可能ファイルの作成
# add_executable(test_control_arm example/test_control_arm.cpp
#                                 src/sobit_pro_joint_controller.cpp
#                                 src/sobit_pro_wheel_controller.cpp)
# ament_target_dependencies(test_control_arm rclcpp)

# add_executable(test_control_head example/test_control_head.cpp
#                                  src/sobit_pro_joint_controller.cpp
#                                  src/sobit_pro_wheel_controller.cpp)
# ament_target_dependencies(test_control_head rclcpp)

# add_executable(test_grasp example/test_grasp.cpp
#                           src/sobit_pro_joint_controller.cpp
#                           src/sobit_pro_wheel_controller.cpp)
# ament_target_dependencies(test_grasp rclcpp)

# add_executable(test_place example/test_place.cpp
#                           src/sobit_pro_joint_controller.cpp
#                           src/sobit_pro_wheel_controller.cpp)
# ament_target_dependencies(test_place rclcpp)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME} sobit_pro_module
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS test_control_arm test_control_head test_grasp test_place
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
