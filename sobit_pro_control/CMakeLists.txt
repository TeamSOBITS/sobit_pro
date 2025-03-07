cmake_minimum_required(VERSION 3.10)
project(sobit_pro_control)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ament packages and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# Include directories
include_directories(
  include
)

# Build the executable
# add_executable(${PROJECT_NAME} 
  # src/sobit_pro_main.cpp
  # src/sobit_pro_control.cpp
  # src/sobit_pro_motor_driver.cpp
  # src/sobit_pro_odometry.cpp
# )

# Link against ament libraries
# ament_target_dependencies(${PROJECT_NAME} 
#   rclcpp
#   rclcpp_components
#   std_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   dynamixel_sdk
# )

# Install the executable
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# Export package information
# ament_package()
