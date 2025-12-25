cmake_minimum_required(VERSION 3.10.2)
project(sobit_pro_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## Find ament_cmake and required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

###########
## Build ##
###########

## Include directories
include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/sobit_pro_control.cpp
  src/sobit_pro_odometry.cpp
  src/sobit_pro_main.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobit_pro::SobitProMain"
  EXECUTABLE sobit_pro_control_node)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  ament_index_cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#############
## Install ##
#############

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

install(DIRECTORY
  mp3
  config
  launch  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## Export package
ament_package()
