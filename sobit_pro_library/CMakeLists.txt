cmake_minimum_required(VERSION 3.8)
project(sobit_pro_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find libraries
include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/sobit_pro_joint_action_server.cpp
  src/sobit_pro_wheel_action_server.cpp
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobit_pro::JointActionServer"
  EXECUTABLE joint_action_server)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobit_pro::WheelActionServer"
  EXECUTABLE wheel_action_server)


ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "sobits_interfaces"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "trajectory_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
  "tf2")


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


#############
## Install ##
#############

install(DIRECTORY
  config
  launch DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
