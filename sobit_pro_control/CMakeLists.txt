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
find_package(rclcpp_action REQUIRED)

# find_package(dynamixel_workbench_toolbox REQUIRED)

###########
## Build ##
###########

## Include directories
include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/sobit_pro_control.cpp
  src/sobit_pro_odometry.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  # 他の必要な依存
)

add_executable(sobit_pro_main src/sobit_pro_main.cpp)
target_link_libraries(sobit_pro_main ${PROJECT_NAME})
ament_target_dependencies(sobit_pro_main
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  rclcpp_action
  # dynamixel_workbench_toolbox
)

install(TARGETS
  sobit_pro_main
  DESTINATION lib/${PROJECT_NAME}
)

# ament_target_dependencies(sobit_pro_control rclcpp nav_msgs tf2_ros)

# target_include_directories(${PROJECT_NAME} PUBLIC
#   include
# )

## Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_msgs"
  "nav_msgs"
  "sensor_msgs"
  "trajectory_msgs"

  "control_msgs"
  "rclcpp_action"
  
  # "dynamixel_workbench_toolbox"
)

#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# install(DIRECTORY urdf
#   DESTINATION share/${PROJECT_NAME}/
# )

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