#!/bin/bash

echo "╔══╣ Setup: SOBIT PRO (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages for SOBIT PRO
ros_packages=(
    "sobits_interfaces"
    "dynamixel_hardware"
    "realsense_ros" \
    "urg_node" \
    # "azure_kinect_ros_driver" \ # TODO
    "sobits_gazebo_worlds"
)

# Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone -b $ROS_DISTRO-devel https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}


# Download required dependencies
python3 -m pip install --break-system-packages \
    transforms3d

# Download required dependencies
sudo apt update
sudo apt install -y \
    mpg321 

# Download ROS packages
sudo apt install -y \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-limits \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-transmission-interface \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-trajectory-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-linux \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-velocity-controllers \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-urdf \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-gz-ros2-control \
    ros-$ROS_DISTRO-actuator-msgs \
    ros-$ROS_DISTRO-gps-msgs \
    ros-$ROS_DISTRO-openni2-camera \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \


# Go back to previous directory
cd ${DIR}

echo "╚══╣ Setup: SOBIT PRO (FINISHED) ╠══╝"
