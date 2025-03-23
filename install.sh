#!/bin/bash

echo "╔══╣ Setup: SOBIT PRO (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages for SOBIT PRO
ros_packages=(
    # "sobits_common" \
    "dynamixel_hardware" \
    "sobits_msgs" \
    "urg_node" \
    # "azure_kinect_ros_driver" \
    "realsense_ros"
)

# Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    # if [ -f ${ros_packages[i]}/install.sh ]; then
    #     echo "Running install.sh in ${ros_packages[i]}."
    #     cd ${ros_packages[i]}
    #     bash install.sh
    #     cd ..
    # fi
}

# Download required dependencies
sudo apt-get update
sudo apt-get install -y \
    mpg321 

# Download ROS packages
# sudo apt-get update
# sudo apt-get install -y \
#     ros-$ROS_DISTRO-pybind11-catkin \
#     ros-$ROS_DISTRO-robot-state-publisher \
#     ros-$ROS_DISTRO-joint-state-controller \
#     ros-$ROS_DISTRO-joint-state-publisher \
#     ros-$ROS_DISTRO-joint-state-publisher-gui \
#     ros-$ROS_DISTRO-joint-limits-interface \
#     ros-$ROS_DISTRO-hardware-interface \
#     ros-$ROS_DISTRO-transmission-interface \
#     ros-$ROS_DISTRO-controller-interface \
#     ros-$ROS_DISTRO-controller-manager \
#     ros-$ROS_DISTRO-ros-control \
#     ros-$ROS_DISTRO-ros-controllers \
#     ros-$ROS_DISTRO-tf2 \
#     ros-$ROS_DISTRO-tf2-ros \
#     ros-$ROS_DISTRO-sensor-msgs \
#     ros-$ROS_DISTRO-trajectory-msgs \
#     ros-$ROS_DISTRO-geometry-msgs \
#     ros-$ROS_DISTRO-joy

# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-ecl-linear-algebra \
    ros-$ROS_DISTRO-laser-proc \
    ros-$ROS_DISTRO-urg-c \
    ros-$ROS_DISTRO-urg-node \
    ros-$ROS_DISTRO-urg-node-msgs \
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
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-velocity-controllers \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-group-impedance-controller \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-robot-controllers \
    ros-$ROS_DISTRO-robot-controllers-interface \
    ros-$ROS_DISTRO-urdf \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-tf-transformations


# Install Gazebo Fortress with binaries
sudo apt-get install -y \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ign-ros2-control \
    ros-$ROS_DISTRO-ign-ros2-control-demos


# Setting up Dynamixel USB configuration (SOBIT PRO: Mobile Robot Mechanism)
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", SYMLINK+=\"input/dx_lower\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dx_lower.rules

# Setting up Dynamixel USB configuration (SOBIT PRO: Head and Arm Robot Mechanism)
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"input/dx_upper\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dx_upper.rules

# Setting up PS4 Joystick USB configuration
echo "KERNEL==\"uinput\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"05c4\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:05C4.*\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"09cc\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:09CC.*\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/50-ds4drv.rules

# Reload udev rules
sudo udevadm control --reload-rules

# Trigger the new rules
sudo udevadm trigger

# Go back to previous directory
cd ${DIR}


echo "╚══╣ Setup: SOBIT PRO (FINISHED) ╠══╝"
