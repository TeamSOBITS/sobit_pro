#!/bin/bash

echo "╔══╣ Setup: SOBIT PRO (STARTING) ╠══╗"


# Keep track of the current directory
CURRENT_DIR=`pwd`
cd ..

# Dowload required packages for SOBIT PRO
ros_packages=(
    "sobit_common" \
    "sobits_msgs" \
    "urg_node" \
    "azure_kinect_ros_driver" \
    "realsense_ros"
)

# Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone -b feature/oss https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}


# Setting up sound configuration
echo "pacmd load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket &> /dev/null" >> ~/.bashrc
echo "#!bin/bash
touch /tmp/pulseaudio.client.conf
echo \"default-server = unix:/tmp/pulseaudio.socket \n 
      # Prevent a server running in the container \n 
      autospawn = no \n 
      daemon-binary = /bin/true \n
      # Prevent the use of shared memory \n
      enable-shm = false\" >> /tmp/pulseaudio.client.conf" | sudo tee /etc/profile.d/sound_setup.sh
sudo bash /etc/profile.d/sound_setup.sh

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
cd ${CURRENT_DIR}


echo "╚══╣ Setup: SOBIT PRO (FINISHED) ╠══╝"
