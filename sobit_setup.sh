#!/bin/bash

cd ~/docker_ws/container/ros_melodic_sobit_pro/src/

# git cloneしたいTeamSOBITSのROSパッケージを記述
ros_packages=(
    "sobit_common" \
    "azure_kinect_ros" \
    "pytorch_yolo" \
    "sobit_navigation_stack" \
    "text_to_speech" \
    "web_speech_recognition" \
    "placeable_position_estimator"
)

for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    # echo "array[$i] = ${array[i]}"
    echo "${ros_packages[i]}"
    git clone https://gitlab.com/TeamSOBITS/${ros_packages[i]}.git
}

# オープンソースのROSパッケージのgit clone
git clone https://github.com/ros/executive_smach.git

cd 

# Setting Sound configure
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

# Seting wheel USB
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", ATTRS{serial}==\"E148\", SYMLINK+=\"input/dxhub\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dxhub.rules
#sudo /etc/init.d/udev reload

# Seting arm_pantilt USB
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", ATTRS{serial}==\"E143\", SYMLINK+=\"input/dongle\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dongle.rules
#sudo /etc/init.d/udev reload

# Seting ps4_joy_control USB
echo "KERNEL==\"uinput\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"05c4\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:05C4.*\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"09cc\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:09CC.*\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/50-ds4drv.rules
#sudo /etc/init.d/udev reload

# Seting azure_kinect USB
echo "# Bus 002 Device 116: ID 045e:097a Microsoft Corp.
      # Bus 001 Device 015: ID 045e:097b Microsoft Corp.
      # Bus 002 Device 118: ID 045e:097c Microsoft Corp.
      # Bus 002 Device 117: ID 045e:097d Microsoft Corp.
      # Bus 001 Device 016: ID 045e:097e Microsoft Corp.
      BUS!=\"usb\", ACTION!=\"add\", SUBSYSTEM!==\"usb_device\", GOTO=\"k4a_logic_rules_end\"
      ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097a\", MODE=\"0666\", GROUP=\"plugdev\"
      ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097b\", MODE=\"0666\", GROUP=\"plugdev\"
      ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097c\", MODE=\"0666\", GROUP=\"plugdev\"
      ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097d\", MODE=\"0666\", GROUP=\"plugdev\"
      ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097e\", MODE=\"0666\", GROUP=\"plugdev\"
      LABEL=\"k4a_logic_rules_end\"" | sudo tee /etc/udev/rules.d/99-k4a.rules
#sudo /etc/init.d/udev reload

#sudo udevadm control --reload-rules
#sudo udevadm trigger

# USB Reload
sudo /etc/init.d/udev reload