#!/bin/sh

echo "Install ros-kinetic-turtlebot*"
apt-get install ros-kinetic-turtlebot*

echo "Install ros-kinetic-urg-node"
apt-get install ros-kinetic-urg-node

echo "Install ros-kinetic-uvc-camera"
apt-get install ros-kinetic-uvc-camera

echo "Install ros-kinetic-roswww"
apt-get install ros-kinetic-roswww

echo "Install ros-kinetic-rosbridge-suite"
apt-get install ros-kinetic-rosbridge-suite

echo "Install ros-kinetic-jsk-rviz-plugins"
apt-get install ros-kinetic-jsk-rviz-plugins

echo "Install mplayer"
apt-get install mplayer

echo "Install smach"
apt-get install ros-kinetic-smach*

echo "Install ros-control"
apt install ros-kinetic-ros-control*

echo "Install ros-control"
apt install ros-kinetic-joint-*

echo "Seting wheel"
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", ATTRS{serial}==\"E148\", SYMLINK+=\"wheel\", MODE=\"0666\"" > /etc/udev/rules.d/wheel.rules
sudo /etc/init.d/udev reload

echo "Setting arm_pantilt"
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", ATTRS{serial}==\"E143\", SYMLINK+=\"arm_pantilt\", MODE=\"0666\"" > /etc/udev/rules.d/arm_pantilt.rules
sudo /etc/init.d/udev reload

echo "Install sympy"
pip install sympy

echo "Install Finished"
