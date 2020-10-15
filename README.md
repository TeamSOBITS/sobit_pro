# ROS Package for SOBIT pro

The project is ROS package to control the sobit_pro

## Prerequisites

- OS: Ubuntu 18.04 
- ROS distribution: melodic Kame

## How to Install

```bash:
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/TeamSOBITS/sobit_pro.git
$ git clone https://gitlab.com/TeamSOBITS/sobit_common_msg.git
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ cd sobit_pro
$ chmod 755 install.sh
$ sudo ./install.sh
$ cd ~/catkin_ws
$ catkin_make
```

### How to use

```bash:
$ roslaunch sobit_pro minimal.launch
```

### service code for moving joints   

```bash:
sobit_pro/sobit_pro_bringup/src/joint_controller.py
```
