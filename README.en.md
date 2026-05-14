<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT PRO

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
      <a href="#launch-and-usage">Launch and Usage</a>
      <ul>
        <li><a href="#how-to-launch-the-simulator">How to Launch the Simulator</a></li>
        <li><a href="#if-only-using-mobile-mechanism">If only using mobile mechanism</a></li>
        <li><a href="#if-only-using-camera">If only using camera</a></li>
        <li><a href="#visualization-on-rviz">Visualization on Rviz</a></li>
      </ul>
    </li>
    <li>
      <a href="#software">Software</a>
      <ul>
        <li><a href="#joint-controller">Joint Controller</a></li>
        <li><a href="#wheel-controller">Wheel Controller</a></li>
      </ul>
    </li>
    <li>
      <a href="#hardware">Hardware</a>
      <ul>
        <li><a href="#how-to-download-3d-parts">How to Download 3D Parts</a></li>
        <li><a href="#electronic-circuit-diagram">Electronic Circuit Diagram</a></li>
        <li><a href="#robot-assembly">Robot Assembly</a></li>
        <li><a href="#robot-features">Robot Features</a></li>
        <li><a href="#bill-of-materials-bom">Bill of Materials (BOM)</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#references">References</a></li>
  </ol>
</details>

<!-- INTRODUCTION -->
## Introduction

![SOBIT PRO](sobit_pro/docs/img/sobit_pro.png)

This repository provides libraries for operating the 4-wheel independently steered mobile manipulator, SOBIT PRO, developed by SOBITS.

> [!WARNING]
> If you are a beginner, make sure to operate the real robot under the supervision of an experienced user.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

This section explains how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Prerequisites

Please prepare the following environment before proceeding with the installation.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS | Jazzy Jalisco |
| Python | 3.12 |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation

1. Move to the `src` directory of your ROS workspace.
   ```sh
    cd ~/colcon_ws/src/
   ```
2. Clone this repository.
   ```sh
   git clone -b jazzy-devel https://github.com/TeamSOBITS/sobit_pro
   ```
3. Move into the repository.
   ```sh
    cd sobit_pro/
   ```
4. Install the dependent packages.
   ```sh
   bash install.sh
   ```
5. Build the packages.
   ```sh
    cd ~/colcon_ws/
    colcon build --symlink-install
    source ~/colcon_ws/install/setup.sh
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE -->
## Launch and Usage

To launch the real robot, execute [real_minimal.launch.py](sobit_pro_bringup/launch/real_minimal.launch.py).

```sh
ros2 launch sobit_pro_bringup real_minimal.launch.py
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### How to Launch the Simulator
Run [gz_minimal.launch.py](sobit_pro_bringup/launch/gz_minimal.launch.py) in the Gazebo Harmonic environment.
```sh
ros2 launch sobit_pro_bringup gz_minimal.launch.py
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### If Only Using the Mobile Base

SOBIT PRO can be operated using only the mobile base.

1. Modify the settings in either [real_minimal.launch.py](sobit_pro_bringup/launch/real_minimal.launch.py) or [gz_minimal.launch.py](sobit_pro_bringup/launch/gz_minimal.launch.py) as follows.

    ```xml
    <!-- Activate Mobile-Base (True), Arm (True), Head (True) -->
    <arg name="enable_mb"           default="True"/>
    <arg name="enable_arm"          default="False"/>
    <arg name="enable_head"         default="False"/>

    <!-- URG: lan-cable (True), usb-cable (False) -->
    <arg name="urg_lan"             default="False"/>
    ```

2. Run [real_minimal.launch.py](sobit_pro_bringup/launch/real_minimal.launch.py) for the real robot, or [gz_minimal.launch.py](sobit_pro_bringup/launch/gz_minimal.launch.py) for the simulator.

    ```sh
    ros2 launch sobit_pro_bringup real_minimal.launch.py
    ```

> [!NOTE]
> Set `use_serial_urg` to `True` when using LAN communication, and `False` when using USB communication.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### If Only Using the Camera

The camera mounted on SOBIT PRO can be operated independently.  
For the Xtion camera, run the following command.

```sh
ros2 launch sobit_pro_bringup xtion.launch.py
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Visualization on Rviz

As a preliminary step before operating the real robot, SOBIT PRO can be visualized in Rviz to display the robot configuration.

```sh
ros2 launch sobit_pro_description display.launch.py
```

If launched successfully, Rviz will appear as shown below.
![SOBIT PRO Display with Rviz](sobit_pro/docs/img/sobit_pro_display.png)

<p align="right">(<a href="#readme-top">back to top</a>)</p> 

## Software

<details>
<summary>Summary of information on SOBIT PRO and related software</summary>


### Joint Controller

This section provides information for controlling the pan-tilt mechanism and manipulator of SOBIT PRO.

#### Motion Functions

1. `move_to_pose()` : Move the robot to a predefined pose described in [pose_list.yaml](sobit_pro_library/config/pose_list.yaml).

    ```cpp
    # MoveToPose.action
    # Goal
    string pose_name                                # Predefined pose name
    builtin_interfaces/Duration time_allowance      # Limmit time
    ---
    # Result
    bool success                                    # SUCCESS/FAILED
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # the time required
    ---
    # Feedback
    string[] current_joint_names                    # Current list of active joint names
    float32[] current_joint_rad                     # Current list of active joint angles
    # float32[] current_joint_vel                     # Current list of active joint angle vellocity
    builtin_interfaces/Duration move_time           # Time taken to date
    ```
> [!NOTE]
> Existing poses can be found in [pose_list.yaml](sobit_pro_library/config/pose_list.yaml).  
> Please refer to [How to Set Poses](#how-to-set-poses) for instructions on creating new poses.

2. `move_joint()` : Moves the specified joint(s) to arbitrary angles.
    ```cpp
    # MoveJoint.action
    # Goal
    string[] target_joint_names                     # List of joint names to move
    float64[] target_joint_rad                      # List of joint angles to move
    builtin_interfaces/Duration time_allowance      # Limmit time
    ---
    # Result
    bool success                                    # SUCCESS/FAILED
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # the time required
    ---
    # Feedback
    string[] current_joint_names                    # Current list of active joint names
    float32[] current_joint_rad                     # Current list of active joint angles
    # float32[] current_joint_vel                     # Current list of active joint angle vellocity
    builtin_interfaces/Duration move_time           # Time taken to date
    ```

> [!NOTE]
> Please refer to [Joint Names](#joint-names) for the available `joint names`.

<p align="right">(<a href="#readme-top">back to top</a>)</p> 
 
#### Joint Names

The joint names of SOBIT PRO are listed below.
- arm_shoulder_1_tilt_joint
- arm_elbow_upper_1_tilt_joint
- arm_elbow_lower_tilt_joint
- arm_elbow_lower_pan_joint
- arm_wrist_tilt_joint
- hand_inner_l_joint
- hand_finger_l_joint
- hand_joint
- hand_finger_r_joint
- hand_outer_l_joint
- hand_outer_r_joint
- arm_elbow_upper_2_tilt_joint
- arm_shoulder_2_tilt_joint
- head_pan_joint
- head_tilt_joint
- wheel_b_l_steer_joint
- wheel_b_l_drive_joint
- wheel_b_r_steer_joint
- wheel_b_r_drive_joint
- wheel_f_l_steer_joint
- wheel_f_l_drive_joint
- wheel_f_r_steer_joint
- wheel_f_r_drive_joint

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### How to Set Poses

Poses can be added and edited in the file [pose_list.yaml](sobit_pro_library/config/pose_list.yaml).  
The format is as follows.

```yaml
ros_parameters:
    poses:
     - pose_name

    pose_name:
      arm_shoulder_1_tilt_joint     : 1.57
      arm_elbow_upper_1_tilt_joint  : 1.57
      arm_elbow_lower_tilt_joint    : -1.57
      arm_elbow_lower_pan_joint     : 0.00
      arm_wrist_tilt_joint          : -1.57
      hand_joint                    : 0.00
      head_pan_joint                : 0.00
      head_tilt_joint               : 0.00
```
Add the pose name you want to define to poses, and then set the angle of each joint under the corresponding pose name.

### Wheel Controller

This is a summary of information for moving the SOBIT PRO moving mechanism.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Motion Functions

1. `move_wheel_linear` : Moves the robot in translation (forward, backward, and lateral motion only). (Unit: meters)
    ```cpp
    # MoveWheelLinear.action
    # Goal
    geometry_msgs/Point target_point                # Distance to be moved (differential-wheel: x, omni-direction: (x, y))
    builtin_interfaces/Duration time_allowance      # Limit time
    ---
    # Result
    bool success                                    # SUCCESS/FAILED
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Total required time
    ---
    # Feedback
    geometry_msgs/Point current_point               # Distance traveled to date
    builtin_interfaces/Duration move_time           # Elapsed time
    ```
2. `move_wheel_rotate` : Performs rotational motion. (Unit: radian)
    ```cpp
    # MoveWheelRotate.action
    # Goal
    float32 target_yaw                              # Target rotation angle
    builtin_interfaces/Duration time_allowance      # Limit time
    ---
    # Result
    bool success                                    # SUCCESS/FAILED
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Total required time
    ---
    # Feedback
    float32 current_yaw                             # Rotated angle to date
    builtin_interfaces/Duration move_time           # Elapsed time
    ```

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Hardware

SOBIT PRO is available as open source hardware at [OnShape](https://cad.onshape.com/documents/4acbecde07fba120a62ec033/w/c6217b66947274dee4e8f911/e/c2e5c16292d7dfc11ee3cc01).

![SOBIT PRO in OnShape](sobit_pro/docs/img/sobit_pro_onshape.png)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<details>
<summary>For more information on hardware, please click here.</summary>

### How to download 3D parts

1. Access Onshape.

> [!NOTE]
> You do not need to create an `OnShape` account to download files. However, if you wish to copy the entire document, we recommend that you create an account.

2. Select the part in `Instances` by right-clicking on it.
3. A list will be displayed, press the `Export` button.
4. In the window that appears, there is a `Format` item. Select `STEP`.
5. Finally, press the blue `Export` button to start the download.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Electronic Circuit Diagram

TBD

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Robot Assembly

TBD

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Features

| Item | Details |
| --- | --- |
| Maximum linear velocity | 0.7[m/s] |
| Maximum Rotational Speed | 0.229[rad/s] |
| Maximum Payload | 0.35[kg] |
| Size (LxWxH) | 450x450x1250[mm] |
| Weight | 16[kg] |
| Remote Controller | PS3/PS4 |
| LiDAR | UST-20LX |
| RGB-D | Azure Kinect DK (head), RealSense D405 (arm) |
| IMU | LSM6DSMUS |
| Speaker | Mono Speaker |
| Microphone | Condenser Microphone |
| Actuator (Arm) | 2 x XM540-W150, 6 x XM430-W320 |
| Actuator (movement mechanism) | 4 x XM430-W320, 4 x XM430-W210 |
| Power Supply | 2 x Makita 6.0Ah 18V |
| PC Connection | USB |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Bill of Materials (BOM)

| Part | Model Number | Quantity | Where to Buy |
| --- | --- | --- | --- |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONE -->
## Milestone

- [ ] Add electronic circuit diagram
- [ ] Add robot assembly instructions
- [ ] Add links for parts list (BOM)

Please refer to the [Issues page][issues-url] for current bugs and requests for new features.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html#)
* [ROS Control](http://wiki.ros.org/ros_control)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_pro/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_pro/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_pro/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_pro/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_pro.svg?style=for-the-badge
[license-url]: LICENSE
