# sobit_turtlebot_library
Library to control SOBIT_PRO

---

## How to Install
```bash:
$ sudo apt update 
$ sudo apt install ros-melodic-pybind11_catkin -y
$ git clone https://gitlab.com/TeamSOBITS/sobit_turtlebot_library.git
$ cm
```

---

## SobitProJointController
SOBIT_PROのカメラパンチルトとマニピュレータを動かすクラス

### Joint
* SOBIT_PROのジョイント名とその定数名(Enum : Joint)
    ```bash
    * "arm_shoulder_1_tilt_joint"     :   ARM_SHOULDER_1_TILT_JOINT
    * "arm_shoulder_2_tilt_joint"     :   ARM_SHOULDER_2_TILT_JOINT
    * "arm_elbow_upper_1_tilt_joint"  :   ARM_ELBOW_UPPER_1_TILT_JOINT
    * "arm_elbow_upper_2_tilt_joint"  :   ARM_ELBOW_UPPER_2_TILT_JOINT
    * "arm_elbow_lower_tilt_joint"    :   ARM_ELBOW_LOWER_TILT_JOINT
    * "arm_elbow_lower_pan_joint"    :   ARM_ELBOW_LOWER_PAN_JOINT
    * "arm_wrist_tilt_joint"          :   ARM_WRIST_TILT_JOINT
    * "hand_joint"                    :   HAND_JOINT
    * "head_camera_pan_joint"         :   HEAD_CAMERA_PAN_JOINT
    * "head_camera_tilt_joint"        :   HEAD_CAMERA_TILT_JOINT
    ```

### Functions
1.  moveJoint()   :   1つのジョイントを動かす(ジョイントの指定は定数名(Enum))
    ```bash
    bool sobit::SobitProJointController::moveJoint (
        const Joint joint_num,          :   ジョイント番号
        const double rad,               :   移動角度
        const double sec,               :   移動時間
        bool is_sleep = true            :   移動中にスリープ(待機)を入れるかどうか
    )
    ```  
2.  moveHeadPanTilt()   :   xtionのパンチルトを任意の角度に動かす
    ```bash
    bool sobit::SobitProJointController::moveHeadPanTilt (
        const double pan_rad,           :   パン角度
        const double tilt_rad,          :   チルト角度
        const double sec,               :   移動時間
        bool is_sleep = true            :   移動中にスリープ(待機)を入れるかどうか
    )
    ```  
3.  moveArm()   :   アームを任意の角度に動かす
    ```bash
    bool sobit::SobitProJointController::moveArm ( 
        const double arm1,              :   アームの関節1の角度
        const double arm2,              :   アームの関節2の角度
        const double arm3,              :   アームの関節3の角度
        const double arm_wrist_tilt               :   アームの関節4の角度
    )
    ```  
4.  moveToRegisterdMotion()   :   予め設定したポーズに動かす
    ```bash
    bool sobit::SobitProJointController::movePose( 
        const std::string &pose_name 

    )
    ```  

    * ポーズの設定方法：sobit_pro_library/config/sobit_pro_pose.yamlでポーズを設定する
    ```bash
    sobit_pro_pose:
         - { 
            pose_name: "initial_pose",
            arm_shoulder_1_tilt_joint: 1.57,
            arm_shoulder_2_tilt_joint: -1.57,
            arm_elbow_upper_1_tilt_joint: 1.57,
            arm_elbow_upper_2_tilt_joint: -1.57,
            arm_elbow_lower_tilt_joint: 0.0,
            arm_elbow_lower_pan_joint: -1.57,
            arm_wrist_tilt_joint: -1.57,
            hand_joint: 0.0,
            head_camera_pan_joint: 0.0,
            head_camera_tilt_joint: 0.0
        }
         - {
            pose_name: "detecting_pose",
            arm_shoulder_1_tilt_joint: 1.57,
            arm_shoulder_2_tilt_joint: -1.57,
            arm_elbow_upper_1_tilt_joint: 1.57,
            arm_elbow_upper_2_tilt_joint: -1.57,
            arm_elbow_lower_tilt_joint: 0.0,
            arm_elbow_lower_pan_joint: -1.57,
            arm_wrist_tilt_joint: -1.57,
            hand_joint: 0.0,
            head_camera_pan_joint: 0.0,
            head_camera_tilt_joint: -0.7
        }
    ```  

---

## SobitProWheelController
SOBIT_PROの車輪を動かすクラス

### Functions
1.  controlWheelLinear()   :   並進運動を行う
    ```bash
    bool sobit::SobitProWheelController::controlWheelLinear (
        const double distance_x,        :   x方向への直進移動距離
        const double distance_y,        :   y方向への直進移動距離
    )
    ```  
2.  controlWheelRotateRad()   :   回転運動を行う(弧度法：Radian)
    ```bash
    bool sobit::SobitProWheelController::controlWheelRotateRad (
        const double angle_rad,               :   回転角度(弧度法：Radian)
    )
    ```  
3.  controlWheelRotateDeg()   :   回転運動を行う(度数法：Degree)
    ```bash
    bool sobit::SobitProWheelController::controlWheelRotateDeg ( 
        const double angle_deg,               :   回転角度(度数法：Degree)
    )
    ```

---