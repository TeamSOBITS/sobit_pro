#include <sobit_pro_library/sobit_pro_library.h>
#include <sobit_pro_library/sobit_pro_joint_controller.h>
#include <sobit_pro_library/sobit_pro_wheel_controller.hpp>

using namespace sobit_pro;

PYBIND11_MODULE( sobit_pro_module, m ) {
    pybind11::enum_<Joint>( m, "Joint" )
        .value( "ARM_SHOULDER_1_TILT_JOINT",    Joint::ARM_SHOULDER_1_TILT_JOINT )
        .value( "ARM_SHOULDER_2_TILT_JOINT",    Joint::ARM_SHOULDER_2_TILT_JOINT )
        .value( "ARM_ELBOW_UPPER_1_TILT_JOINT", Joint::ARM_ELBOW_UPPER_1_TILT_JOINT )
        .value( "ARM_ELBOW_UPPER_2_TILT_JOINT", Joint::ARM_ELBOW_UPPER_2_TILT_JOINT )
        .value( "ARM_ELBOW_LOWER_TILT_JOINT", Joint::ARM_ELBOW_LOWER_TILT_JOINT )
        .value( "ARM_ELBOW_LOWER_PAN_JOINT", Joint::ARM_ELBOW_LOWER_PAN_JOINT )
        .value( "ARM_WRIST_TILT_JOINT", Joint::ARM_WRIST_TILT_JOINT )
        .value( "GRIPPER_JOINT", Joint::GRIPPER_JOINT )
        .value( "HEAD_PAN_JOINT", Joint::HEAD_PAN_JOINT )
        .value( "HEAD_TILT_JOINT", Joint::HEAD_TILT_JOINT )
        .value( "JOINT_NUM", Joint::JOINT_NUM )
        .export_values( );

    pybind11::class_<SobitProJointController>( m, "SobitProJointController" )
        .def( pybind11::init<const std::string&>( ) )
        .def( "moveToPose", &SobitProJointController::moveToPose, "move Pose",
            pybind11::arg( "pose_name" ),
            pybind11::arg( "sec" ) = 5.0,
            pybind11::arg( "is_sleep" ) = true )
        .def( "moveJoint", &SobitProJointController::moveJoint, "move Joint",
            pybind11::arg( "joint_num" ),
            pybind11::arg( "rad" ),
            pybind11::arg( "sec" ) = 5.0,
            pybind11::arg( "is_sleep" ) = true )
        .def( "moveHeadPanTilt", &SobitProJointController::moveHeadPanTilt, "move Head PanTilt",
            pybind11::arg( "head_camera_pan" ),
            pybind11::arg( "head_camera_tilt" ),
            pybind11::arg( "sec" ) = 5.0,
            pybind11::arg( "is_sleep" ) = true )
        .def( "moveArm", &SobitProJointController::moveArm, "move Arm",
            pybind11::arg( "arm_shoulder_tilt_joint" ),
            pybind11::arg( "arm_elbow_upper_tilt_joint" ),
            pybind11::arg( "arm_elbow_lower_tilt_joint" ),
            pybind11::arg( "arm_elbow_lower_pan_joint" ),
            pybind11::arg( "arm_wrist" ),
            pybind11::arg( "sec" ) = 5.0,
            pybind11::arg( "is_sleep" ) = true )
        .def( "moveGripperToTargetCoord", &SobitProJointController::moveGripperToTargetCoord, "move Gripper To Target Coordinate",
            pybind11::arg( "target_pos_x" ),
            pybind11::arg( "target_pos_y" ),
            pybind11::arg( "target_pos_z" ),
            pybind11::arg( "shift_x" ),
            pybind11::arg( "shift_y" ),
            pybind11::arg( "shift_z" ) )
        .def( "moveGripperToTargetTF", &SobitProJointController::moveGripperToTargetTF, "move Gripper To Target TF",
            pybind11::arg( "target_name" ),
            pybind11::arg( "shift_x" ),
            pybind11::arg( "shift_y" ),
            pybind11::arg( "shift_z" ) )
        .def( "moveGripperToPlaceCoord", &SobitProJointController::moveGripperToPlaceCoord, "move Gripper To Placeable Position Coordinate",
            pybind11::arg( "target_pos_x" ),
            pybind11::arg( "target_pos_y" ),
            pybind11::arg( "target_pos_z" ),
            pybind11::arg( "shift_x" ),
            pybind11::arg( "shift_y" ),
            pybind11::arg( "shift_z" ) )
        .def( "moveGripperToPlaceTF", &SobitProJointController::moveGripperToPlaceTF, "move Gripper To Placeable Position TF",
            pybind11::arg( "target_name" ),
            pybind11::arg( "shift_x" ),
            pybind11::arg( "shift_y" ),
            pybind11::arg( "shift_z" ) )
        .def( "graspDecision", &SobitProJointController::graspDecision, "grasp Decision" );

    pybind11::class_<SobitProWheelController>( m, "SobitProWheelController" )
        .def( pybind11::init<const std::string&>( ) )
        .def( "controlWheelLinear", &SobitProWheelController::controlWheelLinear, "control Wheel Linear", 
            pybind11::arg( "distance_x" ),
            pybind11::arg( "distance_y" ) )
        .def( "controlWheelRotateRad", &SobitProWheelController::controlWheelRotateRad, "control Wheel Rotate Rad", 
            pybind11::arg( "angle_rad" ) )
        .def( "controlWheelRotateDeg", &SobitProWheelController::controlWheelRotateDeg,"control Wheel Rotate Deg",
            pybind11::arg( "angle_deg" ) );

}