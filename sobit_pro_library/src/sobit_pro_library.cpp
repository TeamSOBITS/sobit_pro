#include <sobit_pro_library/sobit_pro_library.h>
#include <sobit_pro_library/sobit_pro_joint_controller.h>
#include <sobit_pro_library/sobit_pro_wheel_controller.hpp>

using namespace sobit_pro;
namespace py = pybind11;


PYBIND11_MODULE( sobit_pro_module, m ) {
    py::enum_<Joint>( m, "Joint" )
        .value( "ARM_SHOULDER_1_TILT_JOINT", Joint::ARM_SHOULDER_1_TILT_JOINT )
        .value( "ARM_SHOULDER_2_TILT_JOINT", Joint::ARM_SHOULDER_2_TILT_JOINT )
        .value( "ARM_ELBOW_UPPER_1_TILT_JOINT", Joint::ARM_ELBOW_UPPER_1_TILT_JOINT )
        .value( "ARM_ELBOW_UPPER_2_TILT_JOINT", Joint::ARM_ELBOW_UPPER_2_TILT_JOINT )
        .value( "ARM_ELBOW_LOWER_TILT_JOINT", Joint::ARM_ELBOW_LOWER_TILT_JOINT )
        .value( "ARM_WRIST_TILT_JOINT", Joint::ARM_WRIST_TILT_JOINT )
        .value( "HAND_JOINT", Joint::HAND_JOINT )
        .value( "HEAD_CAMERA_PAN_JOINT", Joint::HEAD_CAMERA_PAN_JOINT )
        .value( "HEAD_CAMERA_TILT_JOINT", Joint::HEAD_CAMERA_TILT_JOINT )
        .value( "JOINT_NUM", Joint::JOINT_NUM )
        .export_values( );

    py::class_<SobitProJointController>( m, "SobitProJointController" )
        .def( py::init<const std::string&>( ) )
        .def( "moveToPose", &SobitProJointController::moveToPose, "move Pose",
            py::arg( "pose_name" ),
            py::arg( "sec" ) = 5.0 )
        .def( "moveJoint", &SobitProJointController::moveJoint, "move Joint",
            py::arg( "joint_num" ),
            py::arg( "rad" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveHeadPanTilt", &SobitProJointController::moveHeadPanTilt, "move Head PanTilt",
            py::arg( "head_camera_pan" ),
            py::arg( "head_camera_tilt" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveArm", &SobitProJointController::moveArm, "move Arm",
            py::arg( "arm1" ),
            py::arg( "arm2" ),
            py::arg( "arm3" ),
            py::arg( "arm4" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveGripperToTargetCoord", &SobitProJointController::moveGripperToTargetCoord, "move Gripper To Target Coordinate",
            py::arg( "goal_position_x" ),
            py::arg( "goal_position_y" ),
            py::arg( "goal_position_z" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "moveGripperToTargetTF", &SobitProJointController::moveGripperToTargetTF, "move Gripper To Target TF",
            py::arg( "target_name" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "moveGripperToPlaceCoord", &SobitProJointController::moveGripperToPlaceCoord, "move Gripper To Placeable Position Coordinate",
            py::arg( "goal_position_x" ),
            py::arg( "goal_position_y" ),
            py::arg( "goal_position_z" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "moveGripperToPlaceTF", &SobitProJointController::moveGripperToPlaceTF, "move Gripper To Placeable Position TF",
            py::arg( "target_name" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "graspDecision", &SobitProJointController::graspDecision, "grasp Decision" );

    py::class_<SobitProWheelController>( m, "SobitProWheelController" )
        .def( py::init<const std::string&>( ) )
        .def( "controlWheelLinear", &SobitProWheelController::controlWheelLinear, "control Wheel Linear", 
            py::arg( "distance_x" ),
            py::arg( "distance_y" ) )
        .def( "controlWheelRotateRad", &SobitProWheelController::controlWheelRotateRad, "control Wheel Rotate Rad", 
            py::arg( "angle_rad" ) )
        .def( "controlWheelRotateDeg", &SobitProWheelController::controlWheelRotateDeg,"control Wheel Rotate Deg",
            py::arg( "angle_deg" ) );

}