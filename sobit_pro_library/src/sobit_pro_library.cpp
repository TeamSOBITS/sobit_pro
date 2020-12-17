#include <sobit_pro_library/sobit_pro_library.h>
#include <sobit_pro_library/sobit_pro_joint_controller.h>
#include <sobit_pro_library/sobit_pro_wheel_controller.hpp>

using namespace sobit;
namespace py = pybind11;


PYBIND11_MODULE(sobit_pro_module, m) {
  py::enum_<Joint>(m, "Joint")
      .value("ARM1_1_JOINT", Joint::ARM1_1_JOINT)
      .value("ARM1_2_JOINT", Joint::ARM1_2_JOINT)
      .value("ARM2_1_JOINT", Joint::ARM2_1_JOINT)
      .value("ARM2_2_JOINT", Joint::ARM2_2_JOINT)
      .value("ARM3_JOINT", Joint::ARM3_JOINT)
      .value("ARM4_JOINT", Joint::ARM4_JOINT)
      .value("GRIPPER_JOINT", Joint::GRIPPER_JOINT)
      .value("HEAD_CAMERA_PAN_JOINT", Joint::HEAD_CAMERA_PAN_JOINT)
      .value("HEAD_CAMERA_TILT_JOINT", Joint::HEAD_CAMERA_TILT_JOINT)
      .value("JOINT_NUM", Joint::JOINT_NUM)
      .export_values();

  py::class_<SobitProJointController>(m, "SobitProJointController")
      .def(py::init<const std::string&>())
      .def("moveJoint", &SobitProJointController::moveJoint, "move Joint", py::arg("joint_num"), py::arg("rad"), py::arg("sec"), py::arg("is_sleep") = true)
      .def("moveHeadPanTilt",
           &SobitProJointController::moveHeadPanTilt,
           "move Head PanTilt",
           py::arg("head_camera_pan"),
           py::arg("head_camera_tilt"),
           py::arg("sec"),
           py::arg("is_sleep") = true)
      .def("moveArm",
           &SobitProJointController::moveArm,
           "move Arm",
           py::arg("arm1"),
           py::arg("arm2"),
           py::arg("arm3"),
           py::arg("arm4"))
      .def("moveToRegisterdMotion", &SobitProJointController::moveToRegisterdMotion, "move to Registerd Motion", py::arg("pose_name"))
      .def("moveGripperToTarget",
           &SobitProJointController::moveGripperToTarget,
           "move Gripper To Target",
           py::arg("target_name"),
           py::arg("diff_goal_position_x"),
           py::arg("diff_goal_position_y"),
           py::arg("diff_goal_position_z"));

  py::class_<SobitProWheelController>(m, "SobitProWheelController")
      .def(py::init<const std::string&>())
      .def("controlWheelLinear", 
           &SobitProWheelController::controlWheelLinear,
           "control Wheel Linear", 
           py::arg("distance_x"),
           py::arg("distance_y"))
      .def("controlWheelRotateRad", 
           &SobitProWheelController::controlWheelRotateRad,
           "control Wheel Rotate Rad", 
           py::arg("angle_rad"))
      .def("controlWheelRotateDeg", 
           &SobitProWheelController::controlWheelRotateDeg,
           "control Wheel Rotate Deg", 
           py::arg("angle_deg"));

}