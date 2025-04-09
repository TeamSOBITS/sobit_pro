#include "sobit_pro_control/sobit_pro_main.hpp"
#include "sobit_pro_control/sobit_pro_control.hpp"
#include "sobit_pro_control/sobit_pro_odometry.hpp"

// Create the instance
// SobitProControl     sobit_pro_control;
// SobitProMotorDriver sobit_pro_motor_driver;
// SobitProOdometry    sobit_pro_odometry;

namespace sobit_pro
{

SobitProMain::SobitProMain(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("sobit_pro_main", options)
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  this->sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "mobile_base/commands/velocity", qos_profile, std::bind(&SobitProMain::callback, this, std::placeholders::_1));

  this->sub_joint_info_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&SobitProMain::joint_callback, this, std::placeholders::_1));

  this->pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos_profile);
  this->pub_steer_joint_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory_controller/joint_trajectory", qos_profile);
  this->pub_wheel_joint_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "wheel_trajectory_controller/command", qos_profile);
  this->pub_wheels_error_ = this->create_publisher<std_msgs::msg::Bool>(
      "wheels_error", qos_profile);
}

// Twist callback
void SobitProMain::callback(const geometry_msgs::msg::Twist::SharedPtr vel_twist)
{
    SobitProControl     sobit_pro_control;
    SobitProOdometry    sobit_pro_odometry;
    // Translational
    if (((std::fabs(vel_twist->linear.x) > 0.000) || (std::fabs(vel_twist->linear.y) > 0.000)) &&
        (std::fabs(vel_twist->angular.z) <= 0.001))
    {
        motion = SobitProControl::TRANSLATIONAL_MOTION;
        wheels_error.data = false;
        pub_wheels_error_->publish(wheels_error);
    }
    // Rotational
    else if (((std::fabs(vel_twist->linear.x) <= 0.001) && (std::fabs(vel_twist->linear.y) <= 0.001)) &&
             (std::fabs(vel_twist->angular.z) > 0.000))
    {
        motion = SobitProControl::ROTATIONAL_MOTION;
        wheels_error.data = false;
        pub_wheels_error_->publish(wheels_error);
    }
    // Swivel
    else if (((std::fabs(vel_twist->linear.x) > 0.000) || (std::fabs(vel_twist->linear.y) > 0.000)) &&
             (std::fabs(vel_twist->angular.z) > 0.000))
    {
        if (std::fabs(2.0 * std::sqrt(std::pow(vel_twist->linear.x, 2.0) + std::pow(vel_twist->linear.y, 2.0))) >
            std::fabs(vel_twist->angular.z * SobitProControl::TRACK))
        {
            motion = SobitProControl::SWIVEL_MOTION;
        }
        else
        {
            motion = SobitProControl::ROTATIONAL_MOTION;
        }
        wheels_error.data = false;
        pub_wheels_error_->publish(wheels_error);
    }
    // Stop
    else
    {
        motion = SobitProControl::STOP_MOTION;
        wheels_error.data = false;
        pub_wheels_error_->publish(wheels_error);
    }

    sobit_pro_control.getMotion(motion);
    sobit_pro_odometry.getMotion(motion);
    sobit_pro_control.setParams(*vel_twist);
}

void SobitProMain::joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_info)
{
    SobitProControl     sobit_pro_control;
    SobitProOdometry    sobit_pro_odometry;
    for (size_t i = 0; i < joint_info->name.size(); ++i) {
        joints_pos[joint_info->name[i]] = joint_info->position[i];
        joints_vel[joint_info->name[i]] = joint_info->velocity[i];
    }
}



// Start up sound
bool SobitProMain::start_up_sound()
{
    SobitProControl     sobit_pro_control;
    SobitProOdometry    sobit_pro_odometry;
    bool is_sound = false;

    // Generate a random number
    std::random_device rnd;
    std::mt19937 gen(rnd());
    std::uniform_int_distribution<int> distribution(1, 100);
    int rand_sound = distribution(gen);

    // パラメータ取得（デフォルト値: 75）
    int sound_param = this->declare_parameter<int>("sound_param", 75);
    this->get_parameter("sound_param", sound_param);

    // sound の決定
    std::string sound = rand_sound <= sound_param ? "start_up" : "soka_univ_gakuseika";

    // パッケージパス取得
    std::string pack_path = ament_index_cpp::get_package_share_directory("sobit_pro_control");
    std::string sound_path = pack_path + "/mp3/" + sound + ".mp3";

    // ログ出力
    std::cout << std::endl;
    std::cout << "rand_sound: " << rand_sound << std::endl;
    std::cout << "sound_param: " << sound_param << std::endl;
    std::cout << "Start Up: " << sound << ".mp3" << std::endl;
    std::cout << std::endl;

    // サウンド再生
    is_sound = std::system(("mpg321 --quiet " + sound_path).c_str());
    rclcpp::sleep_for(std::chrono::seconds(2));

    if (is_sound) {
        RCLCPP_ERROR(this->get_logger(), "There was an error reproducing the start up sound.");
    }

    return is_sound;
}


// Shut down sound
bool SobitProMain::shut_down_sound()
{
    SobitProControl     sobit_pro_control;
    SobitProOdometry    sobit_pro_odometry;
    bool is_sound = false;

    // パッケージパス取得
    std::string package_path = ament_index_cpp::get_package_share_directory("sobit_pro_control");
    std::string sound_path   = package_path + "/mp3/shut_down.mp3";

    // ログ出力
    std::cout << std::endl;
    std::cout << "Shutdown Sound" << std::endl;

    // サウンド再生
    is_sound = std::system(("mpg321 --quiet " + sound_path).c_str());
    rclcpp::sleep_for(std::chrono::seconds(2));

    if (is_sound) {
        RCLCPP_ERROR(this->get_logger(), "There was an error reproducing the shutdown sound.");
    }

    return is_sound;
}


// Control wheel
void SobitProMain::control_wheel()
{
    std::cout << "aaaaaaaaaaa" << "\n";
    SobitProControl     sobit_pro_control;
    SobitProOdometry    sobit_pro_odometry;
    // auto node = std::make_shared<rclcpp::Node>("sobit_pro_control_wheel");
    // // [SIM] Wait for the joint_states to be published
    // while (rclcpp::ok() && (joints_pos.empty() || joints_vel.empty())) {
    //     rclcpp::spin_some(node);
    // }
    std::cout << "bbbbbbbbbbbbbbb" << "\n";

    // [SIM] Set the initial position of the wheel
    wheel_fl_init_pos = SobitProMain::getJointPos("wheel_f_l_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
    wheel_fr_init_pos = SobitProMain::getJointPos("wheel_f_r_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
    wheel_bl_init_pos = SobitProMain::getJointPos("wheel_b_l_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
    wheel_br_init_pos = SobitProMain::getJointPos("wheel_b_r_drive_joint") * 1024. / (M_PI / 2.) + 2048.;

    // Get the robot namespace (ROS2ではノードの名前空間として取得)
    std::string robot_name = (std::strcmp(this->get_namespace(), "/") != 0)
                            ? std::string(this->get_namespace()).substr(1) + "/"
                            : "";
    
    // Initilize Odometry // [NOT for Isaac Sim!]
    prev_odom.header.stamp            = this->get_clock()->now();
    prev_odom.header.frame_id         = robot_name + "odom";
    prev_odom.child_frame_id          = robot_name + "base_footprint";
    prev_odom.pose.pose.position.x    = 0.0;
    prev_odom.pose.pose.position.y    = 0.0;
    prev_odom.pose.pose.position.z    = 0.0;
    prev_odom.pose.pose.orientation.x = 0.0;
    prev_odom.pose.pose.orientation.y = 0.0;
    prev_odom.pose.pose.orientation.z = 0.0;
    prev_odom.pose.pose.orientation.w = 1.0;
    prev_odom.twist.twist.linear.x    = 0.0;
    prev_odom.twist.twist.linear.y    = 0.0;
    prev_odom.twist.twist.linear.z    = 0.0;
    prev_odom.twist.twist.angular.x   = 0.0;
    prev_odom.twist.twist.angular.y   = 0.0;
    prev_odom.twist.twist.angular.z   = 0.0;

    result_odom.header.stamp            = this->get_clock()->now();
    result_odom.header.frame_id         = robot_name + "odom";
    result_odom.child_frame_id          = robot_name + "base_footprint";
    result_odom.pose.pose.position.x    = 0.0;
    result_odom.pose.pose.position.y    = 0.0;
    result_odom.pose.pose.position.z    = 0.0;
    result_odom.pose.pose.orientation.x = 0.0;
    result_odom.pose.pose.orientation.y = 0.0;
    result_odom.pose.pose.orientation.z = 0.0;
    result_odom.pose.pose.orientation.w = 1.0;
    result_odom.twist.twist.linear.x    = 0.0;
    result_odom.twist.twist.linear.y    = 0.0;
    result_odom.twist.twist.linear.z    = 0.0;
    result_odom.twist.twist.angular.x   = 0.0;
    result_odom.twist.twist.angular.y   = 0.0;
    result_odom.twist.twist.angular.z   = 0.0;

    rclcpp::Rate rate(50);

    trajectory_msgs::msg::JointTrajectory steer_joint_trajectory;
    std_msgs::msg::Float64MultiArray wheel_joint_vel;

    while (rclcpp::ok()) {
        std::cout << "cccccccccccccccccc" << "\n";
        set_steer_pos = sobit_pro_control.setSteerPos();

        steer_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_steer_joint") * 1024. / (M_PI / 2.) + 2048.;
        steer_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_steer_joint") * 1024. / (M_PI / 2.) + 2048.;
        steer_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_steer_joint") * 1024. / (M_PI / 2.) + 2048.;
        steer_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_steer_joint") * 1024. / (M_PI / 2.) + 2048.;

        if ((1024 <= fabs(set_steer_pos[0] - steer_fl_curt_pos))
            || (1024 <= fabs(set_steer_pos[1] - steer_fr_curt_pos))
            || (1024 <= fabs(set_steer_pos[2] - steer_bl_curt_pos))
            || (1024 <= fabs(set_steer_pos[3] - steer_br_curt_pos))) {

            RCLCPP_INFO(this->get_logger(),"Changing the direction of the wheel");

            set_wheel_vel[0] = set_wheel_vel[1] = set_wheel_vel[2] = set_wheel_vel[3] = 0.;
            wheel_joint_vel.data.clear();
            wheel_joint_vel.data.push_back(set_wheel_vel[0] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
            wheel_joint_vel.data.push_back(set_wheel_vel[1] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
            wheel_joint_vel.data.push_back(set_wheel_vel[2] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
            wheel_joint_vel.data.push_back(set_wheel_vel[3] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
            checkPublishersConnection(pub_wheel_joint_);
            pub_wheel_joint_->publish(wheel_joint_vel);

            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        steer_joint_trajectory.joint_names.clear();
        steer_joint_trajectory.points.clear();

        setPosJointTrajectory("wheel_f_l_steer_joint", (set_steer_pos[0] - 2048.) * (M_PI / 2. / 1024.), 0.5, &steer_joint_trajectory);
        addPosJointTrajectory("wheel_f_r_steer_joint", (set_steer_pos[1] - 2048.) * (M_PI / 2. / 1024.), 0.5, &steer_joint_trajectory);
        addPosJointTrajectory("wheel_b_l_steer_joint", (set_steer_pos[2] - 2048.) * (M_PI / 2. / 1024.), 0.5, &steer_joint_trajectory);
        addPosJointTrajectory("wheel_b_r_steer_joint", (set_steer_pos[3] - 2048.) * (M_PI / 2. / 1024.), 0.5, &steer_joint_trajectory);

        checkPublishersConnection(pub_steer_joint_);
        pub_steer_joint_->publish(steer_joint_trajectory);

        do {
            // rclcpp::spin_some(node);
            steer_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_steer_joint") * 1024. / (M_PI / 2.) + 2048.;
            steer_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_steer_joint") * 1024. / (M_PI / 2.) + 2048.;
            steer_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_steer_joint") * 1024. / (M_PI / 2.) + 2048.;
            steer_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_steer_joint") * 1024. / (M_PI / 2.) + 2048.;
        } while ((SobitProControl::DXL_MOVING_STATUS_THRESHOLD < fabs(set_steer_pos[0] - steer_fl_curt_pos)) &&
                 (SobitProControl::DXL_MOVING_STATUS_THRESHOLD < fabs(set_steer_pos[1] - steer_fr_curt_pos)) &&
                 (SobitProControl::DXL_MOVING_STATUS_THRESHOLD < fabs(set_steer_pos[2] - steer_bl_curt_pos)) &&
                 (SobitProControl::DXL_MOVING_STATUS_THRESHOLD < fabs(set_steer_pos[3] - steer_br_curt_pos)));

        set_wheel_vel = sobit_pro_control.setWheelVel();

        wheel_joint_vel.data.clear();
        wheel_joint_vel.data.push_back(set_wheel_vel[0] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
        wheel_joint_vel.data.push_back(set_wheel_vel[1] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
        wheel_joint_vel.data.push_back(set_wheel_vel[2] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
        wheel_joint_vel.data.push_back(set_wheel_vel[3] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
        checkPublishersConnection(pub_wheel_joint_);
        pub_wheel_joint_->publish(wheel_joint_vel);

        wheel_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
        wheel_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
        wheel_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
        wheel_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
        
        rclcpp::Time tmp_time = prev_odom.header.stamp;
        // Calculate Odometry based on motion mode (check!) // [NOT for Isaac Sim!]
        sobit_pro_odometry.odom(steer_fl_curt_pos, steer_fr_curt_pos,
                                steer_bl_curt_pos, steer_br_curt_pos,
                                wheel_fl_curt_pos, wheel_fr_curt_pos,
                                wheel_bl_curt_pos, wheel_br_curt_pos,
                                wheel_fl_init_pos, wheel_fr_init_pos,
                                wheel_bl_init_pos, wheel_br_init_pos,
                                prev_motion,
                                prev_odom, result_odom,
                                tmp_time);

        wheel_fl_init_pos = wheel_fl_curt_pos;
        wheel_fr_init_pos = wheel_fr_curt_pos;
        wheel_bl_init_pos = wheel_bl_curt_pos;
        wheel_br_init_pos = wheel_br_curt_pos;

        // Update odom for next loop calculation // [NOT for Isaac Sim!]
        prev_odom.header.stamp            = result_odom.header.stamp;
        prev_odom.pose.pose.position.x    = result_odom.pose.pose.position.x;
        prev_odom.pose.pose.position.y    = result_odom.pose.pose.position.y;
        prev_odom.pose.pose.position.z    = result_odom.pose.pose.position.z;
        prev_odom.pose.pose.orientation.x = result_odom.pose.pose.orientation.x;
        prev_odom.pose.pose.orientation.y = result_odom.pose.pose.orientation.y;
        prev_odom.pose.pose.orientation.z = result_odom.pose.pose.orientation.z;
        prev_odom.pose.pose.orientation.w = result_odom.pose.pose.orientation.w;
        prev_odom.twist.twist.linear.x    = result_odom.twist.twist.linear.x;
        prev_odom.twist.twist.linear.y    = result_odom.twist.twist.linear.y;
        prev_odom.twist.twist.linear.z    = result_odom.twist.twist.linear.z;
        prev_odom.twist.twist.angular.x   = result_odom.twist.twist.angular.x;
        prev_odom.twist.twist.angular.y   = result_odom.twist.twist.angular.y;
        prev_odom.twist.twist.angular.z   = result_odom.twist.twist.angular.z;

        result_odom.header.stamp = this->get_clock()->now();

        pub_odometry_->publish(result_odom);

        std::cout << "ああ、数値を出力！" << "\n";

        rate.sleep();
    }
}

} // namespace sobit_pro


// Bring Up SOBIT PRO main function

int main(int argc, char **argv){
    // ros::init(argc, argv, "sobit_pro_control");
    rclcpp::init(argc, argv);
    auto sobit_pro_main = std::make_shared<SobitProMain>();

    // Initialize SobitProMain class
    // SobitProMain sobit_pro_main;

    // Start up motor
    // sobit_pro_motor_driver.init();
    // sobit_pro_motor_driver.addPresentParam();

    // Start up sound
    // sobit_pro_main->start_up_sound();

    // Control wheel (main loop)
    sobit_pro_main->control_wheel();

    // Shut down sound
    // sobit_pro_main->shut_down_sound();

    // Shut down motor
    // sobit_pro_motor_driver.closeDynamixel();

    return 0;
}

// int main(int argc, char **argv){
//     rclcpp::init(argc, argv);

//     auto node = rclcpp::Node::make_shared("hello");
//     RCLCPP_INFO(node->get_logger(), "Hello, ROS2 world!");

//     rclcpp::shutdown();
//     return 0;
// }
