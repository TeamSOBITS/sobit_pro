#include "sobit_pro_control/sobit_pro_main.hpp"
#include "sobit_pro_control/sobit_pro_control.hpp"
// #include "sobit_pro_control/sobit_pro_motor_driver.hpp"
#include "sobit_pro_control/sobit_pro_odometry.hpp"

// Create the instance
SobitProControl     sobit_pro_control;
// SobitProMotorDriver sobit_pro_motor_driver;
SobitProOdometry    sobit_pro_odometry;

// Twist callback
void SobitProMain::callback(const geometry_msgs::msg::Twist::SharedPtr vel_twist)
{
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
    for (size_t i = 0; i < joint_info->name.size(); ++i) {
        joints_pos[joint_info->name[i]] = joint_info->position[i];
        joints_vel[joint_info->name[i]] = joint_info->velocity[i];
    }
}



// Start up sound
bool SobitProMain::start_up_sound()
{
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
    std::string pack_path = ament_index_cpp::get_package_share_directory("sobit_pro_sim_control");
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
    bool is_sound = false;

    // パッケージパス取得
    std::string package_path = ament_index_cpp::get_package_share_directory("sobit_pro_sim_control");
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
    auto node = std::make_shared<rclcpp::Node>("sobit_pro_control_wheel");
    // [SIM] Wait for the joint_states to be published
    while (rclcpp::ok() && (joints_pos.empty() || joints_vel.empty())) {
        rclcpp::spin_some(node);
    }

    // [SIM] Set the initial position of the wheel
    wheel_fl_init_pos = SobitProMain::getJointPos("wheel_f_l_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
    wheel_fr_init_pos = SobitProMain::getJointPos("wheel_f_r_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
    wheel_bl_init_pos = SobitProMain::getJointPos("wheel_b_l_drive_joint") * 1024. / (M_PI / 2.) + 2048.;
    wheel_br_init_pos = SobitProMain::getJointPos("wheel_b_r_drive_joint") * 1024. / (M_PI / 2.) + 2048.;

    // Get the robot namespace (ROS2ではノードの名前空間として取得)
    std::string robot_name = (std::strcmp(this->get_namespace(), "/") != 0)
                            ? std::string(this->get_namespace()).substr(1) + "/"
                            : "";

    rclcpp::Rate rate(50);

    trajectory_msgs::msg::JointTrajectory steer_joint_trajectory;
    // rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr steer_action_client_;

    std_msgs::msg::Float64MultiArray wheel_joint_vel;

    while (rclcpp::ok()) {
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
            rclcpp::spin_some(node);
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

        wheel_fl_init_pos = wheel_fl_curt_pos;
        wheel_fr_init_pos = wheel_fr_curt_pos;
        wheel_bl_init_pos = wheel_bl_curt_pos;
        wheel_br_init_pos = wheel_br_curt_pos;

        rate.sleep();
    }
}


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
    sobit_pro_main->start_up_sound();

    // Control wheel (main loop)
    sobit_pro_main->control_wheel();

    // Shut down sound
    sobit_pro_main->shut_down_sound();

    // Shut down motor
    // sobit_pro_motor_driver.closeDynamixel();

    return 0;
}
