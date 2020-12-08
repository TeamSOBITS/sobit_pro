#ifndef _SOBIT_PRO_LIBRARY_WHEEL_CONTROLLER_H_
#define _SOBIT_PRO_LIBRARY_WHEEL_CONTROLLER_H_

#include <ros/ros.h>
#include <cmath>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sobit_pro_library/sobit_pro_library.h>

namespace sobit {
    class SobitProWheelController  : private ROSCommonNode {
        protected :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_cmd_vel_;
            ros::Subscriber sub_odom_;
            nav_msgs::Odometry curt_odom_;

            void checkPublishersConnection ( const ros::Publisher& pub );
            void callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg );
            double geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat );
            double rad2Deg ( const double rad );
            double deg2Rad ( const double deg );
        public :
            SobitProWheelController( const std::string &name );
            SobitProWheelController( );
            bool controlWheelLinear( const double distance_x, const double distance_y );
            bool controlWheelRotateRad( const double angle_rad );
            bool controlWheelRotateDeg( const double angle_deg );
    };
}

inline void sobit::SobitProWheelController::checkPublishersConnection ( const ros::Publisher& pub ) {
    ros::Rate loop_rate(10);
    while ( pub.getNumSubscribers()	== 0 && ros::ok() ) {
        try { loop_rate.sleep(); }
        catch ( const std::exception& ex ) { break; }
    }
    return; 
}

inline void sobit::SobitProWheelController::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { curt_odom_ = *odom_msg; }

inline double sobit::SobitProWheelController::geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat ) {
    tf::Quaternion quat;
    double roll, pitch, yaw;
    quaternionMsgToTF(geometry_quat, quat);
    quat.normalize();
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

inline double sobit::SobitProWheelController::rad2Deg ( const double rad ) { return rad * 180.0 / M_PI;  }

inline double sobit::SobitProWheelController::deg2Rad ( const double deg ) { return deg * M_PI / 180.0; }

#endif /* _SOBIT_PRO_LIBRARY_WHEEL_CONTROLLER_H_ */