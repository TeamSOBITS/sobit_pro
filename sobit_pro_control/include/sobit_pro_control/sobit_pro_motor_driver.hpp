#ifndef SOBIT_PRO_MOTOR_DRIVER_H_
#define SOBIT_PRO_MOTOR_DRIVER_H_

#include <iostream>
#include <dynamixel_sdk/dynamixel_sdk.h>


class SobitProMotorDriver{
    private:
        // Control table address (Dynamixel X-series)
        const int ADDR_X_TORQUE_ENABLE            = 64;
        const int ADDR_X_GOAL_POSITION            = 116;
        const int ADDR_X_GOAL_VELOCITY            = 104;
        const int ADDR_X_PRESENT_POSITION         = 132;
        const int ADDR_X_PRESENT_VELOCITY         = 128;

        // Limit values (XM430-W210-R)
        const int LIMIT_X_MAX_VELOCITY            = 330;

        // Data Byte Length
        const int LEN_X_GOAL_POSITION             = 4;
        const int LEN_X_GOAL_VELOCITY             = 4;
        const int LEN_X_PRESENT_POSITION          = 4;
        const int LEN_X_PRESENT_VELOCITY          = 4;

        // Dynamixel protocol version 2.0
        const int PROTOCOL_VERSION                = 2.0; // float

        // Default setting
        const int BAUDRATE                        = 3000000; // Baudrate of Dynamixel
        const char *DEVICENAME                    = "/dev/input/dx_lower";

        const int TORQUE_ENABLE                   = 1;       // Value for enabling the torque
        const int TORQUE_DISABLE                  = 0;       // Value for disabling the torque

        uint32_t baudrate_;
        float  protocol_version_;

        dynamixel::PortHandler    *portHandler_;
        dynamixel::PacketHandler  *packetHandler_;

        dynamixel::GroupSyncWrite *groupSyncWritePos_;
        dynamixel::GroupSyncWrite *groupSyncWriteVel_;
        
        dynamixel::GroupSyncRead  *groupSyncReadPos_;
        dynamixel::GroupSyncRead  *groupSyncReadVel_;

    public:
        static constexpr const int WHEEL_F_L = 2; // Dynamixel ID:2
        static constexpr const int WHEEL_F_R = 1; // Dynamixel ID:1
        static constexpr const int WHEEL_B_L = 4; // Dynamixel ID:4
        static constexpr const int WHEEL_B_R = 3; // Dynamixel ID:3

        static constexpr const int STEER_F_L = 6; // Dynamixel ID:6
        static constexpr const int STEER_F_R = 5; // Dynamixel ID:5
        static constexpr const int STEER_B_L = 8; // Dynamixel ID:8
        static constexpr const int STEER_B_R = 7; // Dynamixel ID:7

        static constexpr const int DXL_MOVING_STATUS_THRESHOLD = 20; // Dynamixel moving status threshold  // old param : 10


        SobitProMotorDriver();
        ~SobitProMotorDriver();
        bool init();
        bool setTorque(uint8_t id, uint8_t is_enable);
        void closeDynamixel();
        bool controlSteersPos(int32_t *value);
        bool controlWheelsVel(int32_t *value);
        bool addPresentParam();
        uint32_t feedbackSteerPos(uint8_t);
        uint32_t feedbackWheelPos(uint8_t);
};

#endif // SOBIT_PRO_MOTOR_DRIVER_H_
