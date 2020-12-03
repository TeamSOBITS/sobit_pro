#ifndef SOBIT_PRO_MOTOR_DRIVER_H_
#define SOBIT_PRO_MOTOR_DRIVER_H_

#include <stdio.h> // printf etc.
#include <dynamixel_sdk/dynamixel_sdk.h> // Uses Dynamixel SDK library

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_PRESENT_POSITION         132
#define ADDR_X_PRESENT_VELOCITY         128

// Limit values (XM430-W210-R)
#define LIMIT_X_MAX_VELOCITY            330

// Data Byte Length
#define LEN_X_GOAL_POSITION             4
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_PRESENT_POSITION          4
#define LEN_X_PRESENT_VELOCITY          4

// Dynamixel protocol version 2.0
#define PROTOCOL_VERSION                2.0

// Default setting
#define WHEEL_F_R                       1              // Dynamixel ID:1
#define WHEEL_F_L                       2              // Dynamixel ID:2
#define WHEEL_B_R                       3              // Dynamixel ID:3
#define WHEEL_B_L                       4              // Dynamixel ID:4

#define STEER_F_R                       5              // Dynamixel ID:5
#define STEER_F_L                       6              // Dynamixel ID:6
#define STEER_B_R                       7              // Dynamixel ID:7
#define STEER_B_L                       8              // Dynamixel ID:8

#define BAUDRATE                        3000000        // baud rate of Dynamixel
#define DEVICENAME                      "/dev/input/wheel"

#define TORQUE_ENABLE                   1              // Value for enabling the torque
#define TORQUE_DISABLE                  0              // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     20             // Dynamixel moving status threshold  // old param : 10

class SobitProMotorDriver{
  public:
    SobitProMotorDriver();
    ~SobitProMotorDriver();
    bool init(void);
    void closeDynamixel(void);
    bool setTorque(uint8_t id, bool onoff);
    bool setProfileAcceleration(uint8_t id, uint32_t value);
    bool setProfileVelocity(uint8_t id, uint32_t value);
    bool controlSteers(int32_t *value);
    bool controlWheels(int32_t *value);
    bool addPresentParam(void);
    int32_t feedbackSteer(uint8_t);
    int32_t feedbackWheel(uint8_t);

  private:
    uint32_t baudrate_;
    float  protocol_version_;

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux
    dynamixel::PortHandler *portHandler_;

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler_;

    // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite *groupSyncWritePosition_;

    // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;

    // Initialize GroupSyncRead instance
    dynamixel::GroupSyncRead *groupSyncReadPosition_;

    // Initialize GroupSyncRead instance
    dynamixel::GroupSyncRead *groupSyncReadVelocity_;
};

#endif // SOBIT_PRO_MOTOR_DRIVER_H_
