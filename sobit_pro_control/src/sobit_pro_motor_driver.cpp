#include "sobit_pro_control/sobit_pro_motor_driver.hpp"

// Constructor
SobitProMotorDriver::SobitProMotorDriver():
baudrate_(BAUDRATE),
protocol_version_(PROTOCOL_VERSION){
}

// Destructor
SobitProMotorDriver::~SobitProMotorDriver(){
    closeDynamixel();
}

bool SobitProMotorDriver::init(){
    portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if( portHandler_->openPort() ) std::cout << "Succeeded to open the port!" << std::endl;
    else{
        std::cout << "Failed to open the port!" << std::endl;
        return false;
    }

    // Set baudrate
    if( portHandler_->setBaudRate(baudrate_) ) std::cout << "Succeeded to change the baudrate!" << std::endl;
    else{
        std::cout << "Failed to change the baudrate!" << std::endl;
        return false;
    }

    // Enable Dynamixel Torque
    setTorque(STEER_F_L, TORQUE_ENABLE); setTorque(STEER_F_R, TORQUE_ENABLE); setTorque(STEER_B_L, TORQUE_ENABLE); setTorque(STEER_B_R, TORQUE_ENABLE);
    setTorque(WHEEL_F_L, TORQUE_ENABLE); setTorque(WHEEL_F_R, TORQUE_ENABLE); setTorque(WHEEL_B_L, TORQUE_ENABLE); setTorque(WHEEL_B_R, TORQUE_ENABLE);

    // Initialize GroupSyncWrite and GroupSyncRead instance
    groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);
    groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
    groupSyncReadPosition_  = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    groupSyncReadVelocity_  = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);


    return true;
}

bool SobitProMotorDriver::setTorque(uint8_t id, uint8_t is_enable){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, is_enable, &dxl_error);
    if( dxl_comm_result != COMM_SUCCESS ) packetHandler_->getTxRxResult(dxl_comm_result);
    else if( dxl_error ) packetHandler_->getRxPacketError(dxl_error);
    else{
        if( is_enable ) std::cout << "Dynamixel ID:" << id << "has been successfully connected!" << std::endl;
        else            std::cout << "Dynamixel ID:" << id << "has been successfully disconnected!" << std::endl;
    }


    return dxl_comm_result;
}

void SobitProMotorDriver::closeDynamixel(void){
    // Disable Dynamixel Torque
    setTorque(WHEEL_F_L, TORQUE_DISABLE); setTorque(WHEEL_F_R, TORQUE_DISABLE); setTorque(WHEEL_B_L, TORQUE_DISABLE); setTorque(WHEEL_B_R, TORQUE_DISABLE);
    setTorque(STEER_F_L, TORQUE_DISABLE); setTorque(STEER_F_R, TORQUE_DISABLE); setTorque(STEER_B_L, TORQUE_DISABLE); setTorque(STEER_B_R, TORQUE_DISABLE);

    // Close port
    portHandler_->closePort();
}

bool SobitProMotorDriver::controlSteers(int32_t *value){
    bool dxl_addparam_result_;
    int8_t dxl_comm_result_;


    // Add parameter storage for Dynamixel goal position
    dxl_addparam_result_ = groupSyncWritePosition_->addParam(STEER_F_L, (uint8_t*)&value[1]);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncWritePosition_->addParam(STEER_F_R, (uint8_t*)&value[0]);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncWritePosition_->addParam(STEER_B_L, (uint8_t*)&value[3]);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncWritePosition_->addParam(STEER_B_R, (uint8_t*)&value[2]);
    if( !dxl_addparam_result_ ) return false;


    // SyncWrite goal position value
    dxl_comm_result_ = groupSyncWritePosition_->txPacket();
    if( dxl_comm_result_ != COMM_SUCCESS ){
        packetHandler_->getTxRxResult(dxl_comm_result_);
        return false;
    }

    // Clear syncwrite parameter storage
    groupSyncWritePosition_->clearParam();


    return true;
}

bool SobitProMotorDriver::controlWheels(int32_t *value){
    bool dxl_addparam_result_;
    int8_t dxl_comm_result_;

    // Add parameter storage for Dynamixel goal velocity
    dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_F_L, (uint8_t*)&value[1]);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_F_R, (uint8_t*)&value[0]);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_B_L, (uint8_t*)&value[3]);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_B_R, (uint8_t*)&value[2]);
    if( !dxl_addparam_result_ ) return false;

    // Syncwrite goal velocity value
    dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
    if( dxl_comm_result_ != COMM_SUCCESS ){
        packetHandler_->getTxRxResult(dxl_comm_result_);
        return false;
    }

    // Clear syncwrite parameter storage
    groupSyncWriteVelocity_->clearParam();


    return true;
}

bool SobitProMotorDriver::addPresentParam(){
    // Add parameter storage for Dynamixel present position
    bool dxl_addparam_result_ = false;

    dxl_addparam_result_ = groupSyncReadPosition_->addParam(STEER_F_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPosition_->addParam(STEER_F_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPosition_->addParam(STEER_B_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPosition_->addParam(STEER_B_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPosition_->addParam(WHEEL_F_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPosition_->addParam(WHEEL_F_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPosition_->addParam(WHEEL_B_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPosition_->addParam(WHEEL_B_R);
    if( !dxl_addparam_result_ ) return false;

/*
    // Add parameter storage for Dynamixel present velocity
    dxl_addparam_result_ = groupSyncReadVelocity_->addParam(STEER_F_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVelocity_->addParam(STEER_F_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVelocity_->addParam(STEER_B_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVelocity_->addParam(STEER_B_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVelocity_->addParam(WHEEL_F_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVelocity_->addParam(WHEEL_F_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVelocity_->addParam(WHEEL_B_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVelocity_->addParam(WHEEL_B_L);
    if( !dxl_addparam_result_ ) return false;
*/

    return true;
}

uint32_t SobitProMotorDriver::feedbackSteer(uint8_t id){
    int dxl_comm_result_ = COMM_TX_FAIL;
    bool dxl_getdata_result_ = false;
    uint8_t dxl_error_ = 0;
    uint32_t steer_present_angle_;

    // SyncRead current position
    dxl_comm_result_ = groupSyncReadPosition_->txRxPacket();
    if( dxl_comm_result_ != COMM_SUCCESS ) std::cout << packetHandler_->getTxRxResult(dxl_comm_result_) << std::endl;
    else if( groupSyncReadPosition_->getError(id, &dxl_error_) ) std::cout << "[ID:" << id << "] " << packetHandler_->getRxPacketError(dxl_error_) << std::endl;

    // Check if GroupSyncRead data of Dynamixel is available
    dxl_getdata_result_ = groupSyncReadPosition_->isAvailable(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if( !dxl_getdata_result_ ){
        std::cout << "[ID:" << id << "] groupSyncRead getData() failed" << std::endl;
        return 0;
    }

    // Get Dynamixel current position value
    steer_present_angle_ = groupSyncReadPosition_->getData(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);


    return steer_present_angle_;
}

uint32_t SobitProMotorDriver::feedbackWheel(uint8_t id){
    int dxl_comm_result_ = COMM_TX_FAIL;
    bool dxl_getdata_result_ = false;
    uint8_t dxl_error_ = 0;
    uint32_t wheel_present_angle_;

    // SyncRead current position
    dxl_comm_result_ = groupSyncReadPosition_->txRxPacket();
    if( dxl_comm_result_ != COMM_SUCCESS ) std::cout << packetHandler_->getTxRxResult(dxl_comm_result_) << std::endl;
    else if( groupSyncReadPosition_->getError(id, &dxl_error_) ) std::cout << "[ID:" << id << "] " << packetHandler_->getRxPacketError(dxl_error_) << std::endl;

    // Check if GroupSyncRead data of Dynamixel is available
    dxl_getdata_result_ = groupSyncReadPosition_->isAvailable(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if( !dxl_getdata_result_ ){
        std::cout << "[ID:" << id << "] groupSyncRead getData() failed" << std::endl;
        return 0;
    }

    // Get Dynamixel present position value
    wheel_present_angle_ = groupSyncReadPosition_->getData(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);


    return wheel_present_angle_;
}

/*
uint32_t SobitProMotorDriver::feedbackSteer(uint8_t id){
    int dxl_comm_result_ = COMM_TX_FAIL;
    bool dxl_getdata_result_ = false;
    uint8_t dxl_error_ = 0;
    uint32_t steer_present_vel_;

    // Syncread present velocity
    dxl_comm_result_ = groupSyncReadVelocity_->txRxPacket();

    if( dxl_comm_result_ != COMM_SUCCESS ) std::cout << packetHandler_->getTxRxResult(dxl_comm_result_) << std::endl;
    else if( groupSyncReadVelocity_->getError(id, &dxl_error_) ) std::cout << "[ID:" << id << "] " << packetHandler_->getRxPacketError(dxl_error_) << std::endl;

    // Check if groupsyncread data of Dynamixel is available
    dxl_getdata_result_ = groupSyncReadVelocity_->isAvailable(id, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    if( !dxl_getdata_result_ ){
        std::cout << "[ID:" << id << "] groupSyncRead getdata failed" << std::endl;
        return 0;
    }

    // Get Dynamixel present velocity value
    steer_present_vel_ = groupSyncReadVelocity_->getData(id, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    return steer_present_vel_;
}
*/

/*
uint32_t SobitProMotorDriver::feedbackWheel(uint8_t id){
    int dxl_comm_result_ = COMM_TX_FAIL;
    bool dxl_getdata_result_ = false;
    uint8_t dxl_error_ = 0;
    uint32_t wheel_present_vel_;

    // Syncread present velocity
    dxl_comm_result_ = groupSyncReadVelocity_->txRxPacket();

    if( dxl_comm_result_ != COMM_SUCCESS ) std::cout << packetHandler_->getTxRxResult(dxl_comm_result_) << std::endl;
    else if( groupSyncReadVelocity_->getError(id, &dxl_error_) ) std::cout << "[ID:" << id << "] " << packetHandler_->getRxPacketError(dxl_error_) << std::endl;

    // Check if groupsyncread data of Dynamixel is available
    dxl_getdata_result_ = groupSyncReadVelocity_->isAvailable(id, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    if( !dxl_getdata_result_ ){
        std::cout << "[ID:" << id << "] groupSyncRead getdata failed" << std::endl;
        return 0;
    }

    // Get Dynamixel present velocity value
    wheel_present_vel_ = groupSyncReadVelocity_->getData(id, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    return wheel_present_vel_;
}
*/
