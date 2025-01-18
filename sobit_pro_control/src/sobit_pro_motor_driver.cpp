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
    if( portHandler_->openPort() ) std::cout << "Succeeded in opening the port!" << std::endl;
    else{
        std::cout << "Failed to open the port!" << std::endl;
        return false;
    }

    // Set baudrate
    if( portHandler_->setBaudRate(baudrate_) ) std::cout << "Succeeded in changing the baudrate!" << std::endl;
    else{
        std::cout << "Failed to change the baudrate!" << std::endl;
        return false;
    }

    // Enable Dynamixel Torque
    setTorque(STEER_F_L, TORQUE_ENABLE); setTorque(STEER_F_R, TORQUE_ENABLE); setTorque(STEER_B_L, TORQUE_ENABLE); setTorque(STEER_B_R, TORQUE_ENABLE);
    setTorque(WHEEL_F_L, TORQUE_ENABLE); setTorque(WHEEL_F_R, TORQUE_ENABLE); setTorque(WHEEL_B_L, TORQUE_ENABLE); setTorque(WHEEL_B_R, TORQUE_ENABLE);

    // Initialize GroupSyncWrite and GroupSyncRead instance
    groupSyncWritePos_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION   , LEN_X_GOAL_POSITION);
    groupSyncWriteVel_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY   , LEN_X_GOAL_VELOCITY);
    groupSyncReadPos_  = new dynamixel::GroupSyncRead (portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    groupSyncReadVel_  = new dynamixel::GroupSyncRead (portHandler_, packetHandler_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);


    return true;
}

bool SobitProMotorDriver::setTorque(uint8_t id, uint8_t is_enable){
    uint8_t dxl_error   = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, is_enable, &dxl_error);
    if( dxl_comm_result != COMM_SUCCESS ) packetHandler_->getTxRxResult(dxl_comm_result);
    else if( dxl_error ) packetHandler_->getRxPacketError(dxl_error);
    else{
        if( is_enable ) std::cout << "Dynamixel ID:" << int(id) << " has been successfully connected!" << std::endl;
        else            std::cout << "Dynamixel ID:" << int(id) << " has been successfully disconnected!" << std::endl;
    }


    return dxl_comm_result;
}

void SobitProMotorDriver::closeDynamixel(){
    // Disable Dynamixel Torque
    setTorque(WHEEL_F_L, TORQUE_DISABLE); setTorque(WHEEL_F_R, TORQUE_DISABLE); setTorque(WHEEL_B_L, TORQUE_DISABLE); setTorque(WHEEL_B_R, TORQUE_DISABLE);
    setTorque(STEER_F_L, TORQUE_DISABLE); setTorque(STEER_F_R, TORQUE_DISABLE); setTorque(STEER_B_L, TORQUE_DISABLE); setTorque(STEER_B_R, TORQUE_DISABLE);

    // Close port
    portHandler_->closePort();
}

bool SobitProMotorDriver::controlSteersPos(int32_t *value){
    bool dxl_addparam_result_;
    int8_t dxl_comm_result_;
    uint8_t value_data_byte[4] = {0, };


    // Add parameter storage for Dynamixel goal position
    // dxl_addparam_result_ = groupSyncWritePos_->addParam(STEER_F_L, (uint8_t*)&value[1]);
    value_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[1]));
    value_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[1]));
    value_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[1]));
    value_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[1]));
    dxl_addparam_result_ = groupSyncWritePos_->addParam(STEER_F_L, (uint8_t*)&value_data_byte);
    if( !dxl_addparam_result_ ) return false;

    // dxl_addparam_result_ = groupSyncWritePos_->addParam(STEER_F_R, (uint8_t*)&value[0]);
    value_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[0]));
    value_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[0]));
    value_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[0]));
    value_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[0]));
    dxl_addparam_result_ = groupSyncWritePos_->addParam(STEER_F_R, (uint8_t*)&value_data_byte);
    if( !dxl_addparam_result_ ) return false;

    // dxl_addparam_result_ = groupSyncWritePos_->addParam(STEER_B_L, (uint8_t*)&value[3]);
    value_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[3]));
    value_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[3]));
    value_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[3]));
    value_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[3]));
    dxl_addparam_result_ = groupSyncWritePos_->addParam(STEER_B_L, (uint8_t*)&value_data_byte);
    if( !dxl_addparam_result_ ) return false;

    // dxl_addparam_result_ = groupSyncWritePos_->addParam(STEER_B_R, (uint8_t*)&value[2]);
    value_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[2]));
    value_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[2]));
    value_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[2]));
    value_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[2]));
    dxl_addparam_result_ = groupSyncWritePos_->addParam(STEER_B_R, (uint8_t*)&value_data_byte);
    if( !dxl_addparam_result_ ) return false;

    // SyncWrite goal position value
    dxl_comm_result_ = groupSyncWritePos_->txPacket();
    if( dxl_comm_result_ != COMM_SUCCESS ){
        packetHandler_->getTxRxResult(dxl_comm_result_);
        return false;
    }

    // Clear syncwrite parameter storage
    groupSyncWritePos_->clearParam();


    return true;
}

bool SobitProMotorDriver::controlWheelsVel(int32_t *value){
    bool dxl_addparam_result_;
    int8_t dxl_comm_result_;
    uint8_t value_data_byte[4] = {0, };

    // Add parameter storage for Dynamixel goal velocity
    // dxl_addparam_result_ = groupSyncWriteVel_->addParam(WHEEL_F_L, (uint8_t*)&value[1]);
    value_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[1]));
    value_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[1]));
    value_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[1]));
    value_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[1]));
    dxl_addparam_result_ = groupSyncWriteVel_->addParam(WHEEL_F_L, (uint8_t*)&value_data_byte);
    if( !dxl_addparam_result_ ) return false;

    // dxl_addparam_result_ = groupSyncWriteVel_->addParam(WHEEL_F_R, (uint8_t*)&value[0]);
    value_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[0]));
    value_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[0]));
    value_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[0]));
    value_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[0]));
    dxl_addparam_result_ = groupSyncWriteVel_->addParam(WHEEL_F_R, (uint8_t*)&value_data_byte);
    if( !dxl_addparam_result_ ) return false;

    // dxl_addparam_result_ = groupSyncWriteVel_->addParam(WHEEL_B_L, (uint8_t*)&value[3]);
    value_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[3]));
    value_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[3]));
    value_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[3]));
    value_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[3]));
    dxl_addparam_result_ = groupSyncWriteVel_->addParam(WHEEL_B_L, (uint8_t*)&value_data_byte);
    if( !dxl_addparam_result_ ) return false;

    // dxl_addparam_result_ = groupSyncWriteVel_->addParam(WHEEL_B_R, (uint8_t*)&value[2]);
    value_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[2]));
    value_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[2]));
    value_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[2]));
    value_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[2]));
    dxl_addparam_result_ = groupSyncWriteVel_->addParam(WHEEL_B_R, (uint8_t*)&value_data_byte);
    if( !dxl_addparam_result_ ) return false;

    // Syncwrite goal velocity value
    dxl_comm_result_ = groupSyncWriteVel_->txPacket();
    if( dxl_comm_result_ != COMM_SUCCESS ){
        packetHandler_->getTxRxResult(dxl_comm_result_);
        return false;
    }

    // Clear syncwrite parameter storage
    groupSyncWriteVel_->clearParam();


    return true;
}

bool SobitProMotorDriver::addPresentParam(){
    // Add parameter storage for Dynamixel present position
    bool dxl_addparam_result_ = false;

    dxl_addparam_result_ = groupSyncReadPos_->addParam(STEER_F_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPos_->addParam(STEER_F_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPos_->addParam(STEER_B_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPos_->addParam(STEER_B_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPos_->addParam(WHEEL_F_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPos_->addParam(WHEEL_F_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPos_->addParam(WHEEL_B_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadPos_->addParam(WHEEL_B_R);
    if( !dxl_addparam_result_ ) return false;

/*
    // Add parameter storage for Dynamixel present velocity
    dxl_addparam_result_ = groupSyncReadVel_->addParam(STEER_F_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVel_->addParam(STEER_F_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVel_->addParam(STEER_B_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVel_->addParam(STEER_B_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVel_->addParam(WHEEL_F_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVel_->addParam(WHEEL_F_R);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVel_->addParam(WHEEL_B_L);
    if( !dxl_addparam_result_ ) return false;

    dxl_addparam_result_ = groupSyncReadVel_->addParam(WHEEL_B_R);
    if( !dxl_addparam_result_ ) return false;
*/

    return true;
}

uint32_t SobitProMotorDriver::feedbackSteerPos(uint8_t id){
    int dxl_comm_result_     = COMM_TX_FAIL;
    bool dxl_getdata_result_ = false;
    uint8_t dxl_error_       = 0;
    uint32_t steer_curr_pos_;

    // SyncRead current position
    dxl_comm_result_ = groupSyncReadPos_->txRxPacket();
    if( dxl_comm_result_ != COMM_SUCCESS ) std::cout << packetHandler_->getTxRxResult(dxl_comm_result_) << std::endl;
    else if( groupSyncReadPos_->getError(id, &dxl_error_) ) std::cout << "[ID:" << int(id) << "] " << packetHandler_->getRxPacketError(dxl_error_) << std::endl;

    // Check if GroupSyncRead data of Dynamixel is available
    dxl_getdata_result_ = groupSyncReadPos_->isAvailable(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if( !dxl_getdata_result_ ){
        std::cout << "[ID:" << int(id) << "] groupSyncRead getData() failed" << std::endl;
        return 0;
    }

    // Get Dynamixel current position value
    steer_curr_pos_ = groupSyncReadPos_->getData(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);


    return steer_curr_pos_;
}

uint32_t SobitProMotorDriver::feedbackWheelPos(uint8_t id){
    int dxl_comm_result_     = COMM_TX_FAIL;
    bool dxl_getdata_result_ = false;
    uint8_t dxl_error_       = 0;
    uint32_t wheel_curr_pos;

    // SyncRead current position
    dxl_comm_result_ = groupSyncReadPos_->txRxPacket();
    if( dxl_comm_result_ != COMM_SUCCESS ) std::cout << packetHandler_->getTxRxResult(dxl_comm_result_) << std::endl;
    else if( groupSyncReadPos_->getError(id, &dxl_error_) ) std::cout << "[ID:" << int(id) << "] " << packetHandler_->getRxPacketError(dxl_error_) << std::endl;

    // Check if GroupSyncRead data of Dynamixel is available
    dxl_getdata_result_ = groupSyncReadPos_->isAvailable(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if( !dxl_getdata_result_ ){
        std::cout << "[ID:" << int(id) << "] groupSyncRead getData() failed" << std::endl;
        return 0;
    }

    // Get Dynamixel present position value
    wheel_curr_pos = groupSyncReadPos_->getData(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);


    return wheel_curr_pos;
}

/*
uint32_t SobitProMotorDriver::feedbackSteerVel(uint8_t id){
    int dxl_comm_result_     = COMM_TX_FAIL;
    bool dxl_getdata_result_ = false;
    uint8_t dxl_error_       = 0;
    uint32_t steer_curr_vel_;

    // Syncread present velocity
    dxl_comm_result_ = groupSyncReadVel_->txRxPacket();

    if( dxl_comm_result_ != COMM_SUCCESS ) std::cout << packetHandler_->getTxRxResult(dxl_comm_result_) << std::endl;
    else if( groupSyncReadVel_->getError(id, &dxl_error_) ) std::cout << "[ID:" << int(id) << "] " << packetHandler_->getRxPacketError(dxl_error_) << std::endl;

    // Check if groupsyncread data of Dynamixel is available
    dxl_getdata_result_ = groupSyncReadVel_->isAvailable(id, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    if( !dxl_getdata_result_ ){
        std::cout << "[ID:" << int(id) << "] groupSyncRead getdata failed" << std::endl;
        return 0;
    }

    // Get Dynamixel present velocity value
    steer_curr_vel_ = groupSyncReadVel_->getData(id, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    return steer_curr_vel_;
}
*/

/*
uint32_t SobitProMotorDriver::feedbackWheelVel(uint8_t id){
    int dxl_comm_result_     = COMM_TX_FAIL;
    bool dxl_getdata_result_ = false;
    uint8_t dxl_error_       = 0;
    uint32_t wheel_curr_vel_;

    // Syncread present velocity
    dxl_comm_result_ = groupSyncReadVel_->txRxPacket();

    if( dxl_comm_result_ != COMM_SUCCESS ) std::cout << packetHandler_->getTxRxResult(dxl_comm_result_) << std::endl;
    else if( groupSyncReadVel_->getError(id, &dxl_error_) ) std::cout << "[ID:" << int(id) << "] " << packetHandler_->getRxPacketError(dxl_error_) << std::endl;

    // Check if groupsyncread data of Dynamixel is available
    dxl_getdata_result_ = groupSyncReadVel_->isAvailable(id, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    if( !dxl_getdata_result_ ){
        std::cout << "[ID:" << int(id) << "] groupSyncRead getdata failed" << std::endl;
        return 0;
    }

    // Get Dynamixel present velocity value
    wheel_curr_vel_ = groupSyncReadVel_->getData(id, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    return wheel_curr_vel_;
}
*/
