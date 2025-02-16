#ifndef motor_handler_h
#define motor_handler_h
#include <Dynamixel2Arduino.h>
#include <Arduino.h>

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t MTR_ID_COUNT = 4;
const uint8_t MTR_ID_LIST[MTR_ID_COUNT] = {1, 2, 3, 4};

// const uint16_t user_pkt_buf_cap = 128;
// uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t PRESENT_POS_ADDR = 132;
const uint16_t PRESENT_POS_LEN = 4; //Length of Position data of X series is 4 byte

const uint16_t GOAL_POS_ADDR = 116;
const uint16_t GOAL_POS_LEN = 4;

struct read_data{
  int32_t present_position;
};

struct write_data{
  int32_t goal_position;
};


class MotorHandler{
    public:
    short mtrA1;
    short mtrA2;
    short mtrA3;

    short volCtrlID;
    short posCtrlID;

    read_data r_data[MTR_ID_COUNT];
    write_data w_data[MTR_ID_COUNT];

    DYNAMIXEL::InfoSyncReadInst_t r_info;
    DYNAMIXEL::XELInfoSyncRead_t info_xels_read[MTR_ID_COUNT];

    DYNAMIXEL::InfoSyncWriteInst_t w_info;
    DYNAMIXEL::XELInfoSyncWrite_t info_xels_write[MTR_ID_COUNT];

    Dynamixel2Arduino mtr_controller;

    MotorHandler(short mtrA1, short mtrA2, short mtrA3, HardwareSerial& serialPort, int pin);
    
    bool setControlMode(short id, short modeID);

    double getPos(short id);
    bool setPos(short id, double pos);
    double getVol(short id);
    bool setVol(short id, double velocity);

    bool setLinearVol(double velocity);
    bool setRotateVol(double velocity);


    bool isShutdown(short id);
};

#endif