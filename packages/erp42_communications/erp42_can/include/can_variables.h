#ifndef CAN_VARIABLES_H
#define CAN_VARIABLES_H

#include <ros/ros.h>

#define PCAN_DEVICE	PCAN_USBBUS1

const uint8_t SPEED_FACTOR=10;
const uint8_t STEER_FACTOR=-71;
const uint8_t CAN_DATA_LENGTH=8;

const uint16_t CAN_COMMAND_ID=0x777;
const uint16_t CAN_FEEDBACK_ID_1=0x778;
const uint16_t CAN_FEEDBACK_ID_2=0x779;
const uint16_t TEST_ID=0x2B0;

bool enable_can = true;


#endif // CAN_VARIABLES_H
