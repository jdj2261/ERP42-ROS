#ifndef CAN_RECEIVER_H
#define CAN_RECEIVER_H

/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file can_reciever.h
 *
 * @brief CAN data receiving from ERP42 to Upper
 *
 * Created on: 2020. 8. 4
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */

#include <iostream>
#include <PCANBasic.h>
#include <ros/ros.h>

#include <erp42_msgs/FeedBack1.h>
#include <erp42_msgs/FeedBack2.h>
#include <erp42_msgs/test.h>

// feedback
typedef struct _erp42_to_pc_1
{
    uint8_t MODE;
    uint8_t MorA;
    uint8_t ESTOP;
    uint8_t GEAR;
    union speed{ uint8_t speed[2]; uint16_t _speed;}; union speed speed;
    union steer{ int8_t steer[2]; int16_t _steer;}; union steer steer;
    uint8_t brake = 0;
    uint8_t alive = 0;
}ERP2PC_1;

// feedback2
typedef struct _erp42_to_pc_2
{
    int32_t Encoder[4];
    uint8_t Brake_Cmd_Raw;
    uint8_t Brake_Raw;
    uint8_t Brake_Echo;
    uint8_t Brake_Init_Max;
}ERP2PC_2;

namespace unmansol
{
namespace erp42
{
class ERP42Receiver
{
public:
  ERP42Receiver();
  virtual ~ERP42Receiver()
  {
    std::cout << " Receiver Finished... " << std::endl;
  }

  bool Connect();
  void Init_node();
  void Read();
  void Update();

protected:
  TPCANMsg m_RMessage;

  TPCANStatus m_RStatus;

  ERP2PC_1 m_erp2pc_1;
  ERP2PC_2 m_erp2pc_2;

  ros::NodeHandle m_nh;
  ros::Publisher m_pub_feedback1;
  ros::Publisher m_pub_feedback2;

  erp42_msgs::FeedBack1 m_feedback1_msg;
  erp42_msgs::FeedBack2 m_feedback2_msg;

  ros::Publisher m_pub_test;
  erp42_msgs::test m_test_msg;

}; // class ERP42Receiver

} // erp42
} // unmansol


#endif // CAN_RECEIVER_H
