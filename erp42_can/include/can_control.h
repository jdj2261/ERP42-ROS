#ifndef CAN_CONTROL_H
#define CAN_CONTROL_H

/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file can_control.h
 *
 * @brief CAN data sending from upper to ERP42
 *
 * Created on: 2020. 8. 4
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */

#include <stdio.h>
#include <ros/ros.h>
#include <PCANBasic.h>

#include <erp42_msgs/CmdControl.h>

#define PCAN_DEVICE	PCAN_USBBUS1

const uint8_t SPEED_FACTOR=10;
const uint8_t STEER_FACTOR=71;

// command
typedef struct _pc_to_erp42
{
    uint8_t MODE;
    union speed{ uint8_t speed[2]; uint16_t _speed;}; union speed speed;
    union steer{ int8_t steer[2]; int16_t _steer;}; union steer steer;
    uint8_t brake = 0;
    uint8_t alive = 0;
}PC2ERP;

namespace unmansol
{
namespace erp42
{
class ERP42Control
{

public:
  ERP42Control();
  virtual ~ERP42Control()
  {
    std::cout << " Control Finished... " << std::endl;
  }

  bool Connect();
  void Init_node();
  void CmdCtrlMsgCallback(const erp42_msgs::CmdControl &msg);
  void Write();
  void Start();

protected:
  TPCANMsg m_send_msg;
  TPCANStatus m_TStatus;

  PC2ERP m_pc2erp;

  ros::NodeHandle m_nh;
  ros::Subscriber m_sub_command;

  erp42_msgs::CmdControl m_cmd_ctrl_msg;

};

} // erp42
} // unmansol

#endif // CAN_CONTROL_H
