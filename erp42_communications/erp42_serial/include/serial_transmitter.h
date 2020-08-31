#ifndef SERIAL_TRANSMITTER_H
#define SERIAL_TRANSMITTER_H

/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file serial_transmitter.h
 *
 * @brief CAN data sending from upper to ERP42
 *
 * Created on: 2020. 8. 31
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */

#include <ros/ros.h>
#include <SerialInterface/serial_interface.h>
#include <string.h>


// command
typedef struct _pc_to_erp42
{
  unsigned char S = 0x53;
  unsigned char T = 0x54;
  unsigned char X = 0x58;
  uint8_t MorA = 0x01;  // Manual : 0x00, Auto : 0x01
  uint8_t E_stop = 0x00;
  uint8_t gear = 0x01;
  union speed{uint8_t speed[2]; uint16_t _speed;};  union speed speed;
  union steer{char steer[2]; short _steer;};  union steer steer;
  uint8_t brake;
  uint8_t alive;
  unsigned char ETX0 = 0x0D;
  unsigned char ETX1 = 0x0A;
}PC2ERP;

namespace unmansol
{
namespace erp42
{
namespace serial
{
class ERP42Transmitter
{

public:
  ERP42Transmitter();
  virtual ~ERP42Transmitter()
  {
    std::cout << " Transmitter Finished... " << std::endl;
  }

  void Init_data();
  void Init_node();
  void Write();

  // Callback (ROS)
  void CmdCtrlMsgCallback(const erp42_msgs::CmdControl::Ptr &msg);


private:
  unmansol::erp42::SerialInterface serial_interface_;

protected:
  PC2ERP m_pc2erp;

  ros::NodeHandle m_nh;
  ros::Subscriber m_sub_command;

  uint8_t m_AlvCnt;

  void Update(unsigned char (&buffer)[14]);


}; // class ERP42Transmitter
} // namespace serial
} // namespace erp42
} // namespace unmansol

#endif // SERIAL_TRANSMITTER_H
