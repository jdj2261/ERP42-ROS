#ifndef CAN_VARIABLES_H
#define CAN_VARIABLES_H

/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file serial_interface.h
 *
 * @brief ERP42 Serial Interface
 *
 * Created on: 2020. 8. 31
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <stdlib.h>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <erp42_msgs/CmdControl.h>
#include <erp42_msgs/SerialFeedBack.h>

const uint8_t SPEED_FACTOR=10;
const uint8_t STEER_FACTOR=71;

namespace unmansol
{
namespace erp42
{
class SerialInterface
{
public:
  SerialInterface();
  virtual ~SerialInterface()
  {
    ser.close();
    std::cout << " Serial Closed..." << std::endl;
    std::cout << " Interface Finished... " << std::endl;
  }

  ros::NodeHandle m_nh;
  serial::Serial ser;
  std::string ns_;

protected:


  void Init_param();
  int Init_serial();

  std::string m_serial_port;
  int m_serial_baudrate;

}; // class SerialInterface
} // namespace erp42
} // namespace unmansol



#endif // CAN_VARIABLES_H
