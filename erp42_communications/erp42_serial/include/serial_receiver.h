#ifndef SERIAL_RECEIVER_H
#define SERIAL_RECEIVER_H

/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file serial_reciever.h
 *
 * @brief Serial data receiving from ERP42 to Upper
 *
 * Created on: 2020. 8. 7
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */

#include <ros/ros.h>
#include <SerialInterface/serial_interface.h>


//// feedback
//typedef struct _erp42_to_pc
//{
//    uint8_t MorA;
//    uint8_t ESTOP;
//    uint8_t GEAR;
//    union speed{ uint8_t speed[2]; uint16_t _speed;}; union speed speed;
//    union steer{ int8_t steer[2]; int16_t _steer;}; union steer steer;
//    uint8_t brake = 0;
//    uint8_t alive = 0;
//}ERP2PC;


namespace unmansol
{
namespace erp42
{
namespace serial
{
class ERP42Receiver
{
public:
  ERP42Receiver();
  virtual ~ERP42Receiver()
  {
    std::cout << " Receiver Finished... " << std::endl;
  }

  void Init_node();
  void Init_data();
  void Read();
  void Update();

  bool isConnect;
private:
  unmansol::erp42::SerialInterface serial_interface_;

protected:

//  ERP2PC m_erp2pc;

  ros::NodeHandle m_nh;
  ros::Publisher m_pub_feedback;
  erp42_msgs::SerialFeedBack m_feedback_msg;

  std::string m_read_data;

}; // class ERP42Receiver
} // namespace serial
} // namespace erp42
} // namespace unmansol


#endif // SERIAL_RECEIVER_H
