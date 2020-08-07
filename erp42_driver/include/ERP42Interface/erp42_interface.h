#ifndef ERP42_INTERFACE_H
#define ERP42_INTERFACE_H

/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file erp42_interface.h
 *
 * @brief Calculation of odometry with encoder value
 *
 * Created on: 2020. 8. 7
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // cmd_vel

#include <erp42_msgs/CmdControl.h>
#include <erp42_msgs/FeedBack1.h>
#include <erp42_msgs/FeedBack2.h>

namespace unmansol
{
namespace erp42
{
class ERP42Interface  //odom
{
public:
  ERP42Interface();
  virtual ~ERP42Interface()
  {
    std::cout << " Interface Finished... " << std::endl;
  }

  bool Connect();
  void Init_node();
  void Read();
  void Update();

protected:
  ros::NodeHandle m_nh;
  ros::Publisher m_pub_feedback1;
  ros::Publisher m_pub_feedback2;

  erp42_msgs::FeedBack1 m_feedback1_msg;
  erp42_msgs::FeedBack2 m_feedback2_msg;

  ros::Publisher m_pub_test;

}; // class ERP42Interface

} // erp42
} // unmansol


#endif // ERP42_INTERFACE_H
