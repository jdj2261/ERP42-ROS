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

  virtual void SetParams(double wheel_diameter_right, double wheel_diameter_left, double tred_width);
  virtual int16_t GetEncoderPacket();
  virtual void CalculateOdometry();
  virtual void resetOdometry();
  // Set new odometry.
  virtual void setOdometry(double new_x, double new_y, double new_yaw);

  double m_odom_x;
  double m_odom_y;
  double m_odom_yaw;
  double m_steer_angle;
  double m_status;

protected:
  ros::NodeHandle m_nh;

  ros::Publisher m_pub_test;

  erp42_msgs::FeedBack1 m_feedback1_msg;
  erp42_msgs::FeedBack2 m_feedback2_msg;


}; // class ERP42Interface

} // erp42
} // unmansol


#endif // ERP42_INTERFACE_H
