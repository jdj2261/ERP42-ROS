#ifndef ERP42_DRIVER_H
#define ERP42_DRIVER_H
/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file erp42_driver.h
 *
 * @brief Ros topic Sub(/cmd_vel)/Pub(/odom, /steer)
 *
 * Created on: 2020. 8. 7
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */

#include <ERP42Interface/erp42_interface.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>    // odom
#include <tf/transform_broadcaster.h> //tf
#include <geometry_msgs/Twist.h>  // cmd_vel
#include <ros/console.h>

namespace unmansol
{
namespace erp42
{
class ERP42Driver
{
public:
  ERP42Driver();
  virtual ~ERP42Driver()
  {
    std::cout << " Driver Finished... " << std::endl;
  }

  void Init_param();
  void Init_node();

  void Run();
  void Update(ros::Time current_time);

  // Callback (ROS)
  void CmdVelCallback(const geometry_msgs::Twist::Ptr &msg);
;

private:
  unmansol::erp42::ERP42Interface erp42_interface_;

protected:
  ros::NodeHandle m_nh;
  ros::Rate rate_;

  ros::Publisher m_pub_odom;
  ros::Publisher m_pub_cmdcontrol;
  ros::Publisher m_pub_test;

  ros::Subscriber m_sub_cmd_vel;

  tf::TransformBroadcaster m_odom_broadcaster;

  ros::Duration m_delta_time;
  ros::Time m_current_time;
  ros::Time m_last_time;

  erp42_msgs::CmdControl m_cmdctrl_msg;

  double m_last_odom_x;

}; // class ERP42Driver

} // erp42
} // unmansol


#endif // ERP42_DRIVER_H
