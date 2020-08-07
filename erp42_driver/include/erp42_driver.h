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

  void Init_node();
  void Update();

protected:
  ros::NodeHandle m_nh;
  ros::Publisher m_pub_odom;
  ros::Publisher m_pub_steer;

  ros::Publisher m_pub_test;

}; // class ERP42Driver

} // erp42
} // unmansol


#endif // ERP42_DRIVER_H
