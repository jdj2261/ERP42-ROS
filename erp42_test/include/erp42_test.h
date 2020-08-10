#ifndef ERP42_TEST_H
#define ERP42_TEST_H
/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file erp42_test.h
 *
 * @brief ERP42 TEST
 *
 *
 * Created on: 2020. 8. 11
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */


#include <ros/ros.h>
#include <erp42_msgs/FeedBack2.h>

namespace unmansol
{
namespace erp42
{
class ERP42Test
{
public:
  ERP42Test();
  virtual ~ERP42Test()
  {
    std::cout << " Test Finished... " << std::endl;
  }

  void Init_node();
  void encoder_test();

  int16_t m_cnt;

protected:
  ros::NodeHandle m_nh;

  ros::Publisher m_pub_test;
  erp42_msgs::FeedBack2 m_encoder_msg;

}; // class ERP42Test

} // erp42
} // unmansol


#endif // ERP42_TEST_H
