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

#include <erp42_msgs/CmdControl.h>
#include <erp42_msgs/FeedBack1.h> // Publish Command
#include <erp42_msgs/FeedBack2.h> // Subscribe Encoder

#include <math.h>

template<typename N, typename M>
inline double MIN(const N& a, const M& b)
{
  return a < b ? a : b;
}

template<typename N, typename M>
inline double MAX(const N& a, const M& b)
{
  return a > b ? a : b;
}

template<typename T>
inline double NORMALIZE(const T& z)
{
  return atan2(sin(z), cos(z));
}

template<typename T>
inline double DEG2RAD(const T& a)
{
  return a * (M_PI / 180);
}

template<typename T>
inline double RAD2DEG(const T& a)
{
  return a * (180 / M_PI);
}

int plus_or_minus(double value);

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


//  virtual void SetParams(double wheel_diameter_right, double wheel_diameter_left, double tred_width);
  virtual void CalculateOdometry(double delta_time);
  virtual void ResetOdometry();
//  // Set new odometry.
  virtual void SetOdometry(double new_x, double new_y, double new_yaw);
  virtual void SetParams(double wheel_diameter, double wheel_base, double wheel_seperation,
                         double max_vel, double min_vel, double max_steer_angle, double min_steer_angle);

  double m_odom_x;
  double m_odom_y;
  double m_odom_yaw;

  double m_steer_angle;
  double m_status;

  int32_t m_delta_encoder;

protected:
  ros::NodeHandle m_nh;

  ros::Publisher m_pub_test;
  ros::Subscriber m_sub_encoder;
  ros::Subscriber m_sub_steer;

  erp42_msgs::FeedBack2 m_feedback2_msg;

  void Init_node();
  void IntegrateExact(double linear, double angular);
  void integrateRungeKutta2(double linear, double angular);

  // callback
  void EncoderCallback(const erp42_msgs::FeedBack2 &msg);
  void SteerCallback(const erp42_msgs::CmdControl &msg);

  // member variable
  int32_t m_encoder;
  int32_t m_last_encoder;
  double m_wheel_pos;
  double m_delta_pos;
  double m_linear_vel;
  double m_angular_vel;

  double m_wheel_radius = 0.265;
  double m_wheel_tread = 0.985;
  double m_wheel_base = 1.040;
  double m_max_vel = 5.0;
  double m_min_vel = -5.0;
  double m_max_steer_angle = 28.169;
  double m_min_steer_angle = -28.169;

}; // class ERP42Interface

} // erp42
} // unmansol


#endif // ERP42_INTERFACE_H
