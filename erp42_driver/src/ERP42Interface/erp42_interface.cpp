#include <ERP42Interface/erp42_interface.h>

using namespace unmansol::erp42;

static constexpr double TICK2RAD {0.06283185307}; // 3.6[deg] * 3.14159265359 / 180 = 0.06283185307

ERP42Interface::ERP42Interface():
  m_odom_x(0.0),
  m_odom_y(0.0),
  m_odom_yaw(0.0),
  m_linear_vel(0.0),
  m_angular_vel(0.0),
  m_wheel_base(0.0),
  m_steer_angle(0.0),
  m_delta_encoder(0),
  m_nh("~"),
  m_encoder(0),
  m_last_encoder(0),
  m_wheel_pos(0.0)
{
  Init_node();
}

void ERP42Interface::Init_node()
{
  // test
  m_sub_steer   = m_nh.subscribe("/erp42_can/command",1, &ERP42Interface::SteerCallback, this);
  // real
//  m_sub_steer   = m_nh.subscribe("/cmd_vel",1, &ERP42Interface::SteerCallback, this);
  m_sub_encoder = m_nh.subscribe("/erp42_can/feedback2",1,&ERP42Interface::EncoderCallback, this);
}

// *****************************************************************************
// Set params
void ERP42Interface::SetParams(double wheel_radius, double wheel_base, double wheel_tread,
                               double max_vel, double min_vel, double max_steer_angle, double min_steer_angle)
{

  //! Wheel Radius
  m_wheel_radius = wheel_radius;
  ROS_INFO("Wheel Radius : %lf [m]", wheel_radius);

  //! Wheel Base
  m_wheel_base = wheel_base;
  ROS_INFO("Wheel Base : %lf [m]", m_wheel_base);

  //! Tread width[m]
  m_wheel_tread = wheel_tread;
  ROS_INFO("Wheel Tread : %lf [m]", m_wheel_tread);

  //! Max Vel
  m_max_vel = max_vel;
  ROS_INFO("Max Velocity : %lf [m/s]", m_max_vel);

  //! Min Vel
  m_min_vel = min_vel;
  ROS_INFO("Min Velocity : %lf [m/s]", m_min_vel);

  //! Max Steer Angle
  m_max_steer_angle = max_steer_angle;
  ROS_INFO("Max Steer Angle : %lf [degree]", m_max_steer_angle);

  //! Min Steer Angle
  m_min_steer_angle = min_steer_angle;
  ROS_INFO("Min Steer Angle : %lf [degree]", m_min_steer_angle);

}

void ERP42Interface::EncoderCallback(const erp42_msgs::FeedBack2::Ptr &msg)
{
  m_encoder = msg->encoder;
}

void ERP42Interface::SteerCallback(const erp42_msgs::CmdControl::Ptr &msg)
{
  m_steer_angle = DEG2RAD(msg->Deg);
}

// *****************************************************************************
// Calculate ERP42 odometry
void ERP42Interface::CalculateOdometry(double delta_time)
{
  m_delta_encoder = m_encoder - m_last_encoder;
  m_wheel_pos = TICK2RAD * m_delta_encoder;
  m_last_encoder = m_encoder;

  m_delta_pos = m_wheel_radius * m_wheel_pos;

  m_linear_vel = m_delta_pos / delta_time;
  m_angular_vel = tan(m_steer_angle) * m_linear_vel / m_wheel_base;

  m_odom_yaw += m_angular_vel;
  m_odom_x += m_delta_pos * cos(m_odom_yaw);
  m_odom_y += m_delta_pos * sin(m_odom_yaw);


//  std::cout << "Delta Pose: " << m_delta_pos << " ";
//  std::cout << "Linear Vel: " << m_linear_vel<< " ";
//  std::cout << "Angular Vel: " << m_steer_angle<< " ";
//  std::cout << "tan Angular Vel: " << tan(m_steer_angle)<<" ";
//  std::cout << "Odom X : " << m_odom_x << " ";
//  std::cout << " Y : " << m_odom_y << " ";
//  std::cout << " Yaw : " << m_odom_yaw << std::endl;


  if (isnan(m_delta_pos)){
      m_delta_pos = 0.0;
  }
  if (isnan(m_angular_vel)){
      m_angular_vel = 0.0;
  }

}

// *****************************************************************************
// Reset ERP42 odometry
void ERP42Interface::ResetOdometry()
{
  SetOdometry(0.0, 0.0, 0.0);
}

// *****************************************************************************
// Set ERP42 odometry
void ERP42Interface::SetOdometry(double new_x, double new_y, double new_yaw)
{
  m_odom_x   = new_x;
  m_odom_y   = new_y;
  m_odom_yaw = new_yaw;
}

//void ERP42Interface::integrateRungeKutta2(double linear, double angular)
//{
//  const double direction = heading_ + angular * 0.5;

//  /// Runge-Kutta 2nd order integration:
//  x_       += linear * cos(direction);
//  y_       += linear * sin(direction);
//  heading_ += angular;
//}

//void ERP42Interface::IntegrateExact(double linear, double angular)
//{
//  if (fabs(angular) < 1e-6)
//    integrateRungeKutta2(linear, angular);
//  else
//  {
//    /// Exact integration (should solve problems when angular is zero):
//    const double heading_old = heading_;
//    const double r = linear/angular;
//    heading_ += angular;
//    x_       +=  r * (sin(heading_) - sin(heading_old));
//    y_       += -r * (cos(heading_) - cos(heading_old));
//  }
//}


int plus_or_minus(double value)
{
  if (value > 0)
  {
    return 1;
  }
  else if(value < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}
