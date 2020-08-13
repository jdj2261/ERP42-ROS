#include <erp42_driver.h>

using namespace unmansol::erp42;

ERP42Driver::ERP42Driver():
  m_nh("~"),
  rate_(50),
  m_last_time(0.0)
{
  Init_param();
  Init_node();
  erp42_interface_.ResetOdometry();
}

void ERP42Driver::Init_param()
{
  if (ros :: console :: set_logger_level (ROSCONSOLE_DEFAULT_NAME, ros :: console :: levels :: Debug)) {
     ros :: console :: notifyLoggerLevelsChanged ();
  }

  // initialize [m]
  double wheel_radius {0.265};
  double wheel_base {1.040};
  double wheel_tread {0.985};
  double max_vel {5.0};
  double min_vel {-5.0};
  double max_steer_angle {28.169};
  double min_steer_angle {-28.169};

  m_nh.param("wheel_radius", wheel_radius, wheel_radius);
  m_nh.param("wheel_base", wheel_base, wheel_base);
  m_nh.param("wheel_tread", wheel_tread, wheel_tread);
  m_nh.param("max_vel", max_vel, max_vel);
  m_nh.param("min_vel", min_vel, min_vel);
  m_nh.param("max_steer_angle", max_steer_angle, max_steer_angle);
  m_nh.param("min_steer_angle", min_steer_angle, min_steer_angle);

  erp42_interface_.SetParams(wheel_radius, wheel_base, wheel_tread,
                             max_vel, min_vel, max_steer_angle, min_steer_angle);
}

void ERP42Driver::Init_node()
{
  m_pub_odom = m_nh.advertise<nav_msgs::Odometry>("/odom",1);
  m_pub_steer = m_nh.advertise<geometry_msgs::Twist>("/steer_ctrl",1);
  m_pub_cmdcontrol = m_nh.advertise<erp42_msgs::CmdControl>("/erp42_can/command",1);
  m_sub_cmd_vel = m_nh.subscribe("/cmd_vel", 1, &ERP42Driver::CmdVelCallback, this);
}

void ERP42Driver::CmdVelCallback(const geometry_msgs::Twist &msg)
{
  // m/s to KPH
  m_cmdctrl_msg.KPH = msg.linear.x;
  m_cmdctrl_msg.Deg = msg.angular.z;
  std::cout << msg.linear.x << std::endl;
  m_pub_cmdcontrol.publish(m_cmdctrl_msg);
  ROS_DEBUG("%0.2lf",msg.linear.x);

  ROS_DEBUG_STREAM_NAMED("test",
                         "Added values to command. "
                         << "Ang: "   << m_cmdctrl_msg.KPH << ", "
                         << "Lin: "   << m_cmdctrl_msg.Deg << ", ");
}

void ERP42Driver::encoder_test()
{
  while(m_nh.ok())
  {
    m_delta_time = ros::Time::now() - m_last_time;
    m_last_time = ros::Time::now();

//    std::cout << "Loop Time : "<<m_delta_time << " ";
    double diff_time = double(m_delta_time.sec) + double(m_delta_time.nsec)*1e-9;

    erp42_interface_.CalculateOdometry(diff_time);


//    std::cout << erp42_interface_.m_delta_encoder << std::endl;

    ros::spinOnce();
    rate_.sleep();
  }
}


