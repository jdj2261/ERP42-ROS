#include "ERP42Interface/erp42_interface.h"

using namespace unmansol::erp42;

ERP42Interface::ERP42Interface()
    : m_odom_x(0.0),
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
    InitNode();
}

void ERP42Interface::InitNode()
{
    ns_ = ros::this_node::getNamespace();

    if (this->ns_ == "/erp42_can")
    {
        m_sub_steer   = m_nh.subscribe(this->ns_ + "/feedback1",1, &ERP42Interface::CANSteerCallback, this);
        m_sub_encoder = m_nh.subscribe(this->ns_ + "/feedback2",1, &ERP42Interface::CANEncoderCallback, this);
    }
    else
        m_sub_encoder = m_nh.subscribe(this->ns_ + "/feedback",1,&ERP42Interface::SerialDriveCallback, this);
}

// *****************************************************************************
// Set params
void ERP42Interface::SetParams(const double &wheel_radius,
                               const double &wheel_base,
                               const double &wheel_tread,
                               const double &max_vel,
                               const double &min_vel,
                               const double &max_steer_angle,
                               const double &min_steer_angle)
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

void ERP42Interface::CANEncoderCallback(const erp42_msgs::CANFeedBack::Ptr &msg)
{
    m_encoder = msg->encoder;
}

void ERP42Interface::CANSteerCallback(const erp42_msgs::CANFeedBack::Ptr &msg)
{
    m_steer_angle = DEG2RAD(msg->steer);
}

void ERP42Interface::SerialDriveCallback(const erp42_msgs::SerialFeedBack::Ptr &msg)
{
    m_encoder = msg->encoder;
    m_steer_angle = DEG2RAD(msg->steer);
}

// *****************************************************************************
// Calculate ERP42 odometry
void ERP42Interface::CalculateOdometry(const double &delta_time)
{
    m_delta_encoder = m_encoder - m_last_encoder;
    m_last_encoder = m_encoder;
    m_wheel_pos = TICK2RAD * m_delta_encoder;

    m_delta_pos = m_wheel_radius * m_wheel_pos;

    m_linear_vel = m_delta_pos / delta_time;
    m_angular_vel = tan(m_steer_angle) * m_linear_vel / m_wheel_base;

    m_odom_yaw += m_angular_vel;
    m_odom_x += m_delta_pos * cos(m_odom_yaw);
    m_odom_y += m_delta_pos * sin(m_odom_yaw);

    std::cout << "Duration: " << delta_time << " ";
    std::cout << "Previous Encoder: " << m_last_encoder << " ";
    //  std::cout << "Delta Encoder: " << m_delta_encoder << " ";
    //  std::cout << "Wheel Pose: " << m_wheel_pos << " ";
    std::cout << "Delta Pose: " << m_delta_pos << " ";
    std::cout << "Linear Vel: " << m_linear_vel<< " ";
    std::cout << "Angular Vel: " << m_angular_vel<< " ";
    std::cout << "Odom X : " << m_odom_x << " ";
    std::cout << " Y : " << m_odom_y << " ";
    std::cout << " Yaw : " << m_odom_yaw << std::endl;
    std::cout << std::endl;

    if (isnan(m_delta_pos)) m_delta_pos = 0.0;
    if (isnan(m_angular_vel)) m_angular_vel = 0.0;

}

// *****************************************************************************
// Reset ERP42 odometry
void ERP42Interface::ResetOdometry()
{
    SetOdometry(0.0, 0.0, 0.0);
}

// *****************************************************************************
// Set ERP42 odometry
void ERP42Interface::SetOdometry(const double &new_x, const double &new_y, const double &new_yaw)
{
    m_odom_x   = new_x;
    m_odom_y   = new_y;
    m_odom_yaw = new_yaw;
}
