#include "erp42_driver.h"

using namespace unmansol::erp42;

ERP42Driver::ERP42Driver()
    : erp42_interface_(new ERP42Interface()),
      m_nh("~"),
      rate_(50),
      m_odom_broadcaster{},
      m_current_time(0.0),
      m_last_time(0.0),
      m_last_odom_x(0.0),
      m_mode_MorA(0),
      m_mode_EStop(0),
      m_mode_Gear(0)
{
    InitParam();
    InitNode();
    erp42_interface_->ResetOdometry();
}

void ERP42Driver::InitParam()
{
    // DEBUG MODE
    if (ros :: console :: set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros :: console :: levels :: Debug)) {
        ros :: console :: notifyLoggerLevelsChanged();
    }

    // Initialize [m]
    double wheel_radius {0.265};
    double wheel_base {1.040};
    double wheel_tread {0.985};
    double max_vel {5.0};
    double min_vel {-5.0};
    double max_steer_angle {28.169};
    double min_steer_angle {-28.169};

    m_nh.param<double>("wheel_radius", wheel_radius, wheel_radius);
    m_nh.param<double>("wheel_base", wheel_base, wheel_base);
    m_nh.param<double>("wheel_tread", wheel_tread, wheel_tread);
    m_nh.param<double>("max_vel", max_vel, max_vel);
    m_nh.param<double>("min_vel", min_vel, min_vel);
    m_nh.param<double>("max_steer_angle", max_steer_angle, max_steer_angle);
    m_nh.param<double>("min_steer_angle", min_steer_angle, min_steer_angle);

    erp42_interface_->SetParams(wheel_radius, wheel_base, wheel_tread,
                               max_vel, min_vel, max_steer_angle, min_steer_angle);
}

void ERP42Driver::InitNode()
{
    m_pub_odom = m_nh.advertise<nav_msgs::Odometry>("/odom",1);
    m_pub_drive = m_nh.advertise<erp42_msgs::DriveCmd>(erp42_interface_->ns_+"/drive",1);
    m_pub_mode = m_nh.advertise<erp42_msgs::ModeCmd>(erp42_interface_->ns_+"/mode",1);
    m_sub_cmd_vel = m_nh.subscribe("/cmd_vel", 1, &ERP42Driver::CmdVelCallback, this);
    m_sub_mode = m_nh.subscribe(erp42_interface_->ns_+"/without_gear_mode", 1, &ERP42Driver::ModeCallback, this);
}

void ERP42Driver::CmdVelCallback(const geometry_msgs::Twist::Ptr &msg)
{
    // m/s to KPH
    const double linear_vel = msg->linear.x;
    const double angular_vel = msg->angular.z;
    double radius = linear_vel / angular_vel;
    double steering_angle = atan(erp42_interface_->m_wheel_base / radius);

    if (linear_vel == 0.0 || angular_vel == 0.0)
    {
        radius = 0.0;
        steering_angle = 0.0;
    }

    m_mode_Gear = (linear_vel >= 0) ? static_cast<uint8_t>(GEAR::FORWARD)
                                    : static_cast<uint8_t>(GEAR::REVERSE);
    m_mode_msg.MorA = m_mode_MorA;
    m_mode_msg.EStop = m_mode_EStop;
    m_mode_msg.Gear = m_mode_Gear;

    std::cout << " MPS2KPH(linear_vel) " << MPS2KPH(linear_vel);
    std::cout << " Linear Vel : " << linear_vel << " Steer Angle: " << steering_angle <<std::endl;

    m_drive_msg.KPH = static_cast<uint16_t>(MPS2KPH(abs(linear_vel)));
    m_drive_msg.Deg = static_cast<int16_t>(RAD2DEG(steering_angle));

    m_drive_msg.KPH = MIN(MAX_KPH, m_drive_msg.KPH);
    m_drive_msg.Deg = MIN(MAX_DEGREE, m_drive_msg.Deg);
    m_drive_msg.Deg = MAX(-MAX_DEGREE, m_drive_msg.Deg);

    m_pub_mode.publish(m_mode_msg);
    m_pub_drive.publish(m_drive_msg);
}

void ERP42Driver::ModeCallback(const erp42_msgs::ModeCmd::Ptr &msg)
{
    m_mode_MorA = msg->MorA;
    m_mode_EStop = msg->EStop;
}

void ERP42Driver::Update(const ros::Time &current_time)
{
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat {tf::createQuaternionMsgFromYaw(erp42_interface_->m_odom_yaw)};

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = erp42_interface_->m_odom_x;
    odom_trans.transform.translation.y = erp42_interface_->m_odom_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = erp42_interface_->m_odom_x;
    odom.pose.pose.position.y = erp42_interface_->m_odom_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = erp42_interface_->m_linear_vel;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = erp42_interface_->m_angular_vel;

    m_odom_broadcaster.sendTransform(odom_trans);
    m_pub_odom.publish(odom);
}

void ERP42Driver::Run()
{
    while(m_nh.ok())
    {
        ros::spinOnce();
        m_current_time = ros::Time::now();
        m_delta_time = m_current_time - m_last_time;
        m_last_time = m_current_time;

        double diff_time = double(m_delta_time.sec) + double(m_delta_time.nsec)*(1e-9);

        erp42_interface_->CalculateOdometry(diff_time);
        Update(m_current_time);
        rate_.sleep();  // rate is 50hz
    }
}
