#pragma once
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

#include "ERP42Interface/erp42_interface.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>    // odom
#include <tf/transform_broadcaster.h> //tf
#include <geometry_msgs/Twist.h>  // cmd_vel
#include <ros/console.h>

namespace unmansol
{
namespace erp42
{
    enum class GEAR
    {
        FORWARD=0x00,
        NEUTRAL,
        REVERSE,
    };

    class ERP42Driver
    {
    public:
        explicit ERP42Driver();
        ~ERP42Driver() noexcept
        {
            std::cout << "Driver Finished... " << std::endl;
        }
        void Run();

    private:
        std::shared_ptr<ERP42Interface> erp42_interface_;

        ros::NodeHandle m_nh;
        ros::Rate rate_;

        ros::Publisher m_pub_odom;
        ros::Publisher m_pub_drive;
        ros::Publisher m_pub_mode;

        ros::Subscriber m_sub_cmd_vel;
        ros::Subscriber m_sub_mode;

        tf::TransformBroadcaster m_odom_broadcaster;

        ros::Duration m_delta_time;
        ros::Time m_current_time;
        ros::Time m_last_time;

        erp42_msgs::DriveCmd m_drive_msg;
        erp42_msgs::ModeCmd m_mode_msg;

        double m_last_odom_x;

        uint8_t m_mode_MorA;
        uint8_t m_mode_EStop;
        uint8_t m_mode_Gear;

        void InitParam();
        void InitNode();
        void Update(const ros::Time &current_time);

        // Callback (ROS)
        void CmdVelCallback(const geometry_msgs::Twist::Ptr &msg);
        void ModeCallback(const erp42_msgs::ModeCmd::Ptr &msg);

    }; // class ERP42Driver

} // erp42
} // unmansol
