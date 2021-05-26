#pragma once
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
#include <erp42_msgs/DriveCmd.h>
#include <erp42_msgs/ModeCmd.h>
#include <erp42_msgs/CANFeedBack.h> // Publish Command
#include <erp42_msgs/SerialFeedBack.h> // Subscribe Encoder
#include <cmath>

namespace unmansol
{
namespace erp42
{
    static constexpr int8_t MAX_KPH = 5;
    static constexpr int8_t MAX_DEGREE = 20;

    // 3.6[deg] * 3.14159265359 / 180 = 0.06283185307
    static constexpr double TICK2RAD = 0.06283185307;

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

    template <typename T>
    inline double KPH2MPS(const T& a)
    {
        return a * 0.2778; // 1000/3600
    }

    template <typename T>
    inline double MPS2KPH(const T& a)
    {
        return a * 3.6; // 3600/1000
    }

    class ERP42Interface
    {
    public:
        explicit ERP42Interface();
        ~ERP42Interface() noexcept
        {
            std::cout << "Interface Finished... " << std::endl;
        }

        //  virtual void SetParams(double wheel_diameter_right, double wheel_diameter_left, double tred_width);
        void CalculateOdometry(const double &delta_time);
        void ResetOdometry();
        //  // Set new odometry.
        void SetOdometry(const double &new_x, const double &new_y, const double &new_yaw);
        void SetParams(const double &wheel_diameter,
                               const double &wheel_base,
                               const double &wheel_seperation,
                               const double &max_vel,
                               const double &min_vel,
                               const double &max_steer_angle,
                               const double &min_steer_angle);

        double m_odom_x;
        double m_odom_y;
        double m_odom_yaw;

        double m_linear_vel;
        double m_angular_vel;

        double m_wheel_base;
        double m_steer_angle;
        double m_status;

        int32_t m_delta_encoder;

        std::string ns_;

    private:
        ros::NodeHandle m_nh;

        ros::Publisher m_pub_test;
        ros::Subscriber m_sub_steer;
        ros::Subscriber m_sub_encoder;

        erp42_msgs::CANFeedBack m_feedback2_msg;

        void InitNode();

        // callback
        void CANEncoderCallback(const erp42_msgs::CANFeedBack::Ptr &msg);
        void CANSteerCallback(const erp42_msgs::CANFeedBack::Ptr &msg);
        void SerialDriveCallback(const erp42_msgs::SerialFeedBack::Ptr &msg);

        // member variable
        int32_t m_encoder;
        int32_t m_last_encoder;
        double m_wheel_pos;
        double m_delta_pos;

        double m_wheel_radius = 0.265;
        double m_wheel_tread = 0.985;

        double m_max_vel = 5.0;
        double m_min_vel = -5.0;
        double m_max_steer_angle = 28.169;
        double m_min_steer_angle = -28.169;

    }; // class ERP42Interface

} // erp42
} // unmansol

