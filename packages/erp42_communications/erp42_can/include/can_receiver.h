#ifndef CAN_RECEIVER_H
#define CAN_RECEIVER_H

/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file can_reciever.h
 *
 * @brief CAN data receiving from ERP42 to Upper
 *
 * Created on: 2020. 8. 4
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */

#include <iostream>
#include <PCANBasic.h>
#include <ros/ros.h>

#include <erp42_msgs/CANFeedBack.h>

namespace unmansol
{
namespace erp42
{
namespace can
{
    class ERP42Receiver
    {
    public:
        ERP42Receiver();
        virtual ~ERP42Receiver()
        {
            std::cout << " Receiver Finished... " << std::endl;
        }

        bool Connect();
        void InitNode();
        void Read();
        void Update();

        bool isConnect;


    private:
        TPCANMsg m_RMessage;

        TPCANStatus m_RStatus;

        //  ERP2PC_1 m_erp2pc_1;
        //  ERP2PC_2 m_erp2pc_2;

        ros::NodeHandle m_nh;
        ros::Publisher m_pub_feedback1;
        ros::Publisher m_pub_feedback2;

        std::string ns_;

        erp42_msgs::CANFeedBack m_feedback_msg;
        ros::Publisher m_pub_test;

    }; // class ERP42Receiver
} // namespace can
} // namespace erp42
} // namespace unmansol


#endif // CAN_RECEIVER_H
