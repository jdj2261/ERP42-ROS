#ifndef CAN_TRANSMITTER_H
#define CAN_TRANSMITTER_H

/*
 * Copyright 2020 UNMANNED SOLUTION CO., LTD.
 * @file can_transmitter.h
 *
 * @brief CAN data sending from upper to ERP42
 *
 * Created on: 2020. 8. 4
 * Contact: <jdj2261@unmansol.com>
 * Author: Daejong Jin (djjin)
 */

#include <stdio.h>
#include <ros/ros.h>
#include <PCANBasic.h>

#include <erp42_msgs/CmdControl.h>
#include <erp42_msgs/DriveCmd.h>
#include <erp42_msgs/ModeCmd.h>


// command
typedef struct _pc_to_erp42
{
    uint8_t MODE;
    union speed{ uint8_t speed[2]; uint16_t _speed;}; union speed speed;
    union steer{ int8_t steer[2]; int16_t _steer;}; union steer steer;
    uint8_t brake = 0;
    uint8_t alive = 0;
}PC2ERP;

namespace unmansol
{
namespace erp42
{
namespace can
{
class ERP42Transmitter
{

public:
    ERP42Transmitter();
    virtual ~ERP42Transmitter()
    {
        std::cout << " Control Finished... " << std::endl;
    }

    bool Connect();
    void InitData();
    void InitNode();
    void Write();

    // Callback (ROS)
    void CmdCtrlMsgCallback(const erp42_msgs::CmdControl::Ptr &msg);

    bool is_enable_can;

protected:
    TPCANMsg m_TMessage;
    TPCANStatus m_TStatus;

    PC2ERP m_pc2erp;

    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_command;
    std::string ns_;

    erp42_msgs::CmdControl m_cmd_ctrl_msg;

    uint8_t m_AlvCnt;

}; // class ERP42Transmitter
} // namespace can
} // namespace erp42
} // namespace unmansol

#endif // CAN_TRANSMITTER_H
