#include "can_transmitter.h"
#include "can_variables.h"
using namespace unmansol::erp42::can;

ERP42Transmitter::ERP42Transmitter():
    is_enable_can(true),
    m_nh("~"),
    m_AlvCnt(0)
{
    InitData();
    InitNode();
}

void ERP42Transmitter::InitData()
{
    m_TMessage.ID = CAN_COMMAND_ID;
    m_TMessage.LEN = CAN_DATA_LENGTH;
    m_TMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
    memset(m_TMessage.DATA, 0, sizeof(m_TMessage.DATA));

    m_pc2erp.MODE = 0x06; // Manual : 0x00, E-Stop ON : 0x02 , Gear-Neutral : 0x04
    m_pc2erp.speed._speed = 0x00;
    m_pc2erp.steer._steer = 0x00;
    m_pc2erp.brake = 0x00;
}

void ERP42Transmitter::InitNode()
{
    this->ns_ = ros::this_node::getNamespace();
    m_nh.param("enable_can",enable_can,enable_can);
    is_enable_can = enable_can;
    std::cout << is_enable_can << std::endl;
    m_sub_command = m_nh.subscribe(this->ns_ + "/command", 1, &ERP42Transmitter::CmdCtrlMsgCallback, this);
}

void ERP42Transmitter::Write()
{
    m_pc2erp.speed.speed[0] = m_pc2erp.speed._speed & 0xff;
    m_pc2erp.speed.speed[1] = (m_pc2erp.speed._speed & 0xff00) >> 8;
    m_pc2erp.steer.steer[0] = (m_pc2erp.steer._steer & 0xff);
    m_pc2erp.steer.steer[1] = (m_pc2erp.steer._steer & 0xff00) >> 8;

    m_TMessage.DATA[0] = m_pc2erp.MODE;
    m_TMessage.DATA[1] = m_pc2erp.speed.speed[1];
    m_TMessage.DATA[2] = m_pc2erp.speed.speed[0];
    m_TMessage.DATA[3] = m_pc2erp.steer.steer[1];
    m_TMessage.DATA[4] = m_pc2erp.steer.steer[0];
    m_TMessage.DATA[5] = m_pc2erp.brake;
    m_TMessage.DATA[7] = m_AlvCnt++;

    for(int i=0; i<8; i++)
    {
        std::cout << std::hex << (int)m_TMessage.DATA[i] << " ";
    }
    std::cout << std::endl;

    m_TStatus = CAN_Write(PCAN_DEVICE, &m_TMessage);

    if (m_TStatus != PCAN_ERROR_OK)
        ROS_WARN(" Failed connect ");
    else ROS_INFO(" Success Write ");
}

bool ERP42Transmitter::Connect()
{
    this->m_TStatus = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_500K, 0, 0, 0);

    //  if (this->m_TStatus != PCAN_ERROR_OK)
    //  {
    //    char error_message[50];
    //    CAN_GetErrorText(this->m_TStatus, 0x09, error_message);
    //    ROS_ERROR("%s\n",error_message);
    //  }

    if (this->m_TStatus) return false;
    else return true;
}

void ERP42Transmitter::CmdCtrlMsgCallback(const erp42_msgs::CmdControl::Ptr &msg)
{
    m_pc2erp.MODE = msg->MorA + msg->EStop + msg->Gear;
    m_pc2erp.speed._speed = msg->KPH * SPEED_FACTOR;
    m_pc2erp.steer._steer = msg->Deg * STEER_FACTOR;
    m_pc2erp.brake = msg->brake;
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "can_transmitter");

    ERP42Transmitter* erp_control = new ERP42Transmitter;

    ros::Rate loop(50); // 50Hz is 0.02s

    if (erp_control->is_enable_can)
    {
        while(ros::ok())
        {
            bool isConnect = erp_control->Connect();
            if (isConnect)
            {
                ROS_INFO(" Success Connection! ");
                break;
            }
            else ROS_ERROR(" Not Connected ");
            loop.sleep();
        }
    }

    while(ros::ok())
    {
        ros::spinOnce();
        erp_control->Write();
        loop.sleep();
    }

    delete erp_control;

    return 0;
}
