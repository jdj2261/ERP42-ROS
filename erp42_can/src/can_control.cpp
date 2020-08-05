#include "can_control.h"
#include "can_variables.h"
using namespace unmansol::erp42;

ERP42Control::ERP42Control():
  m_nh("~"),
  m_AlvCnt(0)
{
  Init_data();
  Init_node();
}

void ERP42Control::Init_data()
{
  m_TMessage.ID = 0x777;
  m_TMessage.LEN = 8;
  m_TMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
  memset(m_TMessage.DATA, 0, sizeof(m_TMessage.DATA));

  m_pc2erp.MODE = 0x06; // Manual : 0x00, E-Stop ON : 0x02 , Gear-Neutral : 0x04
  m_pc2erp.speed._speed = 0x00;
  m_pc2erp.steer._steer = 0x00;
  m_pc2erp.brake = 0x00;
}

void ERP42Control::Init_node()
{
  m_sub_command = m_nh.subscribe("/erp42_can/command", 1, &ERP42Control::CmdCtrlMsgCallback, this);
}

void ERP42Control::Write()
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

bool ERP42Control::Connect()
{
  this->m_TStatus = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_500K, 0, 0, 0);

  //  std::cout << m_TStatus << std::endl;
  //
  if (this->m_TStatus) return false;
  else return true;
}

void ERP42Control::CmdCtrlMsgCallback(const erp42_msgs::CmdControl &msg)
{
  m_pc2erp.MODE = msg.MorA + msg.EStop + msg.Gear;
  m_pc2erp.speed._speed = msg.KPH * SPEED_FACTOR;
  m_pc2erp.steer._steer = msg.Deg * STEER_FACTOR;
  m_pc2erp.brake = msg.brake;
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "can_control");

  ERP42Control erp_control;

  ros::Rate loop(20);

  while(true)
  {
    bool isConnect = erp_control.Connect();
    if (isConnect) break;
    else ROS_WARN(" Not Connect! ");
  }

  while(ros::ok())
  {
    ros::spinOnce();
    erp_control.Write();
    loop.sleep();
  }

  return 0;
}
