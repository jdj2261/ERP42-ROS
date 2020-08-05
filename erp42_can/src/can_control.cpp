#include "can_control.h"

using namespace unmansol::erp42;

ERP42Control::ERP42Control():
  m_nh("~")
{
  Init_node();

  m_send_msg.ID = 0x777;
  m_send_msg.LEN = 8;
  m_send_msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  memset(m_send_msg.DATA, 0, sizeof(m_send_msg.DATA));


}

void ERP42Control::Init_node()
{
  m_sub_command = m_nh.subscribe("/erp42_can/command", 1, &ERP42Control::CmdCtrlMsgCallback, this);
}

void ERP42Control::Write()
{
  uint8_t AlvCnt = 0;

  m_pc2erp.speed.speed[0] = m_pc2erp.speed._speed & 0xff;
  m_pc2erp.speed.speed[1] = (m_pc2erp.speed._speed & 0xff00) >> 8;
  m_pc2erp.steer.steer[0] = (m_pc2erp.steer._steer & 0xff);
  m_pc2erp.steer.steer[1] = (m_pc2erp.steer._steer & 0xff00) >> 8;

  m_send_msg.DATA[0] = m_pc2erp.MODE;
  m_send_msg.DATA[1] = m_pc2erp.speed.speed[1];
  m_send_msg.DATA[2] = m_pc2erp.speed.speed[0];
  m_send_msg.DATA[3] = m_pc2erp.steer.steer[1];
  m_send_msg.DATA[4] = m_pc2erp.steer.steer[0];
  m_send_msg.DATA[5] = m_pc2erp.brake;
  m_send_msg.DATA[7] = AlvCnt++;

  m_TStatus = CAN_Write(PCAN_DEVICE, &m_send_msg);

  if (m_TStatus != PCAN_ERROR_OK)
    ROS_WARN(" Failed connect ");
}

bool ERP42Control::Connect()
{
  this->m_TStatus = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_500K, 0, 0, 0);

//  std::cout << m_TStatus << std::endl;
  //
  if (this->m_TStatus) return false;
  else return true;
}

void ERP42Control::Start()
{
  Write();
  ROS_INFO("Start");
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

  ros::Rate loop(2);

  while(true)
  {
    bool isConnect = erp_control.Connect();
    if (isConnect) break;
    else ROS_WARN(" Not Connect! ");
  }

  while(ros::ok())
  {
    erp_control.Start();
    loop.sleep();
  }

  return 0;
}
