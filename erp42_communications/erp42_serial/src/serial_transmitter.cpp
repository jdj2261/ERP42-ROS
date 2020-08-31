#include "serial_transmitter.h"

using namespace unmansol::erp42::serial;

ERP42Transmitter::ERP42Transmitter():
  m_nh("~"),
  m_AlvCnt(0)
{
  Init_data();
  Init_node();
}

void ERP42Transmitter::Init_data()
{
  m_pc2erp.speed._speed = 0x00;
  m_pc2erp.steer._steer = 0x00;
  m_pc2erp.brake = 0x00;
}

void ERP42Transmitter::Init_node()
{
  std::string ns = serial_interface_.ns_;
  m_sub_command = m_nh.subscribe(ns + "/command", 1, &ERP42Transmitter::CmdCtrlMsgCallback, this);
}

void ERP42Transmitter::Write()
{
  unsigned char writeBuffer[14];

  Update(writeBuffer);
  serial_interface_.ser.write(writeBuffer, 14);
}

void ERP42Transmitter::Update(unsigned char (&buffer)[14])
{
  m_pc2erp.speed.speed[0] = m_pc2erp.speed._speed & 0xff;
  m_pc2erp.speed.speed[1] = (m_pc2erp.speed._speed & 0xff00) >> 8;
  m_pc2erp.steer.steer[0] = (m_pc2erp.steer._steer & 0xff);
  m_pc2erp.steer.steer[1] = (m_pc2erp.steer._steer & 0xff00) >> 8;

  buffer[0] = m_pc2erp.S;
  buffer[1] = m_pc2erp.T;
  buffer[2] = m_pc2erp.X;
  buffer[3] = m_pc2erp.MorA;
  buffer[4] = m_pc2erp.E_stop;
  buffer[5] = m_pc2erp.gear;
  buffer[6] = m_pc2erp.speed.speed[1];
  buffer[7] = m_pc2erp.speed.speed[0];
  buffer[8] = m_pc2erp.steer.steer[1];
  buffer[9] = m_pc2erp.steer.steer[0];
  buffer[10] = m_pc2erp.brake;
  buffer[11] = m_AlvCnt++;
  buffer[12] = m_pc2erp.ETX0;
  buffer[13] = m_pc2erp.ETX1;

  for(int i=0; i<14; i++)
  {
    std::cout << std::hex << (int)buffer[i] << " ";
  }
  std::cout << std::endl;
}

void ERP42Transmitter::CmdCtrlMsgCallback(const erp42_msgs::CmdControl::Ptr &msg)
{
  m_pc2erp.MorA = msg->MorA;
  m_pc2erp.E_stop = msg->EStop;
  m_pc2erp.gear = msg->Gear;
  m_pc2erp.speed._speed = msg->KPH * SPEED_FACTOR;
  m_pc2erp.steer._steer = msg->Deg * STEER_FACTOR;
  m_pc2erp.brake = msg->brake;
}

//int main(int argc, char* argv[])
//{
//  ros::init(argc, argv, "serial_transmitter");

//  ERP42Transmitter erp_transmitter;
//  ros::NodeHandle nh;
//  while(nh.ok())
//  {
//      erp_transmitter.Write();
//      ros::spinOnce();
//  }

//  return 0;
//}
