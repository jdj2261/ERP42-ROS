#include "serial_receiver.h"

using namespace unmansol::erp42::serial;

ERP42Receiver::ERP42Receiver():
  m_nh("~"),
  m_read_data("")
{
  Init_node();
  Init_data();
}


void ERP42Receiver::Init_node()
{
  std::string ns = serial_interface_.ns_;
  m_pub_feedback = m_nh.advertise<erp42_msgs::SerialFeedBack>(ns+"/feedback",1);
}

void ERP42Receiver::Init_data()
{
  m_feedback_msg.MorA = 0x00;
  m_feedback_msg.EStop = 0x01;
  m_feedback_msg.Gear = 0x00;
  m_feedback_msg.speed = 0.0;
  m_feedback_msg.steer = 0.0;
  m_feedback_msg.brake = 0;
  m_feedback_msg.encoder = 0;
  m_feedback_msg.alive = 0;
}

void ERP42Receiver::Read()
{
  unsigned char num_read = 18;

  if(serial_interface_.ser.available())
  {
    serial_interface_.ser.readline(m_read_data, num_read);
    Update();
  }
  else
  {
    ROS_ERROR("Serial Not Connect!!");
  }
}

void ERP42Receiver::Update()
{
  int idx = 0;

  if (char(m_read_data[idx]) == 'S' && char(m_read_data[idx + 1]) == 'T' && char(m_read_data[idx + 2]) == 'X')
  {
    if (char(m_read_data[idx+16]) == 0x0D && char(m_read_data[idx + 17]) == 0x0A)
    {
      m_feedback_msg.MorA  = m_read_data[idx + 3];
      m_feedback_msg.EStop = m_read_data[idx + 4];
      m_feedback_msg.Gear  = m_read_data[idx + 5];

      int speed_now = 0;
      speed_now |= (int)((m_read_data[idx + 6]) & 0xff);
      speed_now |= (int)((m_read_data[idx + 7] << 8) & 0xff00);
      m_feedback_msg.speed = speed_now/SPEED_FACTOR;

      std::cout << m_read_data[idx + 6] << std::endl;

      int steer_now = 0;
      steer_now |= (int)((m_read_data[idx + 8]) & 0xff);
      steer_now |= (int)((m_read_data[idx + 9] << 8) & 0xff00);
      if (steer_now > 30000)
        steer_now = steer_now - 65536;
      m_feedback_msg.steer = steer_now/STEER_FACTOR;

      m_feedback_msg.brake   = m_read_data[idx + 10];

      m_feedback_msg.encoder |= (long)((m_read_data[idx + 11]) & 0xff);
      m_feedback_msg.encoder |= (long)((m_read_data[idx + 12] << 8) & 0xff00);
      m_feedback_msg.encoder |= (long)((m_read_data[idx + 13] << 16) & 0xff0000);
      m_feedback_msg.encoder |= (long)((m_read_data[idx + 14] << 24) & 0xff000000);

      m_feedback_msg.alive = m_read_data[idx+15];

      m_pub_feedback.publish(m_feedback_msg);
    }
  }
  else
  {
    ROS_WARN("Incorrect data has been entered");
  }

}

//int main(int argc, char* argv[])
//{
//  ros::init(argc, argv, "serial_receiver");

//  ERP42Receiver* erp_receiver = new ERP42Receiver ;
//  ros::NodeHandle nh;
//  while(nh.ok())
//  {
//    erp_receiver->Read();
//    ros::spinOnce();
//  }

//  delete erp_receiver;

//  return 0;
//}
