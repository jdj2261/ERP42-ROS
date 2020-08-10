#include <erp42_test.h>

using namespace unmansol::erp42;

ERP42Test::ERP42Test():
  m_cnt(0),
  m_nh("~")
{
  Init_node();
}

void ERP42Test::Init_node()
{
  m_pub_test = m_nh.advertise<erp42_msgs::FeedBack2>("/erp42_can/feedback2",1);
}


void ERP42Test::encoder_test()
{
  m_encoder_msg.encoder = m_cnt++;
  m_pub_test.publish(m_encoder_msg);
  std::cout << m_encoder_msg.encoder << std::endl;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "erp42_test");

  ERP42Test erp_test;
  ros::Rate loop(50);

  while(ros::ok())
  {
    ros::spinOnce();
    erp_test.encoder_test();
    loop.sleep();
  }

  return 0;
}

