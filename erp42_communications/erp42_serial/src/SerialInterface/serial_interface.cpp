#include "SerialInterface/serial_interface.h"

using namespace unmansol::erp42;

SerialInterface::SerialInterface():
  m_nh("~"),
  m_serial_port("/dev/ttyUSB0"),
  m_serial_baudrate(115200)

{
  Init_param();
  Init_serial();
}

void SerialInterface::Init_param()
{
  ns_ = ros::this_node::getNamespace();
  m_nh.param<std::string>("serial_port", m_serial_port, "/dev/ttyUSB0");
  m_nh.param<int>("serial_baudrate", m_serial_baudrate, 115200);

  ROS_INFO("Serial Port     : %s", m_serial_port.c_str());
  ROS_INFO("Serial Baudrate : %d", m_serial_baudrate);

}

int SerialInterface::Init_serial()
{
  try
  {
      ser.setPort(m_serial_port);
      ser.setBaudrate(m_serial_baudrate);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
  }
  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if(ser.isOpen()){
      ROS_INFO_STREAM("Serial Port initialized");
  }else{
      return -1;
  }
}

//int main(int argc, char* argv[])
//{
//  ros::init(argc, argv, "test");

//  SerialInterface test;
//}
