#include <serial_receiver.h>
#include <serial_transmitter.h>

using namespace unmansol::erp42::serial;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "erp42_serial");
  ros::NodeHandle nh;

  ERP42Receiver* erp_receiver = new ERP42Receiver;
  ERP42Transmitter* erp_transmitter = new ERP42Transmitter;

  ros::Rate loop(50); // 50Hz is 0.02s
  while(ros::ok())
  {
    ros::spinOnce();
    erp_receiver->Read();
    erp_transmitter->Write();
    loop.sleep();
  }
  delete erp_receiver;
  delete erp_transmitter;
  return 0;
}


