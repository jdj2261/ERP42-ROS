#include <erp42_serial.h>

using namespace unmansol::erp42;
using namespace std;

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "erp42_serial");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string port_name;

  if (argc < 2)
  {
    port_name = "/dev/ttyUSB0";
  }
  else port_name = argv[1];
  int serial_baudrate = 115200;

  nh_private.getParam("serial_port", port_name);
  nh_private.getParam("serial_baudrate", serial_baudrate);

//  nh_private.getParam("serial_port", port_name);
//  nh_private.getParam("serial_baudrate", serial_baudrate);

  ERP42Serial* erp42_serial = new ERP42Serial(port_name.c_str(), serial_baudrate);

  ros::Rate loop(50); // 50Hz is 0.02s
  while(ros::ok())
  {
    ros::spinOnce();
    erp42_serial->Init_node();
    erp42_serial->Write();
    erp42_serial->Read();
    loop.sleep();
  }

  delete erp42_serial;
  return 0;

}
