#include "erp42_serial.h"

using namespace unmansol::erp42;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "erp42_serial");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::string port_name;

    if (argc < 2) port_name = "/dev/ttyUSB0";
    else port_name = argv[1];
    int serial_baudrate = 115200;

    nh_private.getParam("serial_port", port_name);
    nh_private.getParam("serial_baudrate", serial_baudrate);

    std::shared_ptr<ERP42Serial> erp42_serial = std::make_shared<ERP42Serial>(port_name.c_str(), serial_baudrate);
    erp42_serial->Run();

    return 0;
}
