#include <erp42_driver.h>

using namespace unmansol::erp42;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "erp42_driver");

  ERP42Driver* erp_driver = new ERP42Driver ;
  erp_driver->Run();
  delete erp_driver;

  return 0;
}
