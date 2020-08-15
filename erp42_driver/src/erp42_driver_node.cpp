#include <erp42_driver.h>

using namespace unmansol::erp42;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "erp42_driver");

  ERP42Driver erp_driver;
  erp_driver.run();

  return 0;
}
