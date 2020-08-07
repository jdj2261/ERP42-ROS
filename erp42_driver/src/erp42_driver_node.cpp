#include <erp42_driver.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "erp42_driver");

  while(ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
