#include "can_receiver.h"
#include "can_variables.h"

using namespace unmansol::erp42;

ERP42Receiver::ERP42Receiver()
{

}

void ERP42Receiver::read()
{

}

void ERP42Receiver::update()
{
  std::cout << "test" << std::endl;
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "can_receiver");

  ERP42Receiver erp_receiver;

  while(ros::ok())
  {
    erp_receiver.update();
    ros::spinOnce();
  }

  return 0;
}
