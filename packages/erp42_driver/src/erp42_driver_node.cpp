#include "erp42_driver.h"

using namespace unmansol::erp42;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "erp42_driver");

    std::shared_ptr<ERP42Driver> erp_driver = std::make_shared<ERP42Driver>();
    erp_driver->Run();

    return 0;
}
