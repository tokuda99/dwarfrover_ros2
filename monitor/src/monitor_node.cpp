#include "monitor.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Megarover3Monitor>());
  rclcpp::shutdown();
  return 0;
}
