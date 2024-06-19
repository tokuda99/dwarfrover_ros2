#include "joycon.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyCtrlMegarover>());
  rclcpp::shutdown();
  return 0;
}
