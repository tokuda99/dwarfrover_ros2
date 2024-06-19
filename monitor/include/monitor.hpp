// standard
#include <cfloat>
#include <fstream>

// boost
#include <boost/circular_buffer.hpp>

// ros
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

class Megarover3Monitor : public rclcpp::Node {
  private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void batteryCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    void onTimer();
    void checkButteryVoltage(diagnostic_updater::DiagnosticStatusWrapper &stat);
    double average_twist_, last_played_time_;
    double voltage_;

    rclcpp::Time last_published_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr
        battery_sub_;
    diagnostic_updater::Updater updater_;
    rclcpp::TimerBase::SharedPtr timer_;

  public:
    explicit Megarover3Monitor(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~Megarover3Monitor();
};
