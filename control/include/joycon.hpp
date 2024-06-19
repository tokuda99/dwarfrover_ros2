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
#include "sensor_msgs/msg/joy.hpp"

class JoyCtrlMegarover : public rclcpp::Node {
  private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void onTimer();
    void checkConnection(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void checkConnectionHealth(
        diagnostic_updater::DiagnosticStatusWrapper &stat);

    double round(double number, std::size_t n);
    bool isInit(const std::vector<float> axes);

    std::string device_;
    int linear_, angular_, forward_, backward_;
    double l_scale_, a_scale_;

    rclcpp::Time last_published_time_;
    boost::circular_buffer<bool> connection_history_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    diagnostic_updater::Updater updater_;
    rclcpp::TimerBase::SharedPtr timer_;

  public:
    explicit JoyCtrlMegarover(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~JoyCtrlMegarover();
};
