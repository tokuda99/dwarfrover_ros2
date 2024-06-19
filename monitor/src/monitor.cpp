#include "monitor.hpp"

using namespace std::chrono_literals;

Megarover3Monitor::Megarover3Monitor(const rclcpp::NodeOptions &options)
    : Node("megarover3_monitor", options),
      average_twist_(0.0),
      last_played_time_(0.0),
      voltage_(0.0),
      last_published_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      updater_(this) {
    auto use_sensor_data_qos =
        this->declare_parameter<bool>("use_sensor_data_qos", false);
    const auto qos =
        use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "rover_odo", qos,
        std::bind(
            &Megarover3Monitor::twistCallback, this, std::placeholders::_1));

    battery_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "rover_sensor", qos,
        std::bind(
            &Megarover3Monitor::batteryCallback, this, std::placeholders::_1));

    updater_.setHardwareID("megarover_buttery");
    updater_.add(
        "Buttery Voltage", this, &Megarover3Monitor::checkButteryVoltage);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&Megarover3Monitor::onTimer, this));
}

Megarover3Monitor::~Megarover3Monitor() {}

void Megarover3Monitor::twistCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    double twist =
        std::hypot(std::hypot(msg->linear.x, msg->linear.y), msg->linear.z);

    average_twist_ = (0.5 * twist) + (1.0 - 0.5) * average_twist_;

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Twist " << twist << " Average twist " << average_twist_);

    if (average_twist_ < 1e-12) {
        average_twist_ = 0.0;
    }

    if (average_twist_ > 0.1) {
        // mr3api::audio::playAudio(audio_service_, "alert.wav",
        //                          "/home/tlab/sounds/system/sfx/alert.wav",
        //                          true);
    }
}

void Megarover3Monitor::batteryCallback(
    const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    voltage_ = static_cast<double>(msg->data[1]) / 1000.0;
    std::string percentage;
    if (26.0 < voltage_)
        percentage = "100";
    else if (25.5 <= voltage_ && voltage_ < 26.0)
        percentage = "90";
    else if (25.0 <= voltage_ && voltage_ < 25.5)
        percentage = "80";
    else if (24.5 <= voltage_ && voltage_ < 25.0)
        percentage = "70";
    else if (24.0 <= voltage_ && voltage_ < 24.5)
        percentage = "60";
    else if (23.5 <= voltage_ && voltage_ < 24.0)
        percentage = "50";
    else if (23.0 <= voltage_ && voltage_ < 23.5)
        percentage = "40";
    else if (22.5 <= voltage_ && voltage_ < 23.0)
        percentage = "30";
    else if (22.0 <= voltage_ && voltage_ < 22.5)
        percentage = "20";
    else if (21.5 <= voltage_ && voltage_ < 22.0)
        percentage = "10";
    else if (21.0 <= voltage_ && voltage_ < 21.5)
        percentage = "5";
    else if (voltage_ < 21.0)
        percentage = "0";

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Battery voltage: " << voltage_ << "V, " << percentage << "%");
    last_published_time_ = this->get_clock()->now();
}

void Megarover3Monitor::checkButteryVoltage(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
    auto elapsed = (this->get_clock()->now() - last_published_time_).seconds();
    if (elapsed > 5.0) {
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR,
            "No message received");
    } else {
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::OK,
            "Voltage: " + std::to_string(voltage_) + "V");
    }
}

void Megarover3Monitor::onTimer() { updater_.force_update(); }
