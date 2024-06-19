#include "joycon.hpp"

using namespace std::chrono_literals;

JoyCtrlMegarover::JoyCtrlMegarover(const rclcpp::NodeOptions &options)
    : Node("joycon", options),
      last_published_time_(0.0),
      connection_history_(2),
      updater_(this) {
    device_ = this->declare_parameter<std::string>("dev", "/dev/input/js0");
    linear_ = this->declare_parameter<int>("axis_linear", 4);
    angular_ = this->declare_parameter<int>("axis_angular", 0);
    a_scale_ = this->declare_parameter<double>("scale_angular", 0.8);
    l_scale_ = this->declare_parameter<double>("scale_linear", 0.6);
    forward_ = this->declare_parameter<int>("forward_button", 1);
    backward_ = this->declare_parameter<int>("backward_button", 0);

    auto use_sensor_data_qos =
        this->declare_parameter<bool>("use_sensor_data_qos", false);
    const auto qos =
        use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", qos,
        std::bind(&JoyCtrlMegarover::joyCallback, this, std::placeholders::_1));

    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

    updater_.setHardwareID("megarover_joy");
    updater_.add(
        "Controller Connection", this, &JoyCtrlMegarover::checkConnection);
    updater_.add(
        "Controller Connection Health", this,
        &JoyCtrlMegarover::checkConnectionHealth);

    connection_history_.push_front(false);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&JoyCtrlMegarover::onTimer, this));
}

JoyCtrlMegarover::~JoyCtrlMegarover() {}

void JoyCtrlMegarover::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    geometry_msgs::msg::Twist twist;
    if (msg->buttons[forward_]) {
        twist.angular.z = a_scale_ * msg->axes[angular_];

        double linear = (-msg->axes[linear_] + 1.0) / 2.0;
        if (msg->axes[5] < 0.0 && linear > 0.25) {
            linear = 0.25;
        }
        twist.linear.x = l_scale_ * linear;
    } else if (msg->buttons[backward_]) {
        twist.angular.z = a_scale_ * msg->axes[angular_];

        double linear = (-msg->axes[linear_] + 1.0) / 2.0;
        if (msg->axes[5] < 0.0 && linear > 0.25) {
            linear = 0.25;
        }
        twist.linear.x = l_scale_ * -linear;
    } else {
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
    }

    if (isInit(msg->axes)) {
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
    }

    vel_pub_->publish(twist);
    last_published_time_ = this->get_clock()->now();
}

double JoyCtrlMegarover::round(double number, std::size_t n) {
    number = number * std::pow(10, n - 1);
    number = std::round(number);
    number /= std::pow(10, n - 1);

    return number;
}

bool JoyCtrlMegarover::isInit(const std::vector<float> axes) {
    for (const auto &axe : axes) {
        if (!(fabsf(axe - 0.f) <=
              FLT_EPSILON * fmaxf(1.f, fmaxf(fabsf(axe), fabsf(0.f))))) {
            return false;
        }
    }

    return true;
}

void JoyCtrlMegarover::checkConnection(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
    std::ifstream ifs("/dev/input/js0");
    connection_history_.push_front(!ifs.fail());

    if (connection_history_[0]) {
        if (!connection_history_[1]) {
            // mr3api::audio::playAudio(
            //     audio_service_, mr3api::audio::getConnectFileName(),
            //     mr3api::audio::getConnectPath(), false);
        }

        stat.summaryf(
            diagnostic_msgs::msg::DiagnosticStatus::OK,
            "Connection established");
    } else {
        if (connection_history_[1]) {
            // mr3api::audio::playAudio(
            //     audio_service_, mr3api::audio::getDisconnectFileName(),
            //     mr3api::audio::getDisconnectPath(), false);
        }

        stat.summaryf(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Connection lost");
    }
}

void JoyCtrlMegarover::checkConnectionHealth(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (connection_history_[0]) {
        auto elapsed = (this->get_clock()->now() - last_published_time_).seconds();

        if (elapsed < 60) {
            stat.summary(
                diagnostic_msgs::msg::DiagnosticStatus::OK,
                "Connected (Recieved last message " +
                    std::to_string(static_cast<int>(elapsed)) +
                    " seconds ago)");
        } else if (elapsed >= 60 && elapsed < 300) {
            stat.summary(
                diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "Connection may have lost (Recieved last message " +
                    std::to_string(static_cast<int>(elapsed)) +
                    " seconds ago)");
        } else {
            stat.summary(
                diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "Connection lost (Recieved last message " +
                    std::to_string(static_cast<int>(elapsed)) +
                    " seconds ago)");
        }
    } else {
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Connection lost");
    }
}
void JoyCtrlMegarover::onTimer() { updater_.force_update(); }
