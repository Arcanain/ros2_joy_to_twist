#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
using namespace std::chrono_literals;

class JoyToTwist : public rclcpp::Node
{
public:
  JoyToTwist()
    : Node("joy_to_twist")
  {
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToTwist::joyCallback, this, std::placeholders::_1));

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_joystick", 10);

    // Example of a simple timer callback without arguments
    timer_ = this->create_wall_timer(100ms, std::bind(&JoyToTwist::timerCallback, this));
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() != 8) {
      return;
    }

    last_cmd_vel_joystick_ = geometry_msgs::msg::Twist();

    last_cmd_vel_joystick_.linear.x = msg->axes[1] * 0.5;
    // 無線の時
    //last_cmd_vel_joystick_.angular.z = msg->axes[2] * 1.0;
    // 有線の時
    last_cmd_vel_joystick_.angular.z = msg->axes[3] * 1.0;

  }

  void timerCallback()
  {
    RCLCPP_INFO(
      this->get_logger(), "Publishing: 'linear.x: '%.2f', angular.z: '%.2f'",
      last_cmd_vel_joystick_.linear.x, last_cmd_vel_joystick_.angular.z);
    vel_pub->publish(last_cmd_vel_joystick_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  geometry_msgs::msg::Twist last_cmd_vel_joystick_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyToTwist>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
