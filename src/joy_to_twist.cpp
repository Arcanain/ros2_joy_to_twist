#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
using namespace std::chrono_literals;

class JoyToTwist : public rclcpp::Node
{
public:
  JoyToTwist()
    : Node("joy_to_twist"), previous_linear_x_(0.0), previous_angular_z_(0.0), previous_time_(this->get_clock()->now())
  {
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToTwist::joyCallback, this, std::placeholders::_1));

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_joystick", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&JoyToTwist::timerCallback, this));
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() != 8) {
      return;
    }

    last_cmd_vel_joystick_.linear.x = msg->axes[1] * 0.5;
    last_cmd_vel_joystick_.angular.z = msg->axes[3] * 1.0;
  }

  void timerCallback()
  {
    // 現在の時間を取得
    rclcpp::Time current_time = this->get_clock()->now();
    // 前回からの経過時間を計算
    double dt = (current_time - previous_time_).seconds();

    if (dt > 0) {
      // 角加速度を計算
      double delta_angular_z = last_cmd_vel_joystick_.angular.z - previous_angular_z_;
      double angular_acceleration = delta_angular_z / dt;
      double max_angular_acceleration = 0.5; // rad/s²

      // 線加速度を計算
      double delta_linear_x = last_cmd_vel_joystick_.linear.x - previous_linear_x_;
      double linear_acceleration = delta_linear_x / dt;
      double max_linear_acceleration = 0.3; // m/s²

      // 角加速度が上限を超えた場合に制限を適用
      if (std::abs(angular_acceleration) > max_angular_acceleration) {
        last_cmd_vel_joystick_.angular.z = previous_angular_z_ + std::copysign(max_angular_acceleration * dt, delta_angular_z);
      }

      // 線加速度が上限を超えた場合に制限を適用
      if (std::abs(linear_acceleration) > max_linear_acceleration) {
        last_cmd_vel_joystick_.linear.x = previous_linear_x_ + std::copysign(max_linear_acceleration * dt, delta_linear_x);
      }

      // 前回の速度と時間を更新
      previous_angular_z_ = last_cmd_vel_joystick_.angular.z;
      previous_linear_x_ = last_cmd_vel_joystick_.linear.x;
      previous_time_ = current_time;

      RCLCPP_INFO(
        this->get_logger(), "Publishing: 'linear.x: '%.2f', angular.z: '%.2f', linear_acc: '%.2f', angular_acc: '%.2f'",
        last_cmd_vel_joystick_.linear.x, last_cmd_vel_joystick_.angular.z, linear_acceleration, angular_acceleration);

      vel_pub->publish(last_cmd_vel_joystick_);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  geometry_msgs::msg::Twist last_cmd_vel_joystick_;
  double previous_linear_x_;
  double previous_angular_z_;
  rclcpp::Time previous_time_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyToTwist>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
