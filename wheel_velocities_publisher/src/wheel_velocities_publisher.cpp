#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);
    timer_ = this->create_wall_timer(
        3000ms, std::bind(&WheelVelocitiesPublisher::timerCallback, this));
  }

private:
  void timerCallback() {
    std_msgs::msg::Float32MultiArray msg;

    switch (state_) {
    case 0:
      RCLCPP_INFO(this->get_logger(), "Move forward");
      msg.data = {1, 1, 1, 1};
      break;
    case 1:
      RCLCPP_INFO(this->get_logger(), "Move backward");
      msg.data = {-1, -1, -1, -1};
      break;
    case 2:
      RCLCPP_INFO(this->get_logger(), "Move left");
      msg.data = {-1, 1, -1, 1};
      break;
    case 3:
      RCLCPP_INFO(this->get_logger(), "Move right");
      msg.data = {1, -1, 1, -1};
      break;
    case 4:
      RCLCPP_INFO(this->get_logger(), "Turn clockwise");
      msg.data = {1, -1, 1, -1};
      break;
    case 5:
      RCLCPP_INFO(this->get_logger(), "Turn counter-clockwise");
      msg.data = {-1, 1, -1, 1};
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Stop");
      msg.data = {0, 0, 0, 0};
      this->timer_->cancel();
      rclcpp::shutdown(); // Shutdown the node
      return;             // Exit the callback
    }

    publisher_->publish(msg);
    state_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  int state_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelocitiesPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
