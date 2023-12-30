#include <algorithm> // for std::all_of
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10,
        std::bind(&KinematicModel::wheelSpeedCallback, this,
                  std::placeholders::_1));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void
  wheelSpeedCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

    double r = 0.05;        // radius in meters
    double l = 0.17 / 2;    // half of the wheelbase in meters
    double w = 0.26969 / 2; // half of the track width in meters

    // Kinematic model calculations
    double wz = r / 4 *
                (-1 / (l + w) * msg->data[0] + 1 / (l + w) * msg->data[1] +
                 1 / (l + w) * msg->data[2] - 1 / (l + w) * msg->data[3]);
    double vx =
        r / 4 * (msg->data[0] + msg->data[1] + msg->data[2] + msg->data[3]);
    double vy =
        r / 4 * (-msg->data[0] + msg->data[1] - msg->data[2] + msg->data[3]);

    // Create and publish Twist message
    geometry_msgs::msg::Twist twist;
    twist.angular.z = wz;
    twist.linear.x = vx;
    twist.linear.y = vy;
    publisher_->publish(twist);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicModel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
