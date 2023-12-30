#include <array>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

class EightTrajectoryNode : public rclcpp::Node {
public:
  EightTrajectoryNode() : Node("eight_trajectory") {
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectoryNode::odomCallback, this,
                  std::placeholders::_1));
    wheel_speed_publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed",
                                                                 10);

    waypoints_ = {{{0.0, 1, -1},
                   {0.0, 1, 1},
                   {0.0, 1, 1},
                   {1.5708, 1, -1},
                   {-3.1415, -1, -1},
                   {0.0, -1, 1},
                   {0.0, -1, 1},
                   {0.0, -1, -1}}};
    current_waypoint_index_ = 0;
    dt_ = 3.0; // Time to reach each waypoint (adjust as needed)

    motion_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_), [this]() { moveToNextWaypoint(); });
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
  }

  void moveToNextWaypoint() {
    if (current_waypoint_index_ < waypoints_.size()) {
      auto &waypoint = waypoints_[current_waypoint_index_];
      RCLCPP_INFO(this->get_logger(),
                  "Moving to waypoint %zu: dx=%f, dy=%f, dphi=%f",
                  current_waypoint_index_, waypoint[1], waypoint[2], waypoint[0]);

      double dphi = waypoint[0];
      double dx = waypoint[1];
      double dy = waypoint[2];

      auto [wz, vx, vy] = displacement2velocity(dphi, dx, dy);

      moveRobot(wz, vx, vy);

      current_waypoint_index_++;
    } else {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "Completed all waypoints.");
    }
  }

  void moveRobot(double wz, double vx, double vy) {
    std_msgs::msg::Float32MultiArray wheel_speed_msg;
    auto [twist_wz, twist_vx, twist_vy] = velocity2twist(wz, vx, vy);
    auto wheelSpeeds = twist2wheels(twist_wz, twist_vx, twist_vy);
    wheel_speed_msg.data = {static_cast<float>(wheelSpeeds[0]),
                            static_cast<float>(wheelSpeeds[1]),
                            static_cast<float>(wheelSpeeds[2]),
                            static_cast<float>(wheelSpeeds[3])};
    wheel_speed_publisher_->publish(wheel_speed_msg);
  }

  std::array<double, 3> displacement2velocity(double dphi, double dx,
                                              double dy) {
    return {dphi / dt_, dx / dt_, dy / dt_};
  }

  std::array<double, 3> velocity2twist(double wz, double vx, double vy) {
    double transformed_vx = std::cos(yaw_) * vx - std::sin(yaw_) * vy;
    double transformed_vy = std::sin(yaw_) * vx + std::cos(yaw_) * vy;
    return {wz, transformed_vx, transformed_vy};
  }

  std::array<double, 4> twist2wheels(double wz, double vx, double vy) {
    double r = 0.05;        // radius in meters
    double l = 0.17 / 2;    // half of the wheelbase in meters
    double w = 0.26969 / 2; // half of the track width in meters
    return {(-l - w) / r * wz + vx / r - vy / r,
            (l + w) / r * wz + vx / r + vy / r,
            (l + w) / r * wz + vx / r - vy / r,
            (-l - w) / r * wz + vx / r + vy / r};
  }

  void stopRobot() {
    std_msgs::msg::Float32MultiArray stop_msg;
    stop_msg.data = {0.0f, 0.0f, 0.0f, 0.0f};
    wheel_speed_publisher_->publish(stop_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_publisher_;
  std::vector<std::array<double, 3>> waypoints_;
  size_t current_waypoint_index_;
  struct {
    double x = 0.0;
    double y = 0.0;
  } current_position_;
  double yaw_ = 0.0, roll_ = 0.0, pitch_ = 0.0;
  double dt_;
  rclcpp::TimerBase::SharedPtr motion_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EightTrajectoryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
