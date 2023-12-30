#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tuple>

double phi; // Initialize phi with an appropriate initial value

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  geometry_msgs::msg::Pose pose = msg->pose.pose;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw); // Get roll, pitch, and yaw
  phi = yaw;                  // Update the global phi with the yaw value
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("absolute_motion");
  auto pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "wheel_speed", 10);
  auto position_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, odomCallback);

  rclcpp::sleep_for(std::chrono::seconds(1));

  auto velocity2twist = [](double dphi, double dx, double dy, double phi) {
    double wz = dphi;
    double vx = dx * cos(phi) + dy * sin(phi);
    double vy = -dx * sin(phi) + dy * cos(phi);
    return std::make_tuple(wz, vx, vy);
  };

  auto twist2wheels = [](double wz, double vx, double vy) {
    double r = 0.05;        // radius in meters
    double l = 0.17 / 2;    // half of the wheelbase in meters
    double w = 0.26969 / 2; // half of the track width in meters

    double u[4] = {0.0, 0.0, 0.0, 0.0};

    u[0] = (-wz * (l + w) + vx - vy) / r;
    u[1] = (wz * (l + w) + vx + vy) / r;
    u[2] = (wz * (l + w) + vx - vy) / r;
    u[3] = (-wz * (l + w) + vx + vy) / r;

    return std::array<double, 4>{u[0], u[1], u[2], u[3]};
  };

  std::vector<std::tuple<double, double, double>> motions = {
      {0.0, 1.0, -1.0},         {0.0, 1.0, 1.0},          {0.0, 1.0, 1.0},
      {1.570796326, 1.0, -1.0}, {3.14159265, -1.0, -1.0}, {0.0, -1.0, 1.0},
      {0.0, -1.0, 1.0},         {0.0, -1.0, -1.0},
  };

  for (const auto &motion : motions) {
    double dphi = std::get<0>(motion) / 7; // Get dphi from the tuple
    double dx = std::get<1>(motion) / 7;   // Get dx from the tuple
    double dy = std::get<2>(motion) / 7;   // Get dy from the tuple

    int iterations = 700;
    for (int i = 0; i < iterations; i++) {
      double wz, vx, vy;
      std::tie(wz, vx, vy) = velocity2twist(dphi, dx, dy, phi);
      rclcpp::spin_some(node);
      std::array<double, 4> u = twist2wheels(wz, vx, vy);
      auto msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
      msg->data.assign(u.begin(), u.end()); // Convert std::array to std::vector
      pub->publish(*msg);
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::array<double, 4> stop = {0, 0, 0, 0};
  auto stop_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
  stop_msg->data.assign(stop.begin(),
                        stop.end()); // Convert std::array to std::vector
  pub->publish(*stop_msg);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
