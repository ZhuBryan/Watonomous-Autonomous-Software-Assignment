#include <chrono>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  double lookahead = this->declare_parameter<double>("lookahead_distance", 1.0);
  double goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.3);
  double linear_speed = this->declare_parameter<double>("linear_speed", 0.5);
  double max_angular = this->declare_parameter<double>("max_angular_speed", 1.0);
  control_.configure(lookahead, goal_tolerance, linear_speed, max_angular);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 10 Hz
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) {
    return;
  }

  const auto& pose = robot_odom_->pose.pose;
  double yaw = robot::ControlCore::yawFromQuaternion(pose.orientation);

  bool done = false;
  auto cmd = control_.computeCommand(*current_path_, pose.position.x, pose.position.y, yaw, done);

  if (done) {
    if (!stopped_) {
      cmd_vel_pub_->publish(cmd);  // zeros
      stopped_ = true;
    }
    return;
  }

  stopped_ = false;
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
