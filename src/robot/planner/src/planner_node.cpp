#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  int obstacle_threshold = this->declare_parameter<int>("obstacle_threshold", 90);
  double cost_weight = this->declare_parameter<double>("cost_weight", 5.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  goal_timeout_ = this->declare_parameter<double>("goal_timeout", 90.0);
  planner_.configure(obstacle_threshold, cost_weight);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  have_map_ = true;
  // map changed, old path might go through something new now
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (have_map_ && msg->header.frame_id != current_map_.header.frame_id) {
    RCLCPP_WARN(this->get_logger(), "Goal is in frame '%s' but map is in '%s', set the display frame to %s",
                msg->header.frame_id.c_str(), current_map_.header.frame_id.c_str(),
                current_map_.header.frame_id.c_str());
  }
  goal_ = *msg;
  goal_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    clearPath();
  } else if ((this->now() - goal_start_time_).seconds() > goal_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Timed out trying to reach goal, giving up");
    state_ = State::WAITING_FOR_GOAL;
    clearPath();
  }
}

bool PlannerNode::goalReached() const {
  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Can't plan yet, no map or odom");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id;

  if (!planner_.plan(current_map_, robot_x_, robot_y_, goal_.point.x, goal_.point.y, path)) {
    // keep the goal, next map update will try again
    return;
  }
  for (auto& pose : path.poses) {
    pose.header = path.header;
  }
  path_pub_->publish(path);
}

void PlannerNode::clearPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id;
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
