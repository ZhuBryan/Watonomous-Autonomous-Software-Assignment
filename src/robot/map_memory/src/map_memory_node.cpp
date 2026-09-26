#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  double resolution = this->declare_parameter<double>("resolution", 0.1);
  int width = this->declare_parameter<int>("width", 400);
  int height = this->declare_parameter<int>("height", 400);
  double origin_x = this->declare_parameter<double>("origin_x", -20.0);
  double origin_y = this->declare_parameter<double>("origin_y", -20.0);
  std::string frame_id = this->declare_parameter<std::string>("frame_id", "sim_world");
  update_distance_ = this->declare_parameter<double>("update_distance", 1.5);
  int update_period_ms = this->declare_parameter<int>("update_period_ms", 1000);
  max_fuse_yaw_rate_ = this->declare_parameter<double>("max_fuse_yaw_rate", 0.2);

  map_memory_.configure(resolution, width, height, origin_x, origin_y, frame_id);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(update_period_ms), std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = robot::MapMemoryCore::yawFromQuaternion(msg->pose.pose.orientation);
  robot_yaw_rate_ = msg->twist.twist.angular.z;
  have_odom_ = true;
}

void MapMemoryNode::timerCallback() {
  if (have_costmap_ && have_odom_) {
    double distance = std::hypot(robot_x_ - last_fuse_x_, robot_y_ - last_fuse_y_);
    // costmap and odom can be ~0.1s apart, so while spinning the costmap gets pasted in
    // rotated a few degrees. just wait until we're not turning much
    bool turning = std::abs(robot_yaw_rate_) > max_fuse_yaw_rate_;
    // fuse right away the first time so the planner has something to work with
    if (!has_fused_ || (distance >= update_distance_ && !turning)) {
      map_memory_.fuse(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
      last_fuse_x_ = robot_x_;
      last_fuse_y_ = robot_y_;
      has_fused_ = true;
      RCLCPP_INFO(this->get_logger(), "Fused costmap at (%.2f, %.2f)", robot_x_, robot_y_);
    }
  }

  // publish every tick, not just on fuse, so anything that starts late still gets a map
  auto& map = map_memory_.map();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
