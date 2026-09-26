#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // defaults, params.yaml overrides these
  double resolution = this->declare_parameter<double>("resolution", 0.1);
  int width = this->declare_parameter<int>("width", 300);
  int height = this->declare_parameter<int>("height", 300);
  double inflation_radius = this->declare_parameter<double>("inflation_radius", 1.0);
  int max_cost = this->declare_parameter<int>("max_cost", 100);
  costmap_.configure(resolution, width, height, inflation_radius, max_cost);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_.update(*scan);

  nav_msgs::msg::OccupancyGrid msg = costmap_.toOccupancyGrid();
  // reuse the scan header so the grid is in the lidar frame
  msg.header = scan->header;
  msg.info.map_load_time = scan->header.stamp;
  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
