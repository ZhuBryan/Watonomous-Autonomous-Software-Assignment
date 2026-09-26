#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_ = false;
    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    double robot_yaw_rate_ = 0.0;
    bool have_odom_ = false;

    // pose at the last fuse
    double last_fuse_x_ = 0.0, last_fuse_y_ = 0.0;
    bool has_fused_ = false;

    double update_distance_;
    double max_fuse_yaw_rate_;
};

#endif
