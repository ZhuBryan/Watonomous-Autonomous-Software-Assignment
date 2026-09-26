#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // sets up an empty (all unknown) global map
    void configure(double resolution, int width, int height,
                   double origin_x, double origin_y, const std::string& frame_id);

    // paste a costmap into the global map using the robot pose it was taken at
    void fuse(const nav_msgs::msg::OccupancyGrid& costmap,
              double robot_x, double robot_y, double robot_yaw);

    nav_msgs::msg::OccupancyGrid& map() { return map_; }

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}

#endif
