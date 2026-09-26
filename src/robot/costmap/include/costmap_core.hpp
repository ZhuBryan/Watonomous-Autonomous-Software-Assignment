#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // call this once before update()
    void configure(double resolution, int width, int height,
                   double inflation_radius, int max_cost);

    // rebuilds the whole grid from one scan (grid is centered on the lidar)
    void update(const sensor_msgs::msg::LaserScan& scan);

    // node fills in the header
    nav_msgs::msg::OccupancyGrid toOccupancyGrid() const;

  private:
    // meters -> cell. gx/gy get set even if off the grid, return tells you if it's on
    bool worldToGrid(double x, double y, int& gx, int& gy) const;
    bool inBounds(int gx, int gy) const;
    void markObstacle(int gx, int gy);
    void inflateObstacles();

    rclcpp::Logger logger_;

    double resolution_ = 0.1;  // m/cell
    int width_ = 300;
    int height_ = 300;
    double origin_x_ = 0.0;    // bottom left corner of cell (0,0)
    double origin_y_ = 0.0;
    double inflation_radius_ = 1.0;  // m
    int max_cost_ = 100;

    // row major, index = y * width + x
    // -1 unknown, 0 free, 1-100 cost
    std::vector<int8_t> grid_;
    // cells the lidar actually hit, used for inflation
    std::vector<std::pair<int, int>> obstacles_;
};

}

#endif
