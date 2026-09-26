#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // obstacle_threshold: cells >= this are walls
    // cost_weight: how hard the path tries to stay away from inflated cells
    void configure(int obstacle_threshold, double cost_weight);

    // A* from (start_x, start_y) to (goal_x, goal_y), both in the map frame.
    // returns false if there's no path. path.header is left for the caller
    bool plan(const nav_msgs::msg::OccupancyGrid& map,
              double start_x, double start_y,
              double goal_x, double goal_y,
              nav_msgs::msg::Path& path);

  private:
    rclcpp::Logger logger_;
    int obstacle_threshold_ = 90;
    double cost_weight_ = 5.0;
};

}

#endif
