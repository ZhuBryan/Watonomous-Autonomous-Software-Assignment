#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void configure(double lookahead_distance, double goal_tolerance,
                   double linear_speed, double max_angular_speed);

    // pure pursuit. returns the twist to send, sets done = true when we should just stop
    // (empty path or close enough to the end)
    geometry_msgs::msg::Twist computeCommand(const nav_msgs::msg::Path& path,
                                             double x, double y, double yaw, bool& done) const;

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

  private:
    rclcpp::Logger logger_;
    double lookahead_distance_ = 1.0;
    double goal_tolerance_ = 0.3;
    double linear_speed_ = 0.5;
    double max_angular_speed_ = 1.0;
};

}

#endif
