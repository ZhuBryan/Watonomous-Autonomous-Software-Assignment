#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::configure(double lookahead_distance, double goal_tolerance,
                            double linear_speed, double max_angular_speed) {
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
  max_angular_speed_ = max_angular_speed;
}

double ControlCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Twist ControlCore::computeCommand(const nav_msgs::msg::Path& path,
                                                      double x, double y, double yaw,
                                                      bool& done) const {
  geometry_msgs::msg::Twist cmd;  // all zeros = stop
  done = false;

  if (path.poses.empty()) {
    done = true;
    return cmd;
  }

  const auto& last = path.poses.back().pose.position;
  if (std::hypot(last.x - x, last.y - y) < goal_tolerance_) {
    done = true;
    return cmd;
  }

  // closest point on the path first, so we don't pick a lookahead point we've already passed
  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    double d = std::hypot(p.x - x, p.y - y);
    if (d < best) {
      best = d;
      closest = i;
    }
  }

  // then walk forward until something is at least lookahead_distance away.
  // if nothing is that far, aim at the end of the path
  size_t target = path.poses.size() - 1;
  for (size_t i = closest; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    if (std::hypot(p.x - x, p.y - y) >= lookahead_distance_) {
      target = i;
      break;
    }
  }

  const auto& t = path.poses[target].pose.position;
  double dx = t.x - x;
  double dy = t.y - y;
  double dist = std::hypot(dx, dy);

  // angle between where we're facing and where the target is, wrapped to [-pi, pi]
  double alpha = std::atan2(dy, dx) - yaw;
  alpha = std::atan2(std::sin(alpha), std::cos(alpha));

  if (std::abs(alpha) > M_PI / 2.0) {
    // target is behind us, turn in place first instead of doing a huge loop
    cmd.angular.z = std::copysign(max_angular_speed_, alpha);
    return cmd;
  }

  // pure pursuit: arc through the target has curvature 2 sin(alpha) / L
  double curvature = 2.0 * std::sin(alpha) / dist;
  cmd.linear.x = linear_speed_;
  cmd.angular.z = std::clamp(linear_speed_ * curvature, -max_angular_speed_, max_angular_speed_);
  return cmd;
}

}
