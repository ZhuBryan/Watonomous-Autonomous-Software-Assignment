#include <algorithm>
#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::configure(double resolution, int width, int height,
                              double origin_x, double origin_y, const std::string& frame_id) {
  map_.header.frame_id = frame_id;
  map_.info.resolution = resolution;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.origin.position.x = origin_x;
  map_.info.origin.position.y = origin_y;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(width * height, -1);
}

double MapMemoryCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void MapMemoryCore::fuse(const nav_msgs::msg::OccupancyGrid& costmap,
                         double robot_x, double robot_y, double robot_yaw) {
  // costmap info (lidar frame, assumes no rotation on its origin)
  const double c_res = costmap.info.resolution;
  const int c_w = costmap.info.width;
  const int c_h = costmap.info.height;
  const double c_ox = costmap.info.origin.position.x;
  const double c_oy = costmap.info.origin.position.y;

  // global map info
  const double m_res = map_.info.resolution;
  const int m_w = map_.info.width;
  const int m_h = map_.info.height;
  const double m_ox = map_.info.origin.position.x;
  const double m_oy = map_.info.origin.position.y;

  const double cos_t = std::cos(robot_yaw);
  const double sin_t = std::sin(robot_yaw);

  // rotate + translate the 4 costmap corners into the world to get a bounding box,
  // so we only loop over the part of the map the costmap can actually cover
  const double corners[4][2] = {
    {c_ox, c_oy}, {c_ox + c_w * c_res, c_oy},
    {c_ox, c_oy + c_h * c_res}, {c_ox + c_w * c_res, c_oy + c_h * c_res}};
  double min_wx = 1e9, max_wx = -1e9, min_wy = 1e9, max_wy = -1e9;
  for (const auto& corner : corners) {
    double wx = robot_x + corner[0] * cos_t - corner[1] * sin_t;
    double wy = robot_y + corner[0] * sin_t + corner[1] * cos_t;
    min_wx = std::min(min_wx, wx);
    max_wx = std::max(max_wx, wx);
    min_wy = std::min(min_wy, wy);
    max_wy = std::max(max_wy, wy);
  }
  int mx_start = std::max(0, static_cast<int>(std::floor((min_wx - m_ox) / m_res)));
  int mx_end = std::min(m_w - 1, static_cast<int>(std::floor((max_wx - m_ox) / m_res)));
  int my_start = std::max(0, static_cast<int>(std::floor((min_wy - m_oy) / m_res)));
  int my_end = std::min(m_h - 1, static_cast<int>(std::floor((max_wy - m_oy) / m_res)));

  // go map cell -> costmap cell instead of the other way around, that way there's no holes
  // when the robot is rotated
  for (int my = my_start; my <= my_end; ++my) {
    for (int mx = mx_start; mx <= mx_end; ++mx) {
      double wx = m_ox + (mx + 0.5) * m_res;
      double wy = m_oy + (my + 0.5) * m_res;

      // world -> lidar frame (subtract position, rotate by -yaw)
      double dx = wx - robot_x;
      double dy = wy - robot_y;
      double lx = dx * cos_t + dy * sin_t;
      double ly = -dx * sin_t + dy * cos_t;

      int cx = static_cast<int>(std::floor((lx - c_ox) / c_res));
      int cy = static_cast<int>(std::floor((ly - c_oy) / c_res));
      // corners of the bounding box stick out past the rotated costmap
      if (cx < 0 || cx >= c_w || cy < 0 || cy >= c_h) {
        continue;
      }

      // new data wins, unless it's unknown then keep what we had
      int8_t value = costmap.data[cy * c_w + cx];
      if (value >= 0) {
        map_.data[my * m_w + mx] = value;
      }
    }
  }
}

}
