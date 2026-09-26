#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::configure(double resolution, int width, int height,
                            double inflation_radius, int max_cost) {
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;

  // shift origin so the robot ends up in the middle of the grid
  origin_x_ = -(width_ * resolution_) / 2.0;
  origin_y_ = -(height_ * resolution_) / 2.0;

  grid_.assign(width_ * height_, -1);
}

void CostmapCore::update(const sensor_msgs::msg::LaserScan& scan) {
  // everything starts unknown, only stuff the lidar sees gets filled in
  std::fill(grid_.begin(), grid_.end(), -1);
  obstacles_.clear();

  // lidar is at (0,0) in its own frame
  int robot_gx, robot_gy;
  worldToGrid(0.0, 0.0, robot_gx, robot_gy);

  std::vector<std::pair<int, int>> hits;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    if (std::isnan(range) || range <= scan.range_min) {
      continue;
    }

    // inf / range_max = didn't hit anything, so the whole beam is free space
    bool hit = std::isfinite(range) && range < scan.range_max;
    double length = hit ? range : scan.range_max;

    double angle = scan.angle_min + i * scan.angle_increment;
    double x = length * std::cos(angle);
    double y = length * std::sin(angle);

    int gx, gy;
    bool on_grid = worldToGrid(x, y, gx, gy);
    raytraceFree(robot_gx, robot_gy, gx, gy);
    if (hit && on_grid) {
      hits.emplace_back(gx, gy);
    }
  }

  // mark obstacles after all the raytracing, otherwise a later beam can clear an earlier hit
  for (const auto& [gx, gy] : hits) {
    markObstacle(gx, gy);
  }

  inflateObstacles();
}

bool CostmapCore::worldToGrid(double x, double y, int& gx, int& gy) const {
  // floor not int cast, int cast rounds negatives the wrong way
  gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return inBounds(gx, gy);
}

bool CostmapCore::inBounds(int gx, int gy) const {
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void CostmapCore::raytraceFree(int x0, int y0, int x1, int y1) {
  // bresenham line
  int dx = std::abs(x1 - x0);
  int dy = -std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  while (!(x0 == x1 && y0 == y1)) {
    // starts in the middle so once it's off the grid it's not coming back
    if (!inBounds(x0, y0)) {
      return;
    }
    grid_[y0 * width_ + x0] = 0;

    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void CostmapCore::markObstacle(int gx, int gy) {
  int8_t& cell = grid_[gy * width_ + gx];
  if (cell != 100) {
    cell = 100;
    obstacles_.emplace_back(gx, gy);
  }
}

void CostmapCore::inflateObstacles() {
  int r_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  for (const auto& [ox, oy] : obstacles_) {
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;
        if (!inBounds(nx, ny)) {
          continue;
        }

        double distance = std::hypot(dx, dy) * resolution_;
        if (distance > inflation_radius_) {
          continue;
        }

        // cost = max_cost * (1 - d / r)
        int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
        int8_t& cell = grid_[ny * width_ + nx];
        // only ever raise the cost. cost > 0 so the edge of the circle doesn't mark unknown cells as free
        if (cost > 0 && cost > cell) {
          cell = static_cast<int8_t>(cost);
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::toOccupancyGrid() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;  // foxglove doesn't like an all zero quaternion
  msg.data = grid_;
  return msg;
}

}
