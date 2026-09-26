#include <algorithm>
#include <cmath>

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

  const int n = static_cast<int>(scan.ranges.size());
  if (n == 0) {
    return;
  }

  // how far each beam got before hitting something (range_max if it hit nothing), -1 = bad reading
  std::vector<double> reach(n, -1.0);
  for (int i = 0; i < n; ++i) {
    double range = scan.ranges[i];
    if (std::isnan(range) || range <= scan.range_min) {
      continue;
    }
    bool hit = std::isfinite(range) && range < scan.range_max;
    reach[i] = hit ? range : scan.range_max;
  }

  // free space: for every cell, find the beam pointing at it. if the cell is closer than
  // where that beam stopped, the beam went through it so it's free. doing it per cell
  // instead of tracing each beam means no unknown gaps between beams far from the robot
  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      double x = origin_x_ + (gx + 0.5) * resolution_;
      double y = origin_y_ + (gy + 0.5) * resolution_;
      double r = std::hypot(x, y);
      int i = static_cast<int>(std::lround((std::atan2(y, x) - scan.angle_min) / scan.angle_increment));
      if (i < 0 || i >= n || reach[i] < 0.0) {
        continue;
      }
      if (r < reach[i]) {
        grid_[gy * width_ + gx] = 0;
      }
    }
  }

  // obstacles go on after the free space
  for (int i = 0; i < n; ++i) {
    double range = scan.ranges[i];
    if (!std::isfinite(range) || range <= scan.range_min || range >= scan.range_max) {
      continue;
    }
    double angle = scan.angle_min + i * scan.angle_increment;
    int gx, gy;
    if (worldToGrid(range * std::cos(angle), range * std::sin(angle), gx, gy)) {
      markObstacle(gx, gy);
    }
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
