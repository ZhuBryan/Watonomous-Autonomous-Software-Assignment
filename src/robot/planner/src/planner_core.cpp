#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "planner_core.hpp"

namespace robot
{

namespace
{
// open set entry, sorted by f = g + h
struct AStarNode
{
  int index;
  double f_score;
};

struct CompareF
{
  bool operator()(const AStarNode& a, const AStarNode& b) const
  {
    // smallest f on top
    return a.f_score > b.f_score;
  }
};
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void PlannerCore::configure(int obstacle_threshold, double cost_weight) {
  obstacle_threshold_ = obstacle_threshold;
  cost_weight_ = cost_weight;
}

bool PlannerCore::plan(const nav_msgs::msg::OccupancyGrid& map,
                       double start_x, double start_y,
                       double goal_x, double goal_y,
                       nav_msgs::msg::Path& path) {
  const int w = map.info.width;
  const int h = map.info.height;
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;

  auto toCell = [&](double x, double y, int& cx, int& cy) {
    cx = static_cast<int>(std::floor((x - ox) / res));
    cy = static_cast<int>(std::floor((y - oy) / res));
    return cx >= 0 && cx < w && cy >= 0 && cy < h;
  };

  int sx, sy, gx, gy;
  if (!toCell(start_x, start_y, sx, sy) || !toCell(goal_x, goal_y, gx, gy)) {
    RCLCPP_WARN(logger_, "Start or goal is off the map");
    return false;
  }
  const int start = sy * w + sx;
  const int goal = gy * w + gx;

  // unknown (-1) counts as free, otherwise we could never plan into places we haven't seen
  if (map.data[goal] >= obstacle_threshold_) {
    RCLCPP_WARN(logger_, "Goal is inside an obstacle");
    return false;
  }

  // heuristic = straight line distance in meters. never overestimates since every step
  // costs at least its length, so A* still finds the best path
  auto heuristic = [&](int idx) {
    return std::hypot(idx % w - gx, idx / w - gy) * res;
  };

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(w * h, inf);
  std::vector<int> came_from(w * h, -1);
  std::vector<bool> closed(w * h, false);
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;

  g_score[start] = 0.0;
  open.push({start, heuristic(start)});

  // 8 neighbours
  const int ddx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int ddy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    int current = open.top().index;
    open.pop();

    // same cell can be in the queue more than once, skip the stale copies
    if (closed[current]) {
      continue;
    }
    closed[current] = true;

    if (current == goal) {
      found = true;
      break;
    }

    int cx = current % w;
    int cy = current / w;
    for (int k = 0; k < 8; ++k) {
      int nx = cx + ddx[k];
      int ny = cy + ddy[k];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
        continue;
      }
      int next = ny * w + nx;
      if (closed[next]) {
        continue;
      }
      int cell = map.data[next];
      if (cell >= obstacle_threshold_) {
        continue;
      }

      // step length (1 or sqrt 2 cells), scaled up the closer we get to an obstacle
      double step = std::hypot(ddx[k], ddy[k]) * res;
      double penalty = 1.0 + cost_weight_ * std::max(0, cell) / 100.0;
      double tentative_g = g_score[current] + step * penalty;

      if (tentative_g < g_score[next]) {
        g_score[next] = tentative_g;
        came_from[next] = current;
        open.push({next, tentative_g + heuristic(next)});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* couldn't find a path");
    return false;
  }

  // walk back from the goal, then flip it
  std::vector<int> cells;
  for (int idx = goal; idx != -1; idx = came_from[idx]) {
    cells.push_back(idx);
  }
  std::reverse(cells.begin(), cells.end());

  path.poses.clear();
  for (int idx : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = ox + (idx % w + 0.5) * res;
    pose.pose.position.y = oy + (idx / w + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return true;
}

}
