#include "frontier_exploration_ros2/safety_selection.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace frontier_exploration_ros2
{
namespace
{

constexpr int DX[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
constexpr int DY[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
constexpr double DIAGONAL = 1.4142135623730951;

bool cell_blocked(const OccupancyGrid2d & grid, int mx, int my, int occ_threshold)
{
  const int cost = grid.getCost(mx, my);
  return cost >= 255 || (cost >= 0 && cost >= occ_threshold);
}

std::vector<double> obstacle_clearance_map(
  const OccupancyGrid2d & grid,
  int occ_threshold)
{
  const int width = grid.getSizeX();
  const int height = grid.getSizeY();
  const double resolution = grid.map().info.resolution;
  const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<double> clearance(count, std::numeric_limits<double>::infinity());

  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;

  for (int my = 0; my < height; ++my) {
    for (int mx = 0; mx < width; ++mx) {
      if (cell_blocked(grid, mx, my, occ_threshold)) {
        const std::size_t index = static_cast<std::size_t>(my) * static_cast<std::size_t>(width)
          + static_cast<std::size_t>(mx);
        clearance[index] = 0.0;
        queue.emplace(0.0, index);
      }
    }
  }

  while (!queue.empty()) {
    const auto [distance, index] = queue.top();
    queue.pop();
    if (distance > clearance[index]) {
      continue;
    }

    const int mx = static_cast<int>(index % static_cast<std::size_t>(width));
    const int my = static_cast<int>(index / static_cast<std::size_t>(width));
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = mx + DX[direction];
      const int ny = my + DY[direction];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }

      const std::size_t neighbor = static_cast<std::size_t>(ny) * static_cast<std::size_t>(width)
        + static_cast<std::size_t>(nx);
      const double step = (DX[direction] == 0 || DY[direction] == 0)
        ? resolution : resolution * DIAGONAL;
      const double candidate = distance + step;
      if (candidate < clearance[neighbor]) {
        clearance[neighbor] = candidate;
        queue.emplace(candidate, neighbor);
      }
    }
  }

  return clearance;
}

}  // namespace

SafetyPathScore compute_safety_path_score(
  const OccupancyGrid2d & costmap,
  const geometry_msgs::msg::Pose & start_pose,
  const std::pair<double, double> & goal,
  double robot_radius_m,
  double preferred_clearance_m,
  double distance_weight,
  double clearance_weight,
  int occ_threshold)
{
  const int width = costmap.getSizeX();
  const int height = costmap.getSizeY();
  const double resolution = costmap.map().info.resolution;
  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;

  if (!costmap.worldToMapNoThrow(start_pose.position.x, start_pose.position.y, start_x, start_y) ||
      !costmap.worldToMapNoThrow(goal.first, goal.second, goal_x, goal_y))
  {
    return {};
  }

  const std::vector<double> clearance = obstacle_clearance_map(costmap, occ_threshold);
  const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const std::size_t start_index = static_cast<std::size_t>(start_y) * static_cast<std::size_t>(width)
    + static_cast<std::size_t>(start_x);
  const std::size_t goal_index = static_cast<std::size_t>(goal_y) * static_cast<std::size_t>(width)
    + static_cast<std::size_t>(goal_x);

  if ((std::isfinite(clearance[start_index]) && clearance[start_index] < robot_radius_m) ||
      (std::isfinite(clearance[goal_index]) && clearance[goal_index] < robot_radius_m)) {
    return {};
  }

  std::vector<double> total_cost(count, std::numeric_limits<double>::infinity());
  std::vector<double> path_length(count, 0.0);
  std::vector<double> clearance_cost(count, 0.0);
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  total_cost[start_index] = 0.0;
  queue.emplace(0.0, start_index);

  while (!queue.empty()) {
    const auto [current_cost, index] = queue.top();
    queue.pop();
    if (current_cost > total_cost[index]) {
      continue;
    }
    if (index == goal_index) {
      break;
    }

    const int mx = static_cast<int>(index % static_cast<std::size_t>(width));
    const int my = static_cast<int>(index / static_cast<std::size_t>(width));
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = mx + DX[direction];
      const int ny = my + DY[direction];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }

      if (DX[direction] != 0 && DY[direction] != 0) {
        if (cell_blocked(costmap, mx + DX[direction], my, occ_threshold) ||
            cell_blocked(costmap, mx, my + DY[direction], occ_threshold))
        {
          continue;
        }
      }

      const std::size_t neighbor = static_cast<std::size_t>(ny) * static_cast<std::size_t>(width)
        + static_cast<std::size_t>(nx);
      if (std::isfinite(clearance[neighbor]) && clearance[neighbor] < robot_radius_m) {
        continue;
      }

      const double step = (DX[direction] == 0 || DY[direction] == 0)
        ? resolution : resolution * DIAGONAL;
      const double preferred = std::max(preferred_clearance_m, robot_radius_m);
      const double clearance_ratio = std::clamp(clearance[neighbor] / preferred, 0.0, 1.0);
      const double wall_penalty = step * clearance_weight
        * (1.0 - clearance_ratio) * (1.0 - clearance_ratio);
      const double candidate = current_cost + step * distance_weight + wall_penalty;

      if (candidate < total_cost[neighbor]) {
        total_cost[neighbor] = candidate;
        path_length[neighbor] = path_length[index] + step;
        clearance_cost[neighbor] = clearance_cost[index] + wall_penalty;
        queue.emplace(candidate, neighbor);
      }
    }
  }

  if (!std::isfinite(total_cost[goal_index])) {
    return {};
  }

  return {true, path_length[goal_index], clearance_cost[goal_index]};
}

}  // namespace frontier_exploration_ros2
