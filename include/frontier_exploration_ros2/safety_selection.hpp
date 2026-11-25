#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

struct SafetyPathScore
{
  bool reachable{false};
  double path_length_m{0.0};
  double clearance_cost{0.0};
};

SafetyPathScore compute_safety_path_score(
  const OccupancyGrid2d & costmap,
  const geometry_msgs::msg::Pose & start_pose,
  const std::pair<double, double> & goal,
  double robot_radius_m,
  double preferred_clearance_m,
  double distance_weight,
  double clearance_weight,
  int occ_threshold);

}  // namespace frontier_exploration_ros2
