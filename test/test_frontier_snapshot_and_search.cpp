/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
#include "frontier_exploration_ros2/safety_selection.hpp"

namespace frontier_exploration_ros2
{
namespace
{

// Validates frontier extraction primitives plus snapshot/marker cache behavior in core.

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = std::cos(yaw * 0.5);
  pose.orientation.z = std::sin(yaw * 0.5);
  return pose;
}

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  // Builds compact synthetic occupancy/cost grids for deterministic unit scenarios.
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<uint32_t>(width);
  msg.info.height = static_cast<uint32_t>(height);
  msg.info.resolution = 1.0;
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(width * height), static_cast<int8_t>(default_value));
  return msg;
}

void set_cells(
  nav_msgs::msg::OccupancyGrid & msg,
  const std::vector<std::pair<int, int>> & cells,
  int value)
{
  const int width = static_cast<int>(msg.info.width);
  for (const auto & [x, y] : cells) {
    msg.data[static_cast<std::size_t>(y * width + x)] = static_cast<int8_t>(value);
  }
}

std::pair<OccupancyGrid2d, OccupancyGrid2d> simple_frontier_world(
  const std::vector<std::pair<int, int>> & free_cells,
  const std::vector<std::pair<int, int>> & blocked_cells = {})
{
  // Occupancy starts unknown, then carve free islands and optional blocked cost cells.
  auto map_msg = build_grid(10, 10, -1);
  set_cells(map_msg, free_cells, 0);

  auto costmap_msg = build_grid(10, 10, 0);
  set_cells(costmap_msg, blocked_cells, 100);

  return {OccupancyGrid2d(map_msg), OccupancyGrid2d(costmap_msg)};
}

FrontierCandidate dummy_frontier(double x = 1.0, double y = 1.0)
{
  return FrontierCandidate{{x, y}, {x, y}, 8};
}

FrontierCandidate make_frontier(double x, double y, int size = 1)
{
  return FrontierCandidate{{x, y}, {x, y}, size};
}

void expect_frontier_results_equal(
  const FrontierSearchResult & expected,
  const FrontierSearchResult & actual)
{
  ASSERT_EQ(expected.frontiers.size(), actual.frontiers.size());
  ASSERT_EQ(expected.robot_map_cell, actual.robot_map_cell);
  for (std::size_t i = 0; i < expected.frontiers.size(); ++i) {
    EXPECT_EQ(expected.frontiers[i].centroid, actual.frontiers[i].centroid);
    EXPECT_EQ(expected.frontiers[i].goal_point, actual.frontiers[i].goal_point);
    EXPECT_EQ(expected.frontiers[i].robot_center_distance_m, actual.frontiers[i].robot_center_distance_m);
    EXPECT_EQ(expected.frontiers[i].size, actual.frontiers[i].size);
  }
}

std::unique_ptr<FrontierExplorerCore> make_snapshot_core()
{
  // Creates a core with stable clocks/maps so cache invalidation is fully controlled by tests.
  FrontierExplorerCoreParams params;
  FrontierExplorerCoreCallbacks callbacks;

  auto now_ns = std::make_shared<int64_t>(10'000'000'000);
  callbacks.now_ns = [now_ns]() {
      *now_ns += 1'000'000;
      return *now_ns;
    };
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};

  auto core = std::make_unique<FrontierExplorerCore>(params, callbacks);

  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);

  core->map = OccupancyGrid2d(map_msg);
  core->costmap = OccupancyGrid2d(costmap_msg);
  core->local_costmap = OccupancyGrid2d(costmap_msg);
  core->map_generation = 1;
  core->costmap_generation = 2;
  core->local_costmap_generation = 3;
  core->frontier_stats_log_throttle_seconds = 0.0;
  core->params.frontier_visit_tolerance = 0.3;

  return core;
}

// Frontier extraction behavior.
TEST(FrontierSearchTests, FrontierExtractionWhenRobotIsOnFreeCell)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  auto result = get_frontier(make_pose(4.0, 4.0), occupancy_map, costmap);

  EXPECT_FALSE(result.frontiers.empty());
  for (const auto & frontier : result.frontiers) {
    ASSERT_TRUE(frontier.robot_center_distance_m.has_value());
    EXPECT_GE(*frontier.robot_center_distance_m, 0.0);
  }
}

TEST(FrontierSearchTests, FrontierExtractionWhenRobotStartsUnknownUsesFindFree)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  auto result = get_frontier(make_pose(0.0, 0.0), occupancy_map, costmap);

  EXPECT_FALSE(result.frontiers.empty());
}

TEST(FrontierSearchTests, GlobalCostmapBlockingEliminatesFrontiers)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells, free_cells);
  auto result = get_frontier(make_pose(4.0, 4.0), occupancy_map, costmap);

  EXPECT_TRUE(result.frontiers.empty());
}

TEST(FrontierSearchTests, LocalCostmapBlockingDoesNotEliminateFrontierExtraction)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);

  auto local_costmap_msg = build_grid(10, 10, 0);
  set_cells(local_costmap_msg, free_cells, 100);
  auto local_costmap = OccupancyGrid2d(local_costmap_msg);

  auto result = get_frontier(
    make_pose(4.0, 4.0),
    occupancy_map,
    costmap,
    local_costmap);

  EXPECT_FALSE(result.frontiers.empty());
}

TEST(FrontierSearchTests, FrontierChoiceDeterministicAcrossIdenticalRuns)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 1; x < 4; ++x) {
    for (int y = 1; y < 4; ++y) {
      free_cells.emplace_back(x, y);
    }
  }
  for (int x = 6; x < 9; ++x) {
    for (int y = 6; y < 9; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);

  auto first = get_frontier(make_pose(2.0, 2.0), occupancy_map, costmap);
  auto second = get_frontier(make_pose(2.0, 2.0), occupancy_map, costmap);

  ASSERT_EQ(first.frontiers.size(), second.frontiers.size());
  for (std::size_t i = 0; i < first.frontiers.size(); ++i) {
    EXPECT_EQ(first.frontiers[i].centroid, second.frontiers[i].centroid);
    EXPECT_EQ(first.frontiers[i].goal_point, second.frontiers[i].goal_point);
    EXPECT_EQ(first.frontiers[i].size, second.frontiers[i].size);
  }
}

TEST(FrontierSearchTests, ConcurrentSearchProducesDeterministicEquivalentResults)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 2; x < 8; ++x) {
    for (int y = 2; y < 8; ++y) {
      if (!(x == 4 && y == 4)) {
        free_cells.emplace_back(x, y);
      }
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  const auto baseline = get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);

  auto future_one = std::async(std::launch::async, [&]() {
      return get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);
    });
  auto future_two = std::async(std::launch::async, [&]() {
      return get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);
    });

  const auto result_one = future_one.get();
  const auto result_two = future_two.get();
  expect_frontier_results_equal(baseline, result_one);
  expect_frontier_results_equal(baseline, result_two);
}

TEST(FrontierSearchTests, VisibleRevealGainStopsAtOccupiedWall)
{
  auto map_msg = build_grid(12, 12, -1);
  for (int x = 2; x <= 5; ++x) {
    set_cells(map_msg, {{x, 6}}, 0);
  }
  set_cells(map_msg, {{5, 6}}, 100);

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  const auto visible_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);

  ASSERT_TRUE(visible_gain.has_value());
  EXPECT_EQ(visible_gain->visible_reveal_cell_count, 0);
  EXPECT_DOUBLE_EQ(visible_gain->visible_reveal_length_m, 0.0);
}

TEST(FrontierSearchTests, VisibleRevealGainRespectsYawAndFov)
{
  auto map_msg = build_grid(12, 12, -1);
  for (int x = 2; x <= 4; ++x) {
    set_cells(map_msg, {{x, 6}}, 0);
  }
  set_cells(map_msg, {{3, 5}}, 100);

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  const auto east_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);
  const auto north_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, -1.57079632679),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);

  ASSERT_TRUE(east_gain.has_value());
  ASSERT_TRUE(north_gain.has_value());
  EXPECT_GT(east_gain->visible_reveal_cell_count, 0);
  EXPECT_EQ(north_gain->visible_reveal_cell_count, 0);
}

TEST(FrontierSearchTests, VisibleRevealGainStopsAtVirtualClusterBounds)
{
  auto map_msg = build_grid(12, 12, -1);
  for (int x = 2; x <= 4; ++x) {
    set_cells(map_msg, {{x, 6}}, 0);
  }

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  const auto unclipped_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);
  const auto clipped_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0,
    FrontierCandidate::CellBounds{2, 6, 4, 6});

  ASSERT_TRUE(unclipped_gain.has_value());
  ASSERT_TRUE(clipped_gain.has_value());
  EXPECT_GT(unclipped_gain->visible_reveal_cell_count, 0);
  EXPECT_EQ(clipped_gain->visible_reveal_cell_count, 0);
  EXPECT_DOUBLE_EQ(clipped_gain->visible_reveal_length_m, 0.0);
}

TEST(FrontierSearchTests, ExplorationCompleteCallbackRunsWhenNoFrontiersRemain)
{
  auto core = make_snapshot_core();
  core->params.return_to_start_on_complete = false;

  int completion_calls = 0;
  int frontier_search_calls = 0;
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 2.0));
    };
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.robot_map_cell = {2, 2};
      return result;
    };

  core->try_send_next_goal();
  core->try_send_next_goal();

  EXPECT_EQ(completion_calls, 1);
  EXPECT_EQ(frontier_search_calls, 1);
  EXPECT_TRUE(core->return_to_start_completed);
}

// Frontier snapshot cache behavior.
TEST(FrontierSnapshotTests, SnapshotCacheHitsOnSameGenerationsAndRobotCell)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  const auto first = core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  const auto second = core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 1);
  EXPECT_EQ(first.signature, second.signature);
  EXPECT_EQ(core->frontier_snapshot_cache_hits, 1);
}

TEST(FrontierSnapshotTests, SnapshotReusesCacheWhenOnlyRawMapGenerationChanges)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->map_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 1);
  EXPECT_EQ(core->frontier_snapshot_cache_hits, 1);
}

TEST(FrontierSnapshotTests, RefreshDecisionMapSkipsGenerationBumpWhenOutputIsUnchanged)
{
  auto core = make_snapshot_core();
  core->params.frontier_map_optimization_enabled = true;

  auto map_msg = build_grid(20, 20, -1);
  set_cells(map_msg, {{4, 4}, {4, 5}, {5, 4}, {5, 5}}, 0);
  core->map = OccupancyGrid2d(map_msg);

  core->refresh_decision_map();
  const int generation_after_first_refresh = core->decision_map_generation;
  const int misses_after_first_refresh = core->decision_map_cache_misses;

  core->refresh_decision_map();

  EXPECT_EQ(core->decision_map_generation, generation_after_first_refresh);
  EXPECT_EQ(core->decision_map_cache_misses, misses_after_first_refresh);
  EXPECT_EQ(core->decision_map_cache_hits, 1);
}

TEST(FrontierSnapshotTests, DebugOutputsDoNotChangeSnapshotOrSelection)
{
  auto debug_core = make_snapshot_core();
  auto quiet_core = make_snapshot_core();
  debug_core->params.frontier_map_optimization_enabled = true;
  quiet_core->params.frontier_map_optimization_enabled = true;
  debug_core->callbacks.debug_outputs_enabled = []() {return true;};
  quiet_core->callbacks.debug_outputs_enabled = []() {return false;};

  auto map_msg = build_grid(20, 20, -1);
  set_cells(
    map_msg,
    {
      {3, 3}, {3, 4}, {4, 3}, {4, 4},
      {10, 10}, {10, 11}, {11, 10}, {11, 11},
    },
    0);
  auto costmap_msg = build_grid(20, 20, 0);

  debug_core->map = OccupancyGrid2d(map_msg);
  quiet_core->map = OccupancyGrid2d(map_msg);
  debug_core->costmap = OccupancyGrid2d(costmap_msg);
  quiet_core->costmap = OccupancyGrid2d(costmap_msg);
  debug_core->local_costmap = OccupancyGrid2d(costmap_msg);
  quiet_core->local_costmap = OccupancyGrid2d(costmap_msg);
  debug_core->map_generation = 10;
  quiet_core->map_generation = 10;
  debug_core->costmap_generation = 11;
  quiet_core->costmap_generation = 11;
  debug_core->local_costmap_generation = 12;
  quiet_core->local_costmap_generation = 12;

  auto frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.robot_map_cell = {3, 3};
      result.frontiers.push_back(FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 4});
      result.frontiers.push_back(FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 9});
      return result;
    };
  debug_core->callbacks.frontier_search = frontier_search;
  quiet_core->callbacks.frontier_search = frontier_search;

  const auto debug_snapshot = debug_core->get_frontier_snapshot(make_pose(3.0, 3.0), 0.0);
  const auto quiet_snapshot = quiet_core->get_frontier_snapshot(make_pose(3.0, 3.0), 0.0);

  ASSERT_EQ(debug_snapshot.frontiers.size(), quiet_snapshot.frontiers.size());
  EXPECT_EQ(debug_snapshot.signature, quiet_snapshot.signature);
  for (std::size_t i = 0; i < debug_snapshot.frontiers.size(); ++i) {
    const auto & debug_candidate = debug_snapshot.frontiers[i];
    const auto & quiet_candidate = quiet_snapshot.frontiers[i];
    EXPECT_EQ(debug_candidate.centroid, quiet_candidate.centroid);
    EXPECT_EQ(debug_candidate.goal_point, quiet_candidate.goal_point);
    EXPECT_EQ(debug_candidate.robot_center_distance_m, quiet_candidate.robot_center_distance_m);
    EXPECT_EQ(debug_candidate.size, quiet_candidate.size);
  }

  const auto debug_selection = debug_core->select_frontier(debug_snapshot.frontiers, make_pose(3.0, 3.0));
  const auto quiet_selection = quiet_core->select_frontier(quiet_snapshot.frontiers, make_pose(3.0, 3.0));

  ASSERT_TRUE(debug_selection.frontier.has_value());
  ASSERT_TRUE(quiet_selection.frontier.has_value());
  const auto & debug_selected = *debug_selection.frontier;
  const auto & quiet_selected = *quiet_selection.frontier;
  EXPECT_EQ(debug_selected.centroid, quiet_selected.centroid);
  EXPECT_EQ(debug_selected.goal_point, quiet_selected.goal_point);
  EXPECT_EQ(debug_selected.robot_center_distance_m, quiet_selected.robot_center_distance_m);
  EXPECT_EQ(debug_selected.size, quiet_selected.size);
}

TEST(FrontierSnapshotTests, DecisionMapOptimizationFollowsParameterInMrtspOnlyCore)
{
  auto core = make_snapshot_core();
  core->params.frontier_map_optimization_enabled = false;

  const auto config = core->decision_map_config();

  EXPECT_FALSE(core->frontier_map_optimization_enabled());
  EXPECT_FALSE(config.optimization_enabled);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnDecisionMapGenerationChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->decision_map_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnCostmapGenerationChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->costmap_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, CostmapCallbacksOnlyBumpGenerationWhenSearchInputChanges)
{
  auto core = make_snapshot_core();
  const int initial_costmap_generation = core->costmap_generation;
  const int initial_local_costmap_generation = core->local_costmap_generation;

  auto unchanged_costmap = build_grid(20, 20, 0);
  core->costmapCallback(OccupancyGrid2d(unchanged_costmap));
  core->localCostmapCallback(OccupancyGrid2d(unchanged_costmap));

  EXPECT_EQ(core->costmap_generation, initial_costmap_generation);
  EXPECT_EQ(core->local_costmap_generation, initial_local_costmap_generation);

  auto changed_costmap = unchanged_costmap;
  set_cells(changed_costmap, {{3, 3}}, 100);
  core->costmapCallback(OccupancyGrid2d(changed_costmap));
  core->localCostmapCallback(OccupancyGrid2d(changed_costmap));

  EXPECT_EQ(core->costmap_generation, initial_costmap_generation + 1);
  EXPECT_EQ(core->local_costmap_generation, initial_local_costmap_generation + 1);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnRobotCellChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose & pose,
    const OccupancyGrid2d & occupancy,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = occupancy.worldToMap(pose.position.x, pose.position.y);
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->get_frontier_snapshot(make_pose(2.0, 2.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnMinGoalDistanceChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.4);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, GetFrontierSnapshotRefreshesDirtyDecisionMapBeforeCacheLookup)
{
  auto core = make_snapshot_core();

  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->ingestRawMapUpdate(OccupancyGrid2d(build_grid(20, 20, 0)));
  ASSERT_TRUE(core->decision_map_dirty);
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 1);
  EXPECT_FALSE(core->decision_map_dirty);
}

TEST(FrontierSnapshotTests, PostGoalSettleReadyDependsOnlyOnElapsedTime)
{
  auto core = make_snapshot_core();
  core->params.post_goal_settle_enabled = true;
  core->params.post_goal_min_settle = 0.5;
  core->awaiting_map_refresh = true;
  core->post_goal_settle_active = true;
  core->post_goal_settle_started_at_ns = core->callbacks.now_ns();

  EXPECT_FALSE(core->post_goal_settle_ready());

  core->post_goal_settle_started_at_ns = core->callbacks.now_ns() - 600'000'000;
  EXPECT_TRUE(core->post_goal_settle_ready());
}

TEST(FrontierSelectionTests, SelectFrontierSequenceReturnsFullMrtspOrdering)
{
  auto core = make_snapshot_core();
  const FrontierSequence frontiers{
    make_frontier(1.0, 0.0),
    make_frontier(2.0, 0.0),
    make_frontier(5.0, 0.0),
  };

  const auto frontier_sequence = core->select_frontier_sequence(
    frontiers,
    make_pose(0.0, 0.0),
    frontiers.front());

  const auto selection = core->select_frontier(frontiers, make_pose(0.0, 0.0));

  ASSERT_EQ(frontier_sequence.size(), frontiers.size());
  ASSERT_TRUE(selection.frontier.has_value());
  EXPECT_TRUE(core->are_frontiers_equivalent(frontier_sequence.front(), *selection.frontier));
}

TEST(FrontierSelectionTests, BuildGoalPoseFacesTargetWhenLookAheadUnavailable)
{
  auto core = make_snapshot_core();
  const auto goal_pose = core->build_goal_pose(
    make_frontier(1.0, 1.0),
    make_pose(0.0, 0.0, 1.2));

  EXPECT_NEAR(goal_pose.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(goal_pose.pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(goal_pose.pose.orientation.w, 0.9238795325, 1e-9);
  EXPECT_NEAR(goal_pose.pose.orientation.z, 0.3826834324, 1e-9);
}

// Marker publish deduplication behavior.
TEST(FrontierMarkerTests, MarkerPublishDeduplicatedWhenSignatureUnchanged)
{
  auto core = make_snapshot_core();
  int publish_calls = 0;
  core->callbacks.publish_frontier_markers = [&publish_calls](const FrontierSequence &) {
      publish_calls += 1;
    };

  FrontierSequence frontiers = FrontierExplorerCore::to_frontier_sequence({dummy_frontier(2.0, 2.0)});
  core->publish_frontier_markers(frontiers);
  core->publish_frontier_markers(frontiers);

  EXPECT_EQ(publish_calls, 1);
}

TEST(FrontierMarkerTests, MarkerPublishRunsWhenSignatureChanges)
{
  auto core = make_snapshot_core();
  int publish_calls = 0;
  core->callbacks.publish_frontier_markers = [&publish_calls](const FrontierSequence &) {
      publish_calls += 1;
    };

  core->publish_frontier_markers(
    FrontierExplorerCore::to_frontier_sequence({dummy_frontier(2.0, 2.0)}));
  core->publish_frontier_markers(
    FrontierExplorerCore::to_frontier_sequence({dummy_frontier(3.0, 3.0)}));

  EXPECT_EQ(publish_calls, 2);
}

TEST(FrontierSelectionTests, DISABLED_TravelTimeModePrefersAlignedFrontier)
{
  auto core = make_snapshot_core();
  core->params.selection_mode = "travel_time";
  core->params.max_linear_speed_vmax = 0.5;
  core->params.max_angular_speed_wmax = 1.0;
  core->params.travel_time_turn_weight = 1.0;
  core->params.travel_time_acceleration_weight = 1.0;

  const FrontierSequence frontiers{
    make_frontier(8.0, 0.0),
    make_frontier(-8.0, 0.0),
  };

  const auto selection = core->select_frontier(frontiers, make_pose(0.0, 0.0, 0.0));

  ASSERT_TRUE(selection.frontier.has_value());
  EXPECT_EQ(selection.mode, "travel_time");
  EXPECT_DOUBLE_EQ(frontier_position(*selection.frontier).first, 8.0);
}

TEST(FrontierSelectionTests, TravelTimeModeDelegatesExactNav2Evaluation)
{
  auto core = make_snapshot_core();
  core->params.selection_mode = "travel_time";

  const FrontierSequence frontiers{
    make_frontier(8.0, 0.0),
    make_frontier(-8.0, 0.0),
  };

  const auto selection = core->select_frontier(frontiers, make_pose(0.0, 0.0, 0.0));

  EXPECT_FALSE(selection.frontier.has_value());
  EXPECT_EQ(selection.mode, "travel_time");
}

TEST(FrontierSelectionTests, ObservationModeProducesAViewpointGoal)
{
  auto core = make_snapshot_core();
  core->params.selection_mode = "observation";
  core->params.observation_sensor_range_m = 8.0;
  core->params.observation_lidar_fov_deg = 360.0;
  core->params.observation_lidar_ray_step_deg = 15.0;
  core->params.observation_viewpoint_radius_m = 0.40;
  core->params.observation_viewpoint_count = 8.0;
  core->params.observation_distance_weight = 1.0;
  core->params.observation_gain_weight = 2.0;
  core->params.observation_obstacle_weight = 1.5;

  const FrontierSequence frontiers{
    make_frontier(8.0, 10.0),
  };

  const auto selection = core->select_frontier(frontiers, make_pose(10.0, 10.0, 0.0));

  ASSERT_TRUE(selection.frontier.has_value());
  EXPECT_EQ(selection.mode, "observation");
  ASSERT_TRUE(selection.frontier->goal_point.has_value());
  const auto goal = frontier_position(*selection.frontier);
  EXPECT_LE(std::hypot(goal.first - 8.0, goal.second - 10.0), 0.41);
  const auto sequence = core->select_frontier_sequence(
    frontiers, make_pose(10.0, 10.0, 0.0), selection.frontier);
  ASSERT_EQ(sequence.size(), 1U);
  EXPECT_EQ(frontier_position(sequence.front()), goal);
}

TEST(FrontierSelectionTests, SafetyModeSelectsReachableFrontier)
{
  auto core = make_snapshot_core();
  core->params.selection_mode = "safety";
  core->params.occ_threshold = 65;
  core->params.safety_robot_radius_m = 0.40;
  core->params.safety_preferred_clearance_m = 1.0;
  core->params.safety_distance_weight = 1.0;
  core->params.safety_clearance_weight = 2.0;

  const FrontierSequence frontiers{
    make_frontier(8.0, 10.0),
    make_frontier(12.0, 10.0),
  };

  const auto pose = make_pose(10.0, 10.0, 0.0);
  const auto selection = core->select_frontier(frontiers, pose);

  ASSERT_TRUE(selection.frontier.has_value());
  EXPECT_EQ(selection.mode, "safety");
  const auto sequence = core->select_frontier_sequence(frontiers, pose, selection.frontier);
  ASSERT_EQ(sequence.size(), 1U);
  EXPECT_EQ(frontier_position(sequence.front()), frontier_position(*selection.frontier));
}

TEST(FrontierSelectionTests, SafetyModeDoesNotDispatchLocallyBlockedFrontier)
{
  auto core = make_snapshot_core();
  core->params.selection_mode = "safety";
  core->params.occ_threshold = 65;
  auto local_costmap = build_grid(20, 20, 0);
  set_cells(local_costmap, {{7, 9}, {7, 10}, {8, 9}, {8, 10}}, 67);
  core->local_costmap = OccupancyGrid2d(local_costmap);

  const auto pose = make_pose(10.0, 10.0, 0.0);
  const FrontierSequence blocked{make_frontier(8.0, 10.0)};
  const auto selection = core->select_frontier(blocked, pose);
  EXPECT_EQ(selection.mode, "safety");
  EXPECT_FALSE(selection.frontier.has_value());
  EXPECT_TRUE(core->select_frontier_sequence(blocked, pose, selection.frontier).empty());

  const FrontierSequence mixed{blocked.front(), make_frontier(12.0, 10.0)};
  const auto alternate = core->select_frontier(mixed, pose);
  ASSERT_TRUE(alternate.frontier.has_value());
  EXPECT_EQ(frontier_position(*alternate.frontier), frontier_position(mixed.back()));
}

TEST(FrontierSelectionTests, SafetyModeFindsNearbyApproachForBlockedRepresentative)
{
  auto core = make_snapshot_core();
  core->params.selection_mode = "safety";
  core->params.occ_threshold = 65;
  auto local_costmap = build_grid(20, 20, 0);
  set_cells(local_costmap, {{8, 10}}, 67);
  core->local_costmap = OccupancyGrid2d(local_costmap);

  const auto pose = make_pose(12.0, 10.0, 0.0);
  const FrontierSequence frontiers{make_frontier(8.5, 10.5)};
  const auto selection = core->select_frontier(frontiers, pose);
  ASSERT_TRUE(selection.frontier.has_value());
  EXPECT_EQ(selection.mode, "safety");
  EXPECT_NE(frontier_position(*selection.frontier), frontier_position(frontiers.front()));
  EXPECT_FALSE(core->frontier_cost_status(selection.frontier).has_value());
}

TEST(FrontierSelectionTests, SafetyModePrefersLongerWideCorridorOverShorterNarrowCorridor)
{
  auto core = make_snapshot_core();
  core->params.selection_mode = "safety";
  core->params.occ_threshold = 65;
  core->params.safety_robot_radius_m = 0.28;
  core->params.safety_preferred_clearance_m = 0.80;
  core->params.safety_distance_weight = 1.0;
  core->params.safety_clearance_weight = 3.0;

  auto grid = build_grid(65, 70, 100);
  grid.info.resolution = 0.10;
  for (int y = 12; y <= 64; ++y) {
    for (int x = 11; x <= 29; ++x) {
      grid.data[y * grid.info.width + x] = 0;
    }
  }
  for (int y = 17; y <= 23; ++y) {
    for (int x = 20; x <= 55; ++x) {
      grid.data[y * grid.info.width + x] = 0;
    }
  }
  const OccupancyGrid2d corridors(grid);
  core->map = corridors;
  core->costmap = corridors;
  core->local_costmap = corridors;
  const auto [start_x, start_y] = corridors.mapToWorld(20, 20);
  const auto pose = make_pose(start_x, start_y);
  const auto [near_x, near_y] = corridors.mapToWorld(46, 20);
  const auto [far_x, far_y] = corridors.mapToWorld(20, 51);
  const FrontierSequence frontiers{make_frontier(near_x, near_y), make_frontier(far_x, far_y)};

  const auto score = [&](const FrontierLike & frontier) {
      return compute_safety_path_score(
        corridors, pose, frontier_position(frontier), core->params.safety_robot_radius_m,
        core->params.safety_preferred_clearance_m, core->params.safety_distance_weight,
        core->params.safety_clearance_weight, core->params.occ_threshold);
    };
  const auto near = score(frontiers.front());
  const auto far = score(frontiers.back());
  ASSERT_TRUE(near.reachable);
  ASSERT_TRUE(far.reachable);
  EXPECT_LT(near.path_length_m, far.path_length_m);
  EXPECT_GT(near.clearance_cost, far.clearance_cost);
  EXPECT_GT(near.path_length_m + near.clearance_cost, far.path_length_m + far.clearance_cost);
  RecordProperty("near_distance_m", std::to_string(near.path_length_m));
  RecordProperty("near_clearance_cost", std::to_string(near.clearance_cost));
  RecordProperty("far_distance_m", std::to_string(far.path_length_m));
  RecordProperty("far_clearance_cost", std::to_string(far.clearance_cost));

  const auto safety = core->select_frontier(frontiers, pose);
  ASSERT_TRUE(safety.frontier.has_value());
  EXPECT_EQ(frontier_position(*safety.frontier), frontier_position(frontiers.back()));

  core->params.safety_clearance_weight = 0.0;
  const auto distance_only = core->select_frontier(frontiers, pose);
  ASSERT_TRUE(distance_only.frontier.has_value());
  EXPECT_EQ(frontier_position(*distance_only.frontier), frontier_position(frontiers.front()));

  core->params.safety_robot_radius_m = 0.45;
  EXPECT_FALSE(score(frontiers.front()).reachable);
  const auto minimum_clearance = core->select_frontier(frontiers, pose);
  ASSERT_TRUE(minimum_clearance.frontier.has_value());
  EXPECT_EQ(frontier_position(*minimum_clearance.frontier), frontier_position(frontiers.back()));
}

TEST(FrontierSelectionTests, SafetyModeDoesNotApplyRobotRadiusTwiceToInflation)
{
  auto grid = build_grid(32, 32, 0);
  grid.info.resolution = 0.05;
  for (int y = 0; y < 32; ++y) {
    set_cells(grid, {{0, y}}, 100);
    set_cells(grid, {{1, y}, {2, y}, {3, y}, {4, y}, {5, y}}, 67);
    set_cells(grid, {{6, y}}, 58);
  }
  const OccupancyGrid2d costmap(grid);
  const auto [x, y] = costmap.mapToWorld(6, 5);
  const auto pose = make_pose(x, y);
  EXPECT_TRUE(compute_safety_path_score(
    costmap, pose, costmap.mapToWorld(6, 25), 0.28, 0.80, 1.0, 3.0, 65).reachable);
  EXPECT_FALSE(compute_safety_path_score(
    costmap, pose, costmap.mapToWorld(5, 25), 0.28, 0.80, 1.0, 3.0, 65).reachable);
  EXPECT_FALSE(compute_safety_path_score(
    costmap, pose, costmap.mapToWorld(6, 25), 0.31, 0.80, 1.0, 3.0, 65).reachable);
}

TEST(FrontierSelectionTests, SafetyModeDoesNotTreatUnknownSpaceAsAFreeShortcut)
{
  auto grid = build_grid(40, 30, 0);
  grid.info.resolution = 0.1;
  for (int y = 0; y < 30; ++y) {
    set_cells(grid, {{19, y}, {20, y}}, -1);
  }
  const OccupancyGrid2d disconnected(grid);
  const auto [start_x, start_y] = disconnected.mapToWorld(10, 10);
  const auto pose = make_pose(start_x, start_y);
  const auto goal = disconnected.mapToWorld(30, 10);
  EXPECT_FALSE(compute_safety_path_score(
    disconnected, pose, goal, 0.28, 0.80, 1.0, 3.0, 65).reachable);

  set_cells(grid, {{19, 16}, {20, 16}}, 0);
  const auto detour = compute_safety_path_score(
    OccupancyGrid2d(grid), pose, goal, 0.28, 0.80, 1.0, 3.0, 65);
  ASSERT_TRUE(detour.reachable);
  EXPECT_GT(detour.path_length_m, 2.0);
}

TEST(FrontierSelectionTests, SafetyModeCanExitButCannotEnterInsufficientClearance)
{
  auto grid = build_grid(30, 30, 0);
  grid.info.resolution = 0.05;
  for (int y = 0; y < 30; ++y) {
    set_cells(grid, {{0, y}}, 100);
  }
  const OccupancyGrid2d costmap(grid);
  const auto [start_x, start_y] = costmap.mapToWorld(5, 10);
  const auto [safe_x, safe_y] = costmap.mapToWorld(16, 10);
  const auto pose = make_pose(start_x, start_y);
  const auto safe = make_pose(safe_x, safe_y);
  EXPECT_TRUE(compute_safety_path_score(
    costmap, pose, {safe_x, safe_y}, 0.28, 0.80, 1.0, 3.0, 65).reachable);
  EXPECT_FALSE(compute_safety_path_score(
    costmap, safe, {start_x, start_y}, 0.28, 0.80, 1.0, 3.0, 65).reachable);
  EXPECT_FALSE(compute_safety_path_score(
    costmap, pose, costmap.mapToWorld(4, 10), 0.28, 0.80, 1.0, 3.0, 65).reachable);
}

}  // namespace
}  // namespace frontier_exploration_ros2
