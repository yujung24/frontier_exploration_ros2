# Frontier Exploration ROS 2

This project is based on [mertgulerx/frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2) and extends it with three frontier selection methods for autonomous exploration and performance evaluation using ROS 2, Gazebo, SLAM Toolbox, and Nav2.

## 1. Launch the Simulation

Run each command in a separate terminal.

### Terminal 1: Gazebo

```bash
ros2 launch frontier_exploration_ros2 custom_world.launch.py \
  world:=mixed_rooms.world \
  use_sim_time:=true \
  x_pose:=0.0 y_pose:=0.0
```

### Terminal 2: SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

### Terminal 3: Nav2

```bash
cd ~/colcon_ws/src/frontier_exploration_ros2
ros2 launch ./launch/burger_navigation.launch.py use_sim_time:=true
```

## 2. Frontier Selection Methods

Choose one of the following three methods.
Run only one frontier explorer at a time.

### Method 1: Safety-Based Selection

Selects frontiers based on path cost and obstacle clearance.
Safety rejects blocked targets and paths below the robot's minimum clearance.
If the representative target fails these checks, Safety scores nearby known-free
approach points. It waits for updated map data when no eligible goal remains.
Low observation gain alone does not cancel an active Safety goal.

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  use_sim_time:=true \
  selection_mode:=safety \
  safety_preferred_clearance_m:=0.80 \
  safety_clearance_weight:=3.0 \
  frontier_suppression_enabled:=true \
  frontier_suppression_attempt_threshold:=2 \
  frontier_suppression_timeout_s:=30.0 \
  frontier_visit_tolerance:=0.80
```

### Method 2: Observation-Based Selection

Selects frontiers based on their expected observation or information gain.
If Observation cannot select a viewpoint, the explorer tries the MRTSP order
with the normal costmap dispatch checks. Nav2 reports goal arrival in Observation mode.

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  use_sim_time:=true \
  selection_mode:=observation
```

### Method 3: Travel-Time-Based Selection

Selects frontiers based on estimated travel time using Strategy C.

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  use_sim_time:=true \
  selection_mode:=travel_time \
  time_strategy:=C \
  planner_id:=GridBased
```

## 3. Exploration Performance Evaluation

Run the coverage monitor in a separate terminal, preferably before starting frontier exploration, to evaluate exploration coverage against the ground-truth occupancy map.

The Burger worlds and reference maps use the same basename. Start Gazebo at
`x_pose:=0.0 y_pose:=0.0` to align the reference map with the exploration run.

| Gazebo world | Ground-truth YAML |
| --- | --- |
| `worlds/mixed_rooms.world` | `maps/ground_truth/mixed_rooms.yaml` |
| `worlds/two_passages.world` | `maps/ground_truth/two_passages.yaml` |
| `worlds/different_room_sizes.world` | `maps/ground_truth/different_room_sizes.yaml` |
| `worlds/turning_corridors.world` | `maps/ground_truth/turning_corridors.yaml` |

```bash
cd ~/colcon_ws/src/frontier_exploration_ros2

ros2 run frontier_exploration_ros2 coverage_monitor.py \
  --reference-map "$(pwd)/maps/ground_truth/mixed_rooms.yaml" \
  --target-coverage 0.90 \
  --output mixed_rooms_coverage.json
```

- **Reference map:** `maps/ground_truth/mixed_rooms.yaml`
- **Target coverage:** 90%
- **Output file:** `mixed_rooms_coverage.json`

For a fair comparison, use the same world, initial robot pose, and target coverage for all three methods.

Use a different output filename for each method to keep the evaluation results separate.

## 4. Optional Return-and-Final-Goal Mission

Enable the Mission Manager to stop exploration at 90% coverage, move to the mission start, and ask Nav2 to compute a global path to the final goal. Once planning succeeds, it sends one navigation goal for the final destination. Nav2 may replan during navigation.

Set the route in `config/mission.yaml` or pass another file with `mission_config:=...`. Coordinates use the `map` frame and yaw is in radians. Omit `start` to use the captured exploration start. Existing entries with `position` and `orientation` still specify a final goal.

```yaml
worlds:
  mixed_rooms:
    start: {x: 0.15, y: 0.0, yaw: 0.0}
    goal: {x: 1.5, y: -0.2, yaw: 0.0}
```

Launch the selected frontier method with:

```bash
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  use_sim_time:=true \
  selection_mode:=safety \
  mission_manager_enabled:=true \
  world_name:=mixed_rooms \
  planner_id:=GridBased \
  mission_output:=mixed_rooms_mission.json
```

Run the coverage monitor without its standalone stop request so only the Mission Manager controls the transition:

```bash
ros2 run frontier_exploration_ros2 coverage_monitor.py \
  --reference-map "$(pwd)/maps/ground_truth/mixed_rooms.yaml" \
  --target-coverage 0.90 \
  --no-auto-stop \
  --output mixed_rooms_coverage.json
```

The coverage monitor publishes `coverage_target_reached`. The RViz configuration shows the planned path on `/mission/global_path` and the sampled robot trajectory on `/mission/actual_path`. The mission result stores the planning time, path length, and return and final navigation results.

## Reference

- [Original Frontier Exploration ROS 2 Repository](https://github.com/mertgulerx/frontier_exploration_ros2)
