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
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## 2. Frontier Selection Methods

Choose one of the following three methods.
Run only one frontier explorer at a time.

### Method 1: Safety-Based Selection

Selects frontiers based on path cost and obstacle clearance.

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

## Reference

- [Original Frontier Exploration ROS 2 Repository](https://github.com/mertgulerx/frontier_exploration_ros2)
