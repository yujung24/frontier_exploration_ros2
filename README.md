# frontier_exploration_ros2

이 패키지는 [mertgulerx/frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2)를 기반으로 작성되었습니다.

## 실행 방법

### 1. 의존성 설치

```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 빌드

```bash
cd ~/colcon_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select frontier_exploration_ros2
source install/setup.bash
```

### 3. Gazebo와 TurtleBot3 실행

새 터미널에서 패키지 환경을 적용한 뒤 실행합니다.

```bash
cd ~/colcon_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch frontier_exploration_ros2 custom_world.launch.py \
  world:=mixed_rooms.world \
  use_sim_time:=true
```

다른 월드를 사용할 경우 `world:=mixed_rooms.world`를 다음 파일 이름 중 하나로 변경할 수 있습니다.

- `different_room_sizes.world`
- `map_1.world`
- `map_2.world`
- `map_3.world`
- `map_4.world`
- `turning_corridors.world`
- `two_passages.world`

### 4. SLAM 및 Nav2 실행

별도 터미널에서 SLAM과 Nav2가 실행되어 `/map`, costmap, TF 및 `navigate_to_pose` 액션을 제공해야 합니다.

예시:

```bash
cd ~/colcon_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

다른 터미널에서:

```bash
cd ~/colcon_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

### 5. 탐색 노드 실행

```bash
cd ~/colcon_ws
source install/setup.bash

ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
  use_sim_time:=true
```

설정은 `config/params.yaml`에서 변경할 수 있습니다.
