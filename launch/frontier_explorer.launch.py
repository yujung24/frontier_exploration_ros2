from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SAFETY_PARAMETER_OVERRIDES = {
    "safety_preferred_clearance_m": float,
    "safety_clearance_weight": float,
    "frontier_suppression_enabled": bool,
    "frontier_suppression_attempt_threshold": int,
    "frontier_suppression_timeout_s": float,
    "frontier_visit_tolerance": float,
}


def _parse_optional_bool(value: str, arg_name: str):
    normalized = value.strip().lower()
    if normalized == "":
        return None
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(
        f"Invalid boolean value for launch argument '{arg_name}': '{value}'. "
        "Expected one of: true/false, 1/0, yes/no, on/off"
    )


def _create_frontier_actions(context):
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    autostart_value = LaunchConfiguration("autostart").perform(context)
    control_service_enabled_value = LaunchConfiguration("control_service_enabled").perform(context)
    map_qos_durability = LaunchConfiguration("map_qos_durability")
    map_qos_autodetect_on_startup = LaunchConfiguration("map_qos_autodetect_on_startup")
    map_qos_autodetect_timeout_s = LaunchConfiguration("map_qos_autodetect_timeout_s")
    costmap_qos_reliability = LaunchConfiguration("costmap_qos_reliability")
    selection_mode = LaunchConfiguration("selection_mode")
    time_strategy = LaunchConfiguration("time_strategy")
    planner_id = LaunchConfiguration("planner_id")
    mission_manager_enabled = LaunchConfiguration("mission_manager_enabled")
    mission_config = LaunchConfiguration("mission_config")
    world_name = LaunchConfiguration("world_name")
    mission_output = LaunchConfiguration("mission_output")
    rviz_enabled = LaunchConfiguration("rviz_enabled")
    rviz_config = LaunchConfiguration("rviz_config")

    frontier_overrides: dict[str, Any] = {
        "use_sim_time": use_sim_time,
        "map_qos_durability": map_qos_durability,
        "map_qos_autodetect_on_startup": map_qos_autodetect_on_startup,
        "map_qos_autodetect_timeout_s": map_qos_autodetect_timeout_s,
        "costmap_qos_reliability": costmap_qos_reliability,
    }
    autostart_override = _parse_optional_bool(autostart_value, "autostart")
    if autostart_override is not None:
        frontier_overrides["autostart"] = autostart_override

    control_service_enabled_override = _parse_optional_bool(
        control_service_enabled_value,
        "control_service_enabled",
    )
    if control_service_enabled_override is not None:
        frontier_overrides["control_service_enabled"] = control_service_enabled_override

    mission_enabled_value = mission_manager_enabled.perform(context).strip().lower()
    if mission_enabled_value in ("true", "1", "yes", "on"):
        frontier_overrides["control_service_enabled"] = True

    selection_mode_value = selection_mode.perform(context).strip().lower()
    if selection_mode_value:
        frontier_overrides["selection_mode"] = selection_mode_value

    for name, value_type in SAFETY_PARAMETER_OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            frontier_overrides[name] = (
                _parse_optional_bool(value, name) if value_type is bool else value_type(value)
            )

    frontier_node = Node(
        package="frontier_exploration_ros2",
        executable="frontier_explorer",
        name="frontier_explorer",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            params_file,
            frontier_overrides,
        ],
    )

    selector_node = Node(
        package="frontier_exploration_ros2",
        executable="travel_time_selector.py",
        name="travel_time_selector",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "selection_mode": selection_mode,
                "time_strategy": time_strategy,
                "planner_id": planner_id,
            },
        ],
    )

    actions = [frontier_node]

    rviz_enabled_value = rviz_enabled.perform(context).strip().lower()
    if rviz_enabled_value in ("true", "1", "yes", "on"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_frontier_mission",
                namespace=namespace,
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    if selection_mode.perform(context).strip().lower() == "travel_time":
        actions.append(selector_node)

    mission_enabled_value = mission_manager_enabled.perform(context).strip().lower()
    if mission_enabled_value in ("true", "1", "yes", "on"):
        frontier_overrides["control_service_enabled"] = True
        mission_node = Node(
            package="frontier_exploration_ros2",
            executable="mission_manager.py",
            name="mission_manager",
            namespace=namespace,
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                params_file,
                {
                    "mission_config": mission_config,
                    "world_name": world_name,
                    "output": mission_output,
                    "planner_id": planner_id,
                    "use_sim_time": use_sim_time,
                },
            ],
        )
        actions.append(mission_node)

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory("frontier_exploration_ros2")
    default_params = (Path(pkg_share) / "config" / "params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for the frontier explorer node.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Parameter file for frontier_explorer.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Log level (debug, info, warn, error, fatal).",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="",
                description="Optional override for explorer autostart. Leave empty to use the parameter file.",
            ),
            DeclareLaunchArgument(
                "control_service_enabled",
                default_value="",
                description=(
                    "Optional override for frontier control service availability. "
                    "Leave empty to use the parameter file."
                ),
            ),
            DeclareLaunchArgument(
                "map_qos_durability",
                default_value="transient_local",
                description="Map durability: transient_local | volatile | system_default.",
            ),
            DeclareLaunchArgument(
                "map_qos_autodetect_on_startup",
                default_value="false",
                description="Enable startup-only map durability autodetect.",
            ),
            DeclareLaunchArgument(
                "map_qos_autodetect_timeout_s",
                default_value="2.0",
                description="Autodetect timeout per attempt in seconds.",
            ),
            DeclareLaunchArgument(
                "costmap_qos_reliability",
                default_value="reliable",
                description="Costmap reliability: reliable | best_effort | system_default.",
            ),
            DeclareLaunchArgument(
                "selection_mode",
                default_value="",
                description="Optional selection mode for the frontier explorer.",
            ),
            *[
                DeclareLaunchArgument(
                    name,
                    default_value="",
                    description="Optional explorer override. Leave empty to use the parameter file.",
                )
                for name in SAFETY_PARAMETER_OVERRIDES
            ],
            DeclareLaunchArgument(
                "time_strategy",
                default_value="C",
                description="Optional time strategy for the frontier explorer.",
            ),
            DeclareLaunchArgument(
                "planner_id",
                default_value="GridBased",
                description="Optional planner ID for the frontier explorer.",
            ),
            DeclareLaunchArgument(
                "mission_manager_enabled",
                default_value="false",
                description="Enable post-coverage start-pose return and fixed final-destination navigation.",
            ),
            DeclareLaunchArgument(
                "mission_config",
                default_value=str(Path(pkg_share) / "config" / "mission.yaml"),
                description="YAML file containing fixed final poses by world name.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="default",
                description="World key used in the Mission Manager YAML configuration.",
            ),
            DeclareLaunchArgument(
                "mission_output",
                default_value="mission_result.json",
                description="Output JSON path for mission navigation and timing data.",
            ),
            DeclareLaunchArgument(
                "rviz_enabled",
                default_value="true",
                description="Automatically launch RViz with live map, costmap, TF, laser scan, and frontier displays.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(Path(pkg_share) / "config" / "frontier_mission.rviz"),
                description="RViz configuration used for live exploration visualization.",
            ),
            OpaqueFunction(function=_create_frontier_actions),
        ]
    )
