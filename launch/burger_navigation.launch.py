import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_share = get_package_share_directory("nav2_bringup")
    params_file = RewrittenYaml(
        source_file=os.path.join(nav2_share, "params", "nav2_params.yaml"),
        param_rewrites={
            "robot_radius": "0.10",
            "inflation_radius": "0.35",
            "required_movement_radius": "0.10",
            "movement_time_allowance": "15.0",
            "default_server_timeout": "100",
        },
        convert_types=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": params_file,
            }.items(),
        ),
    ])
