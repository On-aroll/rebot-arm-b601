from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_model():
    try:
        path = Path(
            get_package_share_directory("rebotarm_bringup")
        ) / "config" / "rebotarm_hardware.yaml"
        if path.exists():
            with open(path, "r", encoding="utf-8") as f:
                return str((yaml.safe_load(f) or {}).get("default_model") or "rs")
    except (PackageNotFoundError, OSError):
        pass
    return "rs"


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("rebotarm_moveit_demos"),
            "config",
            "draw_square_rs.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=_default_model(),
                description="RS model (this workspace only contains the RS robot)",
            ),
            Node(
                package="rebotarm_moveit_demos",
                executable="draw_square",
                name="draw_square",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
