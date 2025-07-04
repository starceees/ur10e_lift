#!/usr/bin/env python3
"""
Launch Ignition Gazebo with an empty world (-r empty.sdf).

▪ Adds <ns> argument so every world can live in its own ROS 2 namespace.
▪ Prepends the paths that let Gazebo
      1) find   libgz_ros2_control-system.so
      2) resolve model://ur_description/… meshes.
"""

import os
import pathlib

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # ── launch argument: namespace ──────────────────────────────────────
    ns_arg = DeclareLaunchArgument("ns", default_value="sim")
    ns = LaunchConfiguration("ns")

    # ── resolved paths ──────────────────────────────────────────────────
    ur_root: str = get_package_share_directory("ur_description")          # …/share/ur_description
    ros_lib: str = "/opt/ros/humble/lib"                                  # plugin .so directory

    env_setup = [
        # 1 ▸ tell Gazebo where the ros-control system plugin lives
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            f"{os.getenv('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}:{ros_lib}",
        ),
        # 2 ▸ make model://ur_description/… meshes resolvable (but don't auto-spawn)
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            f"{os.getenv('IGN_GAZEBO_RESOURCE_PATH', '')}:{ur_root}",
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            f"{os.getenv('GZ_SIM_RESOURCE_PATH', '')}:{ur_root}",
        ),
    ]

    # ── include stock Gazebo-Sim launch (empty world) ───────────────────
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 {os.path.join(get_package_share_directory('ur10e_lift_description'), 'worlds', 'custom_empty.sdf')}",
            "ros_namespace": ns,
        }.items(),
    )

    # ── final LaunchDescription ─────────────────────────────────────────
    return LaunchDescription([ns_arg] + env_setup + [world])