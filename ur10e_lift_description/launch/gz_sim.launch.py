#!/usr/bin/env python3
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
    ns_arg = DeclareLaunchArgument("ns", default_value="sim")
    ns = LaunchConfiguration("ns")

    ur_root: str = get_package_share_directory("ur_description")      
    ros_lib: str = "/opt/ros/humble/lib"                            

    env_setup = [
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            f"{os.getenv('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}:{ros_lib}",
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            f"{os.getenv('IGN_GAZEBO_RESOURCE_PATH', '')}:{ur_root}",
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            f"{os.getenv('GZ_SIM_RESOURCE_PATH', '')}:{ur_root}",
        ),
    ]

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

    return LaunchDescription([ns_arg] + env_setup + [world])