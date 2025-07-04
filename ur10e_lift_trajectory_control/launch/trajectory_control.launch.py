#!/usr/bin/env python3

import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get robot description from ur10e_lift_description package
    ur10e_pkg_share = get_package_share_directory('ur10e_lift_description')
    xacro_file = os.path.join(ur10e_pkg_share, 'urdf', 'ur10e_lift.urdf.xacro')
    robot_desc = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }],
        output='screen'
    )

    # Trajectory Planner
    trajectory_planner = Node(
        package='ur10e_lift_trajectory_control',
        executable='trajectory_planner',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        trajectory_planner,
        rviz
    ])
