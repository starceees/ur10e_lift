#!/usr/bin/env python3
"""
Spawn the UR-10e-lift robot into a running Ignition-Gazebo world.

World terminal  ➜  ros2 launch ur10e_lift_description gz_sim.launch.py  ns:=sim
Spawn terminal ➜  ros2 launch ur10e_lift_description spawn.launch.py   ns:=sim
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    # ── namespace (default "sim") --------------------------------------------
    ns_arg = DeclareLaunchArgument('ns', default_value='sim',
                                   description='ROS namespace for the stack')
    ns = LaunchConfiguration('ns')

    # ── full URDF from Xacro --------------------------------------------------
    pkg_share   = get_package_share_directory('ur10e_lift_description')
    xacro_path  = PathJoinSubstitution([pkg_share, 'urdf', 'ur10e_lift.urdf.xacro'])
    
    # FIXED: Use Command with xacro directly
    robot_desc = Command(['xacro ', xacro_path])

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc}],
               namespace=ns, output='screen')

    # ── spawn the entity in Gazebo -------------------------------------------
    spawn = TimerAction(
        period=1.0,                                          # wait for empty world
        actions=[Node(package='ros_gz_sim',
                      executable='create',
                      arguments=['-name','ur10e_lift','-topic','robot_description'],
                      namespace=ns,  output='screen')])

    # ── controller spawners ---------------------------------------------------
    # cm_ns = PathJoinSubstitution(['/', ns, 'controller_manager'])

    # jsb = Node(package='controller_manager', executable='spawner',
    #            arguments=['joint_state_broadcaster',
    #                       '--controller-manager', cm_ns],
    #            namespace=ns, output='screen')

    # traj = Node(package='controller_manager', executable='spawner',
    #             arguments=['jt_controller',
    #                        '--controller-manager', cm_ns],
    #             namespace=ns, output='screen')

    # lift = Node(package='controller_manager', executable='spawner',
    #             arguments=['lift_controller',
    #                        '--controller-manager', cm_ns],
    #             namespace=ns, output='screen')

    # spawners = TimerAction(period=2.0, actions=[jsb, traj, lift])

    # ── RViz ------------------------------------------------------------------
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'ur10e_lift.rviz'])
    rviz = TimerAction(
        period=4.0,
        actions=[Node(package='rviz2', executable='rviz2',
                      arguments=['-d', rviz_cfg],
                      output='screen',
                      remappings=[
                          ('robot_description', [ns, '/robot_description']),
                          ('joint_states', [ns, '/joint_states']),
                          ('/tf', [ns, '/tf']),
                          ('/tf_static', [ns, '/tf_static'])
                      ])])

    # ── final description -----------------------------------------------------
    return LaunchDescription([ns_arg, rsp, spawn, rviz])