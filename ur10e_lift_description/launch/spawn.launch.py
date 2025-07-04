#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    ns_arg = DeclareLaunchArgument('ns', default_value='sim',
                                   description='ROS namespace for the stack')
    ns = LaunchConfiguration('ns')

    pkg_share   = get_package_share_directory('ur10e_lift_description')
    xacro_path  = PathJoinSubstitution([pkg_share, 'urdf', 'ur10e_lift.urdf.xacro'])
    
    robot_desc = Command(['xacro ', xacro_path])

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc}],
               namespace=ns, output='screen')

    spawn = TimerAction(
        period=1.0,                                          
        actions=[Node(package='ros_gz_sim',
                      executable='create',
                      arguments=['-name','ur10e_lift','-topic','robot_description'],
                      namespace=ns,  output='screen')])


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

    return LaunchDescription([ns_arg, rsp, spawn, rviz])