#!/usr/bin/env python3
"""
Robot Loader Node - Loads and publishes robot description and maintains basic joint states
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import subprocess
import os
from ament_index_python.packages import get_package_share_directory

class RobotLoader(Node):
    def __init__(self):
        super().__init__('robot_loader')
        
        # Load robot description
        self.load_robot_description()
        
        # Publisher for robot description
        self.robot_desc_pub = self.create_publisher(String, '/robot_description', 10)
        
        # Publisher for joint states (fallback only)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Joint names
        self.joint_names = [
            'lift_joint',
            'ur10e_shoulder_pan_joint',
            'ur10e_shoulder_lift_joint', 
            'ur10e_elbow_joint',
            'ur10e_wrist_1_joint',
            'ur10e_wrist_2_joint',
            'ur10e_wrist_3_joint'
        ]
        
        # Current joint positions (all start at zero)
        self.joint_positions = [0.0] * len(self.joint_names)
        
        # Subscribe to joint commands
        self.joint_command_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Timer to publish robot description and joint states
        self.robot_desc_timer = self.create_timer(1.0, self.publish_robot_description)
        self.joint_state_timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz
        
        self.get_logger().info('Robot Loader started - robot description loaded')
        self.get_logger().info('Listening for commands on /joint_commands topic')

    def load_robot_description(self):
        """Load robot description from URDF/Xacro"""
        try:
            pkg_share = get_package_share_directory('ur10e_lift_description')
            xacro_file = os.path.join(pkg_share, 'urdf', 'ur10e_lift.urdf.xacro')
            
            # Run xacro to get robot description
            self.robot_description = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')
            self.get_logger().info('Robot description loaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load robot description: {e}')
            self.robot_description = ""

    def publish_robot_description(self):
        """Publish robot description"""
        if self.robot_description:
            msg = String()
            msg.data = self.robot_description
            self.robot_desc_pub.publish(msg)

    def joint_command_callback(self, msg):
        """Update joint positions from commands"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                joint_index = self.joint_names.index(name)
                if i < len(msg.position):
                    self.joint_positions[joint_index] = msg.position[i]
                    self.get_logger().info(f'Updated {name}: {msg.position[i]:.3f}')

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_state_pub.publish(msg)

def main():
    rclpy.init()
    robot_loader = RobotLoader()
    
    try:
        rclpy.spin(robot_loader)
    except KeyboardInterrupt:
        robot_loader.get_logger().info('Shutting down robot loader...')
    finally:
        robot_loader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()