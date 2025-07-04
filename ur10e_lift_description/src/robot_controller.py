#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FixedRobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.joint_names = [
            'lift_joint',
            'ur10e_shoulder_pan_joint',
            'ur10e_shoulder_lift_joint', 
            'ur10e_elbow_joint',
            'ur10e_wrist_1_joint',
            'ur10e_wrist_2_joint',
            'ur10e_wrist_3_joint'
        ]
        
        self.joint_positions = [0.0] * len(self.joint_names)
        
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100Hz
        
        self.demo_step = 0
        self.demo_timer = None
        
        self.get_logger().info('Fixed Robot Controller started - publishing at 100Hz')
        
        self.start_demo_timer = self.create_timer(2.0, self.start_demo)

    def start_demo(self):
        self.start_demo_timer.cancel()
        self.get_logger().info('=== STARTING DEMO SEQUENCE ===')
        self.demo_step = 1
        self.demo_timer = self.create_timer(4.0, self.demo_sequence_step)
        self.demo_sequence_step()  # Run first step immediately

    def demo_sequence_step(self):
        if self.demo_step == 1:
            self.get_logger().info('Step 1: Lift up to 0.8m')
            self.set_joint_position('lift_joint', 0.8)
            
        elif self.demo_step == 2:
            self.get_logger().info('Step 2: Rotate base 90 degrees')
            self.set_joint_position('ur10e_shoulder_pan_joint', 1.57)
            
        elif self.demo_step == 3:
            self.get_logger().info('Step 3: Move shoulder lift down')
            self.set_joint_position('ur10e_shoulder_lift_joint', -1.0)
            
        elif self.demo_step == 4:
            self.get_logger().info('Step 4: Bend elbow')
            self.set_joint_position('ur10e_elbow_joint', 1.5)
            
        elif self.demo_step == 5:
            self.get_logger().info('Step 5: Rotate wrist')
            self.set_joint_position('ur10e_wrist_1_joint', 1.57)
            
        elif self.demo_step == 6:
            self.get_logger().info('Step 6: Reset to home position')
            for i in range(len(self.joint_positions)):
                self.joint_positions[i] = 0.0
            self.get_logger().info('=== DEMO SEQUENCE COMPLETE ===')
            self.demo_timer.cancel()
            return
            
        self.get_logger().info(f'Current positions: {[f"{pos:.2f}" for pos in self.joint_positions]}')
        
        self.demo_step += 1

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_state_pub.publish(msg)

    def set_joint_position(self, joint_name, position):
        if joint_name in self.joint_names:
            index = self.joint_names.index(joint_name)
            old_pos = self.joint_positions[index]
            self.joint_positions[index] = position
            self.get_logger().info(f'Set {joint_name}: {old_pos:.3f} -> {position:.3f}')
        else:
            self.get_logger().error(f'Unknown joint: {joint_name}')

def main():
    rclpy.init()
    controller = FixedRobotController()
    
    try:
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()