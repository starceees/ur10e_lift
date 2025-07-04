#!/usr/bin/env python3
"""
Plot trajectory data logged by the IK trajectory planner.
Usage: python3 plot_trajectory_data.py /tmp/trajectory_log.csv
"""

import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np

def plot_trajectory_data(csv_file):
    try:
        # Read the CSV file
        data = pd.read_csv(csv_file)
        print(f"Loaded {len(data)} data points from {csv_file}")
        
        # Create subplots
        fig, axes = plt.subplots(3, 3, figsize=(15, 12))
        fig.suptitle('UR10e Lift Robot Trajectory Analysis', fontsize=16)
        
        # End-effector position
        axes[0, 0].plot(data['time'], data['ee_x'], 'r-', label='X')
        axes[0, 0].plot(data['time'], data['ee_y'], 'g-', label='Y') 
        axes[0, 0].plot(data['time'], data['ee_z'], 'b-', label='Z')
        axes[0, 0].set_title('End-Effector Position')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # End-effector velocity
        axes[0, 1].plot(data['time'], data['ee_vx'], 'r-', label='Vx')
        axes[0, 1].plot(data['time'], data['ee_vy'], 'g-', label='Vy')
        axes[0, 1].plot(data['time'], data['ee_vz'], 'b-', label='Vz')
        axes[0, 1].set_title('End-Effector Velocity')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Velocity (m/s)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # End-effector acceleration
        axes[0, 2].plot(data['time'], data['ee_ax'], 'r-', label='Ax')
        axes[0, 2].plot(data['time'], data['ee_ay'], 'g-', label='Ay')
        axes[0, 2].plot(data['time'], data['ee_az'], 'b-', label='Az')
        axes[0, 2].set_title('End-Effector Acceleration')
        axes[0, 2].set_xlabel('Time (s)')
        axes[0, 2].set_ylabel('Acceleration (m/s²)')
        axes[0, 2].legend()
        axes[0, 2].grid(True)
        
        # Joint positions
        joint_names = ['lift_joint', 'ur10e_shoulder_pan_joint', 'ur10e_shoulder_lift_joint', 
                       'ur10e_elbow_joint', 'ur10e_wrist_1_joint', 'ur10e_wrist_2_joint', 'ur10e_wrist_3_joint']
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink']
        
        for i, (joint, color) in enumerate(zip(joint_names, colors)):
            if f'{joint}_pos' in data.columns:
                axes[1, 0].plot(data['time'], data[f'{joint}_pos'], color=color, label=joint.replace('ur10e_', ''))
        axes[1, 0].set_title('Joint Positions')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Position (rad/m)')
        axes[1, 0].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        axes[1, 0].grid(True)
        
        # Joint velocities
        for i, (joint, color) in enumerate(zip(joint_names, colors)):
            if f'{joint}_vel' in data.columns:
                axes[1, 1].plot(data['time'], data[f'{joint}_vel'], color=color, label=joint.replace('ur10e_', ''))
        axes[1, 1].set_title('Joint Velocities')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Velocity (rad/s or m/s)')
        axes[1, 1].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        axes[1, 1].grid(True)
        
        # Joint accelerations
        for i, (joint, color) in enumerate(zip(joint_names, colors)):
            if f'{joint}_acc' in data.columns:
                axes[1, 2].plot(data['time'], data[f'{joint}_acc'], color=color, label=joint.replace('ur10e_', ''))
        axes[1, 2].set_title('Joint Accelerations')
        axes[1, 2].set_xlabel('Time (s)')
        axes[1, 2].set_ylabel('Acceleration (rad/s² or m/s²)')
        axes[1, 2].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        axes[1, 2].grid(True)
        
        # 3D trajectory plot
        ax_3d = plt.figure(figsize=(10, 8)).add_subplot(111, projection='3d')
        ax_3d.plot(data['ee_x'], data['ee_y'], data['ee_z'], 'r-', linewidth=2, label='End-effector path')
        ax_3d.scatter(data['ee_x'].iloc[0], data['ee_y'].iloc[0], data['ee_z'].iloc[0], 
                     color='green', s=100, label='Start')
        ax_3d.scatter(data['ee_x'].iloc[-1], data['ee_y'].iloc[-1], data['ee_z'].iloc[-1], 
                     color='red', s=100, label='End')
        ax_3d.set_xlabel('X (m)')
        ax_3d.set_ylabel('Y (m)')
        ax_3d.set_zlabel('Z (m)')
        ax_3d.set_title('3D End-Effector Trajectory')
        ax_3d.legend()
        
        # Speed profile
        speed = np.sqrt(data['ee_vx']**2 + data['ee_vy']**2 + data['ee_vz']**2)
        axes[2, 0].plot(data['time'], speed, 'purple', linewidth=2)
        axes[2, 0].set_title('End-Effector Speed Profile')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_ylabel('Speed (m/s)')
        axes[2, 0].grid(True)
        
        # Acceleration magnitude
        acc_mag = np.sqrt(data['ee_ax']**2 + data['ee_ay']**2 + data['ee_az']**2)
        axes[2, 1].plot(data['time'], acc_mag, 'orange', linewidth=2)
        axes[2, 1].set_title('End-Effector Acceleration Magnitude')
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].set_ylabel('Acceleration (m/s²)')
        axes[2, 1].grid(True)
        
        # Statistics summary
        stats_text = f"""
        Trajectory Statistics:
        Duration: {data['time'].iloc[-1]:.2f} s
        Max Speed: {speed.max():.3f} m/s
        Max Acceleration: {acc_mag.max():.3f} m/s²
        Path Length: {np.sum(np.sqrt(np.diff(data['ee_x'])**2 + np.diff(data['ee_y'])**2 + np.diff(data['ee_z'])**2)):.3f} m
        Avg Speed: {speed.mean():.3f} m/s
        """
        axes[2, 2].text(0.1, 0.5, stats_text, fontsize=10, verticalalignment='center')
        axes[2, 2].set_title('Trajectory Statistics')
        axes[2, 2].axis('off')
        
        plt.tight_layout()
        plt.show()
        
        print("\nTrajectory Analysis Complete!")
        print(f"Max end-effector speed: {speed.max():.3f} m/s")
        print(f"Max end-effector acceleration: {acc_mag.max():.3f} m/s²")
        print(f"Total path length: {np.sum(np.sqrt(np.diff(data['ee_x'])**2 + np.diff(data['ee_y'])**2 + np.diff(data['ee_z'])**2)):.3f} m")
        
    except Exception as e:
        print(f"Error reading or plotting data: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 plot_trajectory_data.py <csv_file>")
        print("Example: python3 plot_trajectory_data.py /tmp/trajectory_log.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    plot_trajectory_data(csv_file)