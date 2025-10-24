#!/usr/bin/env python3
"""
Encoder-based Trajectory Estimation using Secant Model

This ROS node reads encoder data from joint_states and ground truth from gazebo/model_states,
calculates trajectory using the secant model, and compares with ground truth.

Usage:
    python3 encoder_trajectory.py

Parameters:
    - Turtlebot3 Burger standard parameters
    - Secant model for odometry calculation
    - Real-time trajectory plotting
"""

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import math


class EncoderTrajectoryEstimator:
    def __init__(self):
        # Turtlebot3 Burger parameters
        self.wheel_radius = 0.033  # meters
        self.wheel_base = 0.160    # meters

        # Trajectory data
        self.encoder_trajectory = []
        self.ground_truth_trajectory = []
        self.timestamps = []

        # Robot state
        self.current_pose = [0.0, 0.0, 0.0]  # [x, y, theta] - world coordinates
        self.previous_positions = None
        self.last_wheel_positions = None  # Store last wheel positions for incremental calculation

        # Initial pose from ground truth (world coordinates)
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.0

        # Data storage
        self.joint_data = []
        self.ground_truth_data = []

        # Rosbag file
        # self.bag_file = "2025-10-23-18-22-52.bag"
        self.bag_file = "2025-10-24-10-34-42.bag"


    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Convert quaternion to yaw angle using manual calculation"""
        # Manual quaternion to yaw conversion
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_displacement_increment(self, current_positions):
        """Calculate displacement and angle increments using secant model"""
        if self.last_wheel_positions is None:
            self.last_wheel_positions = current_positions
            return 0.0, 0.0

        # Calculate wheel displacements in meters (positions are in radians)
        delta_right = (current_positions[0] - self.last_wheel_positions[0]) * self.wheel_radius
        delta_left = (current_positions[1] - self.last_wheel_positions[1]) * self.wheel_radius

        # Secant model calculations
        delta_s = (delta_right + delta_left) / 2.0  # Linear displacement
        delta_theta = (delta_right - delta_left) / self.wheel_base  # Angular displacement

        self.last_wheel_positions = current_positions
        return delta_s, delta_theta

    def update_trajectory(self, delta_s, delta_theta, timestamp):
        """Update trajectory using secant model formula with world coordinate transformation"""
        if len(self.encoder_trajectory) == 0:
            # First point - use initial pose from ground truth
            self.encoder_trajectory.append([self.initial_x, self.initial_y, self.initial_theta])
            self.timestamps.append(timestamp)
            return

        # Get previous pose (world coordinates)
        prev_x, prev_y, prev_theta = self.encoder_trajectory[-1]

        # Calculate incremental movement in robot-local coordinate system
        # This is the movement relative to the robot's current orientation
        delta_x_local = delta_s * math.cos(delta_theta / 2.0)
        delta_y_local = delta_s * math.sin(delta_theta / 2.0)

        # Transform from robot-local coordinates to world coordinates
        # using the current world orientation
        delta_x_world = delta_x_local * math.cos(prev_theta) - delta_y_local * math.sin(prev_theta)
        delta_y_world = delta_x_local * math.sin(prev_theta) + delta_y_local * math.cos(prev_theta)

        # Update world coordinates
        current_x = prev_x + delta_x_world
        current_y = prev_y + delta_y_world
        current_theta = prev_theta + delta_theta

        self.encoder_trajectory.append([current_x, current_y, current_theta])
        self.timestamps.append(timestamp)

    def process_rosbag(self):
        """Process the rosbag file"""
        print(f"Processing rosbag: {self.bag_file}")

        # Open rosbag
        bag = rosbag.Bag(self.bag_file)

        # Read all messages
        joint_messages = []
        ground_truth_messages = []

        for topic, msg, t in bag.read_messages():
            if topic == '/joint_states':
                joint_messages.append((t, msg))
            elif topic == '/gazebo/model_states':
                ground_truth_messages.append((t, msg))

        bag.close()

        print(f"Read {len(joint_messages)} joint state messages")
        print(f"Read {len(ground_truth_messages)} ground truth messages")

        # Sort by timestamp
        joint_messages.sort(key=lambda x: x[0])
        ground_truth_messages.sort(key=lambda x: x[0])

        # Get initial pose from ground truth (first message)
        if ground_truth_messages:
            first_gt = ground_truth_messages[0][1]
            if len(first_gt.name) > 2 and 'turtlebot3_burger' in first_gt.name[2]:
                initial_pose = first_gt.pose[2]
                self.initial_x = initial_pose.position.x
                self.initial_y = initial_pose.position.y
                self.initial_theta = self.quaternion_to_yaw(
                    initial_pose.orientation.x, initial_pose.orientation.y,
                    initial_pose.orientation.z, initial_pose.orientation.w
                )
                print(f"Initial pose from ground truth: x={self.initial_x:.3f}m, y={self.initial_y:.3f}m, θ={np.degrees(self.initial_theta):.1f}°")

        # Process joint states
        for timestamp, joint_msg in joint_messages:
            if len(joint_msg.name) >= 2:  # Ensure we have both wheels
                # Extract wheel positions (right, left)
                wheel_positions = [0.0, 0.0]
                for i, name in enumerate(joint_msg.name):
                    if 'right' in name:
                        wheel_positions[0] = joint_msg.position[i]
                    elif 'left' in name:
                        wheel_positions[1] = joint_msg.position[i]

                # Calculate increments
                delta_s, delta_theta = self.calculate_displacement_increment(wheel_positions)

                # Update trajectory
                self.update_trajectory(delta_s, delta_theta, timestamp.to_sec())

        # Process ground truth - synchronize with encoder timestamps
        for enc_timestamp, joint_msg in joint_messages:
            # Find the closest ground truth timestamp
            closest_gt = None
            min_time_diff = float('inf')

            for gt_timestamp, gt_msg in ground_truth_messages:
                time_diff = abs(gt_timestamp.to_sec() - enc_timestamp.to_sec())
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_gt = gt_msg

            if closest_gt is not None and len(closest_gt.name) > 2 and 'turtlebot3_burger' in closest_gt.name[2]:
                pose = closest_gt.pose[2]
                yaw = self.quaternion_to_yaw(
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w
                )

                self.ground_truth_trajectory.append([
                    pose.position.x, pose.position.y, yaw
                ])

        print(f"Generated {len(self.encoder_trajectory)} encoder trajectory points")
        print(f"Generated {len(self.ground_truth_trajectory)} ground truth points")

    def plot_trajectories(self):
        """Plot both trajectories for comparison"""
        print("Starting trajectory plotting...")

        if len(self.encoder_trajectory) == 0 or len(self.ground_truth_trajectory) == 0:
            print("No trajectory data to plot")
            return

        # Convert to numpy arrays
        enc_traj = np.array(self.encoder_trajectory)
        gt_traj = np.array(self.ground_truth_trajectory)

        print(f"Encoder trajectory shape: {enc_traj.shape}")
        print(f"Ground truth trajectory shape: {gt_traj.shape}")
        print(f"Trajectory length: {len(self.timestamps)}")

        # Set matplotlib backend for headless operation
        plt.switch_backend('Agg')

        # Create figure with subplots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

        # Plot 1: XY trajectory comparison
        ax1.plot(gt_traj[:, 0], gt_traj[:, 1], 'b-', linewidth=2, label='Ground Truth')
        ax1.plot(enc_traj[:, 0], enc_traj[:, 1], 'r--', linewidth=2, label='Encoder Estimated')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectory Comparison')
        ax1.legend()
        ax1.axis('equal')
        ax1.grid(True)

        # Plot 2: Position error over time
        if len(enc_traj) == len(gt_traj):
            position_error = np.sqrt((enc_traj[:, 0] - gt_traj[:, 0])**2 +
                                   (enc_traj[:, 1] - gt_traj[:, 1])**2)
            ax2.plot(self.timestamps, position_error, 'g-', linewidth=2)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Position Error (m)')
            ax2.set_title(f'Position Error (RMSE: {np.mean(position_error):.3f}m)')
            ax2.grid(True)
            print(f"Position RMSE: {np.mean(position_error):.3f}m")

        # Plot 3: Orientation error over time
        if len(enc_traj) == len(gt_traj):
            orientation_error = np.abs(enc_traj[:, 2] - gt_traj[:, 2])
            # Unwrap angle differences
            orientation_error = np.minimum(orientation_error,
                                         2*np.pi - orientation_error)
            ax3.plot(self.timestamps, np.degrees(orientation_error), 'm-', linewidth=2)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Orientation Error (degrees)')
            ax3.set_title(f'Orientation Error (RMSE: {np.mean(orientation_error):.3f}°)')
            ax3.grid(True)
            print(f"Orientation RMSE: {np.mean(orientation_error):.3f} radians")

        # Plot 4: Error statistics
        if len(enc_traj) == len(gt_traj):
            pos_error = np.sqrt((enc_traj[:, 0] - gt_traj[:, 0])**2 +
                              (enc_traj[:, 1] - gt_traj[:, 1])**2)
            orient_error = np.abs(enc_traj[:, 2] - gt_traj[:, 2])
            orient_error = np.minimum(orient_error, 2*np.pi - orient_error)

            # Calculate trajectory length
            if len(enc_traj) > 1:
                traj_length = np.sum(np.sqrt(np.diff(enc_traj[:,:2], axis=0)**2).sum(axis=1))
            else:
                traj_length = 0.0

            ax4.text(0.1, 0.8, f'Position RMSE: {np.mean(pos_error):.3f}m', fontsize=12)
            ax4.text(0.1, 0.7, f'Max Position Error: {np.max(pos_error):.3f}m', fontsize=12)
            ax4.text(0.1, 0.6, f'Orientation RMSE: {np.mean(orient_error):.3f}rad', fontsize=12)
            ax4.text(0.1, 0.5, f'Max Orientation Error: {np.max(orient_error):.3f}rad', fontsize=12)
            ax4.text(0.1, 0.4, f'Trajectory Length: {traj_length:.3f}m', fontsize=12)
            ax4.axis('off')
            ax4.set_title('Error Statistics')

            print(f"Max Position Error: {np.max(pos_error):.3f}m")
            print(f"Max Orientation Error: {np.max(orient_error):.3f} radians")
            print(f"Trajectory Length: {traj_length:.3f}m")

        plt.tight_layout()
        plt.savefig('encoder_trajectory_comparison.png', dpi=300, bbox_inches='tight')
        print("Plot saved as 'encoder_trajectory_comparison.png'")
        plt.close()  # Close the plot to free memory

    def run(self):
        """Main execution function"""
        print("Starting Encoder Trajectory Estimation...")
        print(f"Wheel radius: {self.wheel_radius}m")
        print(f"Wheel base: {self.wheel_base}m")

        # Process rosbag data
        self.process_rosbag()

        # Plot results
        self.plot_trajectories()

        print("Processing complete!")


def main():
    """Main function"""
    estimator = EncoderTrajectoryEstimator()
    estimator.run()


if __name__ == "__main__":
    main()
