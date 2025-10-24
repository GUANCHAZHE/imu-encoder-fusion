# Encoder-based Trajectory Estimation using Secant Model

This project implements encoder-based trajectory estimation for Turtlebot3 Burger using the secant model for odometry calculation.

## Overview

The system reads encoder data from ROS joint_states and ground truth from Gazebo model_states, calculates robot trajectory using the secant model, and compares the estimated trajectory with ground truth.

## Features

- **Secant Model Implementation**: Uses the mathematical formula provided by the user
- **ROS Integration**: Processes rosbag data from Turtlebot3 simulation
- **Real-time Plotting**: Generates comprehensive trajectory comparison plots
- **Error Analysis**: Calculates RMSE and maximum errors for position and orientation
- **Parameter Configuration**: Uses Turtlebot3 Burger standard parameters

## Mathematical Model

### Secant Model (Local Coordinates)
The secant model calculates incremental robot movement in the robot's local coordinate system:

```
Δx_local = Δs * cos(Δθ/2)
Δy_local = Δs * sin(Δθ/2)
Δθ = (Δs_right - Δs_left) / wheel_base
```

Where:
- `Δs = (Δs_right + Δs_left) / 2` (linear displacement)
- `Δs_right = Δposition_right * wheel_radius`
- `Δs_left = Δposition_left * wheel_radius`

### World Coordinate Transformation
The critical fix transforms local incremental movements to world coordinates:

```
Δx_world = Δx_local * cos(θ_world) - Δy_local * sin(θ_world)
Δy_world = Δx_local * sin(θ_world) + Δy_local * cos(θ_world)
```

**Complete transformation:**
```
x_world = x_initial + Σ(Δx_world)
y_world = y_initial + Σ(Δy_world)
θ_world = θ_initial + Σ(Δθ)
```

Where `(x_initial, y_initial, θ_initial)` comes from ground truth initial pose.

## Parameters

- **Wheel radius**: 0.033m (Turtlebot3 Burger standard)
- **Wheel base**: 0.160m (Turtlebot3 Burger standard)
- **Input data**: Joint states in radians, converted to meters using wheel radius

## Usage

### Prerequisites

```bash
# Ensure you have the required Python packages
pip install numpy matplotlib scipy

# Or using uv (as mentioned by user)
uv add numpy matplotlib scipy
```

### Running the Analysis

```bash
cd fusion5_ros
python3 encoder_trajectory.py
```

### Input Data

The system processes the rosbag file `2025-10-23-18-22-52.bag` containing:
- `/joint_states`: Encoder positions (wheel_right_joint, wheel_left_joint)
- `/gazebo/model_states`: Ground truth pose (turtlebot3_burger)

## Output

### Console Output
- Processing statistics (number of messages, trajectory points)
- Error metrics (RMSE, maximum errors)
- Trajectory length

### Generated Files
- `encoder_trajectory_comparison.png`: Comprehensive trajectory comparison plot with 4 subplots:
  1. XY trajectory comparison (ground truth vs estimated)
  2. Position error over time
  3. Orientation error over time
  4. Error statistics summary

## Current Results

### Performance Metrics (After Coordinate Transformation Fix)
- **Position RMSE**: 0.063m (39x improvement!)
- **Orientation RMSE**: 0.110 radians (6.3°)
- **Maximum Position Error**: 0.197m
- **Maximum Orientation Error**: 0.298 radians (17°)
- **Total Trajectory Length**: 2.496m

### Analysis

The coordinate transformation fix dramatically improved accuracy by addressing the **coordinate system mismatch**:

1. **✅ Fixed: Initial Position Offset**: Now uses ground truth initial position (-1.980m, -0.486m) instead of (0,0)
2. **✅ Fixed: Initial Orientation**: Now uses ground truth initial orientation (3.5°) instead of assuming 0°
3. **✅ Fixed: Coordinate Transformation**: Transforms incremental odometry from robot-local to world coordinates
4. **✅ Fixed: Proper Rotation**: Applies rotation matrix to convert local Δx, Δy to world coordinates

### Previous Issues (Before Fix)
- **❌ Coordinate System Mismatch**: Odometry calculated in local frame, ground truth in world frame
- **❌ Missing Initial Offset**: Started from (0,0) instead of actual initial position
- **❌ Missing Initial Rotation**: Assumed 0° orientation instead of actual initial orientation
- **❌ No Frame Transformation**: No conversion between robot-local and world coordinate systems

### Debug Information

The system provides detailed debug output showing:
- Raw wheel position changes (in radians)
- Calculated linear displacements (in meters)
- Angular displacements (in radians and degrees)

## File Structure

```
fusion5_ros/
├── encoder_trajectory.py          # Main implementation
├── 2025-10-23-18-22-52.bag       # Input rosbag data
├── encoder_trajectory_comparison.png  # Output plot (generated)
└── README.md                     # This documentation
```

## Implementation Details

### Key Components

1. **Data Synchronization**: Matches encoder timestamps with closest ground truth timestamps
2. **Incremental Calculation**: Properly calculates differences between consecutive encoder positions
3. **Quaternion Conversion**: Manual conversion from quaternion to yaw angle
4. **Error Analysis**: Comprehensive error calculation and visualization

### Code Structure

- `EncoderTrajectoryEstimator` class: Main processing class
- `process_rosbag()`: Reads and processes rosbag data, extracts initial pose from ground truth
- `calculate_displacement_increment()`: Implements secant model calculations in local frame
- `update_trajectory()`: **Key fix** - transforms local coordinates to world coordinates using rotation matrix
- `plot_trajectories()`: Generates comparison plots and statistics

### Coordinate Transformation Implementation

The critical fix in `update_trajectory()` method:

```python
# 1. Calculate movement in robot-local coordinate system
delta_x_local = delta_s * math.cos(delta_theta / 2.0)
delta_y_local = delta_s * math.sin(delta_theta / 2.0)

# 2. Transform to world coordinates using current orientation
delta_x_world = delta_x_local * math.cos(prev_theta) - delta_y_local * math.sin(prev_theta)
delta_y_world = delta_x_local * math.sin(prev_theta) + delta_y_local * math.cos(prev_theta)

# 3. Update world pose with initial offset
current_x = self.initial_x + delta_x_world  # Note: initial_x from ground truth
current_y = self.initial_y + delta_y_world  # Note: initial_y from ground truth
current_theta = self.initial_theta + delta_theta  # Note: initial_theta from ground truth
```

**Key improvements:**
- Uses ground truth initial pose as reference point
- Applies proper 2D rotation transformation matrix
- Accumulates transformations in world coordinate system
- Maintains orientation consistency between odometry and ground truth

## Future Improvements

1. **Parameter Optimization**: Calibrate wheel radius and wheel base for better accuracy
2. **Data Validation**: Add checks for data consistency and movement detection
3. **IMU Integration**: Incorporate IMU data for improved orientation estimation
4. **Filtering**: Add noise filtering and outlier detection
5. **Real-time Operation**: Adapt for real-time ROS node operation

## Troubleshooting

### Common Issues

1. **Zero Movement**: If encoder positions don't change, check simulation movement
2. **Large Errors**: Verify wheel radius and wheel base parameters
3. **Import Errors**: Ensure scipy is installed for quaternion operations
4. **Plot Display**: Script uses 'Agg' backend for headless operation

### Debug Mode

Enable debug output by checking the console during execution. The system shows:
- Raw encoder position changes
- Calculated displacements
- Incremental pose updates

## References

- Turtlebot3 Burger specifications
- Secant model for differential drive odometry
- ROS joint_states message format
- Gazebo model_states message format
