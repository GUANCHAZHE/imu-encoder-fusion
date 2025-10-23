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

The secant model calculates robot pose using:

```
x_k = x_{k-1} + Δs_k * cos(θ_k + 1/2 * Δθ_k)
y_k = y_{k-1} + Δs_k * sin(θ_k + 1/2 * Δθ_k)
θ_k = θ_{k-1} + Δθ_k
```

Where:
- `Δs_k = (Δs_right + Δs_left) / 2` (linear displacement)
- `Δθ_k = (Δs_right - Δs_left) / wheel_base` (angular displacement)
- `Δs_right = Δposition_right * wheel_radius`
- `Δs_left = Δposition_left * wheel_radius`

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

### Performance Metrics
- **Position RMSE**: 2.462m
- **Orientation RMSE**: 2.656 radians (152°)
- **Maximum Position Error**: 2.778m
- **Maximum Orientation Error**: 2.669 radians (153°)
- **Total Trajectory Length**: 0.295m

### Analysis

The current implementation shows significant errors, which may be due to:

1. **Parameter Calibration**: The wheel radius (0.033m) or wheel base (0.160m) may need adjustment for the specific simulation setup
2. **Data Scaling**: The encoder positions might be in different units or scale than expected
3. **Simulation Movement**: The robot movement in the simulation appears minimal (only right wheel moving slightly)
4. **Coordinate System**: Potential mismatch between encoder coordinate system and ground truth

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
- `process_rosbag()`: Reads and processes rosbag data
- `calculate_displacement_increment()`: Implements secant model calculations
- `plot_trajectories()`: Generates comparison plots and statistics

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
