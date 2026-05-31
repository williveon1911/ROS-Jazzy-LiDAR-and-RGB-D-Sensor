# ROS 2 Jazzy - LiDAR and RGB-D Sensor Integration

A Robot Operating System 2 (ROS 2) Jazzy-based project for autonomous robot simulation, featuring LiDAR and RGB-D sensor integration, movement control, and SLAM capabilities.

## 📋 Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Configuration](#configuration)
- [Dependencies](#dependencies)
- [SLAM Toolbox Implementation](#slam-toolbox-implementation)
- [Contributing](#contributing)
- [License](#license)

## 📖 Project Overview

This project provides a complete ROS 2 Jazzy simulation environment for a mobile robot equipped with:
- **LiDAR sensors** for distance measurement and mapping
- **RGB-D cameras** for depth sensing and visual perception
- **Movement control** with linear and angular velocity commands
- **SLAM capabilities** for simultaneous localization and mapping

The robot can autonomously navigate, avoid obstacles, and build maps of its environment in a Gazebo simulation.

## ✨ Features

- ✅ **Robot Simulation**: Gazebo integration for realistic robot physics simulation
- ✅ **Multi-Sensor Support**: LiDAR and RGB-D sensor data streaming
- ✅ **Autonomous Movement**: Basic movement patterns and path planning
- ✅ **SLAM Integration**: Real-time mapping using slam_toolbox
- ✅ **ROS 2 Jazzy Compatibility**: Fully compatible with ROS 2 Jazzy distribution
- ✅ **RViz Visualization**: Visualize sensor data and robot state
- ✅ **Modular Design**: Easily extensible for additional features
- ✅ **Kalman Filter Localization**: Multi-sensor fusion (LiDAR + odometry) for real-time pose and velocity estimation

## 🔬 Kalman Filter Localization (`kalman_filter_localization.py`)

Implements a **2D Constant Velocity Kalman Filter** for multi-sensor fusion pose estimation — directly addressing 卡尔曼滤波 and 多传感器融合定位 requirements.

### State Vector

| Index | Symbol  | Description                    | Unit  |
|-------|---------|--------------------------------|-------|
| 0     | `x`     | World-frame X position         | m     |
| 1     | `y`     | World-frame Y position         | m     |
| 2     | `vx`    | World-frame X velocity         | m/s   |
| 3     | `vy`    | World-frame Y velocity         | m/s   |
| 4     | `theta` | Heading angle                  | rad   |

### Kalman Filter Equations

```
Predict:
  x̂⁻ = F · x̂          State prediction (constant-velocity model)
  P⁻  = F · P · Fᵀ + Q  Covariance prediction

Update (per sensor):
  K  = P⁻ · Hᵀ · (H · P⁻ · Hᵀ + R)⁻¹   Kalman gain
  x̂  = x̂⁻ + K · (z − H · x̂⁻)           State update
  P  = (I − K·H) · P⁻ · (I − K·H)ᵀ + K·R·Kᵀ  Covariance update (Joseph form)
```

### Sensor Fusion Strategy

| Sensor        | Topic   | Measurement                        | State indices updated |
|---------------|---------|------------------------------------|-----------------------|
| Wheel odometry| `/odom` | position `(x, y)` and heading `θ`  | 0, 1, 4               |
| LiDAR scanner | `/scan` | forward velocity via Δrange/Δt     | 2 (`vx`)              |

### Topics

| Direction | Topic                | Message type                               |
|-----------|----------------------|--------------------------------------------|
| Subscribe | `/odom`              | `nav_msgs/Odometry`                        |
| Subscribe | `/scan`              | `sensor_msgs/LaserScan`                    |
| Publish   | `/estimated_pose`    | `geometry_msgs/PoseWithCovarianceStamped`  |
| Publish   | `/estimated_velocity`| `geometry_msgs/TwistWithCovarianceStamped` |

### Running

```bash
# Make sure the Gazebo simulation is running first, then:
python3 kalman_filter_localization.py

# Monitor estimated pose in a second terminal:
ros2 topic echo /estimated_pose

# Monitor estimated velocity:
ros2 topic echo /estimated_velocity
```

### Tuning Parameters

Edit the constants at the top of `kalman_filter_localization.py`:

```python
PROCESS_NOISE_POS    = 0.01   # Position model uncertainty (m²/s²)
PROCESS_NOISE_VEL    = 0.10   # Velocity model uncertainty (m²/s⁴)
PROCESS_NOISE_THETA  = 0.005  # Heading model uncertainty (rad²/s²)
ODOM_NOISE_POS       = 0.05   # Odometry position noise (m²)
ODOM_NOISE_THETA     = 0.02   # Odometry heading noise (rad²)
LIDAR_NOISE_VEL      = 0.20   # LiDAR velocity estimate noise (m²/s²)
```

Increase a noise value to make the filter trust that sensor *less*; decrease it to trust the sensor *more*.

## 🗺️ SLAM Toolbox Implementation

This project integrates **slam_toolbox**, a modern SLAM (Simultaneous Localization and Mapping) solution for ROS 2 that creates real-time maps while simultaneously localizing the robot within them.

### Overview

slam_toolbox processes LiDAR scan data to:
- Build occupancy grid maps of the environment
- Track robot pose through scan matching
- Provide loop closure detection to reduce drift
- Generate consistent maps for navigation

### Configuration

The SLAM toolbox is configured via launch files and parameter files. Key configuration includes:
- **Scan matcher settings**: Controls the quality and speed of scan-to-map matching
- **Loop closure parameters**: Threshold values for detecting when the robot revisits areas
- **Map frame settings**: Coordinate frame management and persistence
- **Output frame**: Typically `map` frame for global localization

### Starting SLAM Toolbox

```bash
# Launch SLAM in mapping mode
ros2 launch slam_toolbox online_async.launch.py

# Or for synchronous mapping:
ros2 launch slam_toolbox online_sync.launch.py
```

### RViz Visualization

Monitor SLAM progress in RViz:
- **LaserScan** (`/scan`): Current LiDAR readings with red boundary
- **Map** (`/map`): Occupancy grid built from LiDAR scans
- **TF Frames**: Transformation tree showing `map`, `odom`, and `base_link`
- **Estimated Pose**: Robot position and orientation in the map

![RViz SLAM Visualization](https://github.com/williveon1911/ROS-Jazzy-LiDAR-and-RGB-D-Sensor/assets/image1.png)

### Key Topics

| Topic                | Message Type             | Description                          |
|----------------------|--------------------------|--------------------------------------|
| `/scan`              | `sensor_msgs/LaserScan`  | LiDAR input data                     |
| `/map`               | `nav_msgs/OccupancyGrid` | Output occupancy grid map            |
| `/slam_toolbox/scan_matched_points2`  | `sensor_msgs/PointCloud2` | Processed scan points after matching |
| `/tf`                | `tf2_msgs/TFMessage`     | Transformation broadcasts (`map` → `base_link`) |

### Performance Tuning

For optimal SLAM performance:
1. **Decrease scan matcher resolution** if the robot is moving too fast
2. **Increase loop closure threshold** if false loop closures occur
3. **Adjust motion model parameters** for your robot's dynamics
4. **Fine-tune angle and distance tolerances** based on environment complexity

## Documentation
<img width="1028" height="911" alt="Screenshot from 2026-04-12 18-33-11" src="https://github.com/user-attachments/assets/6ce5430e-361f-45ab-8aeb-26d2002f5d79" />
<img width="1228" height="912" alt="Screenshot from 2026-04-12 18-33-09" src="https://github.com/user-attachments/assets/f9f42793-5934-4105-a451-37929c88f726" />
<img width="1228" height="912" alt="Screenshot from 2026-04-23 20-53-14" src="https://github.com/user-attachments/assets/09de3a14-a07f-4fcb-83e4-87f913d5e34b" />
<img width="1228" height="912" alt="Screenshot from 2026-05-31 14-38-07" src="https://github.com/user-attachments/assets/32844733-a145-4e9d-9f90-e2bd14787e4b" />


