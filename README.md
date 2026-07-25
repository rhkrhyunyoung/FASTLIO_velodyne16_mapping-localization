#  FAST-LIO-Nav2: Seamless Mapping & Localization for ROS2

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License: BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

This repository provides a robust, **ROS2-native solution** for LiDAR-based SLAM and Localization by integrating the enhanced **FAST-LIO** algorithm with the **ROS2 Navigation2 (Nav2)** stack. It optimizes the original FAST-LIO for Velodyne LiDARs and implements a production-ready localization system that replaces AMCL with high-precision LiDAR odometry.

---

## My Contributions

- Ported FAST-LIO localization components from ROS1 to ROS2 Humble
- Removed Livox SDK dependency and adapted the pipeline for Velodyne VLP-16
- Designed TF architecture for Nav2 compatibility
- Implemented dynamic map->odom transformation calculation
- Integrated FAST-LIO localization output with Nav2 navigation stack
- Tuned localization and costmap parameters for autonomous navigation



##  Key Enhancements
### 1. Velodyne-Native Integration
*   **Dependency-Free Build**: Eliminated the mandatory Livox SDK dependency. This version builds flawlessly in environments using only Velodyne drivers.
*   **Universal Compatibility**: Optimized for standard `sensor_msgs/msg/PointCloud2` inputs, ensuring compatibility with VLP-16 and other Velodyne models.

### 2. Integrated Localization Module
*   **ROS1 to ROS2 Porting**: Successfully ported features from ROS1 `FAST_LIO_LOCALIZATION` into the ROS2 framework.
*   **Global Map Relocalization**: Supports loading pre-built `.pcd` maps for stable, drift-free localization in known environments.
*   **Dynamic Re-alignment**: Integrated with RViz2's `/initialpose` for quick alignment between the robot and the global map.

### 3. Full Nav2 Stack Integration
*   **Standardized TF Tree**: Implements the standard `map -> odom -> base_link` hierarchy, making it a "plug-and-play" replacement for AMCL.
*   **AMCL Replacement**: Replaces traditional 2D Monte Carlo Localization with 3D LiDAR EKF-based localization for superior accuracy in complex terrains.
*   **Costmap Optimization**: Pre-configured `nav2_params.yaml` to utilize LiDAR data for obstacle avoidance while filtering ground noise.

---

## TF Transformation Logic

To maintain Nav2-compatible TF hierarchy, the system dynamically computes:

T_map->odom = T_map->base_link × (T_odom->base_link)^-1

This allows FAST-LIO localization and local odometry to coexist without TF conflicts.

### TF Tree Structure
- **`map`**: Global reference frame (provided by FAST-LIO Localization).
- **`odom`**: Local odometry frame (continuous motion from wheel encoders/IMU).
- **`base_link`**: Robot physical center.
- **`sensors`**: Static transforms (LiDAR, IMU, etc.) attached to `base_link`.

<img width="500" height="300" alt="스크린샷 2026-07-14 17-08-03" src="https://github.com/user-attachments/assets/14d908e1-e0c4-4b9f-8ad7-f186978a264f" />


### Transformation Logic (Inverse Calculation)
The node calculates the **map → odom** transform using the following logic to bridge the gap between high-frequency local odometry and high-precision global localization:

$$T_{map \to odom} = T_{map \to base\_link} \times (T_{odom \to base\_link})^{-1}$$

> **Benefit**: This ensures a single parent for `base_link`, allowing Nav2's local planners to use smooth odometry while global planners use accurate map-based localization.

---

##  Workspace Structure
```bash
your_ros2_workspace/
└── src/
    ├── fast_lio/              # Modified FAST-LIO (Mapping & Localization)
    ├── drok_gazebo/           # Custom tracked vehicle model & world
    ├── map/                   # Directory for PCD and YAML maps
    ├── velodyne/              # Velodyne ROS2 drivers
    └── pcd2pgm/               # PCD to PGM converter for Nav2
```
##  How to Run

### 1. Mapping (SLAM)
Generate a point cloud map of your environment.

``` ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml```

https://github.com/user-attachments/assets/37fe88da-bcde-46a3-9a15-cc479d6cf232

   <img width="1617" height="607" alt="img" src="https://github.com/user-attachments/assets/c800e2be-fe2c-49bc-b3f8-baf85aa3185b" />
   
### 2. Save Map
After mapping, save the result to a .pcd file.
``` ros2 service call /map_save std_srvs/srv/Trigger {}```
   <img width="300" height="200" alt="image" src="https://github.com/user-attachments/assets/d7cbc202-58b7-4ccb-a1d3-daa8f775e8a8" /><img width="300" height="200" alt="image" src="https://github.com/user-attachments/assets/4ad02e13-0f27-49f2-a5c3-edb079f2979b" />

### 3. Localization
Run the system using a pre-built map for autonomous navigation.

``` ros2 launch fast_lio localization.launch.py map:=/path/to/your/map_result/my_fast_lio_map.pcd use_sim_time:=true
```

https://github.com/user-attachments/assets/b1537e11-0d6c-43e7-bd98-64d6127a4e74

### 4. Nav2 Stack Integration
Launch the full Navigation2 stack using FAST-LIO as the localization source.
``` ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=true \
    autostart:=true \
    amcl:=false \
    map:=/path/to/your/2d_map.yaml \
    params_file:=/path/to/my_nav2_params.yaml
```

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/d65a8045-4e80-48d7-916f-6a95006cf193" />
<img width="500" height="300" alt="스크린샷 2026-05-14 19-10-34" src="https://github.com/user-attachments/assets/97634a6a-3af0-4f15-898a-6171f7e8e262" />

##  Configuration
*   Mapping: fast_lio/config/velodyne.yaml

*   Localization: fast_lio/config/velodyne_localization.yaml

*   Navigation: nav2_config/my_nav2_params.yaml (Customized for FAST-LIO)

*   Simulation: drok_gazebo/urdf/drok_gazebo.urdf (Optimized friction & limits)

<img width="1280" height="720" alt="img1 daumcdn" src="https://github.com/user-attachments/assets/36f29c00-3de7-4ca8-95cc-bb9a26093a02" />

##  Debugging & Topics
*   Monitor Velocity: ros2 topic echo /cmd_vel
*   Check TF Drift: ros2 run tf2_ros tf2_echo map base_link
*   Visualize Tree: ros2 run tf2_tools view_frames
