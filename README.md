# Assignment 3: Rocky Times Challenge - Search, Map, & Analyze

This ROS2 package implements an autonomous drone system for geological feature detection, mapping, and analysis using an RGBD camera and PX4 SITL simulation.

## Challenge Overview

<img width="1195" height="1020" alt="image" src="https://github.com/user-attachments/assets/6e3d9610-a63a-4949-88a1-a14166a9ed50" />

Students will develop a controller for a PX4-powered drone to efficiently search, map, and analyze 3D objects in an unknown environment. The drone must map the Perseverance rover, and land on it.

### Mission Objectives
Intermediate: 
1. Search and locate the cylinder
2. Map the cylinder in 3D
3. Land safely on top of the cylinder

Advanced (extra credit): 
Execute intermediate objective, and do the following additional tasks. 
1. Search and locate the rover
2. Map the rover in 3D
3. Land safely on top of the rover

In both cases, complete mission while logging time and energy performance. 

### Evaluation Criteria (100 points)

The assignment will be evaluated based on:
- Total time taken to complete the mission
- Total energy units consumed during operation
- Accuracy of rover 3D model
- Landing precision on rover
- Performance across 3 trials

### Key Requirements

- Autonomous takeoff and search strategy implementation
- Real-time rover detection 
- Energy-conscious path planning for mapping using SLAM 
- Safe and precise landing on the rover once mapping is complete
- Robust performance across trials

## Prerequisites

- ROS2 Humble
- PX4 SITL Simulator (Tested with PX4-Autopilot main branch 9ac03f03eb)
- RTAB-Map ROS2 package
- OpenCV
- Python 3.8+

## Repository Setup

### If you already have a fork of the course repository:

```bash
# Navigate to your local copy of the repository
cd ~/RAS-SES-598-Space-Robotics-and-AI

# Add the original repository as upstream (if not already done)
git remote add upstream https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI.git

# Fetch the latest changes from upstream
git fetch upstream

# Checkout your main branch
git checkout main

# Merge upstream changes
git merge upstream/main

# Push the updates to your fork
git push origin main
```

### If you don't have a fork yet:

1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork:
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

### Create Symlink to ROS2 Workspace

```bash
# Create symlink in your ROS2 workspace
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/terrain_mapping_drone_control .
```

### Copy PX4 Model Files

Copy the custom PX4 model files to the PX4-Autopilot folder

```bash
# Navigate to the package
cd ~/ros2_ws/src/terrain_mapping_drone_control

# Make the setup script executable
chmod +x scripts/deploy_px4_model.sh

# Run the setup script to copy model files
./scripts/deploy_px4_model.sh -p /path/to/PX4-Autopilot
```

## Building and Running

This setup runs PX4 SITL, Gazebo, ROS 2, and the mission node inside the Linux VM.
QGroundControl runs on the macOS host. PX4 sends MAVLink UDP traffic from the VM
to QGroundControl on the Mac host, so the launch command needs the Mac host IP
address that is reachable from the VM.

### VM to macOS QGroundControl Link

1. Start QGroundControl on macOS before launching the mission.
2. Find the macOS host IP address on the network shared with the VM.
   - On macOS, check `System Settings > Wi-Fi > Details`, or run:
     ```bash
     ipconfig getifaddr en0
     ```
   - If the Mac is using Ethernet, the interface may be `en1` or another name.
3. Make sure the VM can reach that IP:
   ```bash
   ping <MAC_HOST_IP>
   ```
4. Make sure the VM network mode allows host/guest UDP traffic. Bridged networking is the simplest option. NAT can work too, but it may require explicit UDP forwarding for QGroundControl traffic.
5. QGroundControl normally listens for UDP MAVLink on port `14550`. The launch file exports:
   - `PX4_SIM_HOST_ADDR=<MAC_HOST_IP>`
   - `PX4_SIM_UDP_PORT=14550`

The current default in `launch/cylinder_landing.launch.py` is `192.168.0.92`.
Override it at launch time if your Mac host IP is different.

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch PX4/Gazebo/ROS mission and point PX4 MAVLink at QGroundControl on macOS
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py \
  px4_autopilot_path:=~/PX4-Autopilot \
  qgc_host_ip:=<MAC_HOST_IP>
```

The launch file starts `make px4_sitl gz_x500_depth_mono`, spawns the two
cylinder targets, bridges Gazebo camera/rangefinder/odometry topics into ROS 2,
and starts the current mission node.

To open RViz in a separate terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch terrain_mapping_drone_control visualization.launch.py
```

### RVIO Proof-of-Concept Estimator

The package includes a lightweight Range-Visual-Inertial Odometry proof of
concept for the Mars terrain drone. It estimates pose from:

- PX4 IMU samples on `/fmu/out/sensor_combined` or an optional bridged
  `sensor_msgs/Imu` topic
- PX4 attitude on `/fmu/out/vehicle_attitude`
- downward mono images on `/drone/down_mono`
- downward 1D lidar on `/drone/down_rangefinder`

The estimator publishes:

- `/rvio/odometry` (`nav_msgs/Odometry`)
- `/rvio/path` (`nav_msgs/Path`)
- `/rvio/debug_image` (`sensor_msgs/Image`)
- `/rvio/range_feature_count` (`std_msgs/Int32`)
- `/rvio/visual_update_source` (`std_msgs/String`)
- `/rvio/ground_truth_error` (`geometry_msgs/Vector3Stamped`) when
  `/sim/ground_truth/vehicle_odometry` is available

By default, `estimator_mode:=range_feature_3d` runs the 3D range-feature pose
estimator with base state
`[px, py, pz, vx, vy, vz, roll, pitch, yaw, bax, bay, baz, bgx, bgy, bgz]`.
IMU gyroscope and acceleration samples predict the full pose/velocity state
while estimating accelerometer and gyro bias. The downward
rangefinder is fused as a high-confidence local `pz`/AGL measurement, and the
same range measurement initializes inverse-depth range-feature states
`rho_1...rho_N` for visual reprojection updates. The range-feature front end
searches a small ROI around the projected 1D lidar beam for the best trackable
corner, defers the new feature until the next frame, and tracks it with LK
optical flow so the lidar depth and visual feature are aligned in image location
and time. Tracked range-feature reprojection updates the shared covariance
between the 3D vehicle pose, attitude, and feature inverse depths. PX4 attitude
initializes the filter before `SURVEY`; once RVIO starts, roll, pitch, and yaw
are propagated by gyro in the EKF and PX4 attitude is fused as a noisy
measurement update. This keeps attitude uncertainty in the covariance while
preventing unbounded gyro-only orientation drift from corrupting range-feature
geometry.

In the full mission launch, RVIO and the evaluator are gated by `/mission/state`.
For `mission_mode:=rvio_test`, the vehicle first climbs 5 m relative to its
starting PX4 local position, then when the mission enters `SURVEY`, RVIO resets
its local origin and the metrics start.
This keeps takeoff/platform lidar artifacts out of the x/y accuracy run. Override
with `rvio_start_state:=` to start immediately.

`/rvio/odometry.pose.pose.position.z` is the EKF-estimated local AGL state
strongly constrained by the downward rangefinder, not Gazebo world-frame
altitude. Therefore `/rvio/ground_truth_error.z` defaults to numeric `NaN`
using `ground_truth_error_z_mode:=ignore`; x/y remain the horizontal error
against Gazebo ground truth.

The RVIO evaluator compares relative motion, not raw Gazebo world coordinates.
When gated by `rvio_start_state:=SURVEY`, Gazebo ground truth is zeroed from the
drone model pose at the start of the survey segment. RVIO is compared in that
same local mission frame, horizontal drift is tracked over time, and altitude is
evaluated as AGL against the raw downward rangefinder. It publishes:

- `/rvio_eval/relative_error` (`geometry_msgs/Vector3Stamped`): x/y relative
  pose error, z AGL error
- `/rvio_eval/velocity_error` (`geometry_msgs/Vector3Stamped`): x/y velocity
  error against Gazebo ground truth
- `/rvio_eval/xy_error_norm` (`std_msgs/Float32`)
- `/rvio_eval/velocity_error_norm` (`std_msgs/Float32`)
- `/rvio_eval/agl_error` (`std_msgs/Float32`)
- `/rvio_eval/xy_rmse` (`std_msgs/Float32`)
- `/rvio_eval/velocity_rmse` (`std_msgs/Float32`)
- `/rvio_eval/agl_rmse` (`std_msgs/Float32`)
- `/rvio_eval/drift_percent` (`std_msgs/Float32`)
- `/rvio_eval/path_length_gt` and `/rvio_eval/path_length_rvio`

For live `rqt_plot` metrics, it also publishes:

- `/rvio_plot/estimated_x` and `/rvio_plot/ground_truth_x`
- `/rvio_plot/estimated_y` and `/rvio_plot/ground_truth_y`
- `/rvio_plot/estimated_vx` and `/rvio_plot/ground_truth_vx`
- `/rvio_plot/estimated_vy` and `/rvio_plot/ground_truth_vy`
- `/rvio_plot/final_estimated_x` and `/rvio_plot/final_ground_truth_x`
- `/rvio_plot/final_estimated_y` and `/rvio_plot/final_ground_truth_y`

CSV logs are off by default. Use `write_eval_csv:=true` if you also want files
under `~/.ros/rvio_eval/`.

Run the simulation first, then launch RVIO in another terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch terrain_mapping_drone_control rvio.launch.py
```

For a repeatable metrics run, launch the simple RVIO test mission instead of
auto-hover:

```bash
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py \
  mission_mode:=rvio_test rvio_estimator_mode:=range_feature_3d \
  rvio_test_climb_height:=5.0 headless:=0
```

Useful checks:

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID=15
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source ~/ros2_ws/install/setup.bash

ros2 topic echo /rvio/odometry --once
ros2 topic echo /rvio/ground_truth_error --once
ros2 topic echo /rvio_eval/xy_rmse --once
ros2 topic echo /rvio_eval/velocity_rmse --once
ros2 topic echo /rvio_eval/drift_percent --once
ros2 run rqt_image_view rqt_image_view /rvio/debug_image

# X estimate vs ground truth over time
ros2 run rqt_plot rqt_plot /rvio_plot/estimated_x/data /rvio_plot/ground_truth_x/data

# Y estimate vs ground truth over time
ros2 run rqt_plot rqt_plot /rvio_plot/estimated_y/data /rvio_plot/ground_truth_y/data

# Terminal x/y comparison topics update continuously; at mission end they hold
# the final estimated-vs-ground-truth values.
ros2 run rqt_plot rqt_plot \
  /rvio_plot/final_estimated_x/data /rvio_plot/final_ground_truth_x/data \
  /rvio_plot/final_estimated_y/data /rvio_plot/final_ground_truth_y/data
```

For optical-flow sign and camera-frame calibration, compare RVIO visual velocity
against Gazebo velocity directly:

```bash
ros2 run rqt_plot rqt_plot \
  /rvio_debug/world_vx_mps/data /rvio_debug/ground_truth_vx_mps/data

ros2 run rqt_plot rqt_plot \
  /rvio_debug/world_vy_mps/data /rvio_debug/ground_truth_vy_mps/data
```

If either line is consistently opposite sign or swapped between axes, tune:

```text
flow_x_sign
flow_y_sign
```

Raw flow diagnostics are also available:

```bash
ros2 run rqt_plot rqt_plot \
  /rvio_debug/median_flow_u_px/data /rvio_debug/range_flow_u_px/data

ros2 run rqt_plot rqt_plot \
  /rvio_debug/median_flow_v_px/data /rvio_debug/range_flow_v_px/data

ros2 topic echo /rvio_debug/velocity_sign_score
```

The estimator also publishes the current image-frame time delta and the number
of range-feature tracks accepted/rejected by the tracker sanity checks:

```bash
ros2 topic echo /rvio_debug/image_dt_sec
ros2 topic echo /rvio_debug/accepted_range_feature_count
ros2 topic echo /rvio_debug/rejected_range_feature_count
```

Calibration knobs can be passed to the mission launch:

```bash
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py \
  mission_mode:=rvio_test rvio_estimator_mode:=range_feature_3d \
  rvio_flow_x_sign:=-1.0 rvio_flow_y_sign:=-1.0 \
  rvio_max_range_feature_pixel_rate_pxps:=1000.0
```

### Useful Checks

After the mission launch starts, these checks help verify that the VM, PX4, and
QGroundControl are connected correctly:

```bash
# ROS 2 should see PX4/Gazebo bridged topics
ros2 topic list | grep -E 'fmu|drone|mission'

# Downward rangefinder should publish LaserScan data
ros2 topic echo /drone/down_rangefinder --once

# PX4 odometry should be available to the mission node
ros2 topic echo /fmu/out/vehicle_odometry --once
```

If QGroundControl does not connect, re-check the Mac host IP, VM network mode,
macOS firewall settings, and whether QGroundControl is listening on UDP `14550`.

## Extra credit -- 3D reconstruction (50 points)
Use RTAB-Map or a SLAM ecosystem of your choice to map both rocks, and export the world as a mesh file, and upload to your repo. Use git large file system (LFS) if needed. 

## License

This assignment is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0). 
For more details: https://creativecommons.org/licenses/by-nc-sa/4.0/ 
